// SPDX-License-Identifier: GPL-2.0-only
/*
 * Maxim DS28E16 battery authenticator
 */

#include <crypto/hash.h>
#include <crypto/utils.h>
#include <linux/bitops.h>
#include <linux/crc16.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "qcom_bms.h"

#define DS28E16_CMD_RELEASE_BYTE 0xaa
#define DS28E16_CMD_SKIP_ROM 0xcc
#define DS28E16_CMD_START 0x66
#define DS28E16_CMD_READ_MEM 0x44
#define DS28E16_CMD_READ_STATUS 0xaa
#define DS28E16_CMD_COMP_READ_AUTH 0xa5
#define DS28E16_CMD_COMP_S_SECRET 0x3c
#define DS28E16_RESULT_SUCCESS 0xaa

#define DS28E16_READ_DELAY_MS 50
#define DS28E16_COMPUTE_DELAY_MS 100
#define DS28E16_COMMAND_RETRIES 4
#define DS28E16_INITIAL_DELAY_MS 1500
#define DS28E16_FAILURE_RETRY_MS 60000

#define DS28E16_PAGE_DATA_SIZE 16
#define DS28E16_MAC_SIZE 32
#define DS28E16_ROM_SIZE 8
#define DS28E16_AUTH_PAGE 1
#define DS28E16_SECRET_PAGE 0

#define TLMM_DRV_STRENGTH_16MA (0x7 << 6)
#define TLMM_GPIO_OUTPUT BIT(9)
#define TLMM_GPIO_PULL_UP 0x3
#define TLMM_OUTPUT_HIGH BIT(1)
#define TLMM_OUTPUT_LOW BIT(0)

struct ds28e16 {
	struct device *dev;
	void __iomem *gpio_cfg;
	void __iomem *gpio_io;
	raw_spinlock_t bus_lock;
	struct delayed_work auth_work;
	struct crypto_shash *hmac;
	enum qcom_bms_auth_role role;
	bool authenticated;
	bool manid_set;
	bool force_reauth;
	u8 manid[2];
	u32 last_error;
	struct dentry *debugfs;
};

static const u8 ds28e16_master_secret[DS28E16_MAC_SIZE] = {
	0x0c, 0x99, 0x2b, 0xd3, 0x95, 0xdb, 0xa0, 0xb4, 0xef, 0x07, 0xb3,
	0xd8, 0x75, 0xf3, 0xc7, 0xae, 0xda, 0xc4, 0x41, 0x2f, 0x48, 0x93,
	0xb5, 0xd9, 0xe1, 0xe5, 0x4b, 0x20, 0x9b, 0xf3, 0x77, 0x39,
};

/* DS28E16 anonymous page-authentication MAC input, in wire order. */
struct ds28e16_mac_message {
	u8 rom_id[DS28E16_ROM_SIZE];
	u8 page[DS28E16_PAGE_DATA_SIZE];
	u8 scratchpad[DS28E16_PAGE_DATA_SIZE];
	u8 challenge[DS28E16_MAC_SIZE];
	u8 page_number;
	u8 manid[2];
};

static inline void ds28e16_config_output(struct ds28e16 *ds)
{
	writel_relaxed(TLMM_DRV_STRENGTH_16MA | TLMM_GPIO_OUTPUT |
			       TLMM_GPIO_PULL_UP,
		       ds->gpio_cfg);
}

static inline void ds28e16_config_input(struct ds28e16 *ds)
{
	writel_relaxed(TLMM_DRV_STRENGTH_16MA | TLMM_GPIO_PULL_UP,
		       ds->gpio_cfg);
}

static inline void ds28e16_drive_high(struct ds28e16 *ds)
{
	writel_relaxed(TLMM_OUTPUT_HIGH, ds->gpio_io);
}

static inline void ds28e16_drive_low(struct ds28e16 *ds)
{
	writel_relaxed(TLMM_OUTPUT_LOW, ds->gpio_io);
}

static bool ds28e16_reset(struct ds28e16 *ds)
{
	unsigned long flags;
	bool present;

	raw_spin_lock_irqsave(&ds->bus_lock, flags);
	ds28e16_config_output(ds);
	ds28e16_drive_low(ds);
	udelay(50);
	ds28e16_drive_high(ds);
	ds28e16_config_input(ds);
	udelay(7);
	present = !(readl_relaxed(ds->gpio_io) & BIT(0));
	udelay(50);
	raw_spin_unlock_irqrestore(&ds->bus_lock, flags);

	return present;
}

static u8 ds28e16_read_bit_locked(struct ds28e16 *ds)
{
	u32 value;

	ds28e16_config_output(ds);
	ds28e16_drive_low(ds);
	udelay(1);
	ds28e16_config_input(ds);
	ndelay(500);
	value = readl_relaxed(ds->gpio_io);
	udelay(5);
	ds28e16_drive_high(ds);
	ds28e16_config_output(ds);
	udelay(6);

	return value & BIT(0);
}

static void ds28e16_write_bit_locked(struct ds28e16 *ds, bool bit)
{
	ds28e16_drive_low(ds);
	udelay(1);
	if (bit)
		ds28e16_drive_high(ds);
	udelay(10);
	ds28e16_drive_high(ds);
	udelay(6);
}

static u8 ds28e16_read_byte(struct ds28e16 *ds)
{
	unsigned long flags;
	u8 value = 0;
	int i;

	raw_spin_lock_irqsave(&ds->bus_lock, flags);
	for (i = 0; i < 8; i++)
		value |= ds28e16_read_bit_locked(ds) << i;
	raw_spin_unlock_irqrestore(&ds->bus_lock, flags);

	return value;
}

static void ds28e16_write_byte(struct ds28e16 *ds, u8 value)
{
	unsigned long flags;
	int i;

	raw_spin_lock_irqsave(&ds->bus_lock, flags);
	ds28e16_config_output(ds);
	for (i = 0; i < 8; i++)
		ds28e16_write_bit_locked(ds, value & BIT(i));
	raw_spin_unlock_irqrestore(&ds->bus_lock, flags);
}

static bool ds28e16_crc16_valid(const u8 *data, size_t len)
{
	return crc16(0, data, len) == 0xb001;
}

static int ds28e16_standard_command(struct ds28e16 *ds, const u8 *request,
				    size_t request_len, unsigned int delay_ms,
				    u8 *response, size_t response_len)
{
	u8 tx[40];
	u8 rx[DS28E16_MAC_SIZE + 3];
	size_t tx_len = 0;
	size_t i;
	u8 length;

	if (request_len + 3 > sizeof(tx) || response_len + 2 > sizeof(rx))
		return -EINVAL;

	if (!ds28e16_reset(ds))
		return -ENODEV;

	ds28e16_write_byte(ds, DS28E16_CMD_SKIP_ROM);
	tx[tx_len++] = DS28E16_CMD_START;
	memcpy(&tx[tx_len], request, request_len);
	tx_len += request_len;
	for (i = 0; i < tx_len; i++)
		ds28e16_write_byte(ds, tx[i]);

	tx[tx_len++] = ds28e16_read_byte(ds);
	tx[tx_len++] = ds28e16_read_byte(ds);
	if (!ds28e16_crc16_valid(tx, tx_len))
		return -EBADMSG;

	if (delay_ms) {
		ds28e16_write_byte(ds, DS28E16_CMD_RELEASE_BYTE);
		msleep(delay_ms);
	}

	/* Dummy byte followed by response length. */
	ds28e16_read_byte(ds);
	length = ds28e16_read_byte(ds);
	if (length != response_len)
		return -EMSGSIZE;

	for (i = 0; i < response_len + 2; i++)
		rx[i] = ds28e16_read_byte(ds);

	/* The response CRC covers its length byte as well. */
	tx[0] = length;
	memcpy(&tx[1], rx, response_len + 2);
	if (!ds28e16_crc16_valid(tx, response_len + 3))
		return -EBADMSG;

	memcpy(response, rx, response_len);
	return 0;
}

static int ds28e16_read_status(struct ds28e16 *ds, u8 manid[2])
{
	const u8 request[] = { 1, DS28E16_CMD_READ_STATUS };
	u8 response[7];
	int ret;

	ret = ds28e16_standard_command(ds, request, sizeof(request),
				       DS28E16_READ_DELAY_MS, response,
				       sizeof(response));
	if (ret)
		return ret;
	if (response[0] != DS28E16_RESULT_SUCCESS)
		return -EIO;

	/* Read-Status payload byte 4 is the first MANID byte. */
	manid[0] = response[5];
	manid[1] = 0;
	return 0;
}

static int ds28e16_read_page(struct ds28e16 *ds, u8 page,
			     u8 data[DS28E16_PAGE_DATA_SIZE])
{
	u8 request[] = { 2, DS28E16_CMD_READ_MEM, page & 0x3 };
	u8 response[33];
	int ret;

	ret = ds28e16_standard_command(ds, request, sizeof(request),
				       DS28E16_READ_DELAY_MS, response,
				       sizeof(response));
	if (ret)
		return ret;
	if (response[0] != DS28E16_RESULT_SUCCESS)
		return -EIO;

	memcpy(data, &response[1], DS28E16_PAGE_DATA_SIZE);
	return 0;
}

static int ds28e16_compute_session_secret(struct ds28e16 *ds)
{
	u8 request[36] = { 35, DS28E16_CMD_COMP_S_SECRET };
	u8 response[1];
	int ret;

	request[2] = (DS28E16_SECRET_PAGE & 0x3) | BIT(2) | 0xe0;
	request[3] = 0x08;
	memset(&request[4], 0xaa, DS28E16_MAC_SIZE);

	ret = ds28e16_standard_command(ds, request, sizeof(request),
				       DS28E16_COMPUTE_DELAY_MS, response,
				       sizeof(response));
	if (ret)
		return ret;

	return response[0] == DS28E16_RESULT_SUCCESS ? 0 : -EIO;
}

static int ds28e16_compute_page_mac(struct ds28e16 *ds,
				    const u8 challenge[DS28E16_MAC_SIZE],
				    u8 mac[DS28E16_MAC_SIZE])
{
	u8 request[36] = { 35, DS28E16_CMD_COMP_READ_AUTH };
	u8 response[DS28E16_MAC_SIZE + 1];
	int ret;

	request[2] = (DS28E16_AUTH_PAGE & 0x3) | 0xe0;
	request[3] = 0x02;
	memcpy(&request[4], challenge, DS28E16_MAC_SIZE);

	ret = ds28e16_standard_command(ds, request, sizeof(request),
				       DS28E16_COMPUTE_DELAY_MS, response,
				       sizeof(response));
	if (ret)
		return ret;
	if (response[0] != DS28E16_RESULT_SUCCESS)
		return -EIO;

	memcpy(mac, &response[1], DS28E16_MAC_SIZE);
	return 0;
}

#define ds28e16_retry(__cmd)                                          \
	({                                                            \
		int __ret = -EIO, __i;                                \
		for (__i = 0; __i < DS28E16_COMMAND_RETRIES; __i++) { \
			__ret = (__cmd);                              \
			if (!__ret)                                   \
				break;                                \
		}                                                     \
		__ret;                                                \
	})

static int ds28e16_authenticate(struct ds28e16 *ds)
{
	struct ds28e16_mac_message message = {
		.rom_id = { [0 ... DS28E16_ROM_SIZE - 1] = 0xff },
		.page_number = DS28E16_AUTH_PAGE,
	};
	u8 device_mac[DS28E16_MAC_SIZE];
	u8 host_mac[DS28E16_MAC_SIZE];
	int ret;

	/*
	 * Anonymous mode keeps the ROM ID as 0xff, but the MAC still covers
	 * the chip's MANID.  Read it from Read-Status unless a fixed value was
	 * provided in the device tree.
	 */
	if (ds->manid_set) {
		message.manid[0] = ds->manid[0];
		message.manid[1] = ds->manid[1];
	} else {
		ret = ds28e16_retry(ds28e16_read_status(ds, message.manid));
		if (ret)
			return ret;
	}

	ret = ds28e16_retry(ds28e16_compute_session_secret(ds));
	if (ret)
		return ret;
	ret = ds28e16_retry(
		ds28e16_compute_page_mac(ds, message.challenge, device_mac));
	if (ret)
		return ret;
	ret = ds28e16_retry(
		ds28e16_read_page(ds, DS28E16_AUTH_PAGE, message.page));
	if (ret)
		return ret;

	ret = crypto_shash_tfm_digest(ds->hmac, (u8 *)&message, sizeof(message),
				      host_mac);
	if (ret)
		return ret;

	return crypto_memneq(host_mac, device_mac, sizeof(host_mac)) ?
		       -EKEYREJECTED :
		       0;
}

static void ds28e16_auth_work(struct work_struct *work)
{
	struct ds28e16 *ds = container_of(work, struct ds28e16, auth_work.work);
	bool old_authenticated = READ_ONCE(ds->authenticated);
	bool authenticated;
	int ret;

	if (old_authenticated && !READ_ONCE(ds->force_reauth))
		return;

	WRITE_ONCE(ds->force_reauth, false);
	ret = ds28e16_authenticate(ds);
	ds->last_error = ret < 0 ? -ret : 0;
	authenticated = !ret;
	ds->authenticated = authenticated;

	if (authenticated != old_authenticated)
		qcom_bms_authentication_changed(ds->dev, authenticated);

	if (authenticated) {
		if (!old_authenticated)
			dev_info(ds->dev, "battery authentication passed\n");
		return;
	}

	dev_warn_ratelimited(ds->dev, "battery authentication failed: %d\n",
			     ret);
	mod_delayed_work(system_wq, &ds->auth_work,
			 msecs_to_jiffies(DS28E16_FAILURE_RETRY_MS));
}

static void ds28e16_free_hmac(void *data)
{
	crypto_free_shash(data);
}

static ssize_t ds28e16_reauthenticate_write(struct file *file,
					    const char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct ds28e16 *ds = file->private_data;

	WRITE_ONCE(ds->force_reauth, true);
	mod_delayed_work(system_wq, &ds->auth_work, 0);
	return count;
}

static const struct file_operations ds28e16_reauthenticate_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = ds28e16_reauthenticate_write,
	.llseek = noop_llseek,
};

static void ds28e16_debugfs_init(struct ds28e16 *ds)
{
	ds->debugfs = debugfs_create_dir(dev_name(ds->dev), NULL);
	if (IS_ERR_OR_NULL(ds->debugfs)) {
		ds->debugfs = NULL;
		return;
	}

	debugfs_create_bool("authenticated", 0444, ds->debugfs,
			    &ds->authenticated);
	debugfs_create_u32("last_error", 0444, ds->debugfs, &ds->last_error);
	debugfs_create_file("reauthenticate", 0200, ds->debugfs, ds,
			    &ds28e16_reauthenticate_fops);
}

static int ds28e16_probe(struct platform_device *pdev)
{
	struct gpio_desc *gpio;
	struct ds28e16 *ds;
	const char *role;
	u32 manid[2];
	u32 cfg_addr;
	u32 io_offset;
	int ret;

	ds = devm_kzalloc(&pdev->dev, sizeof(*ds), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

	ds->dev = &pdev->dev;
	raw_spin_lock_init(&ds->bus_lock);
	ds->manid_set = !of_property_read_u32_array(pdev->dev.of_node,
						    "maxim,manid", manid, 2);
	if (ds->manid_set) {
		ds->manid[0] = manid[0];
		ds->manid[1] = manid[1];
	}

	ret = of_property_read_string(pdev->dev.of_node, "maxim,auth-role",
				      &role);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "missing maxim,auth-role\n");
	if (!strcmp(role, "primary")) {
		ds->role = QCOM_BMS_AUTH_PRIMARY;
	} else if (!strcmp(role, "secondary")) {
		ds->role = QCOM_BMS_AUTH_SECONDARY;
	} else {
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid authentication role %s\n", role);
	}

	ret = of_property_read_u32(pdev->dev.of_node,
				   "maxim,gpio-config-address", &cfg_addr);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "missing GPIO config address\n");
	ret = of_property_read_u32(pdev->dev.of_node, "maxim,gpio-io-offset",
				   &io_offset);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "missing GPIO I/O offset\n");
	if (!io_offset || io_offset > 0x1000)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid GPIO I/O offset %#x\n",
				     io_offset);

	gpio = devm_gpiod_get(&pdev->dev, NULL, GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(gpio),
				     "failed to request 1-Wire GPIO\n");

	dev_info(&pdev->dev, "1-Wire GPIO %d, cfg %#x, io offset %#x\n",
		 desc_to_gpio(gpio), cfg_addr, io_offset);

	ds->gpio_cfg =
		devm_ioremap(&pdev->dev, cfg_addr, io_offset + sizeof(u32));
	if (!ds->gpio_cfg)
		return dev_err_probe(&pdev->dev, -ENOMEM,
				     "failed to map GPIO registers\n");
	ds->gpio_io = (u8 __iomem *)ds->gpio_cfg + io_offset;
	ds28e16_drive_high(ds);
	ds28e16_config_output(ds);

	ds->hmac = crypto_alloc_shash("hmac(sha3-256)", 0, 0);
	if (IS_ERR(ds->hmac))
		return dev_err_probe(&pdev->dev, PTR_ERR(ds->hmac),
				     "failed to allocate SHA3 HMAC\n");
	ret = devm_add_action_or_reset(&pdev->dev, ds28e16_free_hmac, ds->hmac);
	if (ret)
		return ret;
	ret = crypto_shash_setkey(ds->hmac, ds28e16_master_secret,
				  DS28E16_MAC_SIZE);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to set authentication key\n");

	ret = devm_delayed_work_autocancel(&pdev->dev, &ds->auth_work,
					   ds28e16_auth_work);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, ds);
	ret = qcom_bms_register_authenticator(&pdev->dev, ds->role);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register authenticator\n");

	ds28e16_debugfs_init(ds);
	mod_delayed_work(system_wq, &ds->auth_work,
			 msecs_to_jiffies(DS28E16_INITIAL_DELAY_MS));
	return 0;
}

static void ds28e16_remove(struct platform_device *pdev)
{
	struct ds28e16 *ds = platform_get_drvdata(pdev);

	debugfs_remove_recursive(ds->debugfs);
	cancel_delayed_work_sync(&ds->auth_work);
	qcom_bms_unregister_authenticator(&pdev->dev);
}

static const struct of_device_id ds28e16_of_match[] = {
	{ .compatible = "maxim,ds28e16-xiaomi" },
	{}
};
MODULE_DEVICE_TABLE(of, ds28e16_of_match);

static struct platform_driver ds28e16_driver = {
	.probe = ds28e16_probe,
	.remove = ds28e16_remove,
	.driver = {
		.name = "ds28e16-xiaomi",
		.of_match_table = ds28e16_of_match,
	},
};
module_platform_driver(ds28e16_driver);

MODULE_DESCRIPTION("Maxim DS28E16 Xiaomi battery authenticator");
MODULE_LICENSE("GPL");
