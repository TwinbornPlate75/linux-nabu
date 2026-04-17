// SPDX-License-Identifier: GPL-2.0
// Simplified IDTP9418 driver for reverse charging and I2C communication
// Supports touch pen charging

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/pinctrl/consumer.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include "idtp9418.h"

#define REVERSE_DPING_CHECK_DELAY_MS 1000
#define REVERSE_CHG_CHECK_DELAY_MS 1000
#define CHARGE_MONITOR_INTERVAL 2 * HZ
#define IDTP_KFIFO_SIZE 128
#define REVERSE_FOD 500
#define LIMIT_SOC 85

#pragma pack(push, 1)
struct idtp9418_event {
	__u8 soc;
	__u8 is_charging;
	__u8 is_attached;
	__u8 charge_limit;
};
#pragma pack(pop)

struct idtp9418_device_info;

struct idtp9418_access_func {
	int (*read)(struct idtp9418_device_info *di, u16 reg, u8 *val);
	int (*write)(struct idtp9418_device_info *di, u16 reg, u8 val);
	int (*read_buf)(struct idtp9418_device_info *di, u16 reg, u8 *buf,
			u32 size);
	int (*write_buf)(struct idtp9418_device_info *di, u16 reg, u8 *buf,
			 u32 size);
	int (*mask_write)(struct idtp9418_device_info *di, u16 reg, u8 mask,
			  u8 val);
};

struct idtp9418_device_info {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct idtp9418_access_func bus;

	struct alarm reverse_dping_alarm;
	struct alarm reverse_chg_alarm;

	struct kfifo kfifo;
	wait_queue_head_t wait_queue;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;

	struct delayed_work irq_work;
	struct delayed_work hall_irq_work;
	struct delayed_work charge_monitor_work;
	struct delayed_work reverse_ept_type_work;

	struct gpio_desc *irq_gpiod;
	struct gpio_desc *hall3_gpiod;
	struct gpio_desc *hall4_gpiod;
	struct gpio_desc *reverse_gpiod;
	struct gpio_desc *reverse_boost_en_gpiod;

	bool is_attached;
	bool is_charging;
	int reverse_pen_soc;
	int charge_limit;
	int hall3_irq;
	int hall4_irq;
	int irq;
};

static enum power_supply_property idtp9418_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_ONLINE,
};

static struct regmap_config i2c_idtp9418_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static struct idtp9418_device_info *idtp9418_global_di;

static void idtp9418_push_event(struct idtp9418_device_info *di)
{
	struct idtp9418_event evt;

	evt.soc = (u8)di->reverse_pen_soc;
	evt.is_charging = (u8)di->is_charging;
	evt.is_attached = (u8)di->is_attached;
	evt.charge_limit = (u8)di->charge_limit;

	if (kfifo_avail(&di->kfifo) < sizeof(evt))
		kfifo_skip(&di->kfifo);
	kfifo_in(&di->kfifo, &evt, sizeof(evt));

	wake_up(&di->wait_queue);
}

static int idtp9418_kfifo_open(struct inode *inode, struct file *filp)
{
	filp->private_data = idtp9418_global_di;
	return 0;
}

static ssize_t idtp9418_kfifo_read(struct file *filp, char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct idtp9418_device_info *di = filp->private_data;
	struct idtp9418_event evt;

	if (count < sizeof(evt))
		return -EINVAL;

	if (kfifo_is_empty(&di->kfifo)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(di->wait_queue,
					     !kfifo_is_empty(&di->kfifo)))
			return -ERESTARTSYS;
	}

	if (kfifo_out(&di->kfifo, &evt, sizeof(evt)) != sizeof(evt))
		return -EIO;

	if (copy_to_user(buf, &evt, sizeof(evt)))
		return -EFAULT;

	return sizeof(evt);
}

static const struct file_operations idtp9418_fops = {
	.owner = THIS_MODULE,
	.open = idtp9418_kfifo_open,
	.read = idtp9418_kfifo_read,
};

static struct miscdevice idtp9418_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "idtp9418",
	.fops = &idtp9418_fops,
	.mode = 0666,
};

static int idtp9418_read(struct idtp9418_device_info *di, u16 reg, u8 *val)
{
	unsigned int temp;
	int rc;

	usleep_range(1000, 2000);
	rc = regmap_read(di->regmap, reg, &temp);
	if (rc >= 0)
		*val = (u8)temp;
	return rc;
}

static int idtp9418_write(struct idtp9418_device_info *di, u16 reg, u8 val)
{
	int rc = 0;

	usleep_range(1000, 2000);
	rc = regmap_write(di->regmap, reg, val);
	if (rc < 0)
		dev_err(di->dev, "[idt] idtp9418 write error: %d\n", rc);
	return rc;
}

static int idtp9418_masked_write(struct idtp9418_device_info *di, u16 reg,
				 u8 mask, u8 val)
{
	int rc = 0;

	rc = regmap_update_bits(di->regmap, reg, mask, val);
	if (rc < 0)
		dev_err(di->dev, "[idt] idtp9418 write mask error: %d\n", rc);
	return rc;
}

static int idtp9418_read_buffer(struct idtp9418_device_info *di, u16 reg,
				u8 *buf, u32 size)
{
	int rc = 0;

	while (size--) {
		rc = di->bus.read(di, reg++, buf++);
		if (rc < 0) {
			dev_err(di->dev, "[idt] read buf error: %d\n", rc);
			return rc;
		}
	}
	return rc;
}

static int idtp9418_write_buffer(struct idtp9418_device_info *di, u16 reg,
				 u8 *buf, u32 size)
{
	int rc = 0;

	while (size--) {
		rc = di->bus.write(di, reg++, *buf++);
		if (rc < 0) {
			dev_err(di->dev, "[idt] write error: %d\n", rc);
			return rc;
		}
	}
	return rc;
}

static void reverse_clrInt(struct idtp9418_device_info *di, u8 *buf, u32 size)
{
	di->bus.write_buf(di, REG_SYS_INT_CLR, buf, size);
	di->bus.write(di, REG_TX_CMD, TX_FOD_EN | TX_CLRINT);
}

static void idtp9418_set_reverse_gpio_state(struct idtp9418_device_info *di,
					    int enable)
{
	gpiod_set_value(di->reverse_gpiod, !!enable);
}

static void
idtp9418_set_reverse_boost_enable_gpio(struct idtp9418_device_info *di,
				       int enable)
{
	gpiod_set_value(di->reverse_boost_en_gpiod, !!enable);
}

static void idtp9418_set_reverse_gpio(struct idtp9418_device_info *di,
				      int enable)
{
	if (enable) {
		idtp9418_set_reverse_gpio_state(di, enable);
		idtp9418_set_reverse_boost_enable_gpio(di, enable);
	} else {
		idtp9418_set_reverse_boost_enable_gpio(di, enable);
		idtp9418_set_reverse_gpio_state(di, enable);
	}
}

static void idt_set_reverse_fod(struct idtp9418_device_info *di, int mw)
{
	u8 mw_l = mw & 0xff;
	u8 mw_h = mw >> 8;

	usleep_range(1000, 2000);
	di->bus.write(di, REG_FOD_LOW, mw_l);
	di->bus.write(di, REG_FOD_HIGH, mw_h);
	dev_info(di->dev, "set reverse fod: %d\n", mw);
}

static void idt_get_reverse_soc(struct idtp9418_device_info *di)
{
	u8 soc = 0;

	di->bus.read(di, REG_CHG_STATUS, &soc);
	if ((soc < 0) || (soc > 0x64)) {
		if (soc == 0xFF) {
			dev_info(di->dev, "[reverse] soc is default 0xFF\n");
			di->reverse_pen_soc = 0xFF;
		} else {
			dev_info(di->dev, "[reverse] soc illegal: %d\n", soc);
			return;
		}
	}
	dev_info(di->dev, "[reverse] soc is %d\n", soc);
	di->reverse_pen_soc = soc;
}

static void idtp9418_reverse_charge_enable(struct idtp9418_device_info *di)
{
	u8 mode = 0;

	idt_set_reverse_fod(di, REVERSE_FOD);
	for (int i = 0; i < 3; i++) {
		di->bus.write(di, REG_TX_CMD, TX_EN | TX_FOD_EN);
		di->bus.read(di, REG_TX_DATA, &mode);
		dev_info(di->dev, "tx data(0078): 0x%x\n", mode);
		if (mode & BIT(0)) {
			dev_info(di->dev, "reverse charging start success\n");
			di->is_charging = true;
			power_supply_changed(di->psy);
			return;
		}
		dev_err(di->dev, "set reverse charge failed, retry: %d\n", i);
	}
	dev_info(di->dev, "reverse charging failed start\n");
	idtp9418_set_reverse_gpio(di, false);
	di->is_charging = false;
	pm_relax(di->dev);
}

static void idtp9418_charge_monitor_work(struct work_struct *work)
{
	struct idtp9418_device_info *di = container_of(
		work, struct idtp9418_device_info, charge_monitor_work.work);
	static bool first_cycle = true;

	idt_get_reverse_soc(di);

	if (di->is_attached && di->reverse_pen_soc >= 0 &&
	    di->reverse_pen_soc < di->charge_limit)
		goto queue_work;

	idtp9418_set_reverse_gpio(di, false);
	first_cycle = false;
	di->is_charging = false;
	power_supply_changed(di->psy);
	goto push;

queue_work:
	schedule_delayed_work(&di->charge_monitor_work,
			      msecs_to_jiffies(CHARGE_MONITOR_INTERVAL));
	if (!first_cycle)
		return;
push:
	idtp9418_push_event(di);
	first_cycle = !first_cycle;
	if (!first_cycle)
		return;
	pm_relax(di->dev);
}

static void reverse_chg_alarm_cb(struct alarm *alarm, ktime_t now)
{
	struct idtp9418_device_info *di = container_of(
		alarm, struct idtp9418_device_info, reverse_chg_alarm);

	dev_info(di->dev, "Reverse Chg Alarm Triggered %lld\n",
		 ktime_to_ms(now));
	idtp9418_set_reverse_gpio(di, false);
	di->is_charging = false;
	power_supply_changed(di->psy);
}

static void reverse_dping_alarm_cb(struct alarm *alarm, ktime_t now)
{
	struct idtp9418_device_info *di = container_of(
		alarm, struct idtp9418_device_info, reverse_dping_alarm);

	dev_info(di->dev,
		 "Reverse Dping Alarm Triggered %lld, is_charging=%d\n",
		 ktime_to_ms(now), di->is_charging);
	idtp9418_set_reverse_gpio(di, false);
	di->is_charging = false;
	power_supply_changed(di->psy);
}

static void reverse_ept_type_get_work(struct work_struct *work)
{
	struct idtp9418_device_info *di = container_of(
		work, struct idtp9418_device_info, reverse_ept_type_work.work);
	int rc;
	u8 buf[2] = { 0 };
	u16 ept_val = 0;

	rc = di->bus.read_buf(di, REG_EPT_TYPE, buf, 2);
	if (rc < 0)
		dev_err(di->dev, "read tx ept type error: %d\n", rc);
	else {
		ept_val = buf[0] | (buf[1] << 8);
		dev_info(di->dev, "tx ept type: 0x%04x\n", ept_val);
		if (ept_val) {
			if ((ept_val & EPT_FOD) || (ept_val & EPT_CMD) ||
			    (ept_val & EPT_OCP) || (ept_val & EPT_OVP) ||
			    (ept_val & EPT_LVP) || (ept_val & EPT_OTP) ||
			    (ept_val & EPT_POCP)) {
				dev_info(
					di->dev,
					"TX mode in ept and disable reverse charging\n");
				idtp9418_set_reverse_gpio(di, false);
			} else if (ept_val & EPT_CEP_TIMEOUT) {
				dev_info(di->dev, "recheck ping state\n");
			}
			di->is_charging = false;
			power_supply_changed(di->psy);
		}
	}
}

static void idtp9418_hall_irq_work(struct work_struct *work)
{
	struct idtp9418_device_info *di = container_of(
		work, struct idtp9418_device_info, hall_irq_work.work);
	bool attached = false;

	if (!gpiod_get_value(di->hall3_gpiod))
		attached = true;
	if (!attached && !gpiod_get_value(di->hall4_gpiod))
		attached = true;
	di->is_attached = attached;

	if (attached) {
		dev_info(di->dev, "hall: pen attach\n");
		idtp9418_set_reverse_gpio(di, true);
		idtp9418_reverse_charge_enable(di);
		alarm_start_relative(&di->reverse_dping_alarm,
				     ms_to_ktime(REVERSE_DPING_CHECK_DELAY_MS));
	} else {
		pm_relax(di->dev);
	}
}

static irqreturn_t idtp9418_irq_handler(int irq, void *dev_id)
{
	struct idtp9418_device_info *di = dev_id;

	pm_stay_awake(di->dev);
	schedule_delayed_work(&di->irq_work, msecs_to_jiffies(10));
	return IRQ_HANDLED;
}

static irqreturn_t idtp9418_hall_irq_handler(int irq, void *dev_id)
{
	struct idtp9418_device_info *di = dev_id;

	pm_stay_awake(di->dev);
	schedule_delayed_work(&di->hall_irq_work, msecs_to_jiffies(10));
	return IRQ_HANDLED;
}

static bool reverse_need_irq_cleared(struct idtp9418_device_info *di, u32 val)
{
	u8 int_buf[4];
	u32 int_val;
	int rc = -1;

	rc = di->bus.read_buf(di, REG_SYS_INT, int_buf, 4);
	if (rc < 0) {
		dev_err(di->dev, "%s: read int state error\n", __func__);
		return false;
	}
	int_val = int_buf[0] | (int_buf[1] << 8) | (int_buf[2] << 16) |
		  (int_buf[3] << 24);
	if (int_val && (int_val == val)) {
		dev_info(di->dev, "irq clear wrong, retry: 0x%08x\n", int_val);
		return true;
	}
	return false;
}

static void idtp9418_irq_work(struct work_struct *work)
{
	struct idtp9418_device_info *di =
		container_of(work, struct idtp9418_device_info, irq_work.work);
	u8 int_buf[4] = { 0 };
	u8 clr_buf[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
	u32 int_val = 0;

	if (gpiod_get_value(di->irq_gpiod))
		goto reverse_out;

	if (di->bus.read_buf(di, REG_SYS_INT, int_buf, 4) < 0) {
		dev_err(di->dev, "[idt]read int state error\n");
		goto reverse_out;
	}

	if (!di->is_charging)
		goto reverse_out;

	reverse_clrInt(di, int_buf, 4);
	int_val = int_buf[0] | (int_buf[1] << 8) | (int_buf[2] << 16) |
		  (int_buf[3] << 24);
	dev_info(di->dev, "[idt] TRX int: 0x%08x\n", int_val);

	msleep(5);
	if (reverse_need_irq_cleared(di, int_val)) {
		reverse_clrInt(di, clr_buf, 4);
		msleep(5);
	}

	if (int_val & INT_EPT_TYPE) {
		schedule_delayed_work(&di->reverse_ept_type_work, 0);
	}

	if (int_val & INT_GET_DPING) {
		dev_info(di->dev,
			 "[idt] TRX get dping and disable reverse charging\n");
		idtp9418_set_reverse_gpio(di, false);
		di->is_charging = false;
		power_supply_changed(di->psy);
	}

	if (int_val & INT_START_DPING) {
		alarm_start_relative(&di->reverse_chg_alarm,
				     ms_to_ktime(REVERSE_CHG_CHECK_DELAY_MS));
		if (alarm_cancel(&di->reverse_dping_alarm) < 0)
			dev_err(di->dev,
				"Couldn't cancel reverse_dping_alarm\n");
	}

	if (int_val & INT_GET_CFG) {
		dev_info(di->dev, "TRX get cfg, cancel alarm\n");
		if (alarm_cancel(&di->reverse_chg_alarm) < 0)
			dev_err(di->dev, "Couldn't cancel reverse_chg_alarm\n");
		schedule_delayed_work(
			&di->charge_monitor_work,
			msecs_to_jiffies(CHARGE_MONITOR_INTERVAL));
	}

reverse_out:
	pm_relax(di->dev);
	return;
}

static int idtp9418_pinctrl_init(struct idtp9418_device_info *di)
{
	int ret;

	di->pinctrl = devm_pinctrl_get(di->dev);
	if (IS_ERR(di->pinctrl)) {
		dev_err(di->dev, "failed to get pinctrl\n");
		return PTR_ERR(di->pinctrl);
	}
	di->pins_active = pinctrl_lookup_state(di->pinctrl, "idt_active");
	if (IS_ERR(di->pins_active)) {
		dev_err(di->dev, "failed to get active pinctrl state\n");
		return PTR_ERR(di->pins_active);
	}
	di->pins_sleep = pinctrl_lookup_state(di->pinctrl, "idt_suspend");
	if (IS_ERR(di->pins_sleep)) {
		dev_err(di->dev, "failed to get sleep pinctrl state\n");
		return PTR_ERR(di->pins_sleep);
	}
	ret = pinctrl_select_state(di->pinctrl, di->pins_active);
	if (ret < 0) {
		dev_err(di->dev, "failed to select active pinctrl state\n");
		return ret;
	}
	dev_info(di->dev, "pinctrl initialization successful\n");
	return 0;
}

static int idtp9418_parse_dt(struct idtp9418_device_info *di)
{
	di->irq_gpiod = devm_gpiod_get(di->dev, "idt,irq", GPIOD_IN);
	if (IS_ERR(di->irq_gpiod)) {
		dev_err(di->dev, "get irq gpio failed: %ld\n",
			PTR_ERR(di->irq_gpiod));
		return PTR_ERR(di->irq_gpiod);
	}

	di->hall3_gpiod =
		devm_gpiod_get_optional(di->dev, "idt,hall3", GPIOD_IN);
	if (IS_ERR(di->hall3_gpiod)) {
		dev_err(di->dev, "get hall3 gpio failed: %ld\n",
			PTR_ERR(di->hall3_gpiod));
		di->hall3_gpiod = NULL;
	}

	di->hall4_gpiod =
		devm_gpiod_get_optional(di->dev, "idt,hall4", GPIOD_IN);
	if (IS_ERR(di->hall4_gpiod)) {
		dev_err(di->dev, "get hall4 gpio failed: %ld\n",
			PTR_ERR(di->hall4_gpiod));
		di->hall4_gpiod = NULL;
	}

	di->reverse_gpiod =
		devm_gpiod_get(di->dev, "idt,reverse-enable", GPIOD_OUT_LOW);
	if (IS_ERR(di->reverse_gpiod)) {
		dev_err(di->dev, "get reverse gpio failed: %ld\n",
			PTR_ERR(di->reverse_gpiod));
		return PTR_ERR(di->reverse_gpiod);
	}

	di->reverse_boost_en_gpiod = devm_gpiod_get(
		di->dev, "idt,reverse-boost-enable", GPIOD_OUT_LOW);
	if (IS_ERR(di->reverse_boost_en_gpiod)) {
		dev_err(di->dev, "get reverse_boost_enable gpio failed: %ld\n",
			PTR_ERR(di->reverse_boost_en_gpiod));
		return PTR_ERR(di->reverse_boost_en_gpiod);
	}

	return 0;
}

static int idtp9418_irq_request(struct idtp9418_device_info *di)
{
	int ret = 0;

	di->irq = gpiod_to_irq(di->irq_gpiod);
	if (di->irq < 0) {
		dev_err(di->dev, "gpiod_to_irq failed, irq=%d\n", di->irq);
		return di->irq;
	}
	ret = devm_request_irq(di->dev, di->irq, idtp9418_irq_handler,
			       IRQF_TRIGGER_FALLING, IDT_DRIVER_NAME, di);
	if (ret < 0) {
		dev_err(di->dev, "request irq failed, ret=%d\n", ret);
		return ret;
	}
	ret = enable_irq_wake(di->irq);
	if (ret < 0) {
		dev_err(di->dev, "enable irq wake failed, ret=%d\n", ret);
		return ret;
	}
	return ret;
}

static int idtp9418_hall_irq_request(struct idtp9418_device_info *di)
{
	int ret = 0;

	di->hall3_irq = gpiod_to_irq(di->hall3_gpiod);
	if (di->hall3_irq < 0)
		return di->hall3_irq;
	ret = devm_request_irq(di->dev, di->hall3_irq,
			       idtp9418_hall_irq_handler,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "idtp_hall3", di);
	if (ret < 0)
		return ret;
	enable_irq_wake(di->hall3_irq);

	di->hall4_irq = gpiod_to_irq(di->hall4_gpiod);
	if (di->hall4_irq < 0)
		return di->hall4_irq;
	ret = devm_request_irq(di->dev, di->hall4_irq,
			       idtp9418_hall_irq_handler,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "idtp_hall4", di);
	if (ret < 0)
		return ret;
	enable_irq_wake(di->hall4_irq);
	return ret;
}

static int idtp9418_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct idtp9418_device_info *di = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!di->is_charging)
			return 0;
		val->intval = di->reverse_pen_soc;
		if (val->intval < 0)
			val->intval = 0;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = di->charge_limit;
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->is_charging;
		return 0;
	default:
		return -EINVAL;
	}
}

static int idtp9418_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
}

static int idtp9418_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct idtp9418_device_info *di = power_supply_get_drvdata(psy);

	if (psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD) {
		if (val->intval < 0 || val->intval > 95)
			return -EINVAL;
		di->charge_limit = val->intval;
		dev_info(di->dev, "Charge limit set to %d%%\n",
			 di->charge_limit);
		return 0;
	}
	return -EINVAL;
}

static int idtp9418_probe(struct i2c_client *client)
{
	struct idtp9418_device_info *di;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct power_supply_config idtp_cfg = { 0 };
	struct power_supply_desc *psy_desc;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &client->dev;
	di->reverse_pen_soc = 255;
	di->charge_limit = LIMIT_SOC;
	di->is_charging = false;
	di->is_attached = false;

	di->regmap = devm_regmap_init_i2c(client, &i2c_idtp9418_regmap_config);
	if (!di->regmap)
		return -ENODEV;

	di->bus.read = idtp9418_read;
	di->bus.write = idtp9418_write;
	di->bus.read_buf = idtp9418_read_buffer;
	di->bus.write_buf = idtp9418_write_buffer;
	di->bus.mask_write = idtp9418_masked_write;

	INIT_DELAYED_WORK(&di->irq_work, idtp9418_irq_work);
	INIT_DELAYED_WORK(&di->hall_irq_work, idtp9418_hall_irq_work);
	init_waitqueue_head(&di->wait_queue);

	ret = kfifo_alloc(&di->kfifo, IDTP_KFIFO_SIZE, GFP_KERNEL);
	if (ret) {
		dev_err(di->dev, "Failed to allocate kfifo\n");
		return ret;
	}

	device_init_wakeup(&client->dev, true);
	i2c_set_clientdata(client, di);

	ret = idtp9418_parse_dt(di);
	if (ret < 0)
		goto cleanup;

	ret = idtp9418_pinctrl_init(di);
	if (ret < 0)
		goto cleanup;

	ret = idtp9418_irq_request(di);
	if (ret < 0)
		goto cleanup;

	ret = idtp9418_hall_irq_request(di);
	if (ret < 0)
		goto cleanup;

	INIT_DELAYED_WORK(&di->reverse_ept_type_work,
			  reverse_ept_type_get_work);
	INIT_DELAYED_WORK(&di->charge_monitor_work,
			  idtp9418_charge_monitor_work);
	alarm_init(&di->reverse_dping_alarm, ALARM_BOOTTIME,
		   reverse_dping_alarm_cb);
	alarm_init(&di->reverse_chg_alarm, ALARM_BOOTTIME,
		   reverse_chg_alarm_cb);

	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc) {
		ret = -ENOMEM;
		goto cleanup;
	}
	psy_desc->name = IDT_DRIVER_NAME;
	psy_desc->type = POWER_SUPPLY_TYPE_WIRELESS;
	psy_desc->properties = idtp9418_props;
	psy_desc->num_properties = ARRAY_SIZE(idtp9418_props);
	psy_desc->get_property = idtp9418_get_property;
	psy_desc->set_property = idtp9418_set_property;
	psy_desc->property_is_writeable = idtp9418_property_is_writeable;

	idtp_cfg.drv_data = di;
	di->psy = devm_power_supply_register(&client->dev, psy_desc, &idtp_cfg);
	if (IS_ERR(di->psy)) {
		dev_err(&client->dev, "Failed to register power supply\n");
		ret = PTR_ERR(di->psy);
		goto cleanup;
	}

	idtp9418_global_di = di;
	ret = misc_register(&idtp9418_miscdev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register misc device\n");
		goto cleanup;
	}

	dev_info(di->dev, "[idt] success probe idtp9418 driver\n");
	return 0;

cleanup:
	misc_deregister(&idtp9418_miscdev);
	idtp9418_global_di = NULL;
	cancel_delayed_work_sync(&di->irq_work);
	cancel_delayed_work_sync(&di->hall_irq_work);
	cancel_delayed_work_sync(&di->reverse_ept_type_work);
	cancel_delayed_work_sync(&di->charge_monitor_work);
	kfifo_free(&di->kfifo);
	if (di->pinctrl)
		pinctrl_put(di->pinctrl);
	i2c_set_clientdata(client, NULL);
	return ret;
}

static void idtp9418_remove(struct i2c_client *client)
{
	struct idtp9418_device_info *di = i2c_get_clientdata(client);

	misc_deregister(&idtp9418_miscdev);
	idtp9418_global_di = NULL;
	cancel_delayed_work_sync(&di->irq_work);
	cancel_delayed_work_sync(&di->hall_irq_work);
	cancel_delayed_work_sync(&di->reverse_ept_type_work);
	cancel_delayed_work_sync(&di->charge_monitor_work);
	kfifo_free(&di->kfifo);
	i2c_set_clientdata(client, NULL);
}

static const struct i2c_device_id idtp9418_id[] = {
	{ IDT_DRIVER_NAME, 0 },
	{}
};

static const struct of_device_id idt_match_table[] = {
	{ .compatible = "idt,p9418" },
	{}
};
MODULE_DEVICE_TABLE(i2c, idtp9418_id);

static struct i2c_driver idtp9418_driver = {
	.driver = {
		.name = IDT_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = idt_match_table,
	},
	.probe = idtp9418_probe,
	.remove = idtp9418_remove,
	.id_table = idtp9418_id,
};

module_i2c_driver(idtp9418_driver);
MODULE_DESCRIPTION("IDTP9418 Reverse Charging Driver");
MODULE_LICENSE("GPL");
