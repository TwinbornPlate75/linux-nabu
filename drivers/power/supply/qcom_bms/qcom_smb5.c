// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Ltd.
 * Author: Casey Connolly <casey.connolly@linaro.org>
 *
 * This driver is for the switch-mode battery charger and boost
 * hardware found in PM8150b / PM7250b PMICs (SMB5 generation).
 */

#include <linux/bitops.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "qcom_bms.h"

#define CHGR_BASE 0x1000
#define DCDC_BASE 0x1100
#define USBIN_BASE 0x1300
#define TYPEC_BASE 0x1500
#define MISC_BASE 0x1600

#define SMB5_REG(chip, addr) ((chip)->base + (addr) - CHGR_BASE)

#define BATTERY_CHARGER_STATUS_1 (CHGR_BASE + 0x06)
#define BATTERY_CHARGER_STATUS_MASK GENMASK(2, 0)
#define BATTERY_CHARGER_STATUS_2 (CHGR_BASE + 0x07)
#define BATTERY_CHARGER_STATUS_2_BAT_OV_BIT BIT(1)
#define BATTERY_CHARGER_STATUS_7 (CHGR_BASE + 0x0D)
#define BAT_TEMP_STATUS_HOT_SOFT_BIT BIT(5)
#define BAT_TEMP_STATUS_COLD_SOFT_BIT BIT(4)
#define BAT_TEMP_STATUS_TOO_HOT_BIT BIT(3)
#define BAT_TEMP_STATUS_TOO_COLD_BIT BIT(2)

#define CHARGING_ENABLE_CMD (CHGR_BASE + 0x42)
#define CHARGING_ENABLE_CMD_BIT BIT(0)

#define CHGR_CFG2 (CHGR_BASE + 0x51)
#define RECHG_MASK GENMASK(2, 1)
#define SOC_BASED_RECHG_BIT GENMASK(2, 1)

#define FAST_CHARGE_CURRENT_CFG (CHGR_BASE + 0x61)

#define CHGR_ADC_ITERM_UP_THD_MSB (CHGR_BASE + 0x67)
#define CHGR_ADC_ITERM_UP_THD_LSB (CHGR_BASE + 0x68)
#define CHGR_ADC_TERM_CFG (CHGR_BASE + 0x6C)
#define TERM_BASED_ON_SAMPLE_CNT BIT(0)

#define FLOAT_VOLTAGE_CFG (CHGR_BASE + 0x70)

#define CHARGE_RCHG_SOC_THRESHOLD_CFG (CHGR_BASE + 0x7D)

#define JEITA_EN_CFG (CHGR_BASE + 0x90)
#define JEITA_EN_MASK GENMASK(3, 0)

#define CHGR_FAST_CHARGE_SAFETY_TIMER_CFG (CHGR_BASE + 0xA2)
#define FAST_CHARGE_SAFETY_TIMER_1536_MIN 0x3

#define POWER_PATH_STATUS (DCDC_BASE + 0x0B)
#define P_PATH_USE_USBIN_BIT BIT(4)
#define P_PATH_VALID_INPUT_POWER_SOURCE_STS_BIT BIT(0)
#define USBIN_SUSPEND_STS_BIT BIT(6)

#define OTG_CFG (DCDC_BASE + 0x53)
#define OTG_EN_SRC_CFG_BIT BIT(1)

#define APSD_STATUS (USBIN_BASE + 0x07)
#define APSD_DTC_STATUS_DONE_BIT BIT(0)

#define APSD_RESULT_STATUS (USBIN_BASE + 0x08)
#define APSD_RESULT_STATUS_MASK GENMASK(6, 0)
#define QC_3P0_CHARGER_BIT BIT(6)
#define QC_2P0_CHARGER_BIT BIT(5)
#define FLOAT_CHARGER_BIT BIT(4)
#define DCP_CHARGER_BIT BIT(3)
#define CDP_CHARGER_BIT BIT(2)
#define OCP_CHARGER_BIT BIT(1)

#define USBIN_CMD_IL (USBIN_BASE + 0x40)
#define USBIN_SUSPEND_BIT BIT(0)

#define CMD_APSD (USBIN_BASE + 0x41)
#define APSD_RERUN_BIT BIT(0)

#define CMD_ICL_OVERRIDE (USBIN_BASE + 0x42)
#define ICL_OVERRIDE_BIT BIT(0)

#define TYPE_C_CFG (USBIN_BASE + 0x58)
#define APSD_START_ON_CC_BIT BIT(7)

#define USBIN_OPTIONS_1_CFG (USBIN_BASE + 0x62)
#define HVDCP_AUTH_ALG_EN_BIT BIT(6)
#define HVDCP_AUTONOMOUS_EN_BIT BIT(5)
#define BC1P2_SRC_DETECT_BIT BIT(3)
#define HVDCP_EN_BIT BIT(2)

#define USBIN_LOAD_CFG (USBIN_BASE + 0x65)
#define ICL_OVERRIDE_AFTER_APSD_BIT BIT(4)

#define USBIN_ICL_OPTIONS (USBIN_BASE + 0x66)
#define CFG_USB3P0_SEL_BIT BIT(2)
#define USB51_MODE_BIT BIT(1)
#define USBIN_MODE_CHG_BIT BIT(0)

#define USBIN_CURRENT_LIMIT_CFG (USBIN_BASE + 0x70)
#define USBIN_AICL_OPTIONS_CFG (USBIN_BASE + 0x80)
#define SUSPEND_ON_COLLAPSE_USBIN_BIT BIT(7)
#define USBIN_AICL_PERIODIC_RERUN_EN_BIT BIT(4)
#define USBIN_AICL_ADC_EN_BIT BIT(3)
#define USBIN_AICL_EN_BIT BIT(2)

#define SMB5_TYPE_C_MODE_CFG (TYPEC_BASE + 0x44)
#define SMB5_EN_TRY_SNK_BIT BIT(4)
#define SMB5_EN_SNK_ONLY_BIT BIT(1)

#define SMB5_TYPEC_VCONN_CONTROL (TYPEC_BASE + 0x46)
#define SMB5_VCONN_EN_ORIENTATION_BIT BIT(2)
#define SMB5_VCONN_EN_VALUE_BIT BIT(1)
#define SMB5_VCONN_EN_SRC_BIT BIT(0)

#define SMB5_TYPE_C_DEBUG_ACCESS_SINK (TYPEC_BASE + 0x4A)
#define SMB5_TYPEC_DEBUG_ACCESS_SINK_MASK GENMASK(4, 0)
#define SMB5_DEBUG_ACCESS_SRC_CFG (TYPEC_BASE + 0x4C)
#define SMB5_EN_UNORIENTED_DEBUG_ACCESS_SRC_BIT BIT(0)

#define SMB5_TYPE_C_EXIT_STATE_CFG (TYPEC_BASE + 0x50)
#define SMB5_SEL_SRC_UPPER_REF_BIT BIT(2)

#define WD_CFG (MISC_BASE + 0x51)
#define WATCHDOG_TRIGGER_AFP_EN_BIT BIT(7)
#define BARK_WDOG_INT_EN_BIT BIT(6)
#define WDOG_TIMER_EN_ON_PLUGIN_BIT BIT(1)

#define SNARL_BARK_BITE_WD_CFG (MISC_BASE + 0x53)

#define AICL_CMD (MISC_BASE + 0x44)
#define RERUN_AICL_BIT BIT(0)

#define AICL_RERUN_TIME_CFG (MISC_BASE + 0x61)
#define AIC_RERUN_TIME_3_SECS 0x0

#define CURRENT_LSB_UA 50000
#define FV_BASE_UV 3600000
#define FV_LSB_UV 10000
#define FV_MAX_RAW 0x76

#define DEFAULT_CURRENT_UA 500000
#define ICL_MAX_UA 4800000

#define DEFAULT_FCC_UA 2000000
#define DEFAULT_FV_UV 4450000
#define DEFAULT_TERM_UA 400000
#define DEFAULT_RECHARGE_SOC 99

enum charger_status {
	INHIBIT_CHARGE = 0,
	TRICKLE_CHARGE,
	PRE_CHARGE,
	FULLON_CHARGE,
	TAPER_CHARGE,
	TERMINATE_CHARGE,
	PAUSE_CHARGE,
	DISABLE_CHARGE,
};

struct smb_init_register {
	u16 addr;
	u8 mask;
	u8 val;
};

struct smb_chip {
	struct device *dev;
	unsigned int base;
	struct regmap *regmap;

	struct delayed_work status_change_work;
	int cable_irq;
	struct iio_channel *usb_in_i_chan;
	struct iio_channel *usb_in_v_chan;

	int usb_type;
	int fcc_max_ua;
	int fv_max_uv;
	int term_ua;
	int recharge_soc;
	unsigned int apsd_retries;
};

static int smb_set_usbin_suspend(void *priv, bool suspend);

static int smb_read(struct smb_chip *chip, u16 addr, unsigned int *val)
{
	return regmap_read(chip->regmap, addr, val);
}

static int smb_write(struct smb_chip *chip, u16 addr, unsigned int val)
{
	return regmap_write(chip->regmap, addr, val);
}

static int smb_update(struct smb_chip *chip, u16 addr, unsigned int mask,
		      unsigned int val)
{
	return regmap_update_bits(chip->regmap, addr, mask, val);
}

static void smb_get_batt_info(struct smb_chip *chip)
{
	struct device_node *batt_np;
	u32 val;

	chip->fcc_max_ua = DEFAULT_FCC_UA;
	chip->fv_max_uv = DEFAULT_FV_UV;
	chip->term_ua = DEFAULT_TERM_UA;
	chip->recharge_soc = DEFAULT_RECHARGE_SOC;

	device_property_read_u32(chip->dev, "qcom,auto-recharge-soc",
				 &chip->recharge_soc);

	batt_np = of_parse_phandle(chip->dev->of_node, "monitored-battery", 0);
	if (!batt_np) {
		dev_warn(chip->dev,
			 "No monitored-battery, using default charge limits\n");
		return;
	}

	if (!of_property_read_u32(batt_np,
				  "constant-charge-current-max-microamp", &val))
		chip->fcc_max_ua = val;
	if (!of_property_read_u32(batt_np,
				  "constant-charge-voltage-max-microvolt", &val))
		chip->fv_max_uv = val;
	if (!of_property_read_u32(batt_np, "charge-term-current-microamp", &val))
		chip->term_ua = val;

	of_node_put(batt_np);
}

static int smb_get_prop_usb_online(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	unsigned int stat;
	int ret;

	ret = smb_read(chip, SMB5_REG(chip, POWER_PATH_STATUS), &stat);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read power path status: %d\n",
			ret);
		return ret;
	}

	*val = (stat & P_PATH_USE_USBIN_BIT) &&
	       (stat & P_PATH_VALID_INPUT_POWER_SOURCE_STS_BIT);
	return 0;
}

static int smb_apsd_get_charger_type(struct smb_chip *chip, int *val)
{
	unsigned int apsd_stat, stat;
	int usb_online = 0;
	int ret;

	ret = smb_get_prop_usb_online(chip, &usb_online);
	if (ret)
		return ret;
	if (!usb_online) {
		*val = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		return 0;
	}

	ret = smb_read(chip, SMB5_REG(chip, APSD_STATUS), &apsd_stat);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read APSD status: %d\n", ret);
		return ret;
	}
	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return -EAGAIN;

	ret = smb_read(chip, SMB5_REG(chip, APSD_RESULT_STATUS), &stat);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read APSD result: %d\n", ret);
		return ret;
	}

	stat &= APSD_RESULT_STATUS_MASK;

	if (stat & CDP_CHARGER_BIT)
		*val = POWER_SUPPLY_USB_TYPE_CDP;
	else if (stat & (DCP_CHARGER_BIT | OCP_CHARGER_BIT | FLOAT_CHARGER_BIT |
			 QC_2P0_CHARGER_BIT | QC_3P0_CHARGER_BIT))
		*val = POWER_SUPPLY_USB_TYPE_DCP;
	else
		*val = POWER_SUPPLY_USB_TYPE_SDP;

	return 0;
}

static bool smb_batt_ov(struct smb_chip *chip, bool *ov)
{
	unsigned int stat;
	int ret;

	ret = smb_read(chip, SMB5_REG(chip, BATTERY_CHARGER_STATUS_2), &stat);
	if (ret)
		return false;

	*ov = !!(stat & BATTERY_CHARGER_STATUS_2_BAT_OV_BIT);
	return true;
}

static int smb_get_prop_status(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	unsigned int stat;
	bool ov;
	int usb_online = 0;
	int ret;

	ret = smb_get_prop_usb_online(chip, &usb_online);
	if (ret)
		return ret;
	if (!usb_online) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = smb_read(chip, SMB5_REG(chip, BATTERY_CHARGER_STATUS_1), &stat);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read charging status: %d\n", ret);
		return ret;
	}

	if (smb_batt_ov(chip, &ov) && ov) {
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		*val = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		*val = POWER_SUPPLY_STATUS_FULL;
		break;
	case PAUSE_CHARGE:
	case DISABLE_CHARGE:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static int smb_get_prop_health(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	unsigned int stat;
	bool ov;
	int ret;

	if (smb_batt_ov(chip, &ov) && ov) {
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	ret = smb_read(chip, SMB5_REG(chip, BATTERY_CHARGER_STATUS_7), &stat);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read charger status 7: %d\n", ret);
		return ret;
	}

	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		*val = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_BIT)
		*val = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_BIT)
		*val = POWER_SUPPLY_HEALTH_WARM;
	else
		*val = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int smb_get_prop_usb_type(void *priv, int *val)
{
	struct smb_chip *chip = priv;

	*val = chip->usb_type;
	return 0;
}

static int smb_get_current_limit(void *priv, int *val)
{
	struct smb_chip *chip = priv;
	unsigned int limit;
	int ret;

	ret = smb_read(chip, SMB5_REG(chip, USBIN_CURRENT_LIMIT_CFG), &limit);
	if (ret)
		return ret;

	*val = limit * CURRENT_LSB_UA;
	return 0;
}

static int smb_get_iio_now(struct smb_chip *chip, struct iio_channel *chan,
			   bool gate_on_charging, int *val)
{
	int status;
	int ret;

	if (IS_ERR_OR_NULL(chan))
		return -ENODATA;

	if (gate_on_charging) {
		ret = smb_get_prop_status(chip, &status);
		if (ret)
			return ret;
		if (status != POWER_SUPPLY_STATUS_CHARGING) {
			*val = 0;
			return 0;
		}
	}

	return iio_read_channel_processed(chan, val);
}

static int smb_get_prop_voltage_now(void *priv, int *val)
{
	struct smb_chip *chip = priv;

	return smb_get_iio_now(chip, chip->usb_in_v_chan, false, val);
}

static int smb_get_prop_current_now(void *priv, int *val)
{
	struct smb_chip *chip = priv;

	return smb_get_iio_now(chip, chip->usb_in_i_chan, true, val);
}

static int smb_run_aicl(struct smb_chip *chip)
{
	unsigned int stat;
	int ret;

	ret = smb_read(chip, SMB5_REG(chip, POWER_PATH_STATUS), &stat);
	if (ret)
		return ret;
	if (stat & USBIN_SUSPEND_STS_BIT)
		return 0;

	return smb_update(chip, SMB5_REG(chip, AICL_CMD), RERUN_AICL_BIT,
			  RERUN_AICL_BIT);
}

static int smb_icl_override(struct smb_chip *chip, int mode)
{
	unsigned int usb51_mode, icl_override, apsd_override;
	int ret;

	switch (mode) {
	case 1:
		usb51_mode = 0;
		icl_override = ICL_OVERRIDE_BIT;
		apsd_override = 0;
		break;
	case 2:
		usb51_mode = USBIN_MODE_CHG_BIT;
		icl_override = 0;
		apsd_override = ICL_OVERRIDE_AFTER_APSD_BIT;
		break;
	default:
		usb51_mode = USBIN_MODE_CHG_BIT;
		icl_override = 0;
		apsd_override = 0;
		break;
	}

	ret = smb_update(chip, SMB5_REG(chip, USBIN_ICL_OPTIONS),
			 USBIN_MODE_CHG_BIT, usb51_mode);
	if (ret)
		return ret;
	ret = smb_update(chip, SMB5_REG(chip, CMD_ICL_OVERRIDE),
			 ICL_OVERRIDE_BIT, icl_override);
	if (ret)
		return ret;
	return smb_update(chip, SMB5_REG(chip, USBIN_LOAD_CFG),
			  ICL_OVERRIDE_AFTER_APSD_BIT, apsd_override);
}

static int smb_set_sdp_current(struct smb_chip *chip, int ua)
{
	unsigned int options;

	switch (ua) {
	case 100000:
		options = 0;
		break;
	case 150000:
		options = CFG_USB3P0_SEL_BIT;
		break;
	case 900000:
		options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
	case 500000:
	default:
		options = USB51_MODE_BIT;
		break;
	}

	return smb_update(chip, SMB5_REG(chip, USBIN_ICL_OPTIONS),
			  CFG_USB3P0_SEL_BIT | USB51_MODE_BIT, options);
}

static int smb_set_input_current_limit(void *priv, int ua)
{
	struct smb_chip *chip = priv;
	bool sdp_mode;
	unsigned int raw;
	int ret;

	if (ua <= 25000)
		return smb_set_usbin_suspend(priv, true);

	ua = min(ua, ICL_MAX_UA);
	sdp_mode = chip->usb_type == POWER_SUPPLY_USB_TYPE_SDP && ua <= 500000;

	if (sdp_mode) {
		ret = smb_set_sdp_current(chip, ua);
		if (ret)
			return ret;
		ret = smb_icl_override(chip, 1);
	} else {
		raw = ua / CURRENT_LSB_UA;
		ret = smb_write(chip, SMB5_REG(chip, USBIN_CURRENT_LIMIT_CFG),
				raw);
		if (ret)
			return ret;
		ret = smb_icl_override(chip, 2);
	}
	if (ret)
		return ret;

	ret = smb_update(chip, SMB5_REG(chip, USBIN_CMD_IL),
			 USBIN_SUSPEND_BIT, 0);
	if (ret)
		return ret;

	return sdp_mode ? 0 : smb_run_aicl(chip);
}

static int smb_set_charge_current(void *priv, int ua)
{
	struct smb_chip *chip = priv;
	unsigned int raw;

	ua = clamp(ua, 0, 8000000);
	raw = ua / CURRENT_LSB_UA;

	return smb_write(chip, SMB5_REG(chip, FAST_CHARGE_CURRENT_CFG), raw);
}

static int smb_set_float_voltage(void *priv, int uv)
{
	struct smb_chip *chip = priv;
	unsigned int raw;

	if (uv < FV_BASE_UV)
		raw = 0;
	else
		raw = min((uv - FV_BASE_UV) / FV_LSB_UV, FV_MAX_RAW);

	return smb_write(chip, SMB5_REG(chip, FLOAT_VOLTAGE_CFG), raw);
}

static int smb_set_charging_enabled(void *priv, bool en)
{
	struct smb_chip *chip = priv;

	return smb_update(chip, SMB5_REG(chip, CHARGING_ENABLE_CMD),
			  CHARGING_ENABLE_CMD_BIT,
			  en ? CHARGING_ENABLE_CMD_BIT : 0);
}

static int smb_set_usbin_suspend(void *priv, bool suspend)
{
	struct smb_chip *chip = priv;

	return smb_update(chip, SMB5_REG(chip, USBIN_CMD_IL),
			  USBIN_SUSPEND_BIT,
			  suspend ? USBIN_SUSPEND_BIT : 0);
}

static const struct qcom_bms_charger_ops smb_charger_ops = {
	.get_status = smb_get_prop_status,
	.get_online = smb_get_prop_usb_online,
	.get_current_now = smb_get_prop_current_now,
	.get_voltage_now = smb_get_prop_voltage_now,
	.get_input_current_limit = smb_get_current_limit,
	.get_usb_type = smb_get_prop_usb_type,
	.get_health = smb_get_prop_health,
	.set_charge_current = smb_set_charge_current,
	.set_float_voltage = smb_set_float_voltage,
	.set_input_current_limit = smb_set_input_current_limit,
	.set_charging_enabled = smb_set_charging_enabled,
	.set_usbin_suspend = smb_set_usbin_suspend,
};

static void smb_status_change_work(struct work_struct *work)
{
	struct smb_chip *chip =
		container_of(work, struct smb_chip, status_change_work.work);
	int charger_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	int usb_online = 0;
	int count, ret;

	ret = smb_get_prop_usb_online(chip, &usb_online);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read USB status: %d\n", ret);
		return;
	}
	if (!usb_online) {
		chip->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		chip->apsd_retries = 0;
		qcom_bms_notify_changed(chip->dev);
		return;
	}

	for (count = 0; count < 5; count++) {
		ret = smb_apsd_get_charger_type(chip, &charger_type);
		if (ret != -EAGAIN)
			break;
		msleep(100);
	}

	if (ret == -EAGAIN) {
		if (chip->apsd_retries++ < 3) {
			ret = smb_update(chip, SMB5_REG(chip, CMD_APSD),
					 APSD_RERUN_BIT, APSD_RERUN_BIT);
			if (ret) {
				dev_err(chip->dev,
					"failed to rerun APSD: %d\n", ret);
				return;
			}
			mod_delayed_work(system_wq, &chip->status_change_work,
					 msecs_to_jiffies(1000));
			return;
		}

		dev_warn(chip->dev, "APSD timeout, assuming SDP\n");
		charger_type = POWER_SUPPLY_USB_TYPE_SDP;
	}

	if (ret < 0) {
		dev_err(chip->dev, "failed to get charger type: %d\n", ret);
		return;
	}

	chip->usb_type = charger_type;
	chip->apsd_retries = 0;
	qcom_bms_notify_changed(chip->dev);
}

static irqreturn_t smb_handle_batt_overvoltage(int irq, void *data)
{
	struct smb_chip *chip = data;
	bool ov;

	if (smb_batt_ov(chip, &ov) && ov) {
		dev_err(chip->dev, "battery overvoltage detected\n");
		qcom_bms_notify_changed(chip->dev);
	}

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usb_plugin(int irq, void *data)
{
	struct smb_chip *chip = data;

	qcom_bms_notify_changed(chip->dev);
	schedule_delayed_work(&chip->status_change_work,
			      msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usbin_icl_change(int irq, void *data)
{
	struct smb_chip *chip = data;

	qcom_bms_notify_changed(chip->dev);
	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_usbin_src_change(int irq, void *data)
{
	struct smb_chip *chip = data;

	qcom_bms_notify_changed(chip->dev);
	schedule_delayed_work(&chip->status_change_work,
			      msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static irqreturn_t smb_handle_chg_state_change(int irq, void *data)
{
	struct smb_chip *chip = data;

	qcom_bms_notify_changed(chip->dev);
	return IRQ_HANDLED;
}

static const struct smb_init_register smb5_init_seq[] = {
	{ .addr = USBIN_CMD_IL, .mask = USBIN_SUSPEND_BIT, .val = 0 },
	{ .addr = SMB5_TYPE_C_MODE_CFG,
	  .mask = SMB5_EN_TRY_SNK_BIT | SMB5_EN_SNK_ONLY_BIT,
	  .val = SMB5_EN_TRY_SNK_BIT },
	{ .addr = SMB5_TYPEC_VCONN_CONTROL,
	  .mask = SMB5_VCONN_EN_ORIENTATION_BIT | SMB5_VCONN_EN_SRC_BIT |
		  SMB5_VCONN_EN_VALUE_BIT,
	  .val = 0 },
	{ .addr = SMB5_DEBUG_ACCESS_SRC_CFG,
	  .mask = SMB5_EN_UNORIENTED_DEBUG_ACCESS_SRC_BIT,
	  .val = SMB5_EN_UNORIENTED_DEBUG_ACCESS_SRC_BIT },
	{ .addr = SMB5_TYPE_C_EXIT_STATE_CFG,
	  .mask = SMB5_SEL_SRC_UPPER_REF_BIT,
	  .val = SMB5_SEL_SRC_UPPER_REF_BIT },
	{ .addr = TYPE_C_CFG, .mask = APSD_START_ON_CC_BIT, .val = 0 },
	{ .addr = SMB5_TYPE_C_DEBUG_ACCESS_SINK,
	  .mask = SMB5_TYPEC_DEBUG_ACCESS_SINK_MASK, .val = 0x17 },
	{ .addr = OTG_CFG, .mask = OTG_EN_SRC_CFG_BIT, .val = 0 },
	{ .addr = USBIN_OPTIONS_1_CFG,
	  .mask = HVDCP_AUTH_ALG_EN_BIT | HVDCP_AUTONOMOUS_EN_BIT |
		  BC1P2_SRC_DETECT_BIT | HVDCP_EN_BIT,
	  .val = BC1P2_SRC_DETECT_BIT },
	{ .addr = USBIN_ICL_OPTIONS,
	  .mask = USBIN_MODE_CHG_BIT, .val = USBIN_MODE_CHG_BIT },
	{ .addr = CMD_ICL_OVERRIDE, .mask = ICL_OVERRIDE_BIT, .val = 0 },
	{ .addr = USBIN_LOAD_CFG,
	  .mask = ICL_OVERRIDE_AFTER_APSD_BIT, .val = 0 },
	{ .addr = SNARL_BARK_BITE_WD_CFG, .mask = GENMASK(7, 0), .val = 0 },
	{ .addr = WD_CFG,
	  .mask = WATCHDOG_TRIGGER_AFP_EN_BIT | WDOG_TIMER_EN_ON_PLUGIN_BIT |
		  BARK_WDOG_INT_EN_BIT,
	  .val = 0 },
	{ .addr = USBIN_AICL_OPTIONS_CFG,
	  .mask = USBIN_AICL_PERIODIC_RERUN_EN_BIT | USBIN_AICL_ADC_EN_BIT |
		  USBIN_AICL_EN_BIT | SUSPEND_ON_COLLAPSE_USBIN_BIT,
	  .val = USBIN_AICL_PERIODIC_RERUN_EN_BIT | USBIN_AICL_ADC_EN_BIT |
		 USBIN_AICL_EN_BIT | SUSPEND_ON_COLLAPSE_USBIN_BIT },
};

static int smb_init_hw(struct smb_chip *chip)
{
	unsigned int raw;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(smb5_init_seq); i++) {
		ret = smb_update(chip, SMB5_REG(chip, smb5_init_seq[i].addr),
				 smb5_init_seq[i].mask, smb5_init_seq[i].val);
		if (ret)
			return dev_err_probe(chip->dev, ret,
					     "init command %d failed\n", i);
	}

	/* Keep the PMIC out of autonomous HVDCP/QC negotiation. */
	ret = smb_update(chip, SMB5_REG(chip, USBIN_OPTIONS_1_CFG),
			 HVDCP_AUTH_ALG_EN_BIT | HVDCP_AUTONOMOUS_EN_BIT |
				 HVDCP_EN_BIT,
			 0);
	if (ret)
		return ret;

	/* Hardware JEITA is handled by qcom_bms; disable the PMIC comparators. */
	ret = smb_update(chip, SMB5_REG(chip, JEITA_EN_CFG), JEITA_EN_MASK, 0);
	if (ret)
		return ret;

	ret = smb_write(chip,
			SMB5_REG(chip, CHGR_FAST_CHARGE_SAFETY_TIMER_CFG),
			FAST_CHARGE_SAFETY_TIMER_1536_MIN);
	if (ret)
		return ret;

	if (chip->term_ua > 0) {
		raw = abs(chip->term_ua) / 1000;
		raw = DIV_ROUND_CLOSEST(raw * 32767, 10000);
		ret = smb_write(chip,
				SMB5_REG(chip, CHGR_ADC_ITERM_UP_THD_MSB),
				(raw >> 8) & 0xff);
		if (ret)
			return ret;
		ret = smb_write(chip,
				SMB5_REG(chip, CHGR_ADC_ITERM_UP_THD_LSB),
				raw & 0xff);
		if (ret)
			return ret;
		ret = smb_update(chip, SMB5_REG(chip, CHGR_ADC_TERM_CFG),
				 TERM_BASED_ON_SAMPLE_CNT,
				 TERM_BASED_ON_SAMPLE_CNT);
		if (ret)
			return ret;
	}

	raw = DIV_ROUND_CLOSEST(min(chip->recharge_soc, 100) * 255, 100) + 1;
	ret = smb_write(chip,
			SMB5_REG(chip, CHARGE_RCHG_SOC_THRESHOLD_CFG), raw);
	if (ret)
		return ret;
	ret = smb_update(chip, SMB5_REG(chip, CHGR_CFG2), RECHG_MASK,
			 SOC_BASED_RECHG_BIT);
	if (ret)
		return ret;

	ret = smb_write(chip, SMB5_REG(chip, AICL_RERUN_TIME_CFG),
			AIC_RERUN_TIME_3_SECS);
	if (ret)
		return ret;

	ret = smb_set_charge_current(chip, DEFAULT_FCC_UA);
	if (ret)
		return ret;
	ret = smb_set_float_voltage(chip, DEFAULT_FV_UV);
	if (ret)
		return ret;
	ret = smb_set_input_current_limit(chip, DEFAULT_CURRENT_UA);
	if (ret)
		return ret;

	return smb_set_charging_enabled(chip, true);
}

static int smb_init_irq(struct smb_chip *chip, int *irq, const char *name,
			irqreturn_t (*handler)(int irq, void *data),
			bool optional)
{
	int irqnum;
	int ret;

	irqnum = platform_get_irq_byname(to_platform_device(chip->dev), name);
	if (irqnum < 0) {
		if (optional && irqnum == -ENXIO)
			return 0;
		return irqnum;
	}

	ret = devm_request_threaded_irq(chip->dev, irqnum, NULL, handler,
					IRQF_ONESHOT, name, chip);
	if (ret < 0)
		return dev_err_probe(chip->dev, ret,
				     "failed to request irq %s\n", name);

	if (irq)
		*irq = irqnum;

	return 0;
}

static int smb_probe(struct platform_device *pdev)
{
	struct smb_chip *chip;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap)
		return dev_err_probe(chip->dev, -ENODEV,
				     "failed to locate the regmap\n");

	ret = device_property_read_u32(chip->dev, "reg", &chip->base);
	if (ret < 0)
		return dev_err_probe(chip->dev, ret,
				     "failed to read base address\n");

	chip->usb_in_v_chan = devm_iio_channel_get(chip->dev, "usbin_v");
	if (IS_ERR(chip->usb_in_v_chan)) {
		dev_warn(chip->dev, "no usbin_v channel: %ld\n",
			 PTR_ERR(chip->usb_in_v_chan));
		chip->usb_in_v_chan = NULL;
	}
	chip->usb_in_i_chan = devm_iio_channel_get(chip->dev, "usbin_i");
	if (IS_ERR(chip->usb_in_i_chan)) {
		dev_warn(chip->dev, "no usbin_i channel: %ld\n",
			 PTR_ERR(chip->usb_in_i_chan));
		chip->usb_in_i_chan = NULL;
	}

	smb_get_batt_info(chip);

	ret = smb_init_hw(chip);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(chip->dev,
					   &chip->status_change_work,
					   smb_status_change_work);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "failed to init status work\n");

	ret = smb_init_irq(chip, NULL, "bat-ov", smb_handle_batt_overvoltage,
			   true);
	if (ret < 0)
		return ret;

	ret = smb_init_irq(chip, &chip->cable_irq, "usb-plugin",
			   smb_handle_usb_plugin, false);
	if (ret < 0)
		return ret;

	ret = smb_init_irq(chip, NULL, "usbin-icl-change",
			   smb_handle_usbin_icl_change, true);
	if (ret < 0)
		return ret;

	ret = smb_init_irq(chip, NULL, "usbin-src-change",
			   smb_handle_usbin_src_change, true);
	if (ret < 0)
		return ret;

	ret = smb_init_irq(chip, NULL, "chg-state-change",
			   smb_handle_chg_state_change, true);
	if (ret < 0)
		return ret;

	devm_device_init_wakeup(chip->dev);

	ret = devm_pm_set_wake_irq(chip->dev, chip->cable_irq);
	if (ret < 0)
		return dev_err_probe(chip->dev, ret,
				     "failed to set wake irq\n");

	platform_set_drvdata(pdev, chip);

	ret = qcom_bms_register_charger(&pdev->dev, QCOM_BMS_CHARGER_PMIC,
					&smb_charger_ops, chip);
	if (ret < 0)
		return dev_err_probe(chip->dev, ret,
				     "failed to register with qcom_bms\n");

	mod_delayed_work(system_wq, &chip->status_change_work, 0);

	return 0;
}

static void smb_remove(struct platform_device *pdev)
{
	qcom_bms_unregister_charger(&pdev->dev);
}

static const struct of_device_id smb_match_id_table[] = {
	{ .compatible = "qcom,pm7250b-charger" },
	{ .compatible = "qcom,pm8150b-charger" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, smb_match_id_table);

static struct platform_driver qcom_spmi_smb5 = {
	.probe = smb_probe,
	.remove = smb_remove,
	.driver = {
		.name = "qcom-smb5-charger",
		.of_match_table = smb_match_id_table,
	},
};
module_platform_driver(qcom_spmi_smb5);

MODULE_AUTHOR("Casey Connolly <casey.connolly@linaro.org>");
MODULE_AUTHOR("TwinbornPlate75 <3342733415@qq.com>");
MODULE_DESCRIPTION("Qualcomm SMB5 Charger Driver (qcom_bms sub-module)");
MODULE_LICENSE("GPL");
