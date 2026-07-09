// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Unified Battery/Charger Manager
 *
 * Monolithic driver that aggregates the Qualcomm fuel gauge, PMIC charger,
 * and optional direct chargers into user-facing battery and USB power supplies.
 *
 * Copyright (C) 2026
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "qcom_bms.h"

#define QCOM_BMS_PSY_NAME "qcom-battery"
#define QCOM_BMS_USB_PSY_NAME "qcom-usb"

#define QCOM_BMS_TICK_ACTIVE_MS 1000
#define QCOM_BMS_TICK_IDLE_MS 5000

#define QCOM_BMS_DC_BACKOFF_MIN_MS 1000
#define QCOM_BMS_DC_BACKOFF_MAX_MS 10000

#define QCOM_BMS_DC_MIN_VBAT_UV 3500000
#define QCOM_BMS_DC_VBUS_MIN_UV 8500000
#define QCOM_BMS_DC_VBUS_MAX_UV 9600000
#define QCOM_BMS_DC_MIN_TEMP 0
#define QCOM_BMS_DC_MAX_TEMP 450

#define QCOM_BMS_UNVERIFIED_FCC_UA 2000000
#define QCOM_BMS_UNVERIFIED_FV_UV 4450000
#define QCOM_BMS_USB_ICL_UA 3000000

#define QCOM_BMS_DEFAULT_TOO_COLD (-100)
#define QCOM_BMS_DEFAULT_COOL_TH 150
#define QCOM_BMS_DEFAULT_WARM_TH 480
#define QCOM_BMS_DEFAULT_TOO_HOT 590
#define QCOM_BMS_DEFAULT_JEITA_HYST 5

enum qcom_bms_state {
	QCOM_BMS_OFFLINE,
	QCOM_BMS_PMIC,
	QCOM_BMS_DC_ACTIVE,
	QCOM_BMS_BACKOFF,
};

struct qcom_bms_fg {
	struct device *dev;
	struct device_link *link;
	const struct qcom_bms_fg_ops *ops;
	void *priv;
	unsigned int active_calls;
	struct completion idle;
	bool unregistering;
};

struct qcom_bms_charger {
	struct device *dev;
	struct device_link *link;
	const struct qcom_bms_charger_ops *ops;
	void *priv;
	unsigned int active_calls;
	struct completion idle;
	bool unregistering;
};

struct qcom_bms_fg_ref {
	struct qcom_bms_fg *backend;
	const struct qcom_bms_fg_ops *ops;
	void *priv;
};

struct qcom_bms_charger_ref {
	struct qcom_bms_charger *backend;
	const struct qcom_bms_charger_ops *ops;
	void *priv;
};

struct qcom_bms_authenticator {
	struct device *dev;
	struct device_link *link;
	bool authenticated;
};

struct qcom_bms_jeita_range {
	int low;
	int high;
	int value;
};

struct qcom_bms_jeita {
	struct qcom_bms_jeita_range *fcc;
	int fcc_len;
	struct qcom_bms_jeita_range *fv;
	int fv_len;
	int hysteresis;
	int index;
	int too_cold;
	int cool_th;
	int warm_th;
	int too_hot;
};

struct qcom_bms_limits {
	int fcc_ua;
	int fv_uv;
	int icl_ua;
	bool charge_enabled;
};

struct qcom_bms_info {
	struct device *dev;
	struct power_supply *bms_psy;
	struct power_supply *usb_psy;

	struct mutex ops_lock;
	struct qcom_bms_fg fg;
	struct qcom_bms_charger pmic_chg;
	struct qcom_bms_charger direct_chg;
	struct qcom_bms_authenticator auth[QCOM_BMS_AUTH_MAX];
	u32 required_auth_mask;
	bool authenticated;

	struct delayed_work work;
	bool removing;
	unsigned int tick;

	/* tcpm source psy (CC-driven attach/detach) drives direct charging. */
	struct notifier_block psy_nb;
	struct power_supply *tcpm_psy;
	struct device_node *tcpm_np;
	bool pmic_usbin_suspended;

	struct power_supply_battery_info *batt_info;
	int fcc_max_ua;
	int fv_max_uv;
	int icl_max_ua;
	int unverified_fcc_ua;
	int unverified_fv_uv;
	int dc_min_vbat_uv;
	int dc_vbus_min_uv;
	int dc_vbus_max_uv;
	int dc_min_temp;
	int dc_max_temp;

	struct qcom_bms_jeita jeita;
	u32 *thermal_icl;
	int thermal_icl_len;
	u32 *thermal_fcc;
	int thermal_fcc_len;
	int thermal_level;

	int status;
	bool charge_done;
	bool input_present;
	bool cc_online;
	bool pmic_online;
	bool direct_online;
	int pmic_status;
	int direct_status;
	int pmic_health;
	int direct_health;
	int usb_type;
	int tcpm_usb_type;
	int tcpm_vbus_uv;
	int pmic_vbus_uv;
	int soc;
	int reported_soc;
	int temp;
	int vbat_uv;

	enum qcom_bms_state state;
	unsigned long backoff_until;
	int backoff_ms;
	int applied_fcc_ua;
	int applied_fv_uv;
	int applied_icl_ua;
	int applied_direct_fcc_ua;
	int applied_direct_fv_uv;
	bool pmic_charging_enabled;

	struct dentry *debugfs;
	u32 dbg_fcc_ua;
	u32 dbg_fv_uv;
	u32 dbg_icl_ua;
	u32 dbg_state;
	u32 dbg_vbat_uv;
	u32 dbg_soc;
};

static struct qcom_bms_info *g_bms;
static DEFINE_MUTEX(qcom_bms_mutex);

static void qcom_bms_refresh_authentication(struct qcom_bms_info *info);

static struct qcom_bms_info *qcom_bms_lock(void)
{
	struct qcom_bms_info *info;

	mutex_lock(&qcom_bms_mutex);
	info = g_bms;
	if (info)
		mutex_lock(&info->ops_lock);
	mutex_unlock(&qcom_bms_mutex);

	return info;
}

static bool qcom_bms_fg_acquire_locked(struct qcom_bms_info *info,
				       struct qcom_bms_fg_ref *ref)
{
	struct qcom_bms_fg *fg = &info->fg;

	if (!fg->ops || fg->unregistering)
		return false;

	if (!fg->active_calls++)
		reinit_completion(&fg->idle);
	ref->backend = fg;
	ref->ops = fg->ops;
	ref->priv = fg->priv;

	return true;
}

static void qcom_bms_fg_release(struct qcom_bms_info *info,
				struct qcom_bms_fg_ref *ref)
{
	mutex_lock(&info->ops_lock);
	if (!--ref->backend->active_calls)
		complete_all(&ref->backend->idle);
	mutex_unlock(&info->ops_lock);
}

static bool qcom_bms_charger_acquire_locked(struct qcom_bms_charger *charger,
					    struct qcom_bms_charger_ref *ref)
{
	if (!charger->ops || charger->unregistering)
		return false;

	if (!charger->active_calls++)
		reinit_completion(&charger->idle);
	ref->backend = charger;
	ref->ops = charger->ops;
	ref->priv = charger->priv;

	return true;
}

static void qcom_bms_charger_release(struct qcom_bms_info *info,
				     struct qcom_bms_charger_ref *ref)
{
	mutex_lock(&info->ops_lock);
	if (!--ref->backend->active_calls)
		complete_all(&ref->backend->idle);
	mutex_unlock(&info->ops_lock);
}

static int qcom_bms_health(struct qcom_bms_info *info)
{
	if (info->input_present && info->pmic_health != POWER_SUPPLY_HEALTH_GOOD)
		return info->pmic_health;

	if (info->temp < info->jeita.too_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	if (info->temp < info->jeita.cool_th)
		return POWER_SUPPLY_HEALTH_COOL;
	if (info->temp > info->jeita.too_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (info->temp > info->jeita.warm_th)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int qcom_bms_psy_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct qcom_bms_info *info = power_supply_get_drvdata(psy);
	struct qcom_bms_fg_ref fg = {};
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&info->ops_lock);
		val->intval = info->status;
		mutex_unlock(&info->ops_lock);
		return 0;
	case POWER_SUPPLY_PROP_AUTHENTIC:
		mutex_lock(&info->ops_lock);
		val->intval = info->authenticated;
		mutex_unlock(&info->ops_lock);
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = qcom_bms_health(info);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = info->thermal_level;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = max(info->thermal_icl_len,
				  info->thermal_fcc_len) - 1;
		if (val->intval < 0)
			val->intval = 0;
		return 0;
	default:
		break;
	}

	mutex_lock(&info->ops_lock);
	if (!qcom_bms_fg_acquire_locked(info, &fg)) {
		mutex_unlock(&info->ops_lock);
		return -ENODATA;
	}
	mutex_unlock(&info->ops_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		ret = fg.ops->get_technology ?
			      fg.ops->get_technology(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = fg.ops->get_capacity ?
			      fg.ops->get_capacity(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = fg.ops->get_capacity_level ?
			      fg.ops->get_capacity_level(fg.priv,
							 &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = fg.ops->get_current ?
			      fg.ops->get_current(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = fg.ops->get_voltage ?
			      fg.ops->get_voltage(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		ret = fg.ops->get_power ?
			      fg.ops->get_power(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = fg.ops->get_voltage_min_design ?
			      fg.ops->get_voltage_min_design(fg.priv,
							     &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		ret = fg.ops->get_voltage_max_design ?
			      fg.ops->get_voltage_max_design(fg.priv,
							     &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = fg.ops->get_charge_full_design ?
			      fg.ops->get_charge_full_design(fg.priv,
							     &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg.ops->get_charge_full ?
			      fg.ops->get_charge_full(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = fg.ops->get_charge_now ?
			      fg.ops->get_charge_now(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		ret = fg.ops->get_energy_full_design ?
			      fg.ops->get_energy_full_design(fg.priv,
							     &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		ret = fg.ops->get_energy_full ?
			      fg.ops->get_energy_full(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = fg.ops->get_energy_now ?
			      fg.ops->get_energy_now(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg.ops->get_cycle_count ?
			      fg.ops->get_cycle_count(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = fg.ops->get_present ?
			      fg.ops->get_present(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = fg.ops->get_temp ?
			      fg.ops->get_temp(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		ret = fg.ops->get_scope ?
			      fg.ops->get_scope(fg.priv, &val->intval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		ret = fg.ops->get_manufacturer ?
			      fg.ops->get_manufacturer(fg.priv, &val->strval) :
			      -ENODATA;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		ret = fg.ops->get_model_name ?
			      fg.ops->get_model_name(fg.priv, &val->strval) :
			      -ENODATA;
		break;
	default:
		ret = -EINVAL;
	}

	qcom_bms_fg_release(info, &fg);
	return ret;
}

static int qcom_bms_psy_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *val)
{
	struct qcom_bms_info *info = power_supply_get_drvdata(psy);
	struct qcom_bms_fg_ref fg = {};
	int max_level = max(info->thermal_icl_len, info->thermal_fcc_len);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		if (val->intval < 0 || val->intval >= max_level)
			return -EINVAL;
		info->thermal_level = val->intval;
		mod_delayed_work(system_wq, &info->work, 0);
		return 0;
	default:
		break;
	}

	mutex_lock(&info->ops_lock);
	if (!qcom_bms_fg_acquire_locked(info, &fg)) {
		mutex_unlock(&info->ops_lock);
		return -ENODATA;
	}
	mutex_unlock(&info->ops_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg.ops->set_charge_full ?
			      fg.ops->set_charge_full(fg.priv, val->intval) :
			      -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	qcom_bms_fg_release(info, &fg);
	return ret;
}

static int qcom_bms_psy_property_is_writeable(struct power_supply *psy,
					      enum power_supply_property psp)
{
	struct qcom_bms_info *info = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return info->thermal_icl_len || info->thermal_fcc_len;
	default:
		break;
	}

	mutex_lock(&info->ops_lock);
	if (!info->fg.ops) {
		mutex_unlock(&info->ops_lock);
		return 0;
	}

	ret = psp == POWER_SUPPLY_PROP_CHARGE_FULL &&
	      info->fg.ops->set_charge_full;
	mutex_unlock(&info->ops_lock);

	return ret;
}

static const enum power_supply_property qcom_bms_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
};

static const struct power_supply_desc qcom_bms_psy_desc = {
	.name = QCOM_BMS_PSY_NAME,
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_bms_psy_props,
	.num_properties = ARRAY_SIZE(qcom_bms_psy_props),
	.get_property = qcom_bms_psy_get_property,
	.set_property = qcom_bms_psy_set_property,
	.property_is_writeable = qcom_bms_psy_property_is_writeable,
	.no_thermal = true,
};

static bool qcom_bms_charger_is_online(struct qcom_bms_charger_ref *charger)
{
	int online;

	return charger->ops && charger->ops->get_online &&
	       !charger->ops->get_online(charger->priv, &online) && online;
}

static int qcom_bms_usb_psy_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_bms_info *info = power_supply_get_drvdata(psy);
	struct qcom_bms_charger_ref pmic = {};
	struct qcom_bms_charger_ref direct = {};
	bool have_direct, have_pmic;
	bool direct_online, pmic_online;
	int ret = 0;

	mutex_lock(&info->ops_lock);
	have_direct =
		qcom_bms_charger_acquire_locked(&info->direct_chg, &direct);
	have_pmic = qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
	mutex_unlock(&info->ops_lock);

	direct_online = have_direct && qcom_bms_charger_is_online(&direct);
	pmic_online = have_pmic && qcom_bms_charger_is_online(&pmic);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = direct_online || pmic_online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (direct_online && direct.ops->get_status)
			ret = direct.ops->get_status(direct.priv, &val->intval);
		else if (pmic_online && pmic.ops->get_status)
			ret = pmic.ops->get_status(pmic.priv, &val->intval);
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (direct_online && direct.ops->get_current_now)
			ret = direct.ops->get_current_now(direct.priv,
							  &val->intval);
		else if (have_pmic && pmic.ops->get_current_now)
			ret = pmic.ops->get_current_now(pmic.priv,
							&val->intval);
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (direct_online && direct.ops->get_voltage_now)
			ret = direct.ops->get_voltage_now(direct.priv,
							  &val->intval);
		else if (have_pmic && pmic.ops->get_voltage_now)
			ret = pmic.ops->get_voltage_now(pmic.priv,
							&val->intval);
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		if (direct_online) {
			val->intval = info->tcpm_usb_type >=
					      POWER_SUPPLY_USB_TYPE_PD ?
					      POWER_SUPPLY_USB_TYPE_PD :
					      POWER_SUPPLY_USB_TYPE_DCP;
		} else if (pmic_online && pmic.ops->get_usb_type) {
			ret = pmic.ops->get_usb_type(pmic.priv, &val->intval);
		} else {
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (direct_online && direct.ops->get_input_current_limit)
			ret = direct.ops->get_input_current_limit(direct.priv,
								  &val->intval);
		else if (have_pmic && pmic.ops->get_input_current_limit)
			ret = pmic.ops->get_input_current_limit(pmic.priv,
								&val->intval);
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = qcom_bms_health(info);
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		ret = -EINVAL;
	}

	if (have_pmic)
		qcom_bms_charger_release(info, &pmic);
	if (have_direct)
		qcom_bms_charger_release(info, &direct);

	return ret;
}

static const enum power_supply_property qcom_bms_usb_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,      POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_USB_TYPE,    POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_HEALTH,      POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc qcom_bms_usb_psy_desc = {
	.name = QCOM_BMS_USB_PSY_NAME,
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_PD),
	.properties = qcom_bms_usb_psy_props,
	.num_properties = ARRAY_SIZE(qcom_bms_usb_psy_props),
	.get_property = qcom_bms_usb_psy_get_property,
};

static void qcom_bms_get_tcpm(struct qcom_bms_info *info, bool *online,
			      int *vbus_uv, int *usb_type)
{
	union power_supply_propval val;
	struct power_supply *psy;

	*online = false;
	*vbus_uv = 0;
	*usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	psy = READ_ONCE(info->tcpm_psy);
	if (!psy && info->tcpm_np) {
		psy = power_supply_get_by_reference(dev_fwnode(info->dev),
						    "qcom,tcpm-psy");
		if (IS_ERR(psy))
			psy = NULL;
		if (psy)
			WRITE_ONCE(info->tcpm_psy, psy);
	}
	if (!psy)
		return;

	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val))
		*online = val.intval != 0;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				       &val))
		*vbus_uv = val.intval;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &val))
		*usb_type = val.intval;
}

static void qcom_bms_read_inputs(struct qcom_bms_info *info)
{
	struct qcom_bms_charger_ref direct = {};
	struct qcom_bms_charger_ref pmic = {};
	struct qcom_bms_fg_ref fg = {};
	bool have_direct, have_pmic, have_fg;
	bool pmic_online = false;
	bool direct_online = false;
	bool cc_online;
	int pmic_status = POWER_SUPPLY_STATUS_UNKNOWN;
	int direct_status = POWER_SUPPLY_STATUS_UNKNOWN;
	int pmic_health = POWER_SUPPLY_HEALTH_GOOD;
	int direct_health = POWER_SUPPLY_HEALTH_GOOD;
	int usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	int pmic_vbus = 0;
	int tcpm_vbus, tcpm_type;
	int online;
	int val;

	qcom_bms_get_tcpm(info, &cc_online, &tcpm_vbus, &tcpm_type);

	mutex_lock(&info->ops_lock);
	have_direct = qcom_bms_charger_acquire_locked(&info->direct_chg,
						      &direct);
	have_pmic = qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
	have_fg = qcom_bms_fg_acquire_locked(info, &fg);
	mutex_unlock(&info->ops_lock);

	if (have_fg) {
		if (fg.ops->poll && !(info->tick % 5))
			fg.ops->poll(fg.priv);
		if (fg.ops->get_temp && !fg.ops->get_temp(fg.priv, &val))
			info->temp = val;
		if (fg.ops->get_voltage &&
		    !fg.ops->get_voltage(fg.priv, &val))
			info->vbat_uv = val;
		if (fg.ops->get_capacity &&
		    !fg.ops->get_capacity(fg.priv, &val))
			info->soc = val;
	}

	if (have_pmic) {
		if (pmic.ops->get_status)
			pmic.ops->get_status(pmic.priv, &pmic_status);
		if (pmic.ops->get_health)
			pmic.ops->get_health(pmic.priv, &pmic_health);
		if (pmic.ops->get_usb_type)
			pmic.ops->get_usb_type(pmic.priv, &usb_type);
		if (pmic.ops->get_voltage_now)
			pmic.ops->get_voltage_now(pmic.priv, &pmic_vbus);
		if (pmic.ops->get_online &&
		    !pmic.ops->get_online(pmic.priv, &online))
			pmic_online = online;
	}

	if (have_direct) {
		if (direct.ops->get_status)
			direct.ops->get_status(direct.priv, &direct_status);
		if (direct.ops->get_health)
			direct.ops->get_health(direct.priv, &direct_health);
		if (direct.ops->get_online &&
		    !direct.ops->get_online(direct.priv, &online))
			direct_online = online;
	}

	mutex_lock(&info->ops_lock);
	info->cc_online = cc_online;
	info->tcpm_vbus_uv = tcpm_vbus;
	info->tcpm_usb_type = tcpm_type;
	info->pmic_status = pmic_status;
	info->direct_status = direct_status;
	info->pmic_health = pmic_health;
	info->direct_health = direct_health;
	info->usb_type = usb_type;
	info->pmic_vbus_uv = pmic_vbus;
	info->pmic_online = pmic_online;
	info->direct_online = direct_online;
	mutex_unlock(&info->ops_lock);

	if (have_direct)
		qcom_bms_charger_release(info, &direct);
	if (have_pmic)
		qcom_bms_charger_release(info, &pmic);
	if (have_fg)
		qcom_bms_fg_release(info, &fg);
}

static void qcom_bms_update_status(struct qcom_bms_info *info)
{
	bool charge_done;
	bool input_present;
	bool changed;
	int new_status;

	charge_done = info->pmic_status == POWER_SUPPLY_STATUS_FULL ||
		      info->direct_status == POWER_SUPPLY_STATUS_FULL ||
		      (info->state == QCOM_BMS_DC_ACTIVE && info->soc >= 100);
	input_present = info->cc_online || info->pmic_online ||
			info->direct_online ||
			(info->state == QCOM_BMS_DC_ACTIVE &&
			 (info->direct_status == POWER_SUPPLY_STATUS_CHARGING ||
			  info->direct_status == POWER_SUPPLY_STATUS_FULL));

	if (charge_done)
		new_status = POWER_SUPPLY_STATUS_FULL;
	else if (info->direct_status == POWER_SUPPLY_STATUS_CHARGING ||
		 info->pmic_status == POWER_SUPPLY_STATUS_CHARGING)
		new_status = POWER_SUPPLY_STATUS_CHARGING;
	else if (!input_present)
		new_status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		new_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	mutex_lock(&info->ops_lock);
	changed = info->status != new_status ||
		  info->charge_done != charge_done ||
		  info->input_present != input_present;
	info->status = new_status;
	info->charge_done = charge_done;
	info->input_present = input_present;
	mutex_unlock(&info->ops_lock);

	if (changed) {
		struct qcom_bms_fg_ref fg = {};

		mutex_lock(&info->ops_lock);
		if (qcom_bms_fg_acquire_locked(info, &fg)) {
			mutex_unlock(&info->ops_lock);
			if (fg.ops->charging_state_changed)
				fg.ops->charging_state_changed(fg.priv,
							       new_status,
							       charge_done,
							       input_present);
			qcom_bms_fg_release(info, &fg);
		} else {
			mutex_unlock(&info->ops_lock);
		}

		power_supply_changed(info->bms_psy);
		power_supply_changed(info->usb_psy);
	}
}

static int qcom_bms_jeita_lookup(const struct qcom_bms_jeita_range *range,
				 int len, int temp)
{
	int i;

	for (i = 0; i < len; i++)
		if (temp >= range[i].low && temp <= range[i].high)
			return i;

	return -ENODATA;
}

static int qcom_bms_jeita_get(struct qcom_bms_info *info, int temp, int *fcc_ua,
			      int *fv_uv)
{
	struct qcom_bms_jeita *j = &info->jeita;
	int index;

	if (!j->fcc_len || !j->fv_len)
		return -ENODATA;

	index = qcom_bms_jeita_lookup(j->fcc, j->fcc_len, temp);
	if (index < 0) {
		j->index = -1;
		return -ENODATA;
	}

	if (j->index >= 0 && j->index < j->fcc_len &&
	    temp >= j->fcc[j->index].low - j->hysteresis &&
	    temp <= j->fcc[j->index].high + j->hysteresis)
		index = j->index;

	j->index = index;
	*fcc_ua = j->fcc[index].value;

	index = qcom_bms_jeita_lookup(j->fv, j->fv_len, temp);
	if (index < 0)
		return -ENODATA;
	*fv_uv = j->fv[index].value;

	return 0;
}

static void qcom_bms_compute_limits(struct qcom_bms_info *info,
				    struct qcom_bms_limits *lim)
{
	int fcc = info->fcc_max_ua;
	int fv = info->fv_max_uv;
	int icl = info->icl_max_ua;
	int max_level = max(info->thermal_icl_len, info->thermal_fcc_len);
	int jeita_fcc = 0, jeita_fv = 0;
	bool authenticated;

	authenticated = READ_ONCE(info->authenticated);

	if (!qcom_bms_jeita_get(info, info->temp, &jeita_fcc, &jeita_fv)) {
		fcc = min(fcc, jeita_fcc);
		fv = min(fv, jeita_fv);
	} else if (info->jeita.fcc_len) {
		fcc = 0;
	}

	if (!authenticated) {
		fcc = min(fcc, info->unverified_fcc_ua);
		fv = min(fv, info->unverified_fv_uv);
	}

	if (info->cc_online &&
	    info->tcpm_usb_type >= POWER_SUPPLY_USB_TYPE_PD)
		icl = min(icl, QCOM_BMS_USB_ICL_UA);
	else
		switch (info->usb_type) {
		case POWER_SUPPLY_USB_TYPE_SDP:
			icl = min(icl, 500000);
			break;
		case POWER_SUPPLY_USB_TYPE_CDP:
			icl = min(icl, 1500000);
			break;
		case POWER_SUPPLY_USB_TYPE_DCP:
			icl = min(icl, QCOM_BMS_USB_ICL_UA);
			break;
		default:
			icl = min(icl, 500000);
			break;
		}

	if (info->thermal_level >= 0) {
		if (info->thermal_icl &&
		    info->thermal_level < info->thermal_icl_len)
			icl = min_t(int, icl,
				    info->thermal_icl[info->thermal_level]);
		if (info->thermal_fcc &&
		    info->thermal_level < info->thermal_fcc_len)
			fcc = min_t(int, fcc,
				    info->thermal_fcc[info->thermal_level]);
		if (max_level && info->thermal_level == max_level - 1)
			fcc = 0;
	}

	lim->fcc_ua = max(fcc, 0);
	lim->fv_uv = max(fv, 0);
	lim->icl_ua = max(icl, 0);
	lim->charge_enabled = lim->fcc_ua > 0;
}

static void qcom_bms_apply_pmic_limits(struct qcom_bms_info *info,
				       const struct qcom_bms_limits *lim)
{
	struct qcom_bms_charger_ref pmic = {};
	bool have_pmic;

	mutex_lock(&info->ops_lock);
	have_pmic = qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
	mutex_unlock(&info->ops_lock);
	if (!have_pmic)
		return;

	if (pmic.ops->set_charge_current &&
	    lim->fcc_ua != info->applied_fcc_ua) {
		if (!pmic.ops->set_charge_current(pmic.priv, lim->fcc_ua))
			info->applied_fcc_ua = lim->fcc_ua;
		else
			dev_warn_ratelimited(info->dev,
					     "failed to set FCC %d uA\n",
					     lim->fcc_ua);
	}

	if (pmic.ops->set_float_voltage &&
	    lim->fv_uv != info->applied_fv_uv) {
		if (!pmic.ops->set_float_voltage(pmic.priv, lim->fv_uv))
			info->applied_fv_uv = lim->fv_uv;
		else
			dev_warn_ratelimited(info->dev,
					     "failed to set FV %d uV\n",
					     lim->fv_uv);
	}

	if (pmic.ops->set_input_current_limit &&
	    lim->icl_ua != info->applied_icl_ua) {
		if (!pmic.ops->set_input_current_limit(pmic.priv, lim->icl_ua))
			info->applied_icl_ua = lim->icl_ua;
		else
			dev_warn_ratelimited(info->dev,
					     "failed to set ICL %d uA\n",
					     lim->icl_ua);
	}

	if (pmic.ops->set_charging_enabled &&
	    lim->charge_enabled != info->pmic_charging_enabled) {
		if (!pmic.ops->set_charging_enabled(pmic.priv,
						    lim->charge_enabled))
			info->pmic_charging_enabled = lim->charge_enabled;
		else
			dev_warn_ratelimited(info->dev,
					     "failed to %s PMIC charging\n",
					     lim->charge_enabled ? "enable" :
								   "disable");
	}

	qcom_bms_charger_release(info, &pmic);
}

static void qcom_bms_apply_direct_limits(struct qcom_bms_info *info,
					 const struct qcom_bms_limits *lim)
{
	struct qcom_bms_charger_ref direct = {};
	bool have_direct;
	int iin = lim->fcc_ua / 2;

	mutex_lock(&info->ops_lock);
	have_direct = qcom_bms_charger_acquire_locked(&info->direct_chg,
						      &direct);
	mutex_unlock(&info->ops_lock);
	if (!have_direct)
		return;

	if (direct.ops->set_input_current_limit &&
	    iin != info->applied_direct_fcc_ua) {
		if (!direct.ops->set_input_current_limit(direct.priv, iin))
			info->applied_direct_fcc_ua = iin;
		else
			dev_warn_ratelimited(
				info->dev,
				"failed to set direct IIN %d uA\n", iin);
	}

	if (direct.ops->set_float_voltage &&
	    lim->fv_uv != info->applied_direct_fv_uv) {
		if (!direct.ops->set_float_voltage(direct.priv, lim->fv_uv))
			info->applied_direct_fv_uv = lim->fv_uv;
		else
			dev_warn_ratelimited(info->dev,
					     "failed to set direct FV %d uV\n",
					     lim->fv_uv);
	}

	qcom_bms_charger_release(info, &direct);
}

static bool qcom_bms_dc_allowed(struct qcom_bms_info *info,
				const struct qcom_bms_limits *lim)
{
	int vbus;

	if (!info->direct_chg.dev || !info->cc_online)
		return false;
	if (!READ_ONCE(info->authenticated))
		return false;
	if (info->direct_health != POWER_SUPPLY_HEALTH_GOOD)
		return false;
	if (info->vbat_uv < info->dc_min_vbat_uv)
		return false;
	if (info->temp < info->dc_min_temp || info->temp > info->dc_max_temp)
		return false;
	if (info->soc >= 100 || info->thermal_level > 0 ||
	    lim->fcc_ua < 5800000)
		return false;

	vbus = info->tcpm_vbus_uv ?: info->pmic_vbus_uv;
	if (vbus < info->dc_vbus_min_uv || vbus > info->dc_vbus_max_uv)
		return false;

	return true;
}

static int qcom_bms_dc_enter(struct qcom_bms_info *info,
			     const struct qcom_bms_limits *lim)
{
	struct qcom_bms_charger_ref direct = {};
	struct qcom_bms_charger_ref pmic = {};
	bool have_direct, have_pmic;
	int online = 0;
	int ret;

	mutex_lock(&info->ops_lock);
	have_direct = qcom_bms_charger_acquire_locked(&info->direct_chg,
						      &direct);
	have_pmic = qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
	mutex_unlock(&info->ops_lock);

	if (!have_direct || !direct.ops->set_charging_enabled) {
		ret = -ENODEV;
		goto out;
	}

	if (have_pmic && pmic.ops->set_charging_enabled &&
	    !info->pmic_charging_enabled) {
		if (!pmic.ops->set_charging_enabled(pmic.priv, true))
			info->pmic_charging_enabled = true;
	}

	if (direct.ops->init_hw) {
		ret = direct.ops->init_hw(direct.priv);
		if (ret) {
			dev_warn(info->dev,
				 "failed to re-init direct charger: %d\n", ret);
			goto out;
		}
	}

	if (direct.ops->set_float_voltage)
		direct.ops->set_float_voltage(direct.priv, lim->fv_uv);
	if (direct.ops->set_input_current_limit)
		direct.ops->set_input_current_limit(direct.priv,
						    lim->fcc_ua / 2);

	ret = direct.ops->set_charging_enabled(direct.priv, true);
	if (ret) {
		dev_dbg(info->dev, "direct charge enable failed: %d\n", ret);
		goto out;
	}

	if (direct.ops->get_online) {
		ret = direct.ops->get_online(direct.priv, &online);
		if (ret || !online) {
			direct.ops->set_charging_enabled(direct.priv, false);
			ret = -EAGAIN;
			goto out;
		}
	}

	if (have_pmic && pmic.ops->set_usbin_suspend) {
		ret = pmic.ops->set_usbin_suspend(pmic.priv, true);
		if (ret) {
			direct.ops->set_charging_enabled(direct.priv, false);
			goto out;
		}
		info->pmic_usbin_suspended = true;
	}

	info->applied_direct_fcc_ua = lim->fcc_ua / 2;
	info->applied_direct_fv_uv = lim->fv_uv;
	ret = 0;

out:
	if (have_pmic)
		qcom_bms_charger_release(info, &pmic);
	if (have_direct)
		qcom_bms_charger_release(info, &direct);
	return ret;
}

static void qcom_bms_dc_exit(struct qcom_bms_info *info)
{
	struct qcom_bms_charger_ref direct = {};
	struct qcom_bms_charger_ref pmic = {};
	bool have_direct, have_pmic;

	mutex_lock(&info->ops_lock);
	have_direct = qcom_bms_charger_acquire_locked(&info->direct_chg,
						      &direct);
	have_pmic = qcom_bms_charger_acquire_locked(&info->pmic_chg, &pmic);
	mutex_unlock(&info->ops_lock);

	if (have_direct && direct.ops->set_charging_enabled)
		direct.ops->set_charging_enabled(direct.priv, false);

	if (have_pmic && pmic.ops->set_usbin_suspend &&
	    info->pmic_usbin_suspended) {
		if (!pmic.ops->set_usbin_suspend(pmic.priv, false))
			info->pmic_usbin_suspended = false;
		else
			dev_warn_ratelimited(info->dev,
					     "failed to resume PMIC USBIN\n");
	}

	info->applied_direct_fcc_ua = 0;
	info->applied_direct_fv_uv = 0;

	if (have_pmic)
		qcom_bms_charger_release(info, &pmic);
	if (have_direct)
		qcom_bms_charger_release(info, &direct);
}

static void qcom_bms_work(struct work_struct *work)
{
	struct qcom_bms_info *info =
		container_of(to_delayed_work(work), struct qcom_bms_info, work);
	struct qcom_bms_limits lim = {};
	bool input;
	bool allowed;
	int delay = QCOM_BMS_TICK_IDLE_MS;
	int ret;

	if (READ_ONCE(info->removing))
		return;

	info->tick++;
	qcom_bms_read_inputs(info);
	qcom_bms_update_status(info);
	if (info->soc != info->reported_soc) {
		info->reported_soc = info->soc;
		power_supply_changed(info->bms_psy);
	}
	qcom_bms_compute_limits(info, &lim);
	qcom_bms_apply_pmic_limits(info, &lim);

	info->dbg_fcc_ua = lim.fcc_ua > 0 ? lim.fcc_ua : 0;
	info->dbg_fv_uv = lim.fv_uv > 0 ? lim.fv_uv : 0;
	info->dbg_icl_ua = lim.icl_ua > 0 ? lim.icl_ua : 0;
	info->dbg_vbat_uv = info->vbat_uv > 0 ? info->vbat_uv : 0;
	info->dbg_soc = info->soc >= 0 ? info->soc : 0;

	input = info->cc_online || info->pmic_online || info->direct_online;

	if (!input) {
		if (info->state == QCOM_BMS_DC_ACTIVE ||
		    info->pmic_usbin_suspended)
			qcom_bms_dc_exit(info);
		info->state = QCOM_BMS_OFFLINE;
		info->backoff_ms = 0;
		delay = QCOM_BMS_TICK_IDLE_MS;
		goto out;
	}

	if (info->state != QCOM_BMS_DC_ACTIVE && info->pmic_usbin_suspended)
		qcom_bms_dc_exit(info);

	if (info->state == QCOM_BMS_OFFLINE)
		info->state = QCOM_BMS_PMIC;

	switch (info->state) {
	case QCOM_BMS_PMIC:
	case QCOM_BMS_BACKOFF:
		if (time_before(jiffies, info->backoff_until))
			break;
		if (!qcom_bms_dc_allowed(info, &lim))
			break;

		ret = qcom_bms_dc_enter(info, &lim);
		if (ret) {
			if (info->backoff_ms)
				info->backoff_ms = min(info->backoff_ms * 2,
						       QCOM_BMS_DC_BACKOFF_MAX_MS);
			else
				info->backoff_ms = QCOM_BMS_DC_BACKOFF_MIN_MS;
			info->backoff_until = jiffies +
					      msecs_to_jiffies(info->backoff_ms);
			info->state = QCOM_BMS_BACKOFF;
			dev_dbg(info->dev,
				"direct charge not ready (%d), retry in %d ms\n",
				ret, info->backoff_ms);
			break;
		}

		info->state = QCOM_BMS_DC_ACTIVE;
		info->backoff_ms = 0;
		dev_info(info->dev, "direct charge active\n");
		break;
	case QCOM_BMS_DC_ACTIVE:
		allowed = qcom_bms_dc_allowed(info, &lim);
		if (!allowed || !info->direct_online) {
			dev_info(info->dev,
				 "leaving direct charge (allowed=%d online=%d)\n",
				 allowed, info->direct_online);
			qcom_bms_dc_exit(info);
			info->state = QCOM_BMS_BACKOFF;
			info->backoff_ms = QCOM_BMS_DC_BACKOFF_MIN_MS;
			info->backoff_until = jiffies +
					      msecs_to_jiffies(info->backoff_ms);
			break;
		}
		qcom_bms_apply_direct_limits(info, &lim);
		break;
	default:
		info->state = QCOM_BMS_PMIC;
		break;
	}

	/*
	 * Safety net: if a direct-charge disable failed earlier, retry it while
	 * the controller is not in DC_ACTIVE.  The PMIC must not be left
	 * suspended with the switched capacitor still switching.
	 */
	if (info->state != QCOM_BMS_DC_ACTIVE && info->direct_online)
		qcom_bms_dc_exit(info);

	info->dbg_state = info->state;
	delay = info->state == QCOM_BMS_DC_ACTIVE || input ?
			QCOM_BMS_TICK_ACTIVE_MS : QCOM_BMS_TICK_IDLE_MS;

out:
	if (!READ_ONCE(info->removing))
		mod_delayed_work(system_wq, &info->work,
				 msecs_to_jiffies(delay));
}

static int qcom_bms_psy_notifier(struct notifier_block *nb, unsigned long event,
				 void *data)
{
	struct qcom_bms_info *info =
		container_of(nb, struct qcom_bms_info, psy_nb);
	struct power_supply *psy = data;

	if (event == PSY_EVENT_PROP_CHANGED && info->tcpm_np &&
	    psy->dev.of_node == info->tcpm_np && !READ_ONCE(info->removing))
		mod_delayed_work(system_wq, &info->work, 0);

	return NOTIFY_OK;
}

int qcom_bms_register_fg(struct device *dev, const struct qcom_bms_fg_ops *ops,
			 void *priv)
{
	struct qcom_bms_info *info;
	int ret;

	if (!dev || !ops)
		return -EINVAL;

	info = qcom_bms_lock();
	if (!info)
		return -EPROBE_DEFER;

	if (info->fg.dev || info->fg.unregistering) {
		ret = -EBUSY;
		goto out_unlock;
	}

	info->fg.link = device_link_add(dev, info->dev, DL_FLAG_STATELESS);
	if (!info->fg.link) {
		dev_err(dev, "failed to link fuel gauge to BMS\n");
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	info->fg.dev = dev;
	info->fg.ops = ops;
	info->fg.priv = priv;
	mutex_unlock(&info->ops_lock);

	qcom_bms_notify_changed(dev);
	return 0;

out_unlock:
	mutex_unlock(&info->ops_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_bms_register_fg);

int qcom_bms_register_charger(struct device *dev,
			      enum qcom_bms_charger_role role,
			      const struct qcom_bms_charger_ops *ops,
			      void *priv)
{
	struct qcom_bms_info *info;
	struct qcom_bms_charger *charger;
	int ret;

	if (!dev || !ops)
		return -EINVAL;
	if (role != QCOM_BMS_CHARGER_PMIC && role != QCOM_BMS_CHARGER_DIRECT)
		return -EINVAL;

	info = qcom_bms_lock();
	if (!info)
		return -EPROBE_DEFER;

	charger = role == QCOM_BMS_CHARGER_PMIC ? &info->pmic_chg :
						  &info->direct_chg;
	if (charger->dev || charger->unregistering) {
		ret = -EBUSY;
		goto out_unlock;
	}

	charger->link = device_link_add(dev, info->dev, DL_FLAG_STATELESS);
	if (!charger->link) {
		dev_err(dev, "failed to link charger to BMS\n");
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	charger->dev = dev;
	charger->ops = ops;
	charger->priv = priv;
	mutex_unlock(&info->ops_lock);

	qcom_bms_notify_changed(dev);
	return 0;

out_unlock:
	mutex_unlock(&info->ops_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_bms_register_charger);

void qcom_bms_unregister_fg(struct device *dev)
{
	struct qcom_bms_info *info;
	struct device_link *link;

	info = qcom_bms_lock();
	if (!info)
		return;

	if (info->fg.dev != dev) {
		mutex_unlock(&info->ops_lock);
		return;
	}

	link = info->fg.link;
	info->fg.dev = NULL;
	info->fg.link = NULL;
	info->fg.ops = NULL;
	info->fg.priv = NULL;
	info->fg.unregistering = true;
	mutex_unlock(&info->ops_lock);

	wait_for_completion(&info->fg.idle);
	if (link)
		device_link_del(link);

	mutex_lock(&info->ops_lock);
	info->fg.unregistering = false;
	mutex_unlock(&info->ops_lock);

	if (!READ_ONCE(info->removing))
		mod_delayed_work(system_wq, &info->work, 0);
}
EXPORT_SYMBOL_GPL(qcom_bms_unregister_fg);

void qcom_bms_unregister_charger(struct device *dev)
{
	struct qcom_bms_info *info;
	struct qcom_bms_charger *charger;
	struct device_link *link;

	info = qcom_bms_lock();
	if (!info)
		return;

	if (info->pmic_chg.dev == dev) {
		charger = &info->pmic_chg;
	} else if (info->direct_chg.dev == dev) {
		charger = &info->direct_chg;
	} else {
		mutex_unlock(&info->ops_lock);
		return;
	}

	link = charger->link;
	charger->dev = NULL;
	charger->link = NULL;
	charger->ops = NULL;
	charger->priv = NULL;
	charger->unregistering = true;
	mutex_unlock(&info->ops_lock);

	wait_for_completion(&charger->idle);
	if (link)
		device_link_del(link);

	mutex_lock(&info->ops_lock);
	charger->unregistering = false;
	mutex_unlock(&info->ops_lock);

	if (!READ_ONCE(info->removing))
		mod_delayed_work(system_wq, &info->work, 0);
}
EXPORT_SYMBOL_GPL(qcom_bms_unregister_charger);

int qcom_bms_register_authenticator(struct device *dev,
				    enum qcom_bms_auth_role role)
{
	struct qcom_bms_authenticator *auth;
	struct qcom_bms_info *info;
	int ret;

	if (!dev || role < 0 || role >= QCOM_BMS_AUTH_MAX)
		return -EINVAL;

	info = qcom_bms_lock();
	if (!info)
		return -EPROBE_DEFER;

	auth = &info->auth[role];
	if (auth->dev) {
		ret = -EBUSY;
		goto out_unlock;
	}

	auth->link = device_link_add(dev, info->dev, DL_FLAG_STATELESS);
	if (!auth->link) {
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	auth->dev = dev;
	mutex_unlock(&info->ops_lock);

	return 0;

out_unlock:
	mutex_unlock(&info->ops_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_bms_register_authenticator);

void qcom_bms_unregister_authenticator(struct device *dev)
{
	struct qcom_bms_info *info;
	struct device_link *link = NULL;
	int i;

	info = qcom_bms_lock();
	if (!info)
		return;

	for (i = 0; i < QCOM_BMS_AUTH_MAX; i++) {
		if (info->auth[i].dev != dev)
			continue;

		link = info->auth[i].link;
		memset(&info->auth[i], 0, sizeof(info->auth[i]));
		break;
	}
	mutex_unlock(&info->ops_lock);

	if (!link)
		return;

	device_link_del(link);
	qcom_bms_refresh_authentication(info);
}
EXPORT_SYMBOL_GPL(qcom_bms_unregister_authenticator);

static void qcom_bms_refresh_authentication(struct qcom_bms_info *info)
{
	u32 authenticated_mask = 0;
	bool authenticated;
	bool changed;
	int i;

	mutex_lock(&info->ops_lock);
	for (i = 0; i < QCOM_BMS_AUTH_MAX; i++) {
		if (info->auth[i].dev && info->auth[i].authenticated)
			authenticated_mask |= BIT(i);
	}

	authenticated = (authenticated_mask & info->required_auth_mask) ==
			info->required_auth_mask;
	changed = info->authenticated != authenticated;
	info->authenticated = authenticated;
	mutex_unlock(&info->ops_lock);

	if (!changed)
		return;

	dev_info(info->dev, "battery authentication %s (mask %#x/%#x)\n",
		 authenticated ? "passed" : "failed", authenticated_mask,
		 info->required_auth_mask);
	power_supply_changed(info->bms_psy);

	if (!READ_ONCE(info->removing))
		mod_delayed_work(system_wq, &info->work, 0);
}

void qcom_bms_authentication_changed(struct device *dev, bool authenticated)
{
	struct qcom_bms_info *info;
	int i;

	info = qcom_bms_lock();
	if (!info)
		return;

	for (i = 0; i < QCOM_BMS_AUTH_MAX; i++) {
		if (info->auth[i].dev != dev)
			continue;

		info->auth[i].authenticated = authenticated;
		break;
	}
	mutex_unlock(&info->ops_lock);

	if (i < QCOM_BMS_AUTH_MAX)
		qcom_bms_refresh_authentication(info);
}
EXPORT_SYMBOL_GPL(qcom_bms_authentication_changed);

void qcom_bms_notify_changed(struct device *dev)
{
	struct qcom_bms_info *info;
	bool registered;

	info = qcom_bms_lock();
	if (!info)
		return;

	registered = info->fg.dev == dev || info->pmic_chg.dev == dev ||
		     info->direct_chg.dev == dev;
	if (registered && !READ_ONCE(info->removing))
		mod_delayed_work(system_wq, &info->work, 0);
	mutex_unlock(&info->ops_lock);
}
EXPORT_SYMBOL_GPL(qcom_bms_notify_changed);

static int qcom_bms_parse_ranges(struct device *dev, const char *prop,
				 struct qcom_bms_jeita_range **out,
				 int *out_len)
{
	struct qcom_bms_jeita_range *range;
	u32 *vals;
	int count;
	int i, ret;

	count = device_property_count_u32(dev, prop);
	if (count < 0)
		return count;
	if (!count)
		return -ENODATA;
	if (count % 3)
		return -EINVAL;

	vals = devm_kcalloc(dev, count, sizeof(*vals), GFP_KERNEL);
	if (!vals)
		return -ENOMEM;

	ret = device_property_read_u32_array(dev, prop, vals, count);
	if (ret)
		return ret;

	count /= 3;
	range = devm_kcalloc(dev, count, sizeof(*range), GFP_KERNEL);
	if (!range)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		range[i].low = (s32)vals[i * 3];
		range[i].high = (s32)vals[i * 3 + 1];
		range[i].value = (s32)vals[i * 3 + 2];
		if (range[i].low > range[i].high)
			return -EINVAL;
	}

	*out = range;
	*out_len = count;
	return 0;
}

static int qcom_bms_parse_u32_array(struct device *dev, const char *prop,
				    u32 **out, int *out_len)
{
	u32 *vals;
	int count;
	int ret;

	count = device_property_count_u32(dev, prop);
	if (count < 0)
		return count;
	if (!count)
		return -ENODATA;

	vals = devm_kcalloc(dev, count, sizeof(*vals), GFP_KERNEL);
	if (!vals)
		return -ENOMEM;

	ret = device_property_read_u32_array(dev, prop, vals, count);
	if (ret)
		return ret;

	*out = vals;
	*out_len = count;
	return 0;
}

static void qcom_bms_parse_dt(struct qcom_bms_info *info)
{
	struct device *dev = info->dev;

	info->fcc_max_ua = 3000000;
	info->fv_max_uv = 4470000;
	info->icl_max_ua = QCOM_BMS_USB_ICL_UA;
	info->unverified_fcc_ua = QCOM_BMS_UNVERIFIED_FCC_UA;
	info->unverified_fv_uv = QCOM_BMS_UNVERIFIED_FV_UV;
	info->dc_min_vbat_uv = QCOM_BMS_DC_MIN_VBAT_UV;
	info->dc_vbus_min_uv = QCOM_BMS_DC_VBUS_MIN_UV;
	info->dc_vbus_max_uv = QCOM_BMS_DC_VBUS_MAX_UV;
	info->dc_min_temp = QCOM_BMS_DC_MIN_TEMP;
	info->dc_max_temp = QCOM_BMS_DC_MAX_TEMP;
	info->jeita.hysteresis = QCOM_BMS_DEFAULT_JEITA_HYST;
	info->jeita.too_cold = QCOM_BMS_DEFAULT_TOO_COLD;
	info->jeita.cool_th = QCOM_BMS_DEFAULT_COOL_TH;
	info->jeita.warm_th = QCOM_BMS_DEFAULT_WARM_TH;
	info->jeita.too_hot = QCOM_BMS_DEFAULT_TOO_HOT;
	info->jeita.index = -1;

	device_property_read_u32(dev, "qcom,required-authenticators",
				 &info->required_auth_mask);
	info->required_auth_mask &= GENMASK(QCOM_BMS_AUTH_MAX - 1, 0);

	device_property_read_u32(dev, "qcom,usb-icl-ua", &info->icl_max_ua);
	device_property_read_u32(dev, "qcom,unverified-fcc-ua",
				 &info->unverified_fcc_ua);
	device_property_read_u32(dev, "qcom,unverified-fv-uv",
				 &info->unverified_fv_uv);
	device_property_read_u32(dev, "qcom,dc-min-vbat-uv",
				 &info->dc_min_vbat_uv);
	device_property_read_u32(dev, "qcom,dc-vbus-min-uv",
				 &info->dc_vbus_min_uv);
	device_property_read_u32(dev, "qcom,dc-vbus-max-uv",
				 &info->dc_vbus_max_uv);
	device_property_read_u32(dev, "qcom,dc-min-temp-decic",
				 &info->dc_min_temp);
	device_property_read_u32(dev, "qcom,dc-max-temp-decic",
				 &info->dc_max_temp);
	device_property_read_u32(dev, "qcom,jeita-hysteresis-decic",
				 &info->jeita.hysteresis);
	device_property_read_u32(dev, "qcom,jeita-too-cold",
				 &info->jeita.too_cold);
	device_property_read_u32(dev, "qcom,jeita-cool-th",
				 &info->jeita.cool_th);
	device_property_read_u32(dev, "qcom,jeita-warm-th",
				 &info->jeita.warm_th);
	device_property_read_u32(dev, "qcom,jeita-too-hot",
				 &info->jeita.too_hot);

	if (qcom_bms_parse_ranges(dev, "qcom,jeita-fcc-ranges",
				  &info->jeita.fcc, &info->jeita.fcc_len))
		info->jeita.fcc_len = 0;
	if (qcom_bms_parse_ranges(dev, "qcom,jeita-fv-ranges",
				  &info->jeita.fv, &info->jeita.fv_len))
		info->jeita.fv_len = 0;

	if (qcom_bms_parse_u32_array(dev, "qcom,thermal-mitigation-icl-ua",
				     &info->thermal_icl,
				     &info->thermal_icl_len))
		info->thermal_icl_len = 0;
	if (qcom_bms_parse_u32_array(dev, "qcom,thermal-mitigation-fcc-ua",
				     &info->thermal_fcc,
				     &info->thermal_fcc_len))
		info->thermal_fcc_len = 0;

	info->thermal_level = 0;
}

static void qcom_bms_debugfs_init(struct qcom_bms_info *info)
{
	info->debugfs = debugfs_create_dir("qcom-bms", NULL);
	if (IS_ERR_OR_NULL(info->debugfs)) {
		info->debugfs = NULL;
		return;
	}

	debugfs_create_u32("fcc_ua", 0444, info->debugfs, &info->dbg_fcc_ua);
	debugfs_create_u32("fv_uv", 0444, info->debugfs, &info->dbg_fv_uv);
	debugfs_create_u32("icl_ua", 0444, info->debugfs, &info->dbg_icl_ua);
	debugfs_create_u32("state", 0444, info->debugfs, &info->dbg_state);
	debugfs_create_u32("vbat_uv", 0444, info->debugfs, &info->dbg_vbat_uv);
	debugfs_create_u32("soc", 0444, info->debugfs, &info->dbg_soc);
	debugfs_create_bool("authenticated", 0444, info->debugfs,
			    &info->authenticated);
	debugfs_create_bool("pmic_usbin_suspended", 0444, info->debugfs,
			    &info->pmic_usbin_suspended);
}

static int qcom_bms_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct qcom_bms_info *info;
	int ret;

	mutex_lock(&qcom_bms_mutex);
	if (g_bms) {
		mutex_unlock(&qcom_bms_mutex);
		return -EBUSY;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		mutex_unlock(&qcom_bms_mutex);
		return -ENOMEM;
	}

	info->dev = &pdev->dev;
	info->status = POWER_SUPPLY_STATUS_UNKNOWN;
	info->soc = -1;
	info->reported_soc = -1;
	info->temp = 250;
	mutex_init(&info->ops_lock);
	init_completion(&info->fg.idle);
	complete_all(&info->fg.idle);
	init_completion(&info->pmic_chg.idle);
	complete_all(&info->pmic_chg.idle);
	init_completion(&info->direct_chg.idle);
	complete_all(&info->direct_chg.idle);
	INIT_DELAYED_WORK(&info->work, qcom_bms_work);
	qcom_bms_parse_dt(info);
	info->authenticated = !info->required_auth_mask;

	psy_cfg.drv_data = info;
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);

	info->bms_psy = devm_power_supply_register(
		&pdev->dev, &qcom_bms_psy_desc, &psy_cfg);
	if (IS_ERR(info->bms_psy)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(info->bms_psy),
				    "failed to register %s\n",
				    QCOM_BMS_PSY_NAME);
		mutex_unlock(&qcom_bms_mutex);
		return ret;
	}

	info->usb_psy = devm_power_supply_register(
		&pdev->dev, &qcom_bms_usb_psy_desc, &psy_cfg);
	if (IS_ERR(info->usb_psy)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(info->usb_psy),
				    "failed to register %s\n",
				    QCOM_BMS_USB_PSY_NAME);
		mutex_unlock(&qcom_bms_mutex);
		return ret;
	}

	ret = power_supply_get_battery_info(info->bms_psy, &info->batt_info);
	if (ret == -EPROBE_DEFER) {
		mutex_unlock(&qcom_bms_mutex);
		return ret;
	}
	if (!ret && info->batt_info) {
		if (info->batt_info->constant_charge_current_max_ua > 0)
			info->fcc_max_ua =
				info->batt_info->constant_charge_current_max_ua;
		if (info->batt_info->constant_charge_voltage_max_uv > 0)
			info->fv_max_uv =
				info->batt_info->constant_charge_voltage_max_uv;
	} else {
		dev_warn(&pdev->dev,
			 "no monitored-battery, using safe defaults: %d\n",
			 ret);
		info->batt_info = NULL;
	}

	g_bms = info;
	platform_set_drvdata(pdev, info);
	mutex_unlock(&qcom_bms_mutex);

	dev_info(&pdev->dev,
		 "Qualcomm BMS registered (auth mask %#x, FCC %d uA, FV %d uV)\n",
		 info->required_auth_mask, info->fcc_max_ua, info->fv_max_uv);

	qcom_bms_debugfs_init(info);

	info->tcpm_np = of_parse_phandle(pdev->dev.of_node, "qcom,tcpm-psy", 0);
	if (!info->tcpm_np)
		dev_warn(&pdev->dev,
			 "qcom,tcpm-psy missing; direct charge disabled\n");
	info->psy_nb.notifier_call = qcom_bms_psy_notifier;
	power_supply_reg_notifier(&info->psy_nb);

	mod_delayed_work(system_wq, &info->work, 0);

	return 0;
}

static void qcom_bms_remove(struct platform_device *pdev)
{
	struct qcom_bms_info *info;

	mutex_lock(&qcom_bms_mutex);
	info = g_bms;
	g_bms = NULL;
	mutex_unlock(&qcom_bms_mutex);

	if (!info)
		return;

	WRITE_ONCE(info->removing, true);
	debugfs_remove_recursive(info->debugfs);
	power_supply_unreg_notifier(&info->psy_nb);
	cancel_delayed_work_sync(&info->work);

	if (info->state == QCOM_BMS_DC_ACTIVE || info->pmic_usbin_suspended)
		qcom_bms_dc_exit(info);

	if (info->tcpm_psy)
		power_supply_put(info->tcpm_psy);
	of_node_put(info->tcpm_np);

	if (info->batt_info)
		power_supply_put_battery_info(info->bms_psy, info->batt_info);
}

static int qcom_bms_suspend(struct device *dev)
{
	struct qcom_bms_info *info = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&info->work);
	return 0;
}

static int qcom_bms_resume(struct device *dev)
{
	struct qcom_bms_info *info = dev_get_drvdata(dev);

	if (!READ_ONCE(info->removing))
		mod_delayed_work(system_wq, &info->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(qcom_bms_pm, qcom_bms_suspend, qcom_bms_resume);

static const struct of_device_id qcom_bms_of_match[] = {
	{ .compatible = "qcom,pm8150b-bms" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_bms_of_match);

static struct platform_driver qcom_bms_driver = {
	.driver = {
		.name = "qcom-bms",
		.of_match_table = qcom_bms_of_match,
		.pm = pm_sleep_ptr(&qcom_bms_pm),
	},
	.probe = qcom_bms_probe,
	.remove = qcom_bms_remove,
};
module_platform_driver(qcom_bms_driver);

MODULE_AUTHOR("TwinbornPlate75 <3342733415@qq.com>");
MODULE_DESCRIPTION("Qualcomm Unified Battery/Charger Manager");
MODULE_LICENSE("GPL");
