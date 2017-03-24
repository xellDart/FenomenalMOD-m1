/* drives/power/rt5058-charger.c
 * Charger Driver for Richtek RT5058
 *
 * Copyright (C) 2015 Richtek Technology Corp.
 * Author: Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <linux/mfd/rt5058/rt5058.h>
#include <linux/power/rt5058-charger.h>
#ifdef CONFIG_LGE_PM_CABLE_DETECTION
#include <soc/qcom/lge/lge_cable_detection.h>
#endif
#if defined (CONFIG_LGE_PM_CHARGING_CONTROLLER)\
	|| defined (CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER)
#include <soc/qcom/lge/lge_charging_scenario.h>
#endif
#ifdef CONFIG_LGE_PM_LG_POWER_CORE
#include <soc/qcom/lge/power/lge_power_class.h>
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
#include <soc/qcom/lge/power/lge_cable_detect.h>
#endif
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
#include <linux/reboot.h>
#endif
#define MINVAL(a, b)	((a <= b) ? a : b)
#define RT5058_USBREPORT_TIMER	(3000) /* 3 sec  */
#define RT5058_BTM_WAKE_LOCK_TIME (65000) /* 65 sec */
#define RT5058_STATUS_WAKE_LOCK_TIME (2000) /* 2 sec */

struct rt5058_charger_info {
	struct i2c_client *client;
	struct device *dev;
	struct rt5058_charger_platform_data *cdata;
	struct power_supply psy;
	struct delayed_work usbinsert_work;
	struct regulator *sldo1;
	struct wake_lock safety_wake_lock;
	struct wake_lock status_wake_lock;
	struct wake_lock ext_psy_wake_lock;
	struct mutex full_charged_lock;
	struct mutex chg_enabled_lock;
	int battery_status;
	int online;
	int batt_present;
	int input_curr_ma;
	int cable_type;
	int chg_curr_ma;
	int batt_health;
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	int usb_curr_ma;
	u8 usb_curr_ma_on:1;
#endif
	u8 temp_status;
	u8 full_charged:1;
	u8 ovp:1;
	u8 otg_en:1;
	u8 usbin_running:1;
	u8 attach_status_now:1;
	u8 attach_status_old:1;
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	u8 safety_timer:1;
	u8 time_expired:1;
#endif
	u8 iinms_done:1;
	u8 chg_enabled:1;
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	u8 testmode:1;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	u8 is_factory_cable:1;
	struct lge_power *lge_cd_lpc;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	struct lge_power *lge_cc_lpc;
	int btm_state;
	int pseudo_ui;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	struct delayed_work current_settled_work;
	struct lge_power *lge_vzw_lpc;
	int current_settled;
	int input_current_trim;
#endif
};
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
static int lgcc_charging_current;
#define BATT_TEMP_OVERHEAT (57)
#define BATT_TEMP_COLD (-10)
#endif

static void rt5058_get_batt_present(struct rt5058_charger_info *ri);

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
static void rt5058_update_battery_health(struct rt5058_charger_info *ri)
{
	int pre_health = ri->batt_health;

	if (ri->btm_state == BTM_HEALTH_OVERHEAT)
		ri->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (ri->btm_state == BTM_HEALTH_COLD)
		ri->batt_health = POWER_SUPPLY_HEALTH_COLD;
	else
		ri->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	if (pre_health != ri->batt_health) {
		pr_debug("%s: batt health changed (%d)->(%d)\n",
				__func__, pre_health, ri->batt_health);
		power_supply_changed(&ri->psy);
	}
}
#endif

/* not used */
static void rt5058_charger_otg_control(
				struct rt5058_charger_info *ri, bool enable)
{
	union power_supply_propval value;
	int regval = 0;
	int ret;

	pr_info("%s: called charger otg control : %s\n", __func__,
						enable ? "on" : "off");

	if (!enable) {
		/* turn off OTG */
		ret = rt5058_clr_bits(ri->client,
				RT5058_REG_CHGCTRL1, RT5058_OPAMODE_MASK);
		ri->otg_en = 0;
	} else {
		/* Set OTG boost vout = bst_volt, turn on OTG */
		regval = (ri->cdata->bst_volt - 3625) / 25;
		ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL8,
			RT5058_OTGVOLT_MASK, regval << RT5058_OTGVOLT_SHIFT);
		ret = rt5058_set_bits(ri->client,
				RT5058_REG_CHGCTRL1, RT5058_OPAMODE_MASK);

		rt5058_clr_bits(ri->client, RT5058_REG_CHGCTRL1,
							RT5058_CHGHZ_MASK);
		ri->cable_type = POWER_SUPPLY_TYPE_OTG;
		ri->otg_en = 1;
		value.intval = POWER_SUPPLY_STATUS_DISCHARGING;
		psy_do_property(rt5058_chg_devname, set,
					POWER_SUPPLY_PROP_STATUS, value);
	}
}

static void rt5058_unlock_chg_pascode(struct rt5058_charger_info *ri)
{
	rt5058_reg_write(ri->client, RT5058_REG_HIDDEN_PAS_CODE1, 0x96);
	rt5058_reg_write(ri->client, RT5058_REG_HIDDEN_PAS_CODE2, 0x69);
	rt5058_reg_write(ri->client, RT5058_REG_HIDDEN_PAS_CODE3, 0xc3);
	rt5058_reg_write(ri->client, RT5058_REG_HIDDEN_PAS_CODE4, 0x3c);
}

static void rt5058_lock_chg_pascode(struct rt5058_charger_info *ri)
{
	u8 regval[4] = {0, 0, 0, 0};
	int ret, count = 10; /* retry 10 times if failed */

	while (count) {
		/* set lock pascode */
		ret = rt5058_block_write(ri->client,
			RT5058_REG_HIDDEN_PAS_CODE1, 4, regval);
		/* check lock */
		ret = rt5058_reg_read(ri->client, RT5058_REG_CHGHIDDENCTRL1);
		if (ret != 0)
			count--;
		else
			return;
	}
	BUG_ON(count == 0);
}

static void rt5058_enable_isink_backbst(struct rt5058_charger_info *ri,
	u8 enable)
{
	rt5058_unlock_chg_pascode(ri);

	if (enable)
		rt5058_set_bits(ri->client, RT5058_REG_CHGHIDDENCTRL2, 0x04);
	else
		rt5058_clr_bits(ri->client, RT5058_REG_CHGHIDDENCTRL2, 0x04);

	rt5058_lock_chg_pascode(ri);
}

static int rt5058_get_charge_type(struct rt5058_charger_info *ri)
{
	int status = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	int ret;

	ret = rt5058_reg_read(ri->client, RT5058_REG_CHGSTAT);
	if (ret < 0)
		dev_err(ri->dev, "%s fail\n", __func__);

	switch (ret & RT5058_CHGVBATFC_MASK) {
	case RT5058_CHGVBATFC_MASK:
		status = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	default:
		status = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	}
	return status;
}

static int rt5058_get_cv(struct rt5058_charger_info *ri)
{
	int cv = 0, regval = 0;
	regval = rt5058_reg_read(ri->client, RT5058_REG_CHGCTRL7);
	if (regval < 0)
		return -EINVAL;
	regval &= RT5058_CHGCV_MASK;
	regval >>= RT5058_CHGCV_SHIFT;
	if (regval <= 36)
		cv = 3750 + regval * 25;
	else
		cv = RT5058_MAX_CHGCV;
	if (cv != ri->cdata->chg_volt)
		cv = ri->cdata->chg_volt;

	return cv;
}

static int rt5058_set_cv(struct rt5058_charger_info *ri, int cv)
{
	int regval = 0, ret = 0;

	if (cv > 4650)
		regval = 32;
	else if (cv < 3750)
		regval = 0;
	else
		regval = (cv - 3750) / 25;

	rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL7,
		RT5058_CHGCV_MASK, regval << RT5058_CHGCV_SHIFT);

	if (ret < 0) {
		pr_err("%s: set error\n", __func__);
		return -EINVAL;
	}
	else {
		return ret;
	}
}

/* get AICR */
static int rt5058_get_input_current_limit(struct rt5058_charger_info *ri)
{
	int ret;

	ret = rt5058_reg_read(ri->client, RT5058_REG_CHGCTRL4);
	if (ret < 0)
		return ret;
	ret &= RT5058_CHGAICR_MASK;
	ret >>= RT5058_CHGAICR_SHIFT;
	if (!ret)
		return ret;
	if (ret >= 40)
		return RT5058_MAX_AICR;
	return ret * 50;
}

/* set AICR */
static int rt5058_set_input_current_regulation(struct rt5058_charger_info *ri,
		int ma)
{
	int ret = 0;
	int regval = 0;

	if (ma == ri->input_curr_ma)
		return 1;

	ri->input_curr_ma = ma;

	/* AICR */
	if (ri->input_curr_ma < 50 && ri->input_curr_ma >= 0)
		regval = 1; /* AICR = 50mA, do not use AICR disable */
	else if (ri->input_curr_ma <= 2000)
		regval = ri->input_curr_ma / 50; /* 50mA step */
	else
		regval = 40; /*101000*/

	pr_err("%s: set AICR = %dmA\n", __func__, regval * 50);

	regval <<= RT5058_CHGAICR_SHIFT;
	ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL4,
				RT5058_CHGAICR_MASK, regval);

	if (ret < 0) {
		pr_err("%s: set error\n", __func__);
		return -EINVAL;
	}
	else {
		return ret;
	}
}

/* set ICC */
static int rt5058_set_charging_current(struct rt5058_charger_info *ri,
		int curr_ma)
{
	int regval = 0;
	int ret = 0;
	int chg_curr_ma = 0;
#if defined (CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER)
	union lge_power_propval lge_value = {0,};

	if (!ri->lge_cc_lpc) {
		chg_curr_ma = curr_ma;
	} else {
		ri->lge_cc_lpc->get_property(ri->lge_cc_lpc,
			LGE_POWER_PROP_IS_CHG_LIMIT, &lge_value);
		if (lge_value.intval)
			chg_curr_ma = curr_ma;
		else
			chg_curr_ma = RT5058_MAX_ICC;
	}
#endif
	if (chg_curr_ma == ri->chg_curr_ma)
		return 1;

	if (!ri->otg_en) {
		if (chg_curr_ma <= 2500 && chg_curr_ma > 0)
			regval = (chg_curr_ma - 100) / 100;
		else
			regval = (RT5058_MAX_ICC - 100) / 100;

		ri->chg_curr_ma = (regval * 100) + 100;

		regval <<= RT5058_CHGICC_SHIFT;
		pr_err("%s: set ICC = 0x%02x\n", __func__, regval);

		ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL6,
					RT5058_CHGICC_MASK, regval);
	}
	else {
		pr_debug("%s: otg enabled\n", __func__);
	}

	if (ret < 0) {
		pr_err("%s: set error\n", __func__);
		return -EINVAL;
	}
	else {
		return ret;
	}
}

/* get ICC */
static int rt5058_get_charging_current(struct rt5058_charger_info *ri)
{
	int regval;

	regval = rt5058_reg_read(ri->client, RT5058_REG_CHGCTRL6);
	if (regval < 0) {
		dev_err(ri->dev, "read current now fail\n");
		return regval;
	}

	regval &= RT5058_CHGICC_MASK;
	regval >>= RT5058_CHGICC_SHIFT;

	regval = 100 + regval * 100;
	if (regval > RT5058_MAX_ICC)
		regval = RT5058_MAX_ICC;

	return regval;
}

/* charging enable/disable */
static void rt5058_enable_charging(struct rt5058_charger_info *ri, u8 en)
{
	mutex_lock(&ri->chg_enabled_lock);
	if (en) {
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
		if (!ri->time_expired) {
			rt5058_set_bits(ri->client, RT5058_REG_CHGCTRL2, RT5058_CHGEN_MASK);
			ri->chg_enabled = 1;
			rt5058_enable_isink_backbst(ri, ri->chg_enabled);
		}
		else {
			pr_err("%s: timeout expired.\n", __func__);
			mutex_unlock(&ri->chg_enabled_lock);
			return;
		}
#else
		rt5058_set_bits(ri->client, RT5058_REG_CHGCTRL2, RT5058_CHGEN_MASK);
		ri->chg_enabled = 1;
		rt5058_enable_isink_backbst(ri, ri->chg_enabled);
#endif
	}
	else {
		rt5058_clr_bits(ri->client, RT5058_REG_CHGCTRL2, RT5058_CHGEN_MASK);
		ri->chg_enabled = 0;
		rt5058_enable_isink_backbst(ri, ri->chg_enabled);
	}

	mutex_unlock(&ri->chg_enabled_lock);
	pr_err("%s: %s\n", __func__, en ? "enable" : "disable");
}

/* Enable/Disable TE Function */
static void rt5058_te_switch(struct rt5058_charger_info *ri, bool on_off)
{
	if (!ri->cdata->use_te) {
		RTINFO("rt5058 not use te function\n");
		return;
	}
	if (on_off) {
		rt5058_set_bits(ri->client,
				RT5058_REG_CHGCTRL1, RT5058_TEEN_MASK);
		RTINFO("TE Enable\n");
	} else {
		rt5058_clr_bits(ri->client,
				RT5058_REG_CHGCTRL1, RT5058_TEEN_MASK);
		RTINFO("TE Disable\n");
	}
}

#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
/* set chg safety timer */
static void rt5058_set_safety_timer(struct rt5058_charger_info *ri, u8 on_off)
{
	int ret = 0;

	if (ri->safety_timer == on_off)
		return;

	if (on_off) {
		ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL3,
				RT5058_WTFC_MASK,
				ri->cdata->fc_timer << RT5058_WTFC_SHIFT);
		if (ret < 0) {
			pr_err("%s: error set FC timer.", __func__);
			return;
		}
		ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL3,
				RT5058_WTPRC_MASK,
				ri->cdata->prc_timer << RT5058_WTPRC_SHIFT);
		if (ret < 0) {
			pr_err("%s: error set PRC timer.", __func__);
			return;
		}
		ri->safety_timer = 1;
	}
	else {
		ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL3,
				RT5058_WTFC_MASK | RT5058_WTPRC_MASK,
				RT5058_WTFC_MASK | RT5058_WTPRC_MASK);
		if (ret < 0) {
			pr_err("%s: error set timer disable.", __func__);
			return;
		}
		ri->safety_timer = 0;
	}

	pr_err("%s: %s.\n", __func__, ri->safety_timer ? "on":"off");
}
#endif

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
static struct rt5058_charger_info *ref;
static ssize_t rt5058_chg_store_attrs_factory(struct device *, struct device_attribute *, const char *, size_t count);
static ssize_t rt5058_chg_show_attrs_factory(struct device *, struct device_attribute *, char *);

#define RT5058_CHG_ATTR_FACTORY(_name)				\
{							\
	.attr = {.name = #_name, .mode = 0664},		\
	.show = rt5058_chg_show_attrs_factory,			\
	.store = rt5058_chg_store_attrs_factory,			\
}

static struct device_attribute rt5058_chg_attrs_factory[] = {
	RT5058_CHG_ATTR_FACTORY(at_pmrst),
	RT5058_CHG_ATTR_FACTORY(at_charge),
};

enum {
	AT_PMRST = 0,
	AT_CHARGE,
};

static ssize_t rt5058_chg_store_attrs_factory(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	const ptrdiff_t offset = attr - rt5058_chg_attrs_factory;
	int ret = 0;

	switch (offset) {
	case AT_PMRST:
			pr_err("%s: [TESTMODE] pmrst.\n", __func__);
			msleep(3000);
			machine_restart(NULL);
			ret = count;
		break;
	case AT_CHARGE:
			if (strncmp(buf, "0", 1) == 0) {
				pr_err("%s: [TESTMODE] charging off.\n", __func__);
				ref->testmode = 1;
				rt5058_enable_charging(ref, 0);
			}
			else if (strncmp(buf, "1", 1) == 0) {
				pr_err("%s: [TESTMODE] charging on.\n", __func__);
				ref->testmode = 1;
				rt5058_enable_charging(ref, 1);
			}
			ret = count;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static ssize_t rt5058_chg_show_attrs_factory(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	const ptrdiff_t offset = attr - rt5058_chg_attrs_factory;
	int ret = 0;

	switch (offset) {
	case AT_PMRST:
			pr_err("%s: [TESTMODE] pmrst.\n", __func__);
			pr_err("%s: [TESTMODE] pmrst show not works.\n", __func__);
			ret = 1;
		break;
	case AT_CHARGE:
		ret = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	ret = snprintf(buf, PAGE_SIZE, "%d\n", ret);

	return ret;
}

static int rt5058_chg_create_attrs_factory(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(rt5058_chg_attrs_factory); i++) {
		rc = device_create_file(dev, &rt5058_chg_attrs_factory[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	pr_err("%s:failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &rt5058_chg_attrs_factory[i]);
create_attrs_succeed:
	return rc;
}
#endif

static enum power_supply_property rt_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_BATT_PRESENT,
	POWER_SUPPLY_PROP_CHARGER_EOC,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	POWER_SUPPLY_PROP_SAFETY_TIMER,
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
#endif
	POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL,
};

static int rt5058_chg_prop_is_writeable(struct power_supply *psy,
			enum power_supply_property psp)
{
	switch(psp) {
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
		case POWER_SUPPLY_PROP_SAFETY_TIMER:
#endif
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			return 1;
		default:
			break;
	}
	return 0;
}
static char *rt_charger_supply_to_list[] = {
	"battery",
};

static int rt5058_chg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct rt5058_charger_info *ri = dev_get_drvdata(psy->dev->parent);
	int ret = 0;
	int aicr = 0, chg_curr = 0;
	union power_supply_propval value;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ri->online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = ri->battery_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = ri->batt_health;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 2500;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG: /* AICR */
		if (ri->cdata->use_aicr)
			val->intval = rt5058_get_input_current_limit(ri);
		else
			val->intval = -1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: /* charging current right now */
		aicr = rt5058_get_input_current_limit(ri);
		chg_curr = rt5058_get_charging_current(ri);

		ret = rt5058_reg_read(ri->client, RT5058_REG_CHGCTRL1);
		if (ret < 0) {
			dev_err(ri->dev, "read opa mode fail\n");
			return -EINVAL;
		}

		if (ret & RT5058_OPAMODE_MASK) /* boost mode */
			val->intval = -1;
		else {
			if (!aicr)
				val->intval = chg_curr;
			else
				val->intval = MINVAL(aicr, chg_curr);
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: /* charging voltage */
		val->intval = rt5058_get_cv(ri);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG: /* MIVR */
		if (ri->cdata->use_mivr)
			val->intval = ri->cdata->mivr;
		else
			val->intval = -1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3750;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4650;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = rt5058_get_charge_type(ri);
		break;
	case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
		val->intval = ri->otg_en;
		break;
	case POWER_SUPPLY_PROP_BATT_PRESENT:
		val->intval = ri->batt_present;
		break;
	case POWER_SUPPLY_PROP_CHARGER_EOC:
		val->intval = ri->full_charged;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (ri->testmode)
			val->intval = ri->chg_enabled;
		else
			val->intval = 1;
		break;
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	case POWER_SUPPLY_PROP_SAFETY_TIMER:
		val->intval = ri->safety_timer;
		break;
#endif
	case POWER_SUPPLY_PROP_TEMP:
		psy_do_property(rt5058_fuel_devname, get,
				POWER_SUPPLY_PROP_TEMP, value);
		if (value.intval == RT_DOPSY_FAIL)
			val->intval = 0;
		else
			val->intval = value.intval;
		break;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = ri->current_settled;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM:
		val->intval = ri->input_current_trim;
		break;
#endif
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

/* not used */
static int rt5058_set_temp_status(struct rt5058_charger_info *ri, int temp)
{
	int status = RT_TEMP_NORMAL;

	if (temp <= ri->cdata->temp_level[0])
		status = RT_TEMP_COLD;
	else if (temp > ri->cdata->temp_level[0] &&
		temp <= ri->cdata->temp_level[1])
		status = RT_TEMP_COOL;
	else if (temp > ri->cdata->temp_level[1] &&
		temp <= ri->cdata->temp_level[2])
		status = RT_TEMP_NORMAL;
	else if (temp > ri->cdata->temp_level[2] &&
		temp <= ri->cdata->temp_level[3])
		status = RT_TEMP_WARM;
	else if (temp > ri->cdata->temp_level[3])
		status = RT_TEMP_WARM;

	return status;
}

/* not used */
static void rt5058_set_chg_jeita(struct rt5058_charger_info *ri)
{
	union power_supply_propval value;

	RTINFO("Temperature Status = %d\n", ri->temp_status);
	switch (ri->temp_status) {
	case RT_TEMP_COLD:
	case RT_TEMP_HOT:
		/* Disable Charging */
		psy_do_property(rt5058_chg_devname, get,
					POWER_SUPPLY_PROP_STATUS, value);
		if (value.intval == POWER_SUPPLY_STATUS_CHARGING) {
			pr_info("%s: Disable Charging & JEITA\n", __func__);
			rt5058_enable_charging(ri, 0);
			rt5058_clr_bits(ri->client, RT5058_REG_CHGCTRL2,
							RT5058_ICCJEITA_MASK);
		}
		/* set overheat|cold to battery health*/
		value.intval = (ri->temp_status == RT_TEMP_COLD ?
				POWER_SUPPLY_HEALTH_COLD :
						POWER_SUPPLY_HEALTH_OVERHEAT);
		psy_do_property(rt5058_batt_devname, set,
					POWER_SUPPLY_PROP_HEALTH, value);
		break;
	case RT_TEMP_COOL:
	case RT_TEMP_WARM:
		/* Enable Charging */
		psy_do_property(rt5058_chg_devname, get,
					POWER_SUPPLY_PROP_STATUS, value);
		if (value.intval == POWER_SUPPLY_STATUS_CHARGING) {
			rt5058_enable_charging(ri, 1);
			rt5058_set_bits(ri->client, RT5058_REG_CHGCTRL2,
							RT5058_ICCJEITA_MASK);
		}
		/* set good to battery health */
		value.intval = POWER_SUPPLY_HEALTH_GOOD;
		psy_do_property(rt5058_batt_devname, set,
					POWER_SUPPLY_PROP_HEALTH, value);
		break;
	case RT_TEMP_NORMAL:
		/* Enable Charging */
		psy_do_property(rt5058_chg_devname, get,
					POWER_SUPPLY_PROP_STATUS, value);
		if (value.intval == POWER_SUPPLY_STATUS_CHARGING) {
			rt5058_enable_charging(ri, 1);
			rt5058_clr_bits(ri->client, RT5058_REG_CHGCTRL2,
							RT5058_ICCJEITA_MASK);
		}
		/* set good to battery health */
		value.intval = POWER_SUPPLY_HEALTH_GOOD;
		psy_do_property(rt5058_batt_devname, set,
					POWER_SUPPLY_PROP_HEALTH, value);
		break;
	default:
		dev_err(ri->dev, "invalid temperature status\n");
		break;
	}
}

static void rt5058_handle_charge_recharge(struct rt5058_charger_info *ri);
static int rt5058_chg_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct rt5058_charger_info *ri = dev_get_drvdata(psy->dev->parent);
	int ret = 0, pre_temp_status;
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
#if !defined(CONFIG_MACH_MSM8909_M1_TMO_US) && !defined(CONFIG_MACH_MSM8909_M1_MPCS_US)
	union lge_power_propval curr_ma;
#endif
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ri->battery_status = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ri->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		/* AICR */
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
		if (val->intval) {
			ri->usb_curr_ma_on = 1;
			ri->usb_curr_ma = val->intval;
		}
		else {
#if defined(CONFIG_MACH_MSM8909_M1_TMO_US) || defined(CONFIG_MACH_MSM8909_M1_MPCS_US)
			ri->usb_curr_ma_on = 0;
			ri->usb_curr_ma = 500;
#else
			ri->lge_cd_lpc->get_property(ri->lge_cd_lpc,
				LGE_POWER_PROP_CURRENT_MAX, &curr_ma);
			ri->usb_curr_ma_on = 0;
			ri->usb_curr_ma = curr_ma.intval / 1000;
#endif
		}
		ret = rt5058_set_input_current_regulation(ri, ri->usb_curr_ma);
#else
		ret = rt5058_set_input_current_regulation(ri, val->intval);
#endif
		if (ret < 0)
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* ICC */
		ret = rt5058_set_charging_current(ri, val->intval);
		if (ret < 0)
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rt5058_set_cv(ri, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
		rt5058_charger_otg_control(ri, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE: /* force recharge */
		rt5058_handle_charge_recharge(ri);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		/* hidden menu support, set enable/disable charging forced
		 * by using testmode flag */
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
		ri->testmode = 1;
#endif
		rt5058_enable_charging(ri, val->intval);
		break;
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	case POWER_SUPPLY_PROP_SAFETY_TIMER:
		rt5058_set_safety_timer(ri, val->intval);
		break;
#endif
	case POWER_SUPPLY_PROP_TEMP:
		if (ri->cdata->use_jeita) {
			pre_temp_status = ri->temp_status;
			ri->temp_status = rt5058_set_temp_status(ri, val->intval);
			if (ri->temp_status != pre_temp_status)
				rt5058_set_chg_jeita(ri);
		}
		break;
	default:
		break;
	}
	return ret;
}

static int rt5058_charger_init(struct rt5058_charger_info *ri)
{
	int ret = 0;
	u32 regval = 0;

	rt5058_enable_charging(ri, 0);
	/* set OCP level of buck mode */
	if (ri->cdata->hocp)
		rt5058_set_bits(ri->client, RT5058_REG_CHGCTRL1,
							RT5058_CHGHOCP_MASK);
	else
		rt5058_clr_bits(ri->client, RT5058_REG_CHGCTRL1,
							RT5058_CHGHOCP_MASK);

	/* enable battery detection */
	if (ri->cdata->use_batdet) {
		ret = rt5058_set_bits(ri->client,
			RT5058_REG_CHGCTRL2, RT5058_BATDET_MASK);
		if (ret < 0)
			goto err_init;
	}

	/* set TE enable */
	rt5058_te_switch(ri, true);

	/* set ICC */
	rt5058_set_charging_current(ri, ri->cdata->icc);

	/* set Charge Voltage */
	rt5058_set_cv(ri, ri->cdata->chg_volt);

	/* set IEOC */
	regval = ri->cdata->ieoc ? (ri->cdata->ieoc-50)/25+1 : 0;
	ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL9,
				RT5058_IEOC_MASK, regval << RT5058_IEOC_SHIFT);
	if (ret < 0)
		goto err_init;

	/* set MIVR */
	if (ri->cdata->use_mivr) {
		regval = ri->cdata->mivr ? (ri->cdata->mivr-3800)/100 + 1 : 0;
		ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL5,
				RT5058_MIVR_MASK, regval << RT5058_MIVR_SHIFT);
		if (ret < 0)
			goto err_init;
	} else {
		ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL5,
							RT5058_MIVR_MASK, 0);
		if (ret < 0)
			goto err_init;
	}

	/* set Pre-charger threshold */
	regval = (ri->cdata->prechg_volt - 2300)/100;
	ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL5,
			RT5058_VPREC_MASK, regval << RT5058_VPREC_SHIFT);
	if (ret < 0)
		goto err_init;

	/*set Pre-charger current */
	regval = (ri->cdata->prechg_curr - 150)/100;
	ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL6,
			RT5058_IPREC_MASK, regval << RT5058_IPREC_SHIFT);
	if (ret < 0)
		goto err_init;

	/* set Low Battery protection voltage */
	if (ri->cdata->use_lbp) {
		regval = (ri->cdata->lbp_volt - 2300)/100;
		ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL11,
			RT5058_CHGLBP_MASK, regval << RT5058_CHGLBP_SHIFT);
		if (ret < 0)
			goto err_init;
	}

	/*set background charging time */
	ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL7,
		RT5058_EOCTIMER_MASK,
		ri->cdata->backchg_time << RT5058_EOCTIMER_SHIFT);
	if (ret < 0)
		goto err_init;

	/* set input current selection type */
	ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL10,
		RT5058_IINLMTSEL_MASK,
		ri->cdata->iinlmt_sel << RT5058_IINLMTSEL_SHIFT);
	if (ret < 0)
		goto err_init;

	/* set switching frequency */
	if (ri->cdata->sel_swfreq)
		ret = rt5058_set_bits(ri->client,
				RT5058_REG_CHGCTRL1, RT5058_SWFREQ_MASK);
	else
		ret = rt5058_clr_bits(ri->client,
				RT5058_REG_CHGCTRL1, RT5058_SWFREQ_MASK);
	if (ret < 0)
		goto err_init;

	if (ri->cdata->fixfreq)
		ret = rt5058_set_bits(ri->client,
				RT5058_REG_CHGCTRL1, RT5058_FIXFREQ_MASK);
	else
		ret = rt5058_clr_bits(ri->client,
				RT5058_REG_CHGCTRL1, RT5058_FIXFREQ_MASK);
	if (ret < 0)
		goto err_init;

#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	/* set charge timer */
	rt5058_set_safety_timer(ri, 1);
#endif

	/* set System minimum regulatrion voltage */
	ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL10,
			RT5058_SYSMIN_MASK,
			ri->cdata->sys_minvolt << RT5058_SYSMIN_SHIFT);
	if (ret < 0)
		goto err_init;

	/* set IIN detection comparator threshold */
	regval = (ri->cdata->chg_iin_vth - 4100)/100;
	ret = rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL13,
			RT5058_CHGIINVTH_MASK,
			regval << RT5058_CHGIINVTH_SHIFT);
	if (ret < 0)
		goto err_init;

	/* Hidden pass register settings */
	rt5058_reg_write(ri->client, RT5058_REG_HIDDEN_PAS_CODE1, 0x96);
	rt5058_reg_write(ri->client, RT5058_REG_HIDDEN_PAS_CODE2, 0x69);
	rt5058_reg_write(ri->client, RT5058_REG_HIDDEN_PAS_CODE3, 0xc3);
	rt5058_reg_write(ri->client, RT5058_REG_HIDDEN_PAS_CODE4, 0x3c);
	rt5058_reg_write(ri->client, RT5058_REG_CHGHIDDENCTRL3, 0xb0);
	rt5058_reg_write(ri->client, RT5058_REG_CHGHIDDENCTRL7, 0x02);
	rt5058_reg_write(ri->client, RT5058_REG_CHGCTRL14, 0x3f);
	rt5058_lock_chg_pascode(ri);

	/* set CHG_TDEG_EOC */
	rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL9, 0x07, 5);

	return 0;
err_init:
	dev_err(ri->dev, "charger register val init fail\n");
	return ret;
}

static int rt_parse_dt(struct device *dev,
			struct rt5058_charger_platform_data *pdata)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	u32 val, regval[5];
	int len;
	const char *charger_name = rt5058_chg_devname;

	of_property_read_string(np, "richtek,charger-name", &charger_name);
	len = strlen(charger_name);
	pdata->charger_name = devm_kzalloc(dev, len+1, GFP_KERNEL);
	if (pdata->charger_name == NULL) {
		pr_err("%s: devm_kzalloc fail.", __func__);
		return 0;
	}

	strcpy(pdata->charger_name, charger_name);

	pdata->use_mivr = of_property_read_bool(np, "rt,use_mivr") ? 1 : 0;
	pdata->use_te = of_property_read_bool(np, "rt,use_te") ? 1 : 0;
	pdata->use_batdet = of_property_read_bool(np, "rt,use_batdet") ? 1 : 0;
	pdata->use_jeita = of_property_read_bool(np, "rt,use_jeita") ? 1 : 0;
	pdata->use_lbp = of_property_read_bool(np, "rt,use_lbp") ? 1 : 0;
	pdata->use_iinmeas =
			of_property_read_bool(np, "rt,use_iinmeas") ? 1 : 0;
	pdata->use_aicr = of_property_read_bool(np, "rt,use_aicr") ? 1 : 0;

	if (of_property_read_u32(np, "rt,aicr_max", &val) >= 0) {
		if (val >= RT5058_MAX_AICR)
			pdata->aicr_ma = RT5058_MAX_AICR;
		else
			pdata->aicr_ma = val;
		} else {
			dev_info(dev, "use default aicr_max 900mA\n");
			pdata->aicr_ma = 900;
	}

	if (of_property_read_u32(np, "rt,chg_iin_vth", &val) >= 0) {
		if (val >= RT5058_MAX_CHGIIN_VTH)
			pdata->chg_iin_vth = RT5058_MAX_CHGIIN_VTH;
		else
			pdata->chg_iin_vth = val;
	} else {
		dev_info(dev, "use default chg_iin_vth 4100mV\n");
		pdata->chg_iin_vth = 4100;
	}


	if (of_property_read_u32(np, "rt,chg_volt", &val) >= 0) {
		if (val >= RT5058_MAX_CHGCV)
			pdata->chg_volt = RT5058_MAX_CHGCV;
		else
			pdata->chg_volt = val;
	} else {
		dev_info(dev, "use default chgcv 4.2V\n");
		pdata->chg_volt = 4200;
	}

	if (of_property_read_u32(np, "rt,icc", &val) >= 0) {
		if (val >= RT5058_MAX_ICC)
			pdata->icc = RT5058_MAX_ICC;
		else
			pdata->icc = val;
	} else {
		dev_info(dev, "use default icc 1A\n");
		pdata->icc = 1000;
	}

	if (of_property_read_u32(np, "rt,bst_volt", &val) >= 0) {
		if (val >= RT5058_MAX_BSTVOLT)
			pdata->bst_volt = RT5058_MAX_BSTVOLT;
		else
			pdata->bst_volt = val;
	} else {
		dev_info(dev, "use default boost volt 4.175A\n");
		pdata->bst_volt = 4175;
	}

	if (of_property_read_u32(np, "rt,ieoc", &val) >= 0) {
		if (val >= RT5058_MAX_IEOC)
			pdata->ieoc = RT5058_MAX_IEOC;
		else
			pdata->ieoc = val;
	} else {
		dev_info(dev, "use default EOC current 250mA\n");
		pdata->ieoc = 250;
	}

	if (of_property_read_u32(np, "rt,mivr", &val) >= 0) {
		if (val >= RT5058_MAX_MIVR)
			pdata->mivr = RT5058_MAX_MIVR;
		else
			pdata->mivr = val;
	} else {
		dev_info(dev, "use default MIVR 4.6V\n");
		pdata->mivr = 4600;
	}

	if (of_property_read_u32(np, "rt,prechg_volt", &val) >= 0) {
		if (val >= RT5058_MAX_PRECHGVOLT)
			pdata->prechg_volt = RT5058_MAX_PRECHGVOLT;
		else
			pdata->prechg_volt = val;
	} else {
		dev_info(dev, "use default pre-chage voltage threshold 2.8V\n");
		pdata->prechg_volt = 2800;
	}

	if (of_property_read_u32(np, "rt,prechg_curr", &val) >= 0) {
		if (val >= RT5058_MAX_PRECHGCURR)
			pdata->prechg_curr = RT5058_MAX_PRECHGCURR;
		else
			pdata->prechg_curr = val;
	} else {
		dev_info(dev, "use default pre-chage current 150mA\n");
		pdata->prechg_curr = 150;
	}

	if (of_property_read_u32(np, "rt,lbp_volt", &val) >= 0) {
		if (val >= RT5058_MAX_LBPVOLT)
			pdata->lbp_volt = RT5058_MAX_LBPVOLT;
		else
			pdata->lbp_volt = val;
	} else {
		dev_info(dev, "use default low battery protection voltage 2.8V\n");
		pdata->lbp_volt = 2800;
	}

	if (of_property_read_u32(np, "rt,backchg_time", &val) >= 0) {
		if (val >= RT5058_BACKCHG_60MIN)
			pdata->backchg_time = RT5058_BACKCHG_60MIN;
		else
			pdata->backchg_time = val;
	} else {
		dev_info(dev, "use default background charging time 0 min\n");
		pdata->backchg_time = RT5058_BACKCHG_0MIN;
	}

	if (of_property_read_u32(np, "rt,iinlmt_sel", &val) >= 0) {
		if (val >= RT5058_IINLMT_MIN)
			pdata->iinlmt_sel = RT5058_IINLMT_MIN;
		else
			pdata->iinlmt_sel = val;
	} else {
		dev_info(dev,
			"use default input current selection by AICR type\n");
		pdata->iinlmt_sel = RT5058_IINLMT_AICR;
	}

	if (of_property_read_u32(np, "rt,sel_swfreq", &val) >= 0) {
		if (val >= RT5058_SWFREQ_0_75MHZ)
			pdata->sel_swfreq = RT5058_SWFREQ_0_75MHZ;
		else
			pdata->sel_swfreq = val;
	} else {
		dev_info(dev,
			"use default switching frequency 1.5MHz type\n");
		pdata->iinlmt_sel = RT5058_SWFREQ_1_5MHZ;
	}

	if (of_property_read_u32(np, "rt,fixfreq", &val) >= 0) {
		if (val >= RT5058_FIXFREQ_FIXED)
			pdata->fixfreq = RT5058_FIXFREQ_FIXED;
		else
			pdata->fixfreq = val;
	} else {
		dev_info(dev,
			"use default varied switching frequency\n");
		pdata->fixfreq = RT5058_FIXFREQ_VARIED;
	}

	if (of_property_read_u32(np, "rt,hocp", &val) >= 0) {
		if (val >= RT5058_HOCP_3_5A)
			pdata->hocp = RT5058_HOCP_3_5A;
		else
			pdata->hocp = val;
	} else {
		dev_info(dev, "use default hocp 2.5A\n");
		pdata->hocp = RT5058_HOCP_2_5A;
	}

	if (of_property_read_u32(np, "rt,fc_timer", &val) >= 0) {
		if (val >= RT5058_FCTIMER_DISABLE)
			pdata->fc_timer = RT5058_FCTIMER_DISABLE;
		else
			pdata->fc_timer = val;
	} else {
		dev_info(dev, "use default fast charge timer 4hrs\n");
		pdata->fc_timer = RT5058_FCTIMER_4HRS;
	}

	if (of_property_read_u32(np, "rt,prc_timer", &val) >= 0) {
		if (val >= RT5058_PRCTIMER_DISABLE)
			pdata->prc_timer = RT5058_PRCTIMER_DISABLE;
		else
			pdata->prc_timer = val;
	} else {
		dev_info(dev, "use default pre-charge timer 30mins\n");
		pdata->prc_timer = RT5058_PRCTIMER_30MIN;
	}

	if (of_property_read_u32(np, "rt,sys_minvolt", &val) >= 0) {
		if (val >= RT5058_SYSMIN_3_8V)
			pdata->sys_minvolt = RT5058_SYSMIN_3_8V;
		else
			pdata->sys_minvolt = val;
	} else {
		dev_info(dev,
			"use default system minimum regulation voltage 3.6V\n");
		pdata->sys_minvolt = RT5058_SYSMIN_3_6V;
	}

	if (of_property_read_u32_array(np, "rt,chg_irq_mask", regval, 5) >= 0) {
		pdata->irq_mask[0] = regval[0];
		pdata->irq_mask[1] = regval[1];
		pdata->irq_mask[2] = regval[2];
		pdata->irq_mask[3] = regval[3];
		pdata->irq_mask[4] = regval[4];
	} else {
		dev_info(dev, "use default irq_mask\n");
		pdata->irq_mask[0] = 0xf0;
		pdata->irq_mask[1] = 0x08;
		pdata->irq_mask[2] = 0xf0;
		pdata->irq_mask[3] = 0x1f;
		pdata->irq_mask[4] = 0xff;
	}

	if (of_property_read_u32_array(np, "rt,temp_level",
					(u32 *)pdata->temp_level, 4) < 0) {
		dev_info(dev, "no temp level config proeprty, use default\n");
		pdata->temp_level[0] = -40;
		pdata->temp_level[1] = 10;
		pdata->temp_level[2] = 25;
		pdata->temp_level[3] = 70;
	}

#ifdef CONFIG_LGE_PM_CABLE_DETECTION
	get_cable_data_from_dt(np);
#endif
#endif /* CONFIG_OF */
	return 0;
}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
static void rt5058_notice_current_settled(struct work_struct *work)
{
	union power_supply_propval cable_type;
	struct rt5058_charger_info *ri =
		container_of(work, struct rt5058_charger_info, current_settled_work.work);

	RTINFO("run\n");
	ri->input_current_trim = ri->input_curr_ma;

	psy_do_property(RT5058_USB_DEVNAME, get,
			POWER_SUPPLY_PROP_TYPE, cable_type);

	if (cable_type.intval != POWER_SUPPLY_TYPE_USB
		&& cable_type.intval != POWER_SUPPLY_TYPE_UNKNOWN) {
		ri->current_settled = 1;
		power_supply_changed(&ri->psy);
	}
}
#endif

static void rt5058_chg_usbinsert(struct work_struct *work)
{
	struct rt5058_charger_info *ri =
		container_of(work, struct rt5058_charger_info,
					usbinsert_work.work);
	union power_supply_propval value;

	/* report present property to usb driver */
	value.intval = 1;
	psy_do_property(rt5058_usb_devname, set,
				POWER_SUPPLY_PROP_PRESENT, value);
	if (value.intval == RT_DOPSY_FAIL)
		goto do_psy_err;

	ri->usbin_running = 0;

	return;
do_psy_err:
	schedule_delayed_work(&ri->usbinsert_work,
			msecs_to_jiffies(RT5058_USBREPORT_TIMER));
}

static void rt5058_handle_charge_recharge(struct rt5058_charger_info *ri)
{
	u8 pre_full_charged = 0;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	union lge_power_propval lge_value = {0,};
#endif
	mutex_lock(&ri->full_charged_lock);
	pre_full_charged = ri->full_charged;
	ri->full_charged = 0;
	dev_info(ri->dev, "IRQ: Recharging request\n");
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	if (!ri->lge_cc_lpc) {
		pr_err("%s: lpc_cc is not yet ready\n", __func__);
		return;
	}
	else {
		ri->lge_cc_lpc->get_property(ri->lge_cc_lpc,
			LGE_POWER_PROP_CHARGING_ENABLED, &lge_value);
		if (lge_value.intval)
			rt5058_enable_charging(ri, 1);
	}
#else
	rt5058_enable_charging(ri, 1);
#endif
	if (!wake_lock_active(&ri->safety_wake_lock)) {
		wake_lock_timeout(&ri->safety_wake_lock,
				msecs_to_jiffies(RT5058_BTM_WAKE_LOCK_TIME));
	}
	if (pre_full_charged != ri->full_charged)
		power_supply_changed(&ri->psy);

	mutex_unlock(&ri->full_charged_lock);
}

static void rt5058_charger_iinmeas(struct rt5058_charger_info *ri)
{
	int ret;

	/* Enable IIN MEAS fucction */
	ri->iinms_done = 1;
	ret = rt5058_set_bits(ri->client, RT5058_REG_CHGCTRL13,
						RT5058_CHG_IINMEAS_MASK);
	pr_info("%s: RT5058 IIN MEAS is Running", __func__);
}

static void rt5058_handle_charge_eoc(struct rt5058_charger_info *ri)
{
	u8 pre_full_charged = 0;
	mutex_lock(&ri->full_charged_lock);

	pre_full_charged = ri->full_charged;
	ri->full_charged = 1;
	dev_info(ri->dev, "IRQ: CHG EOC occured\n");

	if (!wake_lock_active(&ri->safety_wake_lock)) {
		wake_lock_timeout(&ri->safety_wake_lock,
				msecs_to_jiffies(RT5058_BTM_WAKE_LOCK_TIME));
	}
	if (pre_full_charged != ri->full_charged)
		power_supply_changed(&ri->psy);

	mutex_unlock(&ri->full_charged_lock);
}

static void rt5058_handle_charge_iinmeas(struct rt5058_charger_info *ri)
{
	dev_info(ri->dev, "IRQ: %s, IIN MEAS Done\n", __func__);

	ri->input_curr_ma = rt5058_get_input_current_limit(ri);
	pr_err("%s:AICL Result = %dmA\n", __func__, ri->input_curr_ma);

	if (ri->input_curr_ma == 50) {
		rt5058_assign_bits(ri->client, RT5058_REG_CHGCTRL4,
				RT5058_CHGAICR_MASK, 0x08);
		rt5058_unlock_chg_pascode(ri);
		rt5058_set_bits(ri->client, 0x26, 0x02);
		msleep(30);
		rt5058_clr_bits(ri->client, 0x26, 0x02);
		rt5058_lock_chg_pascode(ri);
	}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	/* for USB psy delay */
	schedule_delayed_work(&ri->current_settled_work,
			msecs_to_jiffies(RT5058_USBREPORT_TIMER));
#endif

}

static void rt5058_handle_charge_mivr(struct rt5058_charger_info *ri)
{
	u8 ret, regval;

	ret = rt5058_reg_read(ri->client, RT5058_REG_CHG_STAT1);
	if (ret < 0)
		return;

	if (ret & RT5058_CHGMIVR_MASK) {
		dev_info(ri->dev, "IRQ: MIVR occur\n");
		regval = rt5058_reg_read(ri->client, RT5058_REG_CHGCTRL4);

		if (regval == 0x08)
			return;
		rt5058_charger_iinmeas(ri);
	}
}

static void rt5058_update_battery_status(struct rt5058_charger_info *ri)
{
	int status = POWER_SUPPLY_STATUS_DISCHARGING;
	int pre_status = ri->battery_status;
	struct power_supply *bat_psy;
	union power_supply_propval value;
	int ret, chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	union lge_power_propval lge_value = {0,};
#endif

	bat_psy = power_supply_get_by_name(rt5058_fuel_devname);
	if (!bat_psy) {
		pr_err("%s: fuel_psy is not yet ready\n", __func__);
		/* prevent before created batt psy */
		if (ri->online)
			status = POWER_SUPPLY_STATUS_CHARGING;
		goto update_status;
	}

	psy_do_property(rt5058_fuel_devname, get,
			POWER_SUPPLY_PROP_CAPACITY, value);

	rt5058_get_batt_present(ri);
	if (value.intval >= 100 && ri->batt_present) {
		status = POWER_SUPPLY_STATUS_FULL;
		goto update_status;
	}
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	if (!ri->lge_cc_lpc) {
		pr_err("%s: lpc_cc is not yet ready\n", __func__);
		goto update_status;
	}
	ret = ri->lge_cc_lpc->get_property(ri->lge_cc_lpc,
			LGE_POWER_PROP_PSEUDO_BATT_UI, &lge_value);
	if (ri->online && lge_value.intval) {
		status = POWER_SUPPLY_STATUS_CHARGING;
		goto update_status;
	}
#else
	if (ri->online) {
		status = POWER_SUPPLY_STATUS_CHARGING;
		goto update_status;
	}
#endif
	chg_type = rt5058_get_charge_type(ri);

	if ((chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
		chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST) && ri->chg_enabled) {
		status = POWER_SUPPLY_STATUS_CHARGING;
		goto update_status;
	}

	if (ri->online)
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	/* TEMP_TEST : when OTG is enabled, handle OTG func. */
	if (ri->otg_en) {
		/* For OTG mode, RT5058 would still report "charging" */
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		ret = rt5058_reg_read(ri->client, RT5058_REG_CHG_STAT4);
		if (ret & RT5058_BSTOVP_STAT_MASK) {
			pr_info("%s: otg overcurrent limit\n", __func__);
			rt5058_charger_otg_control(ri, false);
		}
	}
update_status:
	ri->battery_status = status;

	if (ri->battery_status != pre_status) {
		pr_err("%s: battery status changed (%d)->(%d)\n",
				__func__, pre_status, ri->battery_status);
		power_supply_changed(&ri->psy);
	}
}

static void rt5058_handle_plug_event(struct rt5058_charger_info *ri)
{
	int ret;
	union power_supply_propval value;

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	ri->testmode = 0;
#endif

	ret = rt5058_reg_read(ri->client, RT5058_REG_MUIC_STAT2);
	if (ret < 0) {
		dev_err(ri->dev, "fail to get attach info\n");
		return;
	}

	ri->attach_status_now = (ret & RT5058_MUICUL_MASK) ? 0:1;
	if (ri->attach_status_now != ri->attach_status_old) {
		if (ri->attach_status_now) {
			pr_info("%s: TA Plug in\n", __func__);
			if (!ri->full_charged)
				rt5058_enable_charging(ri, 1);
			ri->usbin_running = 1;
			ri->online = 1;
			schedule_delayed_work(&ri->usbinsert_work, msecs_to_jiffies(1));
		}
		else {
			pr_info("%s: TA Plug out\n", __func__);
			rt5058_enable_charging(ri, 0);
			if (ri->usbin_running) {
				cancel_delayed_work(&ri->usbinsert_work);
				ri->usbin_running = 0;
			}
			value.intval = 0;
			ri->online = 0;
			ri->full_charged = 0;
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
			ri->time_expired = 0;
#endif
			psy_do_property(rt5058_usb_devname, set,
					POWER_SUPPLY_PROP_PRESENT, value);
			ri->iinms_done = 0;
			ri->input_curr_ma = 0;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
			ri->current_settled = 0;
			ri->input_current_trim = 0;
#endif
		}
		ri->attach_status_old = ri->attach_status_now;
	}

	if (!wake_lock_active(&ri->status_wake_lock)) {
		wake_lock_timeout(&ri->status_wake_lock,
				msecs_to_jiffies(RT5058_STATUS_WAKE_LOCK_TIME));
	}
}

static void rt5058_handle_charge_termination(struct rt5058_charger_info *ri)
{
	dev_info(ri->dev, "IRQ: CHG charge terminated\n");
	rt5058_enable_charging(ri, 0);
}

static void rt5058_handle_boost_ovp(struct rt5058_charger_info *ri)
{
	int ret;
#ifdef CONFIG_LGE_PM_BATTERY_RT5058
	union power_supply_propval val;
#endif /* CONFIG_LGE_PM_BATTERY_RT5058 */

	ret = rt5058_reg_read(ri->client, RT5058_REG_CHG_STAT4);
	if (ret < 0)
		return;
	if (ret & RT5058_BSTOVP_STAT_MASK) {
		pr_info("%s: IRQ Boost OVP\n", __func__);
		#ifdef CONFIG_LGE_PM_BATTERY_RT5058
		val.intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		psy_do_property(rt5058_chg_devname, set,
				POWER_SUPPLY_PROP_STATUS, val);
		#endif /* CONFIG_LGE_PM_BATTERY_RT5058 */
	}
}

static void rt5058_get_batt_present(struct rt5058_charger_info *ri)
{
	struct power_supply *bat_psy;
	union power_supply_propval value;
	int pre_batt_present = ri->batt_present;

	bat_psy = power_supply_get_by_name(rt5058_batt_devname);
	if (!bat_psy) {
		pr_err("%s: bat_psy is not yet ready\n", __func__);
		ri->batt_present = 1;
		return;
	}

	psy_do_property(rt5058_batt_devname, get,
		POWER_SUPPLY_PROP_TEMP_RAW, value);

	if (value.intval <= -300 || value.intval >= 790) {
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
		if (ri->is_factory_cable) {
			pr_err("%s: battery missing(%d) with factory cable.\n",
					__func__, value.intval);
			ri->batt_present = 1;
		}
		else {
			pr_err("%s: battery missing(%d).\n", __func__, value.intval);
			ri->batt_present = 0;
		}
#else
		pr_err("%s: battery missing(%d).\n", __func__, value.intval);
		ri->batt_present = 0;
#endif
	}
	else {
		ri->batt_present = 1;
	}

	if (pre_batt_present != ri->batt_present) {
		pr_debug("%s: batt present changed (%d)->(%d)\n",
				__func__, pre_batt_present, ri->batt_present);
		power_supply_changed(&ri->psy);
	}
}

static void rt5058_handle_charge_pwrrdy(struct rt5058_charger_info *ri)
{
	int ret;

	ret = rt5058_reg_read(ri->client, RT5058_REG_CHG_STAT1);
	if (ret < 0)
		return;
	if (ret & RT5058_CHGPWRRDY_STAT_MASK)
		pr_info("%s: IRQ Power Ready\n", __func__);
}

static void rt5058_handle_batabs(struct rt5058_charger_info *ri)
{
	int ret;

	ret = rt5058_reg_read(ri->client, RT5058_REG_CHG_STAT2);
	if (ret < 0)
		return;
	if (ret & RT5058_BATABS_STAT_MASK) {
		pr_info("%s: IRQ Battery is absence\n", __func__);
		rt5058_get_batt_present(ri);
		if (!ri->batt_present) {
			pr_err("%s: set BAT_RMV flag\n", __func__ );
			rt5058_assign_bits16(ri->client, RT5058_REG_FGOPCFG1,
				RT5058_FG_BAT_RMV_MASK, RT5058_FG_BAT_RMV_MASK);
		}
	}
}

#if RT5058_CHG_USE_NESTED_IRQ
static irqreturn_t rt5058_chg_iin_meas(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;
	int ret;

	rt5058_handle_charge_iinmeas(ri);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_icc_meas(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_mivr(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;
	u8 ret;

	rt5058_handle_charge_mivr(ri);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_power_ready(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;
	int ret;

	rt5058_handle_charge_pwrrdy(ri);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_bat_abs(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;
	int ret;

	rt5058_handle_batabs(ri);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_sys_uv(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_time_out(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_batov(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_bad_adp(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_reverse_protect(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_thermal_shutdown(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_thermal_regulation(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: Thermal regulation loop active\n");
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_recharge(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: Recharging request\n");
	rt5058_handle_charge_recharge(ri);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_terminated_to(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	rt5058_handle_charge_termination(ri);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_EOC(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	rt5058_handle_charge_eoc(ri);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_boost_lv(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: Protection event of Boost low voltage\n");
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_boost_ol(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	dev_info(ri->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_chg_boost_ovp(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	rt5058_handle_boost_ovp(ri);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_muic_attach(int irq, void *data)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)data;

	rt5058_handle_plug_event(ri);
	return IRQ_HANDLED;
}

static const struct rt5058_irq_handler rt5058_charger_irq_handler[] = {
	{ .irq_name = "MUIC_ATT", .irq_handler = rt5058_muic_attach},
	{ .irq_name = "CHG_IIN_MEAS", .irq_handler = rt5058_chg_iin_meas},
	{ .irq_name = "CHG_ICC_MEAS", .irq_handler = rt5058_chg_icc_meas},
	{ .irq_name = "CHG_MIVR", .irq_handler = rt5058_chg_mivr},
	{ .irq_name = "CHG_PWR_RDY", .irq_handler = rt5058_chg_power_ready},
	{ .irq_name = "CHG_BATABS", .irq_handler = rt5058_chg_bat_abs},
	{ .irq_name = "CHG_SYSUV", .irq_handler = rt5058_chg_sys_uv},
	{ .irq_name = "CHG_TMR", .irq_handler = rt5058_chg_time_out},
	{ .irq_name = "CHG_BATOV", .irq_handler = rt5058_chg_batov},
	{ .irq_name = "CHG_BADADP", .irq_handler = rt5058_chg_bad_adp},
	{ .irq_name = "CHG_RVP", .irq_handler = rt5058_chg_reverse_protect},
	{ .irq_name = "CHG_TSSHD", .irq_handler = rt5058_chg_thermal_shutdown},
	{ .irq_name = "CHG_TREG", .irq_handler = rt5058_chg_thermal_regulation},
	{ .irq_name = "CHG_RCHG", .irq_handler = rt5058_chg_recharge},
	{ .irq_name = "CHG_TERMTMR", .irq_handler = rt5058_chg_terminated_to},
	{ .irq_name = "CHG_IEOC", .irq_handler = rt5058_chg_EOC},
	{ .irq_name = "CHG_BSTLV", .irq_handler = rt5058_chg_boost_lv},
	{ .irq_name = "CHG_BSTOL", .irq_handler = rt5058_chg_boost_ol},
	{ .irq_name = "CHG_BSTOVP", .irq_handler = rt5058_chg_boost_ovp},
};

static int rt5058_charger_irqinit(struct platform_device *pdev)
{
	struct rt5058_charger_info *fi = platform_get_drvdata(pdev);
	int i = 0, ret = 0;
	const char *irq_name = NULL;

	pr_info("%s\n", __func__);

	rt5058_clr_bits(fi->client, RT5058_REG_IRQMSK, RT5058_CHG_IRQMASK);
	rt5058_clr_bits(fi->client, RT5058_REG_IRQMSK, RT5058_MUIC_IRQMASK);
	for (i = 0; i < ARRAY_SIZE(rt5058_charger_irq_handler); i++) {
		irq_name = rt5058_charger_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
			rt5058_charger_irq_handler[i].irq_handler,
			IRQF_TRIGGER_NONE, irq_name, fi);
		if (ret < 0) {
			dev_err(fi->dev, "request %s fail\n", irq_name);
			goto out_irq_init;
		}
	}
	return 0;
out_irq_init:
	while (--i >= 0) {
		irq_name = rt5058_charger_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		devm_free_irq(&pdev->dev, ret, fi);
	}
	return -EINVAL;
}

static void rt5058_charger_irq_deinit(struct platform_device *pdev)
{
	struct rt5058_charger_info *ri = platform_get_drvdata(pdev);
	int i = 0, ret = 0;
	const char *irq_name = NULL;

	pr_info("%s\n", __func__);
	rt5058_set_bits(ri->client, RT5058_REG_IRQMSK, RT5058_CHG_IRQMASK);
	rt5058_set_bits(ri->client, RT5058_REG_IRQMSK, RT5058_MUIC_IRQMASK);
	for (i = 0; i < ARRAY_SIZE(rt5058_charger_irq_handler); i++) {
		irq_name = rt5058_charger_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		devm_free_irq(&pdev->dev, ret, ri);
	}
}
#else
static void rt5058_chgirq_handler(void *info, int eventno)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)info;

	switch (eventno) {
	case CHGEVENT_IINMEAS:
		rt5058_handle_charge_iinmeas(ri);
		break;
	case CHGEVENT_ICCMEAS:
		dev_info(ri->dev, "IRQ: CHG ICC MEAS Done\n");
		break;
	case CHGEVENT_MIVR:
		rt5058_handle_charge_mivr(ri);
		break;
	case CHGEVENT_PWRRDY:
		rt5058_handle_charge_pwrrdy(ri);
		break;
	case CHGEVENT_BATABS:
		rt5058_handle_batabs(ri);
		break;
	case CHGEVENT_SYSUV:
		dev_info(ri->dev, "IRQ: CHG system uv\n");
		break;
	case CHGEVENT_TMR:
		dev_info(ri->dev, "IRQ: CHG Time-out\n");
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
		if (ri->online) {
			ri->time_expired = 1;
			rt5058_enable_charging(ri, 0);
		}
#endif
		break;
	case CHGEVENT_BATOV:
		dev_info(ri->dev, "IRQ: CHG Battery OV\n");
		break;
	case CHGEVENT_BADADP:
		dev_info(ri->dev, "IRQ: CHG Bad Adapter\n");
		break;
	case CHGEVENT_RVP:
		dev_info(ri->dev, "IRQ: CHG reverse protection\n");
		break;
	case CHGEVENT_TSSHD:
		dev_info(ri->dev, "IRQ: CHG Thermal shutdown\n");
		break;
	case CHGEVENT_TREG:
		dev_info(ri->dev, "IRQ: CHG Thermal regulation loop\n");
		break;
	case CHGEVENT_RCHG:
		dev_info(ri->dev, "IRQ: Recharge reqeust\n");
		break;
	case CHGEVENT_TERMTMR:
		rt5058_handle_charge_termination(ri);
		break;
	case CHGEVENT_IEOC:
		rt5058_handle_charge_eoc(ri);
		break;
	case CHGEVENT_BSTLV:
		dev_info(ri->dev, "IRQ: CHG Boost LV\n");
		break;
	case CHGEVENT_BSTOL:
		dev_info(ri->dev, "IRQ: CHG Boost OL\n");
		break;
	case CHGEVENT_BSTOVP:
		rt5058_handle_boost_ovp(ri);
		break;
	case MUIC_UL_EVENT:
		rt5058_handle_plug_event(ri);
		break;
	default:
		break;
	}
}

static rt_irq_handler rt_chgirq_handler[CHGEVENT_MAX] = {
	[CHGEVENT_IINMEAS] = rt5058_chgirq_handler,
	[CHGEVENT_ICCMEAS] = rt5058_chgirq_handler,
	[CHGEVENT_MIVR] = rt5058_chgirq_handler,
	[CHGEVENT_PWRRDY] = rt5058_chgirq_handler,
	[CHGEVENT_BATABS] = rt5058_chgirq_handler,
	[CHGEVENT_SYSUV] = rt5058_chgirq_handler,
	[CHGEVENT_TMR] = rt5058_chgirq_handler,
	[CHGEVENT_BATOV] = rt5058_chgirq_handler,
	[CHGEVENT_BADADP] = rt5058_chgirq_handler,
	[CHGEVENT_RVP] = rt5058_chgirq_handler,
	[CHGEVENT_TSSHD] = rt5058_chgirq_handler,
	[CHGEVENT_TREG] = rt5058_chgirq_handler,
	[CHGEVENT_RCHG] = rt5058_chgirq_handler,
	[CHGEVENT_TERMTMR] = rt5058_chgirq_handler,
	[CHGEVENT_IEOC] = rt5058_chgirq_handler,
	[CHGEVENT_BSTLV] = rt5058_chgirq_handler,
	[CHGEVENT_BSTOL] = rt5058_chgirq_handler,
	[CHGEVENT_BSTOVP] = rt5058_chgirq_handler,
	[MUIC_UL_EVENT] = rt5058_chgirq_handler,
};

static irqreturn_t rt5058_chg_irq_handler(int irqno, void *param)
{
	struct rt5058_charger_info *ri = (struct rt5058_charger_info *)param;
	u8 regval[4];
	int ret, i;

	pr_debug("%s\n", __func__);

	ret = rt5058_block_read(ri->client, RT5058_REG_MUIC_IRQ1, 3, regval);
	if (ret < 0) {
		dev_err(ri->dev, "read muic irq fail\n");
		return IRQ_HANDLED;
	}
	if (regval[1] & RT5058_MUICUL_MASK)
		rt_chgirq_handler[MUIC_UL_EVENT](ri, MUIC_UL_EVENT);

	ret = rt5058_block_read(ri->client, RT5058_REG_CHG_IRQ1, 4, regval);
	if (ret < 0) {
		dev_err(ri->dev, "read chg irq fail\n");
		return IRQ_HANDLED;
	}

	ret = (regval[3] & ~(ri->cdata->irq_mask[3])) << 24 |
		(regval[2] & ~(ri->cdata->irq_mask[2])) << 16|
		(regval[1] & ~(ri->cdata->irq_mask[1])) << 8|
		(regval[0] & ~(ri->cdata->irq_mask[0]));

	RTINFO("irq_status & irq_mask = 0x%08x\n", ret);
	for (i = 0; i < CHGEVENT_MAX; i++) {
		if ((ret & (1 << i)) && rt_chgirq_handler[i])
			rt_chgirq_handler[i](ri, i);
	}

	return IRQ_HANDLED;
}

static void rt5058_chg_check_attach(struct rt5058_charger_info *ri)
{
	u8 regval;

	regval = rt5058_reg_read(ri->client, RT5058_REG_MUIC_STAT2);
	if (regval < 0) {
		pr_err("%s: read muic status fail\n", __func__);
		return;
	}
	if (!(regval & RT5058_MUICUL_MASK)) {
		rt_chgirq_handler[MUIC_UL_EVENT](ri, MUIC_UL_EVENT);
	}

	rt5058_assign_bits16(ri->client, RT5058_REG_CORECTRL1,
		RT5058_RESET_FLAG_MASK, 0);
}

static int rt5058_charger_irqinit(struct platform_device *pdev)
{
	struct rt5058_charger_info *ri = platform_get_drvdata(pdev);
	int ret;

	pr_info("%s\n", __func__);

	/* factory cable booting w/a */
	rt5058_chg_check_attach(ri);

	rt5058_clr_bits(ri->client, RT5058_REG_IRQMSK, RT5058_CHG_IRQMASK);
	rt5058_clr_bits(ri->client, RT5058_REG_IRQMSK, RT5058_MUIC_IRQMASK);
	ret = platform_get_irq_byname(pdev, "CHG_IRQ");
	if (ret < 0)
		return ret;
	ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
			rt5058_chg_irq_handler,
			IRQF_TRIGGER_NONE, "CHG_IRQ", ri);
	if (ret < 0) {
		dev_err(ri->dev, "request CHG_IRQ fail\n");
		return ret;
	}

	ret = rt5058_block_write(ri->client,
			RT5058_REG_CHG_MASK1, 5, ri->cdata->irq_mask);
	rt5058_clr_bits(ri->client, RT5058_REG_MUIC_MASK2, RT5058_MUICUL_MASK);
	return ret;
}

static void rt5058_charger_irq_deinit(struct platform_device *pdev)
{
	struct rt5058_charger_info *ri = platform_get_drvdata(pdev);
	int ret;

	pr_info("%s\n", __func__);
	rt5058_set_bits(ri->client, RT5058_REG_IRQMSK, RT5058_CHG_IRQMASK);
	ret = platform_get_irq_byname(pdev, "CHG_IRQ");
	if (ret < 0)
		return;
	devm_free_irq(&pdev->dev, ret, ri);
}
#endif /* RT5058_CHG_USE_NESTED_IRQ */

static void rt5058_charger_external_power_changed(struct power_supply *psy) {

	struct rt5058_charger_info *ri = dev_get_drvdata(psy->dev->parent);
	int ma;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	union lge_power_propval curr_ma;
#else
	union power_supply_propval curr_ma;
#endif
	if (!wake_lock_active(&ri->ext_psy_wake_lock))
		wake_lock(&ri->ext_psy_wake_lock);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	ri->lge_cd_lpc->get_property(ri->lge_cd_lpc,
				LGE_POWER_PROP_CURRENT_MAX, &curr_ma);
#else
	psy_do_property(RT5058_USB_DEVNAME, get,
			POWER_SUPPLY_PROP_CURRENT_MAX, curr_ma);
#endif
	ma = curr_ma.intval / 1000;
	pr_err("%s: cable_info result=%dmA\n", __func__, ma);
	/* set AICR */
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	if (!ri->iinms_done) {
		if (ri->usb_curr_ma_on && ma > 2)
			ma = (ma < ri->usb_curr_ma) ? ri->usb_curr_ma : ma;
		rt5058_set_input_current_regulation(ri, ma);
	}
#else
	if (!ri->iinms_done)
		rt5058_set_input_current_regulation(ri, ma);
#endif
	rt5058_update_battery_status(ri);

	if (wake_lock_active(&ri->ext_psy_wake_lock))
		wake_unlock(&ri->ext_psy_wake_lock);
}

#if defined (CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER)
static int lgcc_set_ibat_current(struct rt5058_charger_info *ri, int chg_current){

	int ret = 0;
	lgcc_charging_current = chg_current;
	ret = rt5058_set_charging_current(ri, chg_current);

	return ret;
}

static void lgcc_set_charging_enable(struct rt5058_charger_info *ri, int en)
{
	pr_debug("%s:en/chg_enabled/online %d/%d/%d\n",
		 __func__, en, ri->chg_enabled, ri->online);
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	if (ri->testmode) {
		pr_err("%s: testmode on, do not set.\n", __func__);
		return;
	}
#endif
	if (en == ri->chg_enabled)
		return;

	if (!ri->online)
		return;

	if (ri->full_charged)
		pr_debug("%s:full charged, do not set charging enable.\n", __func__);
	else
		rt5058_enable_charging(ri, en);

	return;
}
#endif
#ifdef CONFIG_LGE_PM_LG_POWER_CORE
static void
rt5058_charger_external_lge_power_changed(struct power_supply *psy)
{
	struct rt5058_charger_info *ri = dev_get_drvdata(psy->dev->parent);
	int rc = 0;
	union lge_power_propval lge_val = {0,};
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	union lge_power_propval lge_val2 = {0,};
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	int factory_cable = 0;
	int factory_cable_boot = 0;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	int lgcc_ibatt_current = 0;
	int lpc_chg_enable = 0;
#endif
	if (!wake_lock_active(&ri->ext_psy_wake_lock))
		wake_lock(&ri->ext_psy_wake_lock);

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	rc = ri->lge_cd_lpc->get_property(ri->lge_cd_lpc,
			LGE_POWER_PROP_IS_FACTORY_CABLE, &lge_val);
	factory_cable = lge_val.intval;

	rc = ri->lge_cd_lpc->get_property(ri->lge_cd_lpc,
			LGE_POWER_PROP_IS_FACTORY_MODE_BOOT, &lge_val);
	factory_cable_boot = lge_val.intval;
	if (factory_cable || factory_cable_boot)
		ri->is_factory_cable = 1;
	else
		ri->is_factory_cable = 0;
#endif

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGING_CONTROLLER
	ri->lge_cc_lpc = lge_power_get_by_name("cc");

	if (!ri->lge_cc_lpc) {
		pr_err("%s : lge_cc_lpc is not yet ready\n", __func__);
	} else {
		rc = ri->lge_cc_lpc->get_property(ri->lge_cc_lpc,
				LGE_POWER_PROP_BTM_STATE, &lge_val);
		ri->btm_state = lge_val.intval;

		rt5058_update_battery_health(ri);

		rc = ri->lge_cc_lpc->get_property(ri->lge_cc_lpc,
				LGE_POWER_PROP_OTP_CURRENT, &lge_val);
		if (lge_val.intval == -1)
			goto skip_current_config2;
		lgcc_ibatt_current = lge_val.intval;

		rc = ri->lge_cc_lpc->get_property(ri->lge_cc_lpc,
				LGE_POWER_PROP_CHARGING_ENABLED, &lge_val);
		if (lge_val.intval == -1)
			goto skip_current_config2;

		lpc_chg_enable = lge_val.intval;

		/* set ibat current from cc_lpc */
		lgcc_set_ibat_current(ri, lgcc_ibatt_current);

		rc = ri->lge_cc_lpc->get_property(ri->lge_cc_lpc,
			LGE_POWER_PROP_PSEUDO_BATT_UI, &lge_val);
		ri->pseudo_ui = lge_val.intval;
	}
skip_current_config2:
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	ri->lge_vzw_lpc = lge_power_get_by_name("vzw");
	if (!ri->lge_vzw_lpc) {
		pr_err("%s : lge_vzw_lpc is not yet ready\n", __func__);
	} else {
		rc = ri->lge_vzw_lpc->get_property(ri->lge_vzw_lpc,
				LGE_POWER_PROP_CHARGING_ENABLED, &lge_val);
		rc = ri->lge_vzw_lpc->get_property(ri->lge_vzw_lpc,
				LGE_POWER_PROP_STORE_DEMO_ENABLED, &lge_val2);

		/* LLK mode */
		if (lge_val2.intval) {
			rt5058_set_safety_timer(ri, 0);

			if (lge_val.intval)
				lpc_chg_enable = 1;
			else
				lpc_chg_enable = 0;
		}
	}
#endif
	lgcc_set_charging_enable(ri, lpc_chg_enable);
	rt5058_update_battery_status(ri);

	if (wake_lock_active(&ri->ext_psy_wake_lock))
		wake_unlock(&ri->ext_psy_wake_lock);
	return;
}
#endif

static int rt5058_charger_probe(struct platform_device *pdev)
{
	struct rt5058_mfd_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct rt5058_charger_info *ri;
	struct rt5058_charger_platform_data *cdata;
	bool use_dt = pdev->dev.of_node;
	int ret = 0;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	union lge_power_propval lge_val = {0,};
	int factory_cable = 0;
	int factory_cable_boot = 0;
#endif
	RTINFO("start.\n");
	if (use_dt) {
		cdata = devm_kzalloc(&pdev->dev, sizeof(*cdata), GFP_KERNEL);
		if (!cdata) {
			dev_err(&pdev->dev, "fail to allocate memory\n");
			return -ENOMEM;
		}
		rt_parse_dt(&pdev->dev, cdata);
	} else {
		dev_err(&pdev->dev, "no dts node\n");
		return -ENODEV;
	}

	ri = devm_kzalloc(&pdev->dev, sizeof(*ri), GFP_KERNEL);

	if (ri == NULL) {
		pr_err("%s : devm_kzalloc fail.\n", __func__);
		return -EPROBE_DEFER;
	}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	ri->lge_cd_lpc = lge_power_get_by_name("cable_detect");
	if (!ri->lge_cd_lpc) {
		pr_err("%s : lge_cd_lpc is not yet ready\n", __func__);
		devm_kfree(&pdev->dev, ri);
		return -EPROBE_DEFER;
	}

	ret = ri->lge_cd_lpc->get_property(ri->lge_cd_lpc,
			LGE_POWER_PROP_IS_FACTORY_CABLE, &lge_val);
	factory_cable = lge_val.intval;

	ret = ri->lge_cd_lpc->get_property(ri->lge_cd_lpc,
			LGE_POWER_PROP_IS_FACTORY_MODE_BOOT, &lge_val);
	factory_cable_boot = lge_val.intval;
	if (factory_cable == 1 || factory_cable_boot == 1)
		ri->is_factory_cable = 1;
	else
		ri->is_factory_cable = 0;
#endif

	ri->client = chip->client;
	ri->dev = chip->dev;
	ri->cdata = cdata;
	platform_set_drvdata(pdev, ri);
	chip->chg_info = ri;
	ri->otg_en = 0;
	ri->batt_present = 1;
	ri->chg_enabled = 1;
	ri->full_charged = 0;
	ri->batt_health = POWER_SUPPLY_HEALTH_GOOD;
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	ri->time_expired = 0;
#endif

	ri->psy.name = cdata->charger_name;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
	ri->psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
#else
	ri->psy.type = -1;
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)) */
	ri->psy.properties = rt_charger_props;
	ri->psy.num_properties = ARRAY_SIZE(rt_charger_props);
	ri->psy.get_property = rt5058_chg_get_property;
	ri->psy.set_property = rt5058_chg_set_property;
	ri->psy.supplied_to = rt_charger_supply_to_list;
	ri->psy.num_supplicants =
		ARRAY_SIZE(rt_charger_supply_to_list);
	ri->psy.external_power_changed =
		rt5058_charger_external_power_changed;
	ri->psy.property_is_writeable =
		rt5058_chg_prop_is_writeable;
#ifdef CONFIG_LGE_PM_LG_POWER_CORE
	ri->psy.external_lge_power_changed =
		rt5058_charger_external_lge_power_changed;
#endif
	/* must register in platform_device(&pdev->dev) or
				i2c_client(&i2c->dev) device */
	ret = power_supply_register(&pdev->dev, &ri->psy);
	if (ret < 0) {
		dev_err(ri->dev, "power supply regiseter fail\n");
		return -EINVAL;
	}

	mutex_init(&ri->full_charged_lock);
	mutex_init(&ri->chg_enabled_lock);

	ret = rt5058_charger_init(ri);
	if (ret < 0) {
		dev_err(ri->dev, "reg init fail\n");
		goto err_init;
	}

	ri->sldo1 = regulator_get(ri->dev, "rt5058-sldo1");
	if (IS_ERR(ri->sldo1)) {
		dev_err(ri->dev, "Regulator get fail\n");
		goto err_regu;
	}

	/* set sldo1 3.3V */
	regulator_set_voltage(ri->sldo1, 3300000, 3300000);
	ret = regulator_enable(ri->sldo1);

	INIT_DELAYED_WORK(&ri->usbinsert_work, rt5058_chg_usbinsert);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	INIT_DELAYED_WORK(&ri->current_settled_work, rt5058_notice_current_settled);
#endif
	ri->temp_status = RT_TEMP_NORMAL;

	wake_lock_init(&ri->safety_wake_lock, WAKE_LOCK_SUSPEND,
			"rt5058_btm_wake_lock");
	wake_lock_init(&ri->status_wake_lock, WAKE_LOCK_SUSPEND,
			"rt5058_status_wake_lock");
	wake_lock_init(&ri->ext_psy_wake_lock, WAKE_LOCK_SUSPEND,
			"rt5058_ext_chged_wake_lock");

	/* enable irq mask */
	ret = rt5058_charger_irqinit(pdev);
	if (ret < 0) {
		dev_err(ri->dev, "charger register irq fail\n");
		goto irq_err;
	}

	/* update charging status */
	rt5058_update_battery_status(ri);
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	ref = ri;
	rt5058_chg_create_attrs_factory(&pdev->dev);
#endif
	RTINFO("done.\n");

	return 0;

err_regu:
	rt5058_charger_irq_deinit(pdev);
irq_err:
err_init:
	power_supply_unregister(&ri->psy);
	devm_kfree(&pdev->dev, ri);
	return ret;
}

static int rt5058_charger_remove(struct platform_device *pdev)
{
	struct rt5058_charger_info *ri = platform_get_drvdata(pdev);

	if (ri) {
		mutex_destroy(&ri->full_charged_lock);
		mutex_destroy(&ri->chg_enabled_lock);
		rt5058_charger_irq_deinit(pdev);
		power_supply_unregister(&ri->psy);
		devm_kfree(&pdev->dev, ri);
	}
	return 0;
}

static void rt5058_charger_shutdown(struct platform_device *pdev)
{
	struct rt5058_charger_info *ri = platform_get_drvdata(pdev);
	int ret;

	/* buck1 disable for leakage current */
	rt5058_clr_bits(ri->client, RT5058_REG_BUCK1VOUT, 0x80);
	ret = rt5058_reg_read(ri->client, RT5058_REG_BUCK1VOUT);
	if (ret < 0)
		dev_err(ri->dev, "%s fail\n", __func__);

	pr_debug("%s: buck1 = 0x%02x\n", __func__, ret);
}

static struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,rt5058-charger",},
};

static struct platform_driver rt5058_charger_driver = {
	.driver = {
		.name = RT5058_CHG_DEVNAME,
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
	},
	.probe = rt5058_charger_probe,
	.remove = rt5058_charger_remove,
	.shutdown = rt5058_charger_shutdown,
};

static int __init rt5058_charger_module_init(void)
{
	return platform_driver_register(&rt5058_charger_driver);
}
module_init(rt5058_charger_module_init);

static void __exit rt5058_charger_module_exit(void)
{
	platform_driver_unregister(&rt5058_charger_driver);
}
module_exit(rt5058_charger_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com");
MODULE_DESCRIPTION("Charger driver for RT5058");
MODULE_ALIAS("platform:" RT5058_DEVICE_NAME "-charger");
MODULE_VERSION("1.0.0_LG");
