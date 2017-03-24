/*
 *  drivers/power/rt-battery.c
 *  Driver for Richtek Battery driver
 *
 *  Copyright (C) 2014 Richtek Technology Corp.
 *  Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include <linux/power/rt-battery.h>

#include <linux/qpnp/qpnp-adc.h>
#ifdef CONFIG_LGE_PM_LG_POWER_CORE
#include <soc/qcom/lge/power/lge_power_class.h>
#endif
#define RT_CAPACITY_ROUND(x)	((x + 5) / 10)
#define RT_BATTERY_TECHNOLOGY	POWER_SUPPLY_TECHNOLOGY_LION
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
#include <soc/qcom/lge/lge_pseudo_batt.h>
#endif

struct rt_battery_info {
	struct device *dev;
	struct mutex param_lock;
	struct power_supply bat;
	struct power_supply ac;
	struct power_supply usb;
	struct wake_lock bat_wake_lock;
	struct delayed_work monitor_work;
	struct delayed_work log_work;
	struct qpnp_vadc_chip *vadc_dev;
	ktime_t last_polling_time;
	int chg_status;
	int health;
	int capacity;
	int cable_type;
	int technology;
	int voltage_now; /* unit : mV */
	int current_now; /* unit : mA */
	int max_volt;
	int min_volt;
	int temp;
	int temp_amb;
	int batt_present;
	char *bat_name;
	char *fuel_name;
	char *chg_name;
	u8 suspend:1;
	u8 ac_online:1;
	u8 usb_online:1;
	u8 ac_pres:1;
	u8 usb_pres:1;
	u8 bat_online:1;
	u8 charging_full:1;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	struct lge_power *lge_vzw_lpc;
	int vzw_chg;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	struct lge_power *lge_cd_lpc;
	bool is_factory_cable;
#endif
};

#define DEFAULT_TEMP   250
int rt5058_battery_get_batt_temp(struct rt_battery_info *rbi)
{
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	int rc = 0;
	struct qpnp_vadc_result results;

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	if (get_pseudo_batt_info(PSEUDO_BATT_MODE)) {
		pr_debug("battery fake mode : %d\n", get_pseudo_batt_info(PSEUDO_BATT_MODE));
		return get_pseudo_batt_info(PSEUDO_BATT_TEMP) * 10;
	}
#endif
	rc = qpnp_vadc_read(rbi->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("%s(): Unable to read batt temperature rc=%d\n", __func__, rc);
		pr_err("%s(): Report default_batt_temp %d again\n",
				__func__, DEFAULT_TEMP);
		return DEFAULT_TEMP;
	} else {
		pr_debug("%s(): get_batt_temp %d %lld\n", __func__,
				results.adc_code, results.physical);
		return (int)results.physical;
	}
#else
	pr_err("CONFIG_SENSORS_QPNP_ADC_VOLTAGE is not defined.\n");
	return DEFAULT_TEMP;
#endif
}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
static int rt5058_is_batt_present(struct rt_battery_info *rbi)
{
	int batt_temp = 0;
	batt_temp = rt5058_battery_get_batt_temp(rbi);
	if (batt_temp <= -300 || batt_temp >= 790) {
		return 0;
	} else {
		return 1;
	}
}

static int rt5058_is_factory_cable(struct rt_battery_info *rbi)
{
	union lge_power_propval lge_val = {0,};
	rbi->lge_cd_lpc = lge_power_get_by_name("cable_detect");
	if (!rbi->lge_cd_lpc) {
		pr_err("cable_detect power_supply get failed\n");
		return 0;
	}
	rbi->lge_cd_lpc->get_property(rbi->lge_cd_lpc,
			LGE_POWER_PROP_IS_FACTORY_CABLE, &lge_val);
	if (lge_val.intval == 1)
		return 1;
	else
		return 0;
}
#endif

static void rt_update_battery_info(struct rt_battery_info *rbi)
{
	union power_supply_propval value;

	wake_lock(&rbi->bat_wake_lock);

	psy_do_property(rbi->chg_name, get,
			POWER_SUPPLY_PROP_BATT_PRESENT, value);
	rbi->batt_present = value.intval;

	psy_do_property(rbi->chg_name, get,
			POWER_SUPPLY_PROP_STATUS, value);
	rbi->chg_status = value.intval;

	psy_do_property(rbi->chg_name, get,
			POWER_SUPPLY_PROP_HEALTH, value);
	rbi->health = value.intval;

	psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, value);
	rbi->voltage_now = value.intval / 1000;

	psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_CURRENT_NOW, value);
	rbi->current_now = value.intval / 1000;

	psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_CAPACITY, value);
	rbi->capacity = value.intval;

	rbi->temp = rt5058_battery_get_batt_temp(rbi);

	dev_info(rbi->dev,
			"%s: Vnow(%dmV), Inow(%dmA), SOC(%d%%), Tbat(%d)\n",
			rt_battery_status_txt[rbi->chg_status], rbi->voltage_now,
			rbi->current_now, rbi->capacity, rbi->temp);

	power_supply_changed(&rbi->bat);
	if (wake_lock_active(&rbi->bat_wake_lock)) {
		wake_unlock(&rbi->bat_wake_lock);
	}
	return;
}

static enum power_supply_property rt_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
    POWER_SUPPLY_PROP_PSEUDO_BATT,
#endif
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
    POWER_SUPPLY_PROP_USB_CURRENT_MAX,
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	POWER_SUPPLY_PROP_SAFETY_TIMER,
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER
/* TODO : valid battery id node will be removed in battery code. */
#else
	POWER_SUPPLY_PROP_VALID_BATT,
#endif
#ifdef CONFIG_LGE_PM_OLD_PSY_NODE
	POWER_SUPPLY_PROP_VZW_CHG,
#endif
};

static int rt_bat_prop_is_writeable(struct power_supply *psy,
			enum power_supply_property psp)
{
	switch(psp) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
		case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
		case POWER_SUPPLY_PROP_SAFETY_TIMER:
#endif
			return 1;
		default:
			break;
	}
	return 0;
}

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
#ifdef CONFIG_MACH_MSM8909_E1Q_VZW
#define AICR_MA 900
#else
#define AICR_MA 900
#endif
#endif
static int rt_battery_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct rt_battery_info *rbi = dev_get_drvdata(psy->dev->parent);
	int ret = 0;
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	union power_supply_propval value = {0,};
#endif

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		rbi->chg_status = val->intval;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		rbi->health = val->intval;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		rbi->batt_present = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		pr_err("%s: set charging enabled %s\n",
				__func__, val->intval ? "on" : "off");
		if (val->intval)
			value.intval = 1;
		else
			value.intval = 0;
		psy_do_property(rbi->chg_name, set,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, value);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rt_update_battery_info(rbi);
		break;
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
		if (val->intval) {
			value.intval = AICR_MA;
			pr_err("%s: set usb current max on = %dmA\n", __func__, value.intval);
			value.intval = AICR_MA;
			psy_do_property(rbi->chg_name, set,
					POWER_SUPPLY_PROP_CURRENT_AVG, value);
		}
		else {
			pr_err("%s: usb current max off\n", __func__);
			value.intval = 0;
			psy_do_property(rbi->chg_name, set,
					POWER_SUPPLY_PROP_CURRENT_AVG, value);
		}
		break;
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	case POWER_SUPPLY_PROP_SAFETY_TIMER:
		if (val->intval)
			value.intval = 1;
		else
			value.intval = 0;
		pr_err("%s: set safety timer %s\n",
				__func__, value.intval ? "on" : "off");
		psy_do_property(rbi->chg_name, set,
				POWER_SUPPLY_PROP_SAFETY_TIMER, value);
		break;
#endif
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER
/* TODO : valid battery id node will be removed in battery code. */
#else
	case POWER_SUPPLY_PROP_VALID_BATT:
		break;
#endif
	default:
		ret = -EINVAL;
		break;
	}
	power_supply_changed(&rbi->bat);
	return ret;
}

static int rt_battery_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct rt_battery_info *rbi = dev_get_drvdata(psy->dev->parent);
	union power_supply_propval value;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = rbi->chg_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = rbi->health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = rbi->batt_present;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		psy_do_property(rbi->chg_name, get,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, value);
		val->intval = value.intval;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = rbi->technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, value);
		val->intval = value.intval;
		rbi->voltage_now = val->intval;
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
		if (get_pseudo_batt_info(PSEUDO_BATT_MODE)) {
			val->intval = get_pseudo_batt_info(PSEUDO_BATT_VOLT);
		}
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_CURRENT_NOW, value);
		val->intval = value.intval;
		rbi->current_now = val->intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = rbi->max_volt;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = rbi->min_volt;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
		if (rt5058_is_factory_cable(rbi) && !rt5058_is_batt_present(rbi)) {
			rbi->capacity = 80;
			val->intval = 80;
		} else {
			val->intval = (rbi->capacity > 100) ?
				100 : rbi->capacity;
		}
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
		if (get_pseudo_batt_info(PSEUDO_BATT_MODE)) {
			val->intval = get_pseudo_batt_info(PSEUDO_BATT_CAPACITY);
		}
#endif
#else
		val->intval = (rbi->capacity > 100) ?
			100 : rbi->capacity;
#endif
		break;
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
		if (!rt5058_is_batt_present(rbi)) {
			val->intval = 250;
		} else {
			val->intval = rt5058_battery_get_batt_temp(rbi);
		}
#else
		val->intval = rt5058_battery_get_batt_temp(rbi);
#endif
		break;
	case POWER_SUPPLY_PROP_TEMP_RAW:
		val->intval = rt5058_battery_get_batt_temp(rbi);
		break;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER
/* TODO : valid battery id node will be removed in battery code. */
#else
	case POWER_SUPPLY_PROP_VALID_BATT:
		val->intval = 1;
		break;
#endif
#ifdef CONFIG_LGE_PM_OLD_PSY_NODE
	case POWER_SUPPLY_PROP_VZW_CHG:
		val->intval = rbi->vzw_chg;
		break;
#endif
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
		val->intval = 1;
		break;
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TIMEOUT
	case POWER_SUPPLY_PROP_SAFETY_TIMER:
		psy_do_property(rbi->chg_name, get,
			POWER_SUPPLY_PROP_SAFETY_TIMER, value);
		val->intval = value.intval;
		break;
#endif
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		val->intval = get_pseudo_batt_info(PSEUDO_BATT_MODE);
		break;
#endif
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int rt_parse_dt(struct device *dev,
				struct rt_battery_info *rbi)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	u32 val;
	int len = 0;
	const char *bat_name = "battery";
	const char *fuel_name = "rt5058-fuelgauge";
	const char *chg_name = "rt5058-charger";

	of_property_read_string(np,
			"rt,fuel_name", (char const **)&fuel_name);
	len = strlen(fuel_name);
	rbi->fuel_name = devm_kzalloc(dev, len+1, GFP_KERNEL);
	if (!rbi->fuel_name)
		return -ENOMEM;
	strcpy(rbi->fuel_name, fuel_name);

	of_property_read_string(np,
			"rt,chg_name", (char const **)&chg_name);
	len = strlen(chg_name);
	rbi->chg_name = devm_kzalloc(dev, len+1, GFP_KERNEL);
	if (!rbi->chg_name)
		return -ENOMEM;
	strcpy(rbi->chg_name, chg_name);

	of_property_read_string(np,
			"rt,battery_name", (char const **)&bat_name);
	len = strlen(bat_name);
	rbi->bat_name = devm_kzalloc(dev, len+1, GFP_KERNEL);
	if (!rbi->bat_name)
		return -ENOMEM;
	strcpy(rbi->bat_name, bat_name);

	if (of_property_read_u32(np, "rt,max_volt", &val) >= 0)
		rbi->max_volt = val;
	else {
		pr_info("use default max voltage 4400mV\n");
		rbi->max_volt = 4400;
	}

	if (of_property_read_u32(np, "rt,min_volt", &val) >= 0)
		rbi->min_volt = val;
	else {
		pr_info("use default max voltage 3600mV\n");
		rbi->min_volt = 3600;
	}
#endif /* OCNFIG_OF */
	return 0;
}

static void rt_battery_log_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rt_battery_info *rbi = container_of(dwork, struct rt_battery_info,
			log_work);

	int vnow, inow, capacity, tbat, chg_stat = 0;
	union power_supply_propval value = {0,};

	wake_lock(&rbi->bat_wake_lock);

	chg_stat = rbi->chg_status;

	psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, value);
	vnow = value.intval / 1000;

	psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_CURRENT_NOW, value);
	inow = value.intval / 1000;

	psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_CAPACITY, value);
	capacity = value.intval;

	tbat = rt5058_battery_get_batt_temp(rbi);

	pr_err("RT5058FG: %s, Vnow(%dmV), Inow(%dmA), SOC(%d%%), Tbat(%d)\n",
			rt_battery_status_txt[chg_stat], vnow, inow, capacity, tbat);

	schedule_delayed_work(&rbi->log_work,
			round_jiffies_relative(msecs_to_jiffies(60000)));
	if (wake_lock_active(&rbi->bat_wake_lock)) {
		wake_unlock(&rbi->bat_wake_lock);
	}
	return;
}

static void rt5058_bat_external_power_changed(struct power_supply *psy)
{
	struct rt_battery_info *rbi = dev_get_drvdata(psy->dev->parent);
	union power_supply_propval value;

	psy_do_property(rbi->fuel_name, get,
			POWER_SUPPLY_PROP_CAPACITY, value);
	rbi->capacity = value.intval;

	psy_do_property(rbi->chg_name, get,
			POWER_SUPPLY_PROP_STATUS, value);
	rbi->chg_status = value.intval;

	psy_do_property(rbi->chg_name, get,
			POWER_SUPPLY_PROP_HEALTH, value);
	rbi->health = value.intval;

	psy_do_property(rbi->chg_name, get,
			POWER_SUPPLY_PROP_BATT_PRESENT, value);
	rbi->batt_present = value.intval;

	pr_err("%s: run bat_psy changed\n", __func__);

	power_supply_changed(&rbi->bat);
}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
static void rt5058_bat_external_lge_power_changed(struct power_supply *psy)
{
	struct rt_battery_info *rbi = dev_get_drvdata(psy->dev->parent);
	union lge_power_propval lge_val = {0,};
	int rc = 0;
	int prev_vzw_chg = rbi->vzw_chg;

	rbi->lge_vzw_lpc = lge_power_get_by_name("vzw");
	if (!rbi->lge_vzw_lpc) {
		pr_err("%s : lge_vzw_lpc is not yet ready\n", __func__);
	} else {
		rc = rbi->lge_vzw_lpc->get_property(rbi->lge_vzw_lpc,
				LGE_POWER_PROP_VZW_CHG, &lge_val);
		rbi->vzw_chg = lge_val.intval;
		if (prev_vzw_chg != rbi->vzw_chg) {
			power_supply_changed(&rbi->bat);
		}
	}
}
#endif

static int rt_battery_probe(struct platform_device *pdev)
{
	struct rt_battery_info *rbi;
	struct power_supply *psy;
	bool use_dt = pdev->dev.of_node;
	int ret;

	pr_info("%s\n", __func__);

	psy = power_supply_get_by_name("rt5058-charger");
	if (!psy) {
		pr_err("%s: chg psy get fail.", __func__);
		return -EPROBE_DEFER;
	}

	psy = power_supply_get_by_name("rt5058-fuelgauge");
	if (!psy) {
		pr_err("%s: fg psy get fail.", __func__);
		return -EPROBE_DEFER;
	}

	rbi = devm_kzalloc(&pdev->dev, sizeof(*rbi), GFP_KERNEL);
	if (!rbi)
		return -ENOMEM;

	if (use_dt) {
		ret = rt_parse_dt(&pdev->dev, rbi);
		if (ret)
			return -ENOMEM;
		rbi->vadc_dev = qpnp_get_vadc(&pdev->dev, "rt5058");
		if (IS_ERR(rbi->vadc_dev)) {
			ret = PTR_ERR(rbi->vadc_dev);
			if (ret != -EPROBE_DEFER)
				pr_err("vadc property missing\n");
			else
				pr_err("probe defer due to not initializing vadc\n");
			goto out_bat;
		}
	}
	else {
		dev_err(&pdev->dev, "no battery device\n");
		return -ENODEV;
	}

	rbi->dev = &pdev->dev;
	rbi->batt_present = 1;
	rbi->technology = RT_BATTERY_TECHNOLOGY;
	platform_set_drvdata(pdev, rbi);

	rbi->bat.name = rbi->bat_name;
	rbi->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	rbi->bat.set_property = rt_battery_set_property;
	rbi->bat.get_property = rt_battery_get_property;
	rbi->bat.external_power_changed =
		rt5058_bat_external_power_changed;
	rbi->bat.properties = rt_battery_props;
	rbi->bat.num_properties = ARRAY_SIZE(rt_battery_props);
	rbi->bat.property_is_writeable =
		rt_bat_prop_is_writeable;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_VZW_REQ
	rbi->bat.external_lge_power_changed =
		rt5058_bat_external_lge_power_changed;
#endif
	ret = power_supply_register(&pdev->dev, &rbi->bat);
	if (ret < 0) {
		dev_err(&pdev->dev, "battery power supply register fail\n");
		goto out_bat;
	}

	wake_lock_init(&rbi->bat_wake_lock,
			WAKE_LOCK_SUSPEND, "rt-battery-monitor");
	mutex_init(&rbi->param_lock);
	rt_update_battery_info(rbi);

	INIT_DELAYED_WORK(&rbi->log_work, rt_battery_log_worker);
	schedule_delayed_work(&rbi->log_work,
			round_jiffies_relative(msecs_to_jiffies(60000)));

	dev_info(&pdev->dev, "driver successfully loaded\n");
	return 0;
out_bat:
	devm_kfree(&pdev->dev, rbi);
	return ret;
}

static int rt_battery_remove(struct platform_device *pdev)
{
	struct rt_battery_info *rbi = platform_get_drvdata(pdev);

	power_supply_unregister(&rbi->bat);
	return 0;
}

static int rt_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rt_battery_info *rbi = platform_get_drvdata(pdev);

	rbi->suspend = 1;
	return 0;
}

static int rt_battery_resume(struct platform_device *pdev)
{
	struct rt_battery_info *rbi = platform_get_drvdata(pdev);

	rbi->suspend = 0;
	return 0;
}

static const struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,battery",},
	{},
};

static struct platform_driver rt_battery_driver = {
	.driver = {
		   .name = RT_BATT_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = rt_match_table,
		   },
	.probe = rt_battery_probe,
	.remove = rt_battery_remove,
	.suspend = rt_battery_suspend,
	.resume = rt_battery_resume,
};

static int __init rt_battery_init(void)
{
	return platform_driver_register(&rt_battery_driver);
}
late_initcall(rt_battery_init);
//module_init(rt_battery_init);

static void __exit rt_battery_exit(void)
{
	platform_driver_unregister(&rt_battery_driver);
}

module_exit(rt_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_DESCRIPTION("Richtek Battery driver");
MODULE_ALIAS("platform:rt-battery");
MODULE_VERSION("1.0.0_LG");
