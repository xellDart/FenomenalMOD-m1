/* drivers/regulator/rt5058-regulator.c
 * Driver for Richtek RT5058 Regulator
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/of.h>

#include <linux/mfd/rt5058/rt5058.h>
#include <linux/regulator/rt5058-regulator.h>

#define ALIAS_NAME "rt5058_regulator"

struct rt5058_regulator_info {
	struct regulator_desc *desc;
	struct regulator_dev *regulator;
	struct i2c_client *client;
	struct device *dev;
	const unsigned int *vol_output_list;
	int min_uV;
	int max_uV;
	int step_uV;
	u8 irq_mask[2];
	u8 vol_reg;
	u8 vol_shift;
	u8 vol_mask;
	u8 enable_reg;
	u8 enable_bit;
};

static const unsigned int rt5058_buck_vol_output_list[] = {
	1000*1000, 1100*1000, 1200*1000, 1300*1000,
	1400*1000, 1500*1000, 1600*1000, 1700*1000,
	1800*1000, 1900*1000, 2000*1000, 2100*1000,
	2200*1000, 2300*1000, 2400*1000, 2500*1000,
	2600*1000, 2700*1000, 2800*1000, 2900*1000,
	3000*1000, 3000*1000, 3000*1000, 3000*1000,
	3000*1000, 3000*1000, 3000*1000, 3000*1000,
	3000*1000, 3000*1000, 3000*1000, 3000*1000,
};

static const unsigned int rt5058_ldo_vol_output_list[] = {
	1200*1000, 1300*1000, 1400*1000, 1500*1000,
	1600*1000, 1700*1000, 1800*1000, 1900*1000,
	2000*1000, 2100*1000, 2200*1000, 2300*1000,
	2400*1000, 2500*1000, 2600*1000, 2700*1000,
	2800*1000, 2900*1000, 3000*1000, 3000*1000,
	3000*1000, 3000*1000, 3000*1000, 3000*1000,
	3000*1000, 3000*1000, 3000*1000, 3000*1000,
	3000*1000, 3000*1000, 3000*1000, 3000*1000,
};

static const unsigned int rt5058_sldo_vol_output_list[] = {
	3300*1000, 4850*1000, 4900*1000, 4950*1000,
};

static inline int check_range(struct rt5058_regulator_info *info,
						int min_uV, int max_uV)
{
	if (min_uV < info->min_uV || min_uV > info->max_uV)
		return -EINVAL;
	return 0;
}

static int rt5058_list_voltage(struct regulator_dev *rdev, unsigned index)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);

	return (index >= rdev->desc->n_voltages) ?
			-EINVAL :
			info->vol_output_list[index];
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38))
static int rt5058_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);
	u8 data;
	const int count = rdev->desc->n_voltages;

	if (selector > count)
		return -EINVAL;

	data = (u8)selector;
	data <<= info->vol_shift;
	return rt5058_assign_bits(info->client,
				info->vol_reg, info->vol_mask, data);
}

static int rt5058_get_voltage_sel(struct regulator_dev *rdev)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);
	int ret;

	ret = rt5058_reg_read(info->client, info->vol_reg);
	if (ret < 0)
		return ret;
	return (ret&info->vol_mask)>>info->vol_shift;
}

#else
static int rt5058_find_voltage(struct regulator_dev *rdev,
					int min_uV, int max_uV)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);
	int i = 0;

	for (i = 0; i < rdev->desc->n_voltages; i++) {
		if ((info->vol_output_list[i] >= min_uV)
			&& (info->vol_output_list[i] <= max_uV))
			return i;
	}
	return -EINVAL;
}

static int rt5058_set_voltage(struct regulator_dev *rdev,
			int min_uV, int max_uV, unsigned *selector)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);
	u8 data;

	if (check_range(info, min_uV, max_uV)) {
		dev_err(&rdev->dev,
			"invalid voltage range (%d, %d) uV\n", min_uV, max_uV);
		return -EINVAL;
	}
	data = rt5058_find_voltage(rdev, min_uV, max_uV);
	data <<= info->vol_shift;
	return rt5058_assign_bits(info->client,
				info->vol_reg, info->vol_mask, data);
}

static int rt5058_get_voltage(struct regulator_dev *rdev)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);
	int ret;

	ret = rt5058_reg_read(info->client, info->vol_reg);
	if (ret < 0) {
		pr_err("rt5058 read regulator voltage fail\n");
		return ret;
	}
	ret = (ret&info->vol_mask)>>info->vol_shift;
	RTINFO("index = %d\n", ret);
	return rt5058_list_voltage(rdev, ret);
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38) */

static int rt5058_enable(struct regulator_dev *rdev)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);

	pr_info("%s: Enable regulator %s\n", ALIAS_NAME, rdev->desc->name);
	return rt5058_set_bits(info->client,
				info->enable_reg, info->enable_bit);
}

static int rt5058_disable(struct regulator_dev *rdev)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);

	pr_info("%s: Disable regulator %s\n", ALIAS_NAME, rdev->desc->name);
	return rt5058_clr_bits(info->client,
				info->enable_reg, info->enable_bit);
}

static int rt5058_is_enabled(struct regulator_dev *rdev)
{
	struct rt5058_regulator_info *info = rdev_get_drvdata(rdev);
	int ret;

	ret = rt5058_reg_read(info->client, info->enable_reg);
	if (ret < 0)
		return ret;

	ret = (ret&info->enable_bit) ? 1:0;
	pr_info("%s %s %s enabled\n",
		ALIAS_NAME, rdev->desc->name, ret ? "is":"is not");
	return ret;
}

static struct regulator_ops rt5058_regulator_ops = {
	.list_voltage		= rt5058_list_voltage,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38))
	.get_voltage_sel	= rt5058_get_voltage_sel,
	.set_voltage_sel	= rt5058_set_voltage_sel,
#else
	.set_voltage		= rt5058_set_voltage,
	.get_voltage		= rt5058_get_voltage,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38) */
	.enable			= rt5058_enable,
	.disable		= rt5058_disable,
	.is_enabled		= rt5058_is_enabled,
};

#define RT5058_REGULATOR_VOLLIST0	rt5058_buck_vol_output_list
#define RT5058_REGULATOR_VOLLIST1	rt5058_ldo_vol_output_list
#define RT5058_REGULATOR_VOLLIST2	rt5058_ldo_vol_output_list
#define RT5058_REGULATOR_VOLLIST3	rt5058_ldo_vol_output_list
#define RT5058_REGULATOR_VOLLIST4	rt5058_sldo_vol_output_list
#define RT5058_REGULATOR_VOLLIST5	rt5058_sldo_vol_output_list

#define RT5058_BUCK_VOUT_SIZE	ARRAY_SIZE(rt5058_buck_vol_output_list)
#define RT5058_LDO_VOUT_SIZE	ARRAY_SIZE(rt5058_ldo_vol_output_list)
#define RT5058_SLDO_VOUT_SIZE	ARRAY_SIZE(rt5058_sldo_vol_output_list)

#define RT5058_REGULATOR_DESC_DECL(_id, _name, _n_voltages)	\
{								\
	.id = _id,						\
	.name = _name,						\
	.n_voltages = _n_voltages,				\
	.ops = &rt5058_regulator_ops,				\
	.type = REGULATOR_VOLTAGE,				\
	.owner = THIS_MODULE,					\
}

static struct regulator_desc rt5058_regulator_desc[RT5058_MAX_REGULATOR] = {
	RT5058_REGULATOR_DESC_DECL(RT5058_ID_BUCK1,
			"rt5058-buck1", RT5058_BUCK_VOUT_SIZE),
	RT5058_REGULATOR_DESC_DECL(RT5058_ID_LDO1,
			"rt5058-ldo1", RT5058_LDO_VOUT_SIZE),
	RT5058_REGULATOR_DESC_DECL(RT5058_ID_LDO2,
			"rt5058-ldo2", RT5058_LDO_VOUT_SIZE),
	RT5058_REGULATOR_DESC_DECL(RT5058_ID_LDO3,
			"rt5058-ldo3", RT5058_LDO_VOUT_SIZE),
	RT5058_REGULATOR_DESC_DECL(RT5058_ID_SLDO1,
			"rt5058-sldo1", RT5058_SLDO_VOUT_SIZE),
	RT5058_REGULATOR_DESC_DECL(RT5058_ID_SLDO2,
			"rt5058-sldo2", RT5058_SLDO_VOUT_SIZE),
};

#define RT5058_REGULATOR_INFO_DECL(_id, min, max, step)		\
{								\
	.desc = &rt5058_regulator_desc[_id],			\
	.min_uV = min * 1000,					\
	.max_uV = max * 1000,					\
	.step_uV = step * 1000,					\
	.vol_reg = RT5058_REGULATOR_REG##_id,			\
	.vol_shift = RT5058_REGULATOR_SHIFT,			\
	.vol_mask = RT5058_REGULATOR_MASK##_id,			\
	.enable_reg = RT5058_REGULATOR_EN_REG##_id,		\
	.enable_bit = RT5058_REGULATOR_EN_MASK,			\
	.vol_output_list = RT5058_REGULATOR_VOLLIST##_id,	\
}

static struct rt5058_regulator_info
		rt5058_regulator_infos[RT5058_MAX_REGULATOR] = {
	RT5058_REGULATOR_INFO_DECL(0, 1000, 3000, 100),
	RT5058_REGULATOR_INFO_DECL(1, 1200, 3000, 100),
	RT5058_REGULATOR_INFO_DECL(2, 1200, 3000, 100),
	RT5058_REGULATOR_INFO_DECL(3, 1200, 3000, 100),
	RT5058_REGULATOR_INFO_DECL(4, 3300, 4950, 0),
	RT5058_REGULATOR_INFO_DECL(5, 3300, 4950, 0),
};

static struct rt5058_regulator_info *find_regulator_info(int id)
{
	struct rt5058_regulator_info *ri;
	int i;

	for (i = 0; i < ARRAY_SIZE(rt5058_regulator_infos); i++) {
		ri = &rt5058_regulator_infos[i];
		if (ri->desc->id == id)
			return ri;
	}
	return NULL;
}

inline struct regulator_dev *rt5058_regulator_register(
		struct regulator_desc *desc, struct device *dev,
		struct regulator_init_data *init_data, void *driver_data)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
	struct regulator_config config = {
		.dev = dev,
		.init_data = init_data,
		.driver_data = driver_data,
		.of_node = dev->of_node,
	};
	return regulator_register(desc, &config);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	return regulator_register(desc, dev,
			init_data, driver_data, dev->of_node);
#else
	return regulatro_reguster(desc, dev, init_data, driver_data);
#endif /* LINUX_VERSION >= KERNEL_VERSION(3, 5, 9) */
}

static struct regulator_init_data *rt_parse_dt(
		struct rt5058_regulator_info *info, struct device *dev)
{
	struct regulator_init_data *init_data = NULL;
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	u32 val[2];
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
	init_data = of_get_regulator_init_data(dev, np);
	if (!init_data) {
		pr_err("%s(): init_data is NULL\n", __func__);
		return NULL;
	}
	#else
	init_dat = of_get_regulatr_init_data(dev);
	#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)) */

	if (of_property_read_bool(np, "rt,allow_mode_mask")) {
		if (init_data) {
			init_data->constraints.valid_modes_mask |=
				(REGULATOR_MODE_NORMAL|REGULATOR_MODE_IDLE);
			init_data->constraints.valid_ops_mask |=
				REGULATOR_CHANGE_MODE;
		}
	}
	if (of_property_read_u32_array(np, "rt,pmic_irq_mask", val, 2) >= 0) {
		info->irq_mask[0] = val[0];
		info->irq_mask[1] = val[1];
	} else
		info->irq_mask[0] = info->irq_mask[1] = 0xff;
#endif /* CONFIG_OF */
	return init_data;
}

static bool pmicirq_isnot_init = true;
#if RT5058_PMIC_USE_NESTED_IRQ
static irqreturn_t rt5058_buck1_ovp_irq(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_buck1_lv_irq(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_ldo1_lv_irq(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_ldo2_lv_irq(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_ldo3_lv_irq(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_sldo1_lv_irq(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_sldo2_lv_irq(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_pmic_otp(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_pmic_vdda_ocp(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_pmic_vdda_uv(int irq, void *data)
{
	struct rt5058_regulator_info *ri = (struct rt5058_regulator_info *)data;

	dev_info(&ri->regulator->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static const struct rt5058_irq_handler rt5058_irq_handler[] = {
	{ .irq_name = "PMIC_BUCK1_OCP", .irq_handler = rt5058_buck1_ovp_irq},
	{ .irq_name = "PMIC_BUCK1_LV", .irq_handler = rt5058_buck1_lv_irq},
	{ .irq_name = "PMIC_LDO1_LV", .irq_handler = rt5058_ldo1_lv_irq},
	{ .irq_name = "PMIC_LDO2_LV", .irq_handler = rt5058_ldo2_lv_irq},
	{ .irq_name = "PMIC_LDO3_LV", .irq_handler = rt5058_ldo3_lv_irq},
	{ .irq_name = "PMIC_SLDO1_LV", .irq_handler = rt5058_sldo1_lv_irq},
	{ .irq_name = "PMIC_SLDO2_LV", .irq_handler = rt5058_sldo2_lv_irq},
	{ .irq_name = "PMIC_OTP", .irq_handler = rt5058_pmic_otp},
	{ .irq_name = "PMIC_VDDA_OVP", .irq_handler = rt5058_pmic_vdda_ocp},
	{ .irq_name = "PMIC_VDDA_UV", .irq_handler = rt5058_pmic_vdda_uv},
};

static int rt5058_regulator_irqinit(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	struct rt5058_regulator_info *ri = rdev_get_drvdata(rdev);
	int i = 0, ret = 0;
	const char *irq_name = NULL;

	if (!pmicirq_isnot_init)
		return 0;

	pr_info("%s\n", __func__);
	rt5058_clr_bits(ri->client, RT5058_REG_IRQMSK, RT5058_PMIC_IRQMASK);

	for (i = 0; i < ARRAY_SIZE(rt5058_irq_handler); i++) {
		irq_name = rt5058_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
			rt5058_irq_handler[i].irq_handler, IRQF_TRIGGER_NONE,
			irq_name, ri);
		if (ret < 0) {
			dev_err(&pdev->dev, "request %s fail\n", irq_name);
			goto out_irq_init;
		}
	}
	pmicirq_isnot_init = false;
	return 0;
out_irq_init:
	while (--i >= 0) {
		irq_name = rt5058_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		devm_free_irq(&pdev->dev, ret, ri);
	}
	return  -EINTR;
}

static void rt5058_regulator_irqdeinit(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	struct rt5058_regulator_info *ri = rdev_get_drvdata(rdev);
	int i = 0, ret = 0;
	const char *irq_name = NULL;

	pr_info("%s\n", __func__);
	rt5058_set_bits(ri->client, RT5058_REG_IRQMSK, RT5058_PMIC_IRQMASK);
	for (i = 0; i < ARRAY_SIZE(rt5058_irq_handler); i++) {
		irq_name = rt5058_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		devm_free_irq(&pdev->dev, ret, ri);
	}
	pmicirq_isnot_init = true;
}
#else
static void rt5058_pmicirq_handler(void *info, int eventno)
{
	struct rt5058_regulator_info *ri = rt5058_regulator_infos;

	dev_info(ri->dev, "%s: eventno = %d\n", __func__, eventno);

	switch (eventno) {
	case PMICEVENT_BUCK1OCP:
		dev_info(ri->dev, "IRQ: Pmic buck1 ocp\n");
		break;
	case PMICEVENT_BUCK1LV:
		dev_info(ri->dev, "IRQ: Pmic buck1 lv\n");
		break;
	case PMICEVENT_OTP:
		dev_info(ri->dev, "IRQ: Pmic OTP\n");
		break;
	case PMICEVENT_VDDAOVP:
		dev_info(ri->dev, "IRQ: Pmic VDDA OVP\n");
		break;
	case PMICEVENT_VDDAUV:
		dev_info(ri->dev, "IRQ: Pmic VDDA UV\n");
		break;
	case PMICEVENT_SLDO2LV:
		dev_info(ri->dev, "IRQ: Pmic SLDO2 LV\n");
		break;
	case PMICEVENT_SLDO1LV:
		dev_info(ri->dev, "IRQ: Pmic SLDO1 LV\n");
		break;
	case PMICEVENT_LDO3LV:
		dev_info(ri->dev, "IRQ: Pmic LDO3 LV\n");
		break;
	case PMICEVENT_LDO2LV:
		dev_info(ri->dev, "IRQ: Pmic LDO2 LV\n");
		break;
	case PMICEVENT_LDO1LV:
		dev_info(ri->dev, "IRQ: Pmic LDO1 LV\n");
		break;
	default:
		break;
	}
}

static rt_irq_handler rt_pmicirq_handler[PMICEVENT_MAX] = {
	[PMICEVENT_BUCK1OCP] = rt5058_pmicirq_handler,
	[PMICEVENT_BUCK1LV] = rt5058_pmicirq_handler,
	[PMICEVENT_OTP] = rt5058_pmicirq_handler,
	[PMICEVENT_VDDAOVP] = rt5058_pmicirq_handler,
	[PMICEVENT_VDDAUV] = rt5058_pmicirq_handler,
	[PMICEVENT_SLDO2LV] = rt5058_pmicirq_handler,
	[PMICEVENT_SLDO1LV] = rt5058_pmicirq_handler,
	[PMICEVENT_LDO3LV] = rt5058_pmicirq_handler,
	[PMICEVENT_LDO2LV] = rt5058_pmicirq_handler,
	[PMICEVENT_LDO1LV] = rt5058_pmicirq_handler,
};

static irqreturn_t rt5058_pmic_irq_handler(int irqno, void *param)
{
	struct rt5058_regulator_info *ri = rt5058_regulator_infos;
	u8 regval[2];
	int ret, i;

	RTINFO("%s\n", __func__);
	ret = rt5058_block_read(ri->client, RT5058_REG_PMIC_IRQ1, 2, regval);
	if (ret < 0) {
		dev_err(ri->dev, "read muic irq fail\n");
		return IRQ_HANDLED;
	}

	ret = (regval[1] & ~(ri->irq_mask[1])) << 8 |
		(regval[0] & ~(ri->irq_mask[0]));

	RTINFO("irq_status & irq_mask = 0x%04x\n", ret);
	for (i = 0; i < PMICEVENT_MAX; i++) {
		if ((ret & (1 << i)) && rt_pmicirq_handler[i])
			rt_pmicirq_handler[i](ri, i);
	}
	return IRQ_HANDLED;
}

static int rt5058_regulator_irqinit(struct platform_device *pdev)
{
	struct rt5058_regulator_info *ri = rt5058_regulator_infos;
	int ret;

	if (!pmicirq_isnot_init)
		return 0;

	pr_info("%s\n", __func__);
	rt5058_clr_bits(ri->client, RT5058_REG_IRQMSK, RT5058_PMIC_IRQMASK);
	ret = platform_get_irq_byname(pdev, "PMIC_IRQ");
	if (ret < 0)
		return ret;
	ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
			rt5058_pmic_irq_handler,
			IRQF_TRIGGER_NONE, "PMIC_IRQ", ri);
	if (ret < 0) {
		dev_err(ri->dev, "request PMIC_IRQ fail\n");
		return ret;
	}

	ret = rt5058_block_write(ri->client,
			RT5058_REG_PMIC_MASK1, 2, ri->irq_mask);

	pmicirq_isnot_init = false;
	return ret;

}

static void rt5058_regulator_irqdeinit(struct platform_device *pdev)
{
	struct rt5058_regulator_info *ri = rt5058_regulator_infos;
	int ret;

	pr_info("%s\n", __func__);
	rt5058_set_bits(ri->client, RT5058_REG_IRQMSK, RT5058_PMIC_IRQMASK);
	ret = platform_get_irq_byname(pdev, "PMIC_IRQ");
	if (ret < 0)
		return;
	devm_free_irq(&pdev->dev, ret, ri);
	pmicirq_isnot_init = true;
}
#endif /* RT5058_PMIC_USE_NESTED_IRQ */

static int rt5058_regulator_probe(struct platform_device *pdev)
{
	struct rt5058_mfd_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct rt5058_regulator_info *ri;
	struct regulator_dev *rdev;
	struct regulator_init_data *init_data;
	bool use_dt = pdev->dev.of_node;
	int ret = 0;

	ri = find_regulator_info(pdev->id);
	if (ri == NULL) {
		dev_err(chip->dev, "invalid regulator ID sepcified\n");
		return -EINVAL;
	}

	ri->client = chip->client;
	ri->dev = chip->dev;
	if (use_dt)
		init_data = rt_parse_dt(ri, &pdev->dev);
	else {
		dev_err(&pdev->dev, "no dts node\n");
		return -ENODEV;
	}

	if (init_data == NULL) {
		dev_err(chip->dev, "no initializing data\n");
		return -EINVAL;
	}

	rdev = rt5058_regulator_register(ri->desc, &pdev->dev, init_data, ri);
	if (IS_ERR(rdev)) {
		dev_err(chip->dev,
			"fail to register regulator %s\n", ri->desc->name);
		ret = PTR_ERR(rdev);
		goto out_probe;
	}
	platform_set_drvdata(pdev, rdev);

	ret = rt5058_regulator_irqinit(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "error request irq\n");
		goto out_irq_init;
	}
	pr_info("%s: probe successfully\n", init_data->constraints.name);
	return 0;
out_irq_init:
	regulator_unregister(rdev);
out_probe:
	return ret;
}

static int rt5058_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	rt5058_regulator_irqdeinit(pdev);
	regulator_unregister(rdev);
	return 0;
}

static const struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,rt5058-buck1",},
	{.compatible = "richtek,rt5058-ldo1",},
	{.compatible = "richtek,rt5058-ldo2",},
	{.compatible = "richtek,rt5058-ldo3",},
	{.compatible = "richtek,rt5058-sldo1",},
	{.compatible = "richtek,rt5058-sldo2",},
};

static struct platform_driver rt5058_regulator_driver = {
	.driver = {
		.name = RT5058_DEV_NAME "-regulator",
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
	},
	.probe = rt5058_regulator_probe,
	.remove = rt5058_regulator_remove,
};

static int __init rt5058_regulator_init(void)
{
	return platform_driver_register(&rt5058_regulator_driver);
}
subsys_initcall(rt5058_regulator_init);

static void __exit rt5058_regulator_exit(void)
{
	platform_driver_unregister(&rt5058_regulator_driver);
}
module_exit(rt5058_regulator_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com");
MODULE_DESCRIPTION("Regulator driver for RT5058");
MODULE_ALIAS("platform:" RT5058_DEV_NAME "-regulator");
MODULE_VERSION("1.0.0_LG");
