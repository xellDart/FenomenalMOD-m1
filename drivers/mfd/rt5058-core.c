/* drivers/mfd/rt5058-core.c
 *  Driver for Richtek RT5058 Core PMIC
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
#include <linux/version.h>
#include <linux/mfd/core.h>
#include <linux/of_irq.h>

#include <linux/mfd/rt5058/rt5058.h>

#ifdef CONFIG_REGULATOR_RT5058
#ifdef CONFIG_OF
#define RT5058_BUCK_DEVS(_id, _idx)				\
{								\
	.name		= RT5058_DEV_NAME "-regulator",		\
	.num_resources  = 0,					\
	.of_compatible  = "richtek," RT5058_DEV_NAME "-buck" #_idx,	\
	.id		= RT5058_ID_##_id,			\
}
#define RT5058_LDO_DEVS(_id, _idx)				\
{								\
	.name		= RT5058_DEV_NAME "-regulator",		\
	.num_resources  = 0,					\
	.of_compatible  = "richtek," RT5058_DEV_NAME "-ldo" #_idx,	\
	.id		= RT5058_ID_##_id,			\
}
#define RT5058_SLDO_DEVS(_id, _idx)				\
{								\
	.name		= RT5058_DEV_NAME "-regulator",		\
	.num_resources  = 0,					\
	.of_compatible  = "richtek," RT5058_DEV_NAME "-sldo" #_idx,	\
	.id		= RT5058_ID_##_id,			\
}
#else
#define RT5058_BUCK_DEVS(_id, _idx)				\
{								\
	.name		= RT5058_DEV_NAME "-regulator",		\
	.num_resources	= 0,					\
	.id		= RT5058_ID_##_id,			\
}
#define RT5058_LDO_DEVS(_id, _idx)				\
{								\
	.name		= RT5058_DEV_NAME "-regulator",		\
	.num_resources	= 0,					\
	.id		= RT5058_ID_##_id,			\
}
#define RT5058_SLDO_DEVS(_id, _idx)				\
{								\
	.name		= RT5058_DEV_NAME "-regulator",		\
	.num_resources	= 0,					\
	.id		= RT5058_ID_##_id,			\
}
#endif /* CONFIG_OF */

static struct mfd_cell regulator_devs[] = {
	RT5058_BUCK_DEVS(BUCK1, 1),
	RT5058_LDO_DEVS(LDO1, 1),
	RT5058_LDO_DEVS(LDO2, 2),
	RT5058_LDO_DEVS(LDO3, 3),
	RT5058_SLDO_DEVS(SLDO1, 1),
	RT5058_SLDO_DEVS(SLDO2, 2),
};
#endif /* CONFIG_REGULATOR_RT5058 */

#ifdef CONFIG_FLED_RT5058
static struct mfd_cell fled_devs[] = {
	{
		.name = RT5058_DEV_NAME "-fled",
		.id = -1,
		.num_resources = 0,
		#ifdef CONFIG_OF
		.of_compatible = "richtek,rt5058-fled",
		#endif /* CONFIG_OF */
	},
};
#endif /* CONFIG_FLED_RT5058 */

#ifdef CONFIG_LGE_PM_CHARGING_RT5058_CHARGER
static struct mfd_cell charger_devs[] = {
	{
		.name = RT5058_DEV_NAME "-charger",
		.id = -1,
		.num_resources = 0,
		#ifdef CONFIG_OF
		.of_compatible = "richtek,rt5058-charger",
		#endif /* CONFIG_OF */
	},
};
#endif /* CONFIG_LGE_PM_CHARGING_RT5058_CHARGER */

#ifdef CONFIG_MUIC_RT5058
static struct mfd_cell muic_devs[] = {
	{
		.name = RT5058_DEV_NAME "-muic",
		.id = -1,
		.num_resources = 0,
		#ifdef CONFIG_OF
		.of_compatible = "richtek,rt5058-muic",
		#endif /* CONFIG_OF */
	},
};
#endif /* CONFIG_MUIC_RT5058 */

#ifdef CONFIG_LGE_PM_FUELGAUGE_RT5058
static struct mfd_cell fuelgauge_devs[] = {
	{
		.name = RT5058_DEV_NAME "-fuelgauge",
		.id = -1,
		.num_resources = 0,
		#ifdef CONFIG_OF
		.of_compatible = "richtek,rt5058-fuelgauge",
		#endif /* CONFIG_OF */
	}
};
#endif /* CONFIG_LGE_PM_FUELGAUGE_RT5058 */

enum {
	IRQ_TYPE_REGULATOR,
	IRQ_TYPE_FUELGAUGE,
	IRQ_TYPE_FLASHLED,
	IRQ_TYPE_CHARGER,
	IRQ_TYPE_MUIC,
	IRQ_TYPE_MAX,
};

static void rt5058_parse_pdata_irq(struct rt5058_mfd_chip *chip,
	struct mfd_cell *cell, int cell_cnt, int irq_type)
{
	struct rt5058_mfd_platform_data *pdata = chip->dev->platform_data;
	struct rt5058_regulator_platform_data *reg_pdata =
		pdata->regulator_platform_data;
	struct rt5058_irq_enable_t *enable_irq = NULL;
	struct resource *m_res = NULL;
	int i = 0, j = 0;
	int ret = 0;

	switch (irq_type) {
	case IRQ_TYPE_REGULATOR:
		for (i = 0; i < cell_cnt; i++) {
			enable_irq = reg_pdata->enable_irq[i];
			if (!enable_irq)
				continue;
			j = enable_irq[i].irq_count;
			m_res = devm_kzalloc(chip->dev, sizeof(*m_res) * j,
					GFP_KERNEL);
			if (!m_res)
				continue;
			for (j = 0; j < enable_irq->irq_count; j++) {
				ret = rt5058_get_irq_index_byname(
						enable_irq->irq_name[j]);
				if (ret < 0)
					continue;
				ret += chip->irq_base;
				m_res[j].start = m_res[j].end = ret;
				m_res[j].name = enable_irq->irq_name[j];
				m_res[j].flags = IORESOURCE_IRQ;
			}
			cell[i].resources = m_res;
			cell[i].num_resources = j;
		}
		break;
	default:
		dev_err(chip->dev, "Not supported IRQ_TYPE\n");
		break;
	}
}

static void rt5058_parse_dt_irq(struct rt5058_mfd_chip *chip,
	struct mfd_cell *cell)
{
#ifdef CONFIG_OF
	struct device_node *np = NULL;
	struct resource *m_res = NULL;
	int irq_cnt = 0;
	int ret = -EINVAL;

	if (chip->dev->of_node && cell->of_compatible) {
		for_each_child_of_node(chip->dev->of_node, np) {
			if (of_device_is_compatible(np, cell->of_compatible)) {
				ret = 0;
				break;
			}
		}
	}
	if (ret < 0)
		return;
	if (np) {
		irq_cnt = of_irq_count(np);
		if (irq_cnt <= 0)
			return;
		m_res = devm_kzalloc(chip->dev, sizeof(*m_res) * irq_cnt,
				     GFP_KERNEL);
		if (!m_res) {
			pr_err("%s(): kzalloc failed\n", __func__);
			return;
		}
		cell->num_resources = of_irq_to_resource_table(np, m_res,
								irq_cnt);
		cell->resources = m_res;
	}
	for (irq_cnt = 0; irq_cnt < cell->num_resources; irq_cnt++) {
		m_res = (struct resource *)cell->resources + irq_cnt;
		ret = rt5058_get_irq_index_byname(m_res->name);
		if (ret < 0)
			continue;
		ret = irq_find_mapping(chip->irq_domain, ret);
		if (ret <= 0)
			m_res->flags = 0x00000000;

		if (ret != m_res->start) {
			dev_info(chip->dev, "name = %s\n", m_res->name);
			dev_info(chip->dev, "try to fix index %d\n", ret);
			m_res->start = m_res->end = ret;
		}
	}
#endif /* #ifdef CONFIG_OF */
}

static int rt5058_parse_irq_to_resources(struct rt5058_mfd_chip *chip,
	struct mfd_cell *cell, int cell_cnt, int irq_type)
{
	int i = 0;
	bool use_dt = chip->dev->of_node;

	if (use_dt) {
		for (i = 0; i < cell_cnt; i++)
			rt5058_parse_dt_irq(chip, &cell[i]);
	} else
		rt5058_parse_pdata_irq(chip, cell, cell_cnt, irq_type);
	return 0;
}

int rt5058_core_init(struct rt5058_mfd_chip *chip,
			struct rt5058_mfd_platform_data *pdata)
{
	int ret = 0;
	bool use_dt = chip->dev->of_node;

	pr_info("Start to initialize all device\n");
#ifdef CONFIG_REGULATOR_RT5058
	if (use_dt || (pdata && pdata->regulator_platform_data)) {
		RTINFO("mfd add regulator devices\n");
		/* if not dt, parse irq to resource */
		rt5058_parse_irq_to_resources(chip, regulator_devs,
			ARRAY_SIZE(regulator_devs), IRQ_TYPE_REGULATOR);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
		ret = mfd_add_devices(chip->dev, 0, &regulator_devs[0],
				ARRAY_SIZE(regulator_devs), NULL,
						chip->irq_base, NULL);
#else
		ret = mfd_add_devices(chip->dev, 0, &regulator_devs[0],
				ARRAY_SIZE(regulator_devs), NULL,
							chip->irq_base);
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0) */
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add regulator device\n");
			goto out_dev;
		}
	}
#endif /* CONFIG_REGULATOR_RT5058 */


#ifdef CONFIG_FLED_RT5058
	if (use_dt || (pdata && pdata->fled_pdata)) {
		RTINFO("mfd add flashlight led devices\n");
		rt5058_parse_irq_to_resources(chip, fled_devs,
				ARRAY_SIZE(fled_devs), IRQ_TYPE_FLASHLED);
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
		ret = mfd_add_devices(chip->dev, 0, &fled_devs[0],
				ARRAY_SIZE(fled_devs), NULL,
						chip->irq_base, NULL);
	#else
		ret = mfd_add_devices(chip->dev, 0, &fled_devs[0],
				ARRAY_SIZE(fled_devs), NULL, chip->irq_base);
	#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0) */
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add fled device\n");
			goto out_dev;
		}
	}
#endif /* CONFIG_FLED_RT5058 */

#ifdef CONFIG_LGE_PM_CHARGING_RT5058_CHARGER
	if (use_dt || (pdata && pdata->chg_pdata)) {
		RTINFO("mfd add charger devices\n");
		rt5058_parse_irq_to_resources(chip, charger_devs,
				ARRAY_SIZE(charger_devs), IRQ_TYPE_CHARGER);
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
		ret = mfd_add_devices(chip->dev, 0, &charger_devs[0],
				ARRAY_SIZE(charger_devs), NULL,
						chip->irq_base, NULL);
	#else
		ret = mfd_add_devices(chip->dev, 0, &charger_devs[0],
					ARRAY_SIZE(charger_devs),
						NULL, chip->irq_base);
	#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0) */
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add charger device\n");
			goto out_dev;
		}
	}
#endif /* CONFIG_LGE_PM_CHARGING_RT5058_CHARGER */

#ifdef CONFIG_MUIC_RT5058
	if (use_dt || pdata) {
		RTINFO("mfd add muic devices\n");
		rt5058_parse_irq_to_resources(chip, muic_devs,
				ARRAY_SIZE(muic_devs), IRQ_TYPE_MUIC);
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
		ret = mfd_add_devices(chip->dev, 0, &muic_devs[0],
				ARRAY_SIZE(muic_devs), NULL,
						chip->irq_base, NULL);
	#else
		ret = mfd_add_devices(chip->dev, 0, &muic_devs[0],
						ARRAY_SIZE(muic_devs),
						NULL, chip->irq_base);
	#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0) */
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add muic device\n");
			goto out_dev;
		}
	}
#endif /* CONFIG_MUIC_RT5058 */

#ifdef CONFIG_LGE_PM_FUELGAUGE_RT5058
	if (use_dt || (pdata)) {
		RTINFO("mfd add fuelgauge devices\n");
		rt5058_parse_irq_to_resources(chip, fuelgauge_devs,
				ARRAY_SIZE(fuelgauge_devs), IRQ_TYPE_FUELGAUGE);
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
		ret = mfd_add_devices(chip->dev, 0, &fuelgauge_devs[0],
				ARRAY_SIZE(fuelgauge_devs), NULL,
						chip->irq_base, NULL);
	#else
		ret = mfd_add_devices(chip->dev, 0, &fuelgauge_devs[0],
						ARRAY_SIZE(fuelgauge_devs),
							NULL, chip->irq_base);
	#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0) */
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add fuelgauge device\n");
			goto out_dev;
		}
	}
#endif /* CONFIG_LGE_PM_FUELGAUGE_RT5058 */
	pr_info("Initialize all device successfully\n");
	return 0;
out_dev:
	mfd_remove_devices(chip->dev);
	return ret;
}
EXPORT_SYMBOL(rt5058_core_init);

int rt5058_core_deinit(struct device *dev)
{
	mfd_remove_devices(dev);
	return 0;
}
EXPORT_SYMBOL(rt5058_core_deinit);
