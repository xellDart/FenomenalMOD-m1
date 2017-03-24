/* drivers/mfd/rt5058-irq.c
 * RT5058 Multifunction Drvice Driver
 * Charger / Buck / LDOs / FlashLED / MUIC
 *
 * Copyright (C) 2015 Richtek Technology Corp.
 * Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/mfd/rt5058/rt5058.h>
#include <linux/mfd/rt5058/rt5058-irq.h>
#include <linux/module.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/i2c.h>

struct irq_name_map {
	const char *name;
	int index;
};

#define RT5058_DECL_IRQMAP(_name) { .name = #_name, .index = RT5058_##_name}
static const struct irq_name_map rt5058_irq_name_map[] = {
	RT5058_DECL_IRQMAP(MUIC_IRQ),
	RT5058_DECL_IRQMAP(CHG_IRQ),
	RT5058_DECL_IRQMAP(FLED_IRQ),
	RT5058_DECL_IRQMAP(PMIC_IRQ),
	RT5058_DECL_IRQMAP(FUEL_IRQ),
	/* below is the reserved irq map for muic ++ */
	RT5058_DECL_IRQMAP(MUIC_OVP),
	RT5058_DECL_IRQMAP(MUIC_OVP_D),
	RT5058_DECL_IRQMAP(MUIC_OCP_LATCH),
	RT5058_DECL_IRQMAP(MUIC_OCP),
	RT5058_DECL_IRQMAP(MUIC_ATT),
	RT5058_DECL_IRQMAP(MUIC_UL),
	RT5058_DECL_IRQMAP(MUIC_CON),
	RT5058_DECL_IRQMAP(MUIC_CHGDET),
	RT5058_DECL_IRQMAP(MUIC_DCDT),
	RT5058_DECL_IRQMAP(MUIC_ADC_CHG),
	RT5058_DECL_IRQMAP(MUIC_ADC_US),
	RT5058_DECL_IRQMAP(MUIC_SKPS),
	RT5058_DECL_IRQMAP(MUIC_SKR),
	RT5058_DECL_IRQMAP(MUIC_KPS),
	RT5058_DECL_IRQMAP(MUIC_LKPS),
	RT5058_DECL_IRQMAP(MUIC_LKR),
	RT5058_DECL_IRQMAP(MUIC_EJ_SE),
	RT5058_DECL_IRQMAP(MUIC_EJ_DET),
	/* above is the reserved irq map for muic -- */
	RT5058_DECL_IRQMAP(CHG_IIN_MEAS),
	RT5058_DECL_IRQMAP(CHG_ICC_MEAS),
	RT5058_DECL_IRQMAP(CHG_MIVR),
	RT5058_DECL_IRQMAP(CHG_PWR_RDY),
	RT5058_DECL_IRQMAP(CHG_BATABS),
	RT5058_DECL_IRQMAP(CHG_SYSUV),
	RT5058_DECL_IRQMAP(CHG_TMR),
	RT5058_DECL_IRQMAP(CHG_BATOV),
	RT5058_DECL_IRQMAP(CHG_BADADP),
	RT5058_DECL_IRQMAP(CHG_RVP),
	RT5058_DECL_IRQMAP(CHG_TSSHD),
	RT5058_DECL_IRQMAP(CHG_TREG),
	RT5058_DECL_IRQMAP(CHG_RCHG),
	RT5058_DECL_IRQMAP(CHG_TERMTMR),
	RT5058_DECL_IRQMAP(CHG_IEOC),
	RT5058_DECL_IRQMAP(CHG_BSTLV),
	RT5058_DECL_IRQMAP(CHG_BSTOL),
	RT5058_DECL_IRQMAP(CHG_BSTOVP),
	RT5058_DECL_IRQMAP(FLED_STRBPIN),
	RT5058_DECL_IRQMAP(FLED_TORPIN),
	RT5058_DECL_IRQMAP(FLED_TX),
	RT5058_DECL_IRQMAP(FLED_LVF),
	RT5058_DECL_IRQMAP(FLED_LED2_SHORT),
	RT5058_DECL_IRQMAP(FLED_LED1_SHORT),
	RT5058_DECL_IRQMAP(FLED_LED2_STRB),
	RT5058_DECL_IRQMAP(FLED_LED1_STRB),
	RT5058_DECL_IRQMAP(FLED_LED2_STRB_TO),
	RT5058_DECL_IRQMAP(FLED_LED1_STRB_TO),
	RT5058_DECL_IRQMAP(FLED_LED2_TOR),
	RT5058_DECL_IRQMAP(FLED_LED1_TOR),
	RT5058_DECL_IRQMAP(PMIC_BUCK1_OCP),
	RT5058_DECL_IRQMAP(PMIC_BUCK1_LV),
	RT5058_DECL_IRQMAP(PMIC_OTP),
	RT5058_DECL_IRQMAP(PMIC_VDDA_OVP),
	RT5058_DECL_IRQMAP(PMIC_VDDA_UV),
	RT5058_DECL_IRQMAP(PMIC_SLDO2_LV),
	RT5058_DECL_IRQMAP(PMIC_SLDO1_LV),
	RT5058_DECL_IRQMAP(PMIC_LDO3_LV),
	RT5058_DECL_IRQMAP(PMIC_LDO2_LV),
	RT5058_DECL_IRQMAP(PMIC_LDO1_LV),
	/* below is the reserved irq map for fuelgauge */
	RT5058_DECL_IRQMAP(FUEL_FG_SC),
	RT5058_DECL_IRQMAP(FUEL_FG_EOD),
	RT5058_DECL_IRQMAP(FUEL_FG_EOC),
	RT5058_DECL_IRQMAP(FUEL_FG_US),
	RT5058_DECL_IRQMAP(FUEL_FG_OS),
	RT5058_DECL_IRQMAP(FUEL_FG_UV),
	RT5058_DECL_IRQMAP(FUEL_FG_UT),
	RT5058_DECL_IRQMAP(FUEL_FG_OT),
	RT5058_DECL_IRQMAP(FUEL_FG_RI),
	RT5058_DECL_IRQMAP(FUEL_FG_SHDN),
	RT5058_DECL_IRQMAP(FUEL_FG_SLP),
	RT5058_DECL_IRQMAP(FUEL_FG_PRES_RDY),
	RT5058_DECL_IRQMAP(FUEL_FG_OEP_ACT),
	RT5058_DECL_IRQMAP(FUEL_FG_BAT_TYPE),
	RT5058_DECL_IRQMAP(FUEL_FG_BAT_PRES),
	RT5058_DECL_IRQMAP(FUEL_FG_DSG),
	RT5058_DECL_IRQMAP(FUEL_FG_RDY),
	RT5058_DECL_IRQMAP(FUEL_FG_QSDONE),
	/* above is the reserved irq map for fuelgauge -- */
};

int rt5058_get_irq_index_byname(const char *irq_name)
{
	int i = 0, ret = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(rt5058_irq_name_map); i++) {
		if (!strcmp(irq_name, rt5058_irq_name_map[i].name)) {
			ret = rt5058_irq_name_map[i].index;
			break;
		}
	}
	return ret;
}

static int rt5058_irq_init_read(struct rt5058_mfd_chip *chip)
{
	unsigned char temp[12] = {0};
	int ret = 0;

	ret = rt5058_i2c_read_word(chip->client, RT5058_REG_FGOPCFG1);
	if (ret < 0) {
		dev_err(chip->dev, "read rt5058 reset flag fail\n");
		goto out_init_read;
	}

	if (!(ret & RT5058_RESET_FLAG_MASK)) {
		ret = rt5058_block_read(chip->client, RT5058_REG_MUIC_IRQ1, 12, temp);
		if (ret < 0)
			goto out_init_read;
		ret = rt5058_block_read(chip->client, RT5058_REG_FG_IRQ1, 4, temp);
		if (ret < 0)
			goto out_init_read;
	}
out_init_read:
	return ret;
}

static int rt5058_mask_all_irq(struct rt5058_mfd_chip *chip)
{
	int ret = 0;

	ret = rt5058_reg_write(chip->client, RT5058_REG_IRQMSK, 0xff);
	if (ret < 0)
		return ret;

	memset(&chip->irq_mask, 0xff, sizeof(chip->irq_mask));
	ret = rt5058_block_write(chip->client, RT5058_REG_MUIC_MASK1,
			RT5058_MUIC_IRQ_REGS_NR, chip->irq_mask.muic_irq);
	if (ret < 0)
		return ret;
	ret = rt5058_block_write(chip->client, RT5058_REG_CHG_MASK1,
			RT5058_CHG_IRQ_REGS_NR, chip->irq_mask.chg_irq);
	if (ret < 0)
		return ret;
	ret = rt5058_block_write(chip->client, RT5058_REG_FLED_MASK1,
			RT5058_FLED_IRQ_REGS_NR, chip->irq_mask.fled_irq);
	if (ret < 0)
		return ret;
	ret = rt5058_block_write(chip->client, RT5058_REG_PMIC_MASK1,
			RT5058_PMIC_IRQ_REGS_NR, chip->irq_mask.pmic_irq);
	if (ret < 0)
		return ret;
	ret = rt5058_block_write(chip->client, RT5058_REG_FG_MASK1,
			RT5058_FUEL_IRQ_REGS_NR, chip->irq_mask.fuel_irq);
	if (ret < 0)
		return ret;
	return 0;
}

static void rt5058_irq_unmask(struct irq_data *data)
{
	struct rt5058_mfd_chip *chip = irq_get_chip_data(data->irq);
	union rt5058_irq_maskstatus prev_irq_mask;
	int index = 0, index_mod = 0;

	if (!chip) {
		pr_err("%s(): chip is NULL\n", __func__);
		return;
	}

	memcpy(&prev_irq_mask, &chip->irq_mask, sizeof(prev_irq_mask));
	index = data->hwirq / 8;
	index_mod = data->hwirq % 8;
	if (data->hwirq != RT5058_MUIC_IRQ && data->hwirq != RT5058_FUEL_IRQ &&
	    data->hwirq != RT5058_FLED_IRQ && data->hwirq != RT5058_PMIC_IRQ &&
	    data->hwirq != RT5058_CHG_IRQ) {
		chip->irq_mask.regs[index] &= ~(1 << index_mod);
		chip->irq_mask_changed = 1;
		return;
	}

	switch (data->hwirq) {
	case RT5058_MUIC_IRQ:
		chip->muic_irq_bypass = 1;
		break;
	case RT5058_FUEL_IRQ:
		chip->fuel_irq_bypass = 1;
		break;
	case RT5058_CHG_IRQ:
		chip->chg_irq_bypass = 1;
		break;
	case RT5058_FLED_IRQ:
		chip->fled_irq_bypass = 1;
		break;
	case RT5058_PMIC_IRQ:
		chip->pmic_irq_bypass = 1;
		break;
	default:
		dev_err(chip->dev, "Not support irq number\n");
		goto out_unmask_irq;
	}
	return;
out_unmask_irq:
	/* if error, restore */
	memcpy(&chip->irq_mask, &prev_irq_mask, sizeof(prev_irq_mask));
}

static void rt5058_irq_mask(struct irq_data *data)
{
	struct rt5058_mfd_chip *chip = irq_get_chip_data(data->irq);
	union rt5058_irq_maskstatus prev_irq_mask;
	int index = 0, index_mod = 0;

	if (!chip) {
		pr_err("%s(): chip is NULL\n", __func__);
		return;
	}

	pr_info("%s\n", __func__);
	memcpy(&prev_irq_mask, &chip->irq_mask, sizeof(prev_irq_mask));
	index = data->hwirq / 8;
	index_mod = data->hwirq % 8;
	if (data->hwirq != RT5058_MUIC_IRQ && data->hwirq != RT5058_FUEL_IRQ &&
	    data->hwirq != RT5058_FLED_IRQ && data->hwirq != RT5058_PMIC_IRQ &&
	    data->hwirq != RT5058_CHG_IRQ) {
		chip->irq_mask.regs[index] &= ~(1 << index_mod);
		return;
	}

	switch (data->hwirq) {
	case RT5058_MUIC_IRQ:
		chip->muic_irq_bypass = 0;
		memset(chip->irq_mask.muic_irq, 0xff, RT5058_MUIC_IRQ_REGS_NR);
		break;
	case RT5058_FUEL_IRQ:
		chip->fuel_irq_bypass = 0;
		memset(chip->irq_mask.fuel_irq, 0xff, RT5058_FUEL_IRQ_REGS_NR);
		break;
	case RT5058_CHG_IRQ:
		chip->chg_irq_bypass = 0;
		memset(chip->irq_mask.chg_irq, 0xff, RT5058_CHG_IRQ_REGS_NR);
		break;
	case RT5058_FLED_IRQ:
		chip->fled_irq_bypass = 0;
		memset(chip->irq_mask.fled_irq, 0xff, RT5058_FLED_IRQ_REGS_NR);
		break;
	case RT5058_PMIC_IRQ:
		chip->pmic_irq_bypass = 0;
		memset(chip->irq_mask.pmic_irq, 0xff, RT5058_PMIC_IRQ_REGS_NR);
		break;
	default:
		dev_err(chip->dev, "Not support irq number\n");
		goto out_mask_irq;
	}
	return;
out_mask_irq:
	/* if error, restore */
	memcpy(&chip->irq_mask, &prev_irq_mask, sizeof(prev_irq_mask));
}

static void rt5058_irq_bus_lock(struct irq_data *data)
{
	struct rt5058_mfd_chip *chip = irq_get_chip_data(data->irq);

	down(&chip->irq_lock);
}

static void rt5058_irq_bus_sync_unlock(struct irq_data *data)
{
	struct rt5058_mfd_chip *chip = irq_get_chip_data(data->irq);
	int ret;

	if (!chip) {
		pr_err("%s(): chip is NULL\n", __func__);
		return;
	}

	switch (data->hwirq) {
	case RT5058_MUIC_IRQ_START ... RT5058_MUIC_IRQ_END:
		ret = rt5058_block_write(chip->client, RT5058_REG_MUIC_MASK1,
			RT5058_MUIC_IRQ_REGS_NR, chip->irq_mask.muic_irq);
		break;
	case RT5058_CHG_IRQ_START ... RT5058_CHG_IRQ_END:
		ret = rt5058_block_write(chip->client, RT5058_REG_CHG_MASK1,
			RT5058_CHG_IRQ_REGS_NR, chip->irq_mask.chg_irq);
		break;
	case RT5058_FLED_IRQ_START ... RT5058_FLED_IRQ_END:
		ret = rt5058_block_write(chip->client, RT5058_REG_FLED_MASK1,
			RT5058_FLED_IRQ_REGS_NR, chip->irq_mask.fled_irq);
		break;
	case RT5058_PMIC_IRQ_START ... RT5058_PMIC_IRQ_END:
		ret = rt5058_block_write(chip->client, RT5058_REG_PMIC_MASK1,
			RT5058_PMIC_IRQ_REGS_NR, chip->irq_mask.pmic_irq);
		break;
	case RT5058_FUEL_IRQ_START ... RT5058_FUEL_IRQ_END:
		ret = rt5058_block_write(chip->client, RT5058_REG_FG_MASK1,
			RT5058_FUEL_IRQ_REGS_NR, chip->irq_mask.fuel_irq);
		break;
	default:
		break;
	}

	chip->irq_mask_changed = 0;
	up(&chip->irq_lock);
}

static struct irq_chip rt5058_irq_chip = {
	.name = "rt5058-irq",
	.irq_unmask = rt5058_irq_unmask,
	.irq_mask = rt5058_irq_mask,
	.irq_bus_lock = rt5058_irq_bus_lock,
	.irq_bus_sync_unlock = rt5058_irq_bus_sync_unlock,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0))
static int rt5058_irqdomain_ops_map(struct irq_domain *d, unsigned int virq,
		irq_hw_number_t hwirq)
{
	struct rt5058_mfd_chip *chip = d->host_data;

	irq_set_chip_data(virq, chip);
	irq_set_chip_and_handler(virq, &rt5058_irq_chip, handle_simple_irq);
	irq_set_nested_thread(virq, true);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
	irq_set_parent(virq, chip->irq);
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)) */
#ifdef CONFIG_ARM
	/*
	 * ARM requires an extra step to clear IRQ_NOREQUEST, which it
	 * sets on behalf of every irq_chip.  Also sets IRQ_NOPROBE.
	 */
	set_irq_flags(virq, IRQF_VALID);
#else
	/* same effect on other architectures */
	irq_set_noprobe(virq);
#endif /* #ifdef CONFIG_ARM */
	return 0;
}

static void rt5058_irqdomain_ops_unmap(struct irq_domain *d, unsigned int virq)
{
#ifdef CONFIG_ARM
	set_irq_flags(virq, 0);
#endif /* #ifdef CONFIG_ARM */
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops rt5058_irqdomain_ops = {
	.map = rt5058_irqdomain_ops_map,
	.unmap = rt5058_irqdomain_ops_unmap,
	.xlate = irq_domain_xlate_onetwocell,
};
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)) */

static irqreturn_t rt5058_irq_handler(int irq, void *data)
{
	struct rt5058_mfd_chip *chip = data;
	int regdata = 0, i = 0, virq = 0;

	down(&chip->suspend_lock);
	regdata = rt5058_reg_read(chip->client, RT5058_REG_IRQIND);
	if (regdata < 0) {
		dev_err(chip->dev, "read irq indicator fail\n");
		goto out_irq_handler;
	}
	RTINFO("read 0x0a = 0x%02x\n", regdata);
	/* read irq event */
	if (regdata & 0x80 && !chip->chg_irq_bypass) {
		rt5058_block_read(chip->client, RT5058_REG_CHG_IRQ1,
			RT5058_CHG_IRQ_REGS_NR, chip->irq_event.chg_irq);
		RTINFO("chg_irq = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
						chip->irq_event.chg_irq[0],
						chip->irq_event.chg_irq[1],
						chip->irq_event.chg_irq[2],
						chip->irq_event.chg_irq[3],
						chip->irq_event.chg_irq[4]);
	} else
		memset(chip->irq_event.chg_irq, 0, RT5058_CHG_IRQ_REGS_NR);
	if (regdata & 0x40 && !chip->fled_irq_bypass)
		rt5058_block_read(chip->client, RT5058_REG_FLED_IRQ1,
			RT5058_FLED_IRQ_REGS_NR, chip->irq_event.fled_irq);
	else
		memset(chip->irq_event.fled_irq, 0, RT5058_FLED_IRQ_REGS_NR);
	if (regdata & 0x10 && !chip->pmic_irq_bypass) {
		rt5058_block_read(chip->client, RT5058_REG_PMIC_IRQ1,
			RT5058_PMIC_IRQ_REGS_NR, chip->irq_event.pmic_irq);
		RTINFO("pmic_irq = 0x%02x 0x%02x\n",
				chip->irq_event.pmic_irq[0],
				chip->irq_event.pmic_irq[1]);
	} else
		memset(chip->irq_event.pmic_irq, 0, RT5058_PMIC_IRQ_REGS_NR);
	if ((regdata & 0x08) && !chip->fuel_irq_bypass) {
		rt5058_block_read(chip->client, RT5058_REG_FG_IRQ1,
			RT5058_FUEL_IRQ_REGS_NR, chip->irq_event.fuel_irq);
		RTINFO("fuel_irq = 0x%02x%02x 0x%02x%02x\n",
					chip->irq_event.fuel_irq[0],
					chip->irq_event.fuel_irq[1],
					chip->irq_event.fuel_irq[2],
					chip->irq_event.fuel_irq[3]);
	} else
		memset(chip->irq_event.fuel_irq, 0x00, RT5058_FUEL_IRQ_REGS_NR);
	/* re-check enabled irq event */
	for (i = 0; i < RT5058_IRQ_REGS_NR; i++)
		chip->irq_event.regs[i] &= ~chip->irq_mask.regs[i];
	/* dispatch irq event */
	if (((regdata & 0x80) || (regdata & 0x20)) && chip->chg_irq_bypass) {
		virq = irq_find_mapping(chip->irq_domain, RT5058_CHG_IRQ);
		handle_nested_irq(virq);
	}
	if ((regdata & 0x08) && chip->fuel_irq_bypass) {
		virq = irq_find_mapping(chip->irq_domain, RT5058_FUEL_IRQ);
		handle_nested_irq(virq);
	}
	if ((regdata & 0x40) && chip->fled_irq_bypass) {
		virq = irq_find_mapping(chip->irq_domain, RT5058_FLED_IRQ);
		handle_nested_irq(virq);
	}
	if ((regdata & 0x10) && chip->pmic_irq_bypass) {
		virq = irq_find_mapping(chip->irq_domain, RT5058_PMIC_IRQ);
		handle_nested_irq(virq);
	}
	for (i = 0; i < RT5058_MAX_IRQS_NR; i++) {
		if (chip->irq_event.regs[i/8] & (1<<(i % 8))) {
			virq = irq_find_mapping(chip->irq_domain, i);
			handle_nested_irq(virq);
		}
	}
out_irq_handler:
	up(&chip->suspend_lock);
	return IRQ_HANDLED;
}

int rt5058_init_irq(struct rt5058_mfd_chip *chip)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0))
	int i = 0, curr_irq = 0;
#endif /* #if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)) */
	int ret = 0;
	bool use_dt = chip->dev->of_node;

	if (use_dt) {
		chip->irq_domain = irq_domain_add_linear(chip->dev->of_node,
				    RT5058_MAX_IRQS_NR, &rt5058_irqdomain_ops,
				    chip);
		if (!chip->irq_domain) {
			dev_err(chip->dev, "error irq_domain request\n");
			goto err_init_base;
		}
	} else {
		chip->irq_base = irq_alloc_descs(chip->irq_base, 0,
						 RT5058_MAX_IRQS_NR, 0);
		if (chip->irq_base < 0) {
			dev_err(chip->dev, "irq base is not valid\n");
			ret = -EINVAL;
			goto err_init_base;
		}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0))
		chip->irq_domain = irq_domain_add_legacy(chip->dev->of_node,
					RT5058_MAX_IRQS_NR, chip->irq_base, 0,
					&rt5058_irqdomain_ops, chip);
		if (!chip->irq_domain) {
			dev_err(chip->dev, "error irq_domain request\n");
			irq_free_descs(chip->irq_base, RT5058_MAX_IRQS_NR);
			ret = -EINVAL;
			goto err_init_base;
		}
#else
		chip->irq_domain = devm_kzalloc(chip->dev,
					sizeof(*chip->irq_domain), GFP_KERNEL);
		if (!chip->irq_domain) {
			dev_err(chip->dev, "error irq_domain alloc\n");
			devm_kfree(chip->dev, chip->irq_domain);
			irq_free_descs(chip->irq_base, RT5058_MAX_IRQS_NR);
			ret = -ENOMEM;
			goto err_init_base;
		}
		chip->irq_domain->irq_base = chip->irq_base;
		chip->irq_domain->nr_irq = RT5058_MAX_IRQS_NR;
		chip->irq_domain->ops = irq_domain_simple_ops;
		irq_domain_add(chip->irq_domain);
		for (i = 0; i < RT5058_IRQS_NR; i++) {
			curr_irq = i + chip->irq_base;
			irq_set_chip_data(curr_irq, chip);
			irq_set_chip_and_handler(curr_irq,
				&rt5058_irq_chip, handle_simple_irq);
			irq_set_nested_thread(curr_irq, true);
#ifdef CONFIG_ARM
			set_irq_flags(curr_irq, IRQF_VALID);
#else
			irq_set_noprobe(curr_irq);
#endif /* #ifdef CONFIG_ARM */
		}
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)) */
	}
	ret = rt5058_mask_all_irq(chip);
	if (ret < 0) {
		dev_err(chip->dev, "rt5058 can't mask all irqs(%d)\n", ret);
		goto err_mask_all_irqs;
	}

	ret = rt5058_irq_init_read(chip);
	if (ret < 0) {
		dev_err(chip->dev, "irq init read fail\n");
		goto err_irq_init_read;
	}

	ret = gpio_request_one(chip->irq_gpio, GPIOF_IN, "rt5058_mfd_irq");
	if (ret < 0) {
		dev_err(chip->dev, "GPIO %d\n", chip->irq_gpio);
		dev_err(chip->dev, "Set GPIO direction input failed\n");
		goto err_request_gpio;
	}

	chip->irq = gpio_to_irq(chip->irq_gpio);
	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
			rt5058_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"rt5058-irq", chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ(%d)\n", ret);
		goto err_request_irq;
	}
	device_init_wakeup(chip->dev, true);
	dev_info(chip->dev, "%s\n", __func__);
	return ret;
err_request_irq:
	gpio_free(chip->irq_gpio);
err_request_gpio:
err_irq_init_read:
err_mask_all_irqs:
	if (use_dt) {
		dev_err(chip->dev, "free irq domain\n");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
		irq_domain_remove(chip->irq_domain);
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)) */
	} else {
		dev_err(chip->dev, "free irq domain\n");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
		irq_domain_remove(chip->irq_domain);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0))
		for (curr_irq = chip->irq_base;
			curr_irq < chip->irq_base + RT5058_MAX_IRQS_NR;
								curr_irq++) {
#ifdef CONFIG_ARM
			set_irq_flags(curr_irq, 0);
#endif /* #ifdef CONFIG_ARM */
			irq_set_chip_and_handler(curr_irq, NULL, NULL);
			irq_set_chip_data(curr_irq, NULL);
		}
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)) */
		irq_free_descs(chip->irq_base, RT5058_MAX_IRQS_NR);
	}
err_init_base:
	return ret;
}

int rt5058_exit_irq(struct rt5058_mfd_chip *chip)
{
	bool use_dt = chip->dev->of_node;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0))
	int curr_irq = 0;
#endif /* #if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)) */

	devm_free_irq(chip->dev, chip->irq, chip);
	gpio_free(chip->irq_gpio);
	if (use_dt) {
		dev_err(chip->dev, "free irq domain\n");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
		irq_domain_remove(chip->irq_domain);
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)) */
	} else {
		dev_err(chip->dev, "free irq domain\n");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
		irq_domain_remove(chip->irq_domain);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0))
		for (curr_irq = chip->irq_base;
			curr_irq < chip->irq_base + RT5058_MAX_IRQS_NR;
								curr_irq++) {
#ifdef CONFIG_ARM
			set_irq_flags(curr_irq, 0);
#endif /* #ifdef CONFIG_ARM */
			irq_set_chip_and_handler(curr_irq, NULL, NULL);
			irq_set_chip_data(curr_irq, NULL);
		}
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)) */
		irq_free_descs(chip->irq_base, RT5058_MAX_IRQS_NR);
	}
	return 0;
}

#ifdef CONFIG_PM
int rt5058_irq_suspend(struct rt5058_mfd_chip *chip)
{
	disable_irq(chip->irq);
	if (device_may_wakeup(chip->dev))
		enable_irq_wake(chip->irq);
	return 0;
}

int rt5058_irq_resume(struct rt5058_mfd_chip *chip)
{
	if (device_may_wakeup(chip->dev))
		disable_irq_wake(chip->irq);
	enable_irq(chip->irq);
	return 0;
}
#endif /* CONFIG_PM */
