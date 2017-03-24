/*drivers/leds/leds-rt5058.c
 *  Driver for Richtek RT5058 flash led function
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
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/mfd/rt5058/rt5058.h>
#include <linux/leds/rtfled.h>
#include <linux/leds/flashlight.h>
#include <linux/leds/rt5058-fled.h>
#include <linux/misc/rt-regmap.h>

static struct platform_device rt_fled_pdev = {
	.name = "rt-flash-led",
	.id = -1,
};

struct rt5058_fled_ctrl {
	u8 mode_reg;
	u8 strb_bit;
	u8 en_bit;
	u8 tor_cur_reg;
	u8 strb_cur_reg;
	u8 timeout_reg;
	u8 strb_to_reg;
};

struct rt5058_fled_info {
	rt_fled_info_t base;
	struct i2c_client *client;
	struct device *dev;
	struct rt5058_fled_platform_data *pdata;
	struct rt5058_fled_ctrl *ctrl[2];
	u8 fled_status:2;
	u8 fled_en:1;
};

static int rt5058_fled_reg_init(struct rt5058_fled_info *fi)
{
	int ret;

	RTINFO("%s\n", __func__);
	rt5058_reg_write(fi->client, RT5058_REG_STRBEN, 0x00);

	if (fi->pdata->control_mode == FLED_PIN_CONTROL)
		ret = rt5058_set_bits(fi->client,
			RT5058_REG_FLEDCFG, RT5058_FLED_PIN_CTRL);
	else
		ret = rt5058_clr_bits(fi->client,
			RT5058_REG_FLEDCFG, RT5058_FLED_PIN_CTRL);
	if (ret < 0)
		return -EINVAL;
	return ret;
}

static inline int rt5058_fled_reset(struct rt5058_fled_info *fi)
{
	int ret;

	RTINFO("%s\n", __func__);
	ret = rt5058_fled_reg_init(fi);
	if (ret < 0) {
		dev_err(fi->dev, "rt5058 fled init fail\n");
		return -EINVAL;
	}
	return ret;
}

static int rt5058_fled_init(struct rt_fled_info *info)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *) info;
	int ret = 0;

	RTINFO("%s\n", __func__);
	ret = rt5058_fled_reset(fi);
	if (ret < 0) {
		dev_err(fi->dev, "reset rt5058 fled fail\n");
		goto err_init;
	}
	ret = rt5058_fled_reg_init(fi);
	if (ret < 0)
		dev_err(fi->dev, "init rt5058 fled register fail\n");
err_init:
	return ret;
}

static int rt5058_fled_suspend(struct rt_fled_info *info, pm_message_t state)
{
	RTINFO("SUSPEND\n");
	return 0;
}

static int rt5058_fled_resume(struct rt_fled_info *info)
{
	RTINFO("RESUME\n");
	return 0;
}

static int rt5058_fled_strobe(struct rt_fled_info *info)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret = 0;

	if (fi->pdata->control_mode == FLED_PIN_CONTROL) {
		dev_info(fi->dev, "use pin contrl strobe\n");
		return 0;
	}

	ret = rt5058_clr_bits(fi->client, RT5058_REG_STRBEN,
			RT5058_FLED1STRB_MASK|RT5058_FLED2STRB_MASK);
	if (fi->pdata->flash1_en)
		ret = rt5058_set_bits(fi->client,
			RT5058_REG_STRBEN, fi->ctrl[0]->strb_bit);
	if (fi->pdata->flash2_en)
		ret = rt5058_set_bits(fi->client,
			RT5058_REG_STRBEN, fi->ctrl[1]->strb_bit);
	if (ret < 0)
		dev_err(fi->dev, "set strobe bit fail\n");
	return ret;
}

static int rt5058_fled_set_mode(
			struct rt_fled_info *info, flashlight_mode_t mode)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret = 0, i;
	u8 regval = 0;

	switch (mode) {
	case FLASHLIGHT_MODE_MIXED:
	case FLASHLIGHT_MODE_TORCH:
		if (fi->fled_status == RT5058_FLED_TORCH)
			break;

		rt5058_reg_write(fi->client, RT5058_REG_STRBEN, 0x00);
		if (fi->pdata->torch1_en)
			regval |= fi->ctrl[0]->en_bit;
		if (fi->pdata->torch2_en)
			regval |= fi->ctrl[1]->en_bit;

		if (fi->pdata->control_mode == FLED_PIN_CONTROL) {
			rt5058_reg_write(fi->client, RT5058_REG_STRBEN, regval);
			break;
		}

		/* 0-> Torch Mode  1->Strobe Mode */
		for (i = 0; i < RT5058_FLED_MAX; i++)
			ret = rt5058_clr_bits(fi->client,
				fi->ctrl[i]->mode_reg, RT5058_FLEDMODE_MASK);

		regval |= RT5058_FLED_ALLEN_MASK;

		pr_info("regval = 0x%02x\n", regval);
		ret = rt5058_reg_write(fi->client, RT5058_REG_STRBEN, regval);
		if (ret < 0)
			goto err_set_mode;
		fi->fled_status = RT5058_FLED_TORCH;
		fi->fled_en = 1;
		dev_info(fi->dev, "set to torch mode\n");
		break;
	case FLASHLIGHT_MODE_FLASH:
		rt5058_reg_write(fi->client, RT5058_REG_STRBEN, 0x00);

		if (fi->pdata->flash1_en)
			regval |= fi->ctrl[0]->en_bit;
		if (fi->pdata->flash2_en)
			regval |= fi->ctrl[1]->en_bit;

		if (fi->pdata->control_mode == FLED_PIN_CONTROL) {
			rt5058_reg_write(fi->client, RT5058_REG_STRBEN, regval);
			break;
		}

		for (i = 0; i < RT5058_FLED_MAX; i++)
			ret = rt5058_set_bits(fi->client, fi->ctrl[i]->mode_reg,
							RT5058_FLEDMODE_MASK);

		regval |= RT5058_FLED_FLASH_MASK;
		regval |= RT5058_FLED_ALLEN_MASK;

		pr_info("regval = 0x%02x\n", regval);
		ret = rt5058_reg_write(fi->client, RT5058_REG_STRBEN, regval);
		if (ret < 0)
			goto err_set_mode;

		fi->fled_status = RT5058_FLED_STROBE;
		fi->fled_en = 1;

		dev_info(fi->dev, "set to flash mode\n");
		break;
	case FLASHLIGHT_MODE_OFF:
		if (fi->fled_status == RT5058_FLED_OFF)
			break;

		rt5058_reg_write(fi->client, RT5058_REG_STRBEN, 0x00);

		fi->fled_status = RT5058_FLED_OFF;
		fi->fled_en = 0;

		dev_info(fi->dev, "set to off mode\n");
		break;
	case FLASHLIGHT_MODE_MAX:
	default:
		ret = -EINVAL;
		break;
	}
	fi->base.flashlight_dev->props.mode = mode;
	return 0;

err_set_mode:
	dev_err(fi->dev, "set fled mode fail\n");
	return ret;
}

static int rt5058_fled_get_mode(struct rt_fled_info *info)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;

	return fi->base.flashlight_dev->props.mode;
}

static int rt5058_fled_torch_current_list(
				struct rt_fled_info *info, int selector)
{
	return (selector >= info->flashlight_dev->props.torch_max_brightness)
			? -EINVAL :
			  RT5058_MIN_TOR + selector * RT5058_TOR_STEP;
}

static int rt5058_fled_strobe_current_list(
				struct rt_fled_info *info, int selector)
{
	return (selector >=
			info->flashlight_dev->props.strobe_max_brightness)
			? -EINVAL :
			  RT5058_MIN_STRB + selector * RT5058_STRB_STEP;
}

static int rt5058_fled_timeout_level_list(
				struct rt_fled_info *info, int selector)
{
	return (selector >= RT5058_TIMEOUT_SIZE) ? -EINVAL :
			RT5058_MIN_TIMEOUT + selector * RT5058_TIMEOUT_STEP;
}

static int rt5058_fled_lv_protection_list(
			struct rt_fled_info *info, int selector)
{
	return 0;
}

static int rt5058_fled_strobe_timeout_list(
			struct rt_fled_info *info, int selector)
{
	return (selector > RT5058_STRBTO_SIZE) ? -EINVAL :
		RT5058_MIN_STRBTO + selector * RT5058_STRBTO_STEP;
}

static int rt5058_fled_set_torch_current_sel(
			struct rt_fled_info *info, int selector)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret, i;

	for (i = 0; i < RT5058_FLED_MAX; i++)
		ret = rt5058_assign_bits(fi->client, fi->ctrl[i]->tor_cur_reg,
						RT5058_TORCHCUR_MASK, selector);
	if (ret == 0)
		fi->base.flashlight_dev->props.torch_brightness = selector;
	return ret;
}

static int rt5058_fled_set_strobe_current_sel(
			struct rt_fled_info *info, int selector)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret;

	ret = rt5058_assign_bits(fi->client, 0x44,
						RT5058_STRBCUR_MASK, selector);
	ret = rt5058_assign_bits(fi->client, 0x48,
						RT5058_STRBCUR_MASK, selector);
	if (ret == 0)
		fi->base.flashlight_dev->props.strobe_brightness = selector;
	return ret;
}

static int rt5058_fled_set_timeout_level_sel(
			struct rt_fled_info *info, int selector)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret, i;

	for (i = 0; i < RT5058_FLED_MAX; i++)
		ret = rt5058_assign_bits(fi->client, fi->ctrl[i]->timeout_reg,
			RT5058_TIMEOUT_MASK, selector<<RT5058_TIMEOUT_SHIFT);

	return ret;
}

static int rt5058_fled_set_lv_protection_sel(
			struct rt_fled_info *info, int selector)
{
	return 0;
}

static int rt5058_fled_set_strobe_timeout_sel(
			struct rt_fled_info *info, int selector)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret, i;

	for (i = 0; i < RT5058_FLED_MAX; i++)
		ret = rt5058_assign_bits(fi->client, fi->ctrl[i]->strb_to_reg,
					RT5058_STRBTIMEOUT_MASK, selector);
	return ret;
}

static int rt5058_fled_get_torch_current_sel(struct rt_fled_info *info)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret = 0;

	ret = rt5058_reg_read(fi->client, fi->ctrl[0]->tor_cur_reg);
	if (ret >= 0)
		ret &= RT5058_TORCHCUR_MASK;
	return ret;
}

static int rt5058_fled_get_strobe_current_sel(struct rt_fled_info *info)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret = 0;

	ret = rt5058_reg_read(fi->client, fi->ctrl[0]->strb_cur_reg);
	if (ret >= 0)
		ret &= RT5058_STRBCUR_MASK;
	return ret;
}

static int rt5058_fled_get_timeout_level_sel(struct rt_fled_info *info)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret;

	ret = rt5058_reg_read(fi->client, fi->ctrl[0]->timeout_reg);
	if (ret >= 0) {
		ret &= RT5058_TIMEOUT_MASK;
		ret >>= RT5058_TIMEOUT_SHIFT;
	}
	return ret;
}

static int rt5058_fled_get_lv_protection_sel(struct rt_fled_info *info)
{
	return 0;
}

static int rt5058_fled_get_strobe_timeout_sel(struct rt_fled_info *info)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret = 0;

	ret = rt5058_reg_read(fi->client, fi->ctrl[0]->strb_to_reg);
	if (ret >= 0)
		ret &= RT5058_STRBTIMEOUT_MASK;
	return ret;
}

static void rt5058_fled_shutdown(struct rt_fled_info *info)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;
	int ret, i;

	for (i = 0; i < RT5058_FLED_MAX; i++)
		rt5058_clr_bits(fi->client,
			RT5058_REG_STRBEN, fi->ctrl[i]->en_bit);

	ret = rt5058_clr_bits(fi->client,
		RT5058_REG_STRBEN, RT5058_FLED_ALLEN_MASK);
	if (ret < 0) {
		dev_err(fi->dev, "clr fled all en fail\n");
		return;
	}
}

static struct rt_fled_hal rt5058_fled_hal = {
	.fled_init = rt5058_fled_init,
	.fled_suspend = rt5058_fled_suspend,
	.fled_resume = rt5058_fled_resume,
	.fled_set_mode = rt5058_fled_set_mode,
	.fled_get_mode = rt5058_fled_get_mode,
	.fled_strobe = rt5058_fled_strobe,
	.fled_troch_current_list = rt5058_fled_torch_current_list,
	.fled_strobe_current_list = rt5058_fled_strobe_current_list,
	.fled_timeout_level_list = rt5058_fled_timeout_level_list,
	.fled_lv_protection_list = rt5058_fled_lv_protection_list,
	.fled_strobe_timeout_list = rt5058_fled_strobe_timeout_list,
	/* method to set */
	.fled_set_torch_current_sel = rt5058_fled_set_torch_current_sel,
	.fled_set_strobe_current_sel = rt5058_fled_set_strobe_current_sel,
	.fled_set_timeout_level_sel = rt5058_fled_set_timeout_level_sel,
	.fled_set_lv_protection_sel = rt5058_fled_set_lv_protection_sel,
	.fled_set_strobe_timeout_sel = rt5058_fled_set_strobe_timeout_sel,
	/* method to get */
	.fled_get_torch_current_sel = rt5058_fled_get_torch_current_sel,
	.fled_get_strobe_current_sel = rt5058_fled_get_strobe_current_sel,
	.fled_get_timeout_level_sel = rt5058_fled_get_timeout_level_sel,
	.fled_get_lv_protection_sel = rt5058_fled_get_lv_protection_sel,
	.fled_get_strobe_timeout_sel = rt5058_fled_get_strobe_timeout_sel,

	.fled_shutdown = rt5058_fled_shutdown,
};

static struct flashlight_properties rt5058_fled_props = {
	.type = FLASHLIGHT_TYPE_LED,
	.torch_brightness = 2,
	.torch_max_brightness = RT5058_TOR_SIZE - 1,
	.strobe_brightness = 13,
	.strobe_max_brightness = RT5058_STRB_SIZE - 1,
	.strobe_delay = 2,
	.strobe_timeout = 544,
	.alias_name = "rt5058-fled",
};

#define RT5058_FLED_DEVICE(_id)				\
{							\
	.mode_reg	= RT5058_FLED##_id,		\
	.strb_bit	= RT5058_STRBBIT##_id,		\
	.en_bit		= RT5058_ENBIT##_id,		\
	.tor_cur_reg	= RT5058_TORCUR##_id,		\
	.strb_cur_reg	= RT5058_STRBCUR##_id,		\
	.timeout_reg	= RT5058_FLEDTIMEOUT##_id,	\
	.strb_to_reg	= RT5058_STRBTIMEOUT##_id,	\
}

static struct rt5058_fled_ctrl rt5058_fled_ctrls[] = {
	RT5058_FLED_DEVICE(0),
	RT5058_FLED_DEVICE(1),
};

static int rt_parse_dt(struct device *dev,
			struct rt5058_fled_platform_data *pdata)
{
	#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	u32 val[2];

	if (of_property_read_u32(np, "rt,control_mode", val) >= 0)
		pdata->control_mode = val[0] ?
				FLED_PIN_CONTROL : FLED_I2C_CONTROL;
	else {
		dev_info(dev, "set default control mode (I2C)\n");
		pdata->control_mode = FLED_I2C_CONTROL;
	}

	if (of_property_read_u32_array(np, "rt,torch_en", val, 2) >= 0) {
		if (val[0])
			pdata->torch1_en = 1;
		if (val[1])
			pdata->torch2_en = 1;
	} else {
		dev_info(dev, "use 2 led in torch mode\n");
		pdata->torch1_en = pdata->torch2_en = 1;
	}

	if (of_property_read_u32_array(np, "rt,flash_en", val, 2) >= 0) {
		if (val[0])
			pdata->flash1_en = 1;
		if (val[1])
			pdata->flash2_en = 1;
	} else {
		dev_info(dev, "use 2 led in flash mode\n");
		pdata->flash1_en = pdata->torch2_en = 1;
	}

	if (of_property_read_u32_array(np, "rt,fled_irq_mask", val, 2) >= 0) {
		pdata->irq_mask[0] = val[0];
		pdata->irq_mask[1] = val[1];
	} else {
		dev_info(dev, "use default irq mask\n");
		pdata->irq_mask[0] = 0x30;
		pdata->irq_mask[1] = 0xc0;
	}
	#endif /* CONFIG_OF */
	return 0;
}

#ifdef CONFIG_RT_REGMAP
static ssize_t rt5058_fled_write(struct file *file,
			const char __user *ubuf, size_t count, loff_t *pos)
{
	struct flashlight_device *flash;
	int ret;
	char buf[200] = {0};
	unsigned long val;

	flash = find_flashlight_by_name("rt-flash-led");
	if (!flash)
		return count;
	simple_write_to_buffer(buf, sizeof(buf), pos, ubuf, count);
	ret = kstrtoul(buf, 16, &val);
	switch (val) {
	case 0:
		flashlight_set_mode(flash, FLASHLIGHT_MODE_OFF);
		break;
	case 1: /* torch */
		flashlight_set_torch_brightness(flash, 7);
		flashlight_set_mode(flash, FLASHLIGHT_MODE_TORCH);
		break;
	case 2: /* strobe */
		flashlight_set_strobe_timeout(flash, 800, 900);
		flashlight_set_torch_brightness(flash, 0);
		flashlight_set_strobe_brightness(flash, 20);
		flashlight_set_mode(flash, FLASHLIGHT_MODE_FLASH);
		break;
	default:
		flashlight_set_mode(flash, FLASHLIGHT_MODE_OFF);
		break;
	}

	return count;
}

static int rt5058_fled_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations rt5058_fled_fops = {
	.open = rt5058_fled_open,
	.write = rt5058_fled_write,
};
#endif /* CONFIG_RT_REGMAP */

#if RT5058_FLED_USE_NESTED_IRQ
static irqreturn_t rt5058_fled1_short(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled2_short(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreutrn_t rt5058_fled_tx(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled_lvf(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled_torpin(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled_strbpin(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled1_tor(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled2_tor(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled1_strbto(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled2_strbto(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled1_strobe(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5058_fled2_strobe(int irq, void *data)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)data;

	dev_info(fi->dev, "IRQ: %s\n", __func__);
	return IRQ_HANDLED;
}

static const struct rt5058_irq_handler rt5058_irq_handler[] = {
	{ .irq_name = "FLED_STRBPIN", .irq_handler = rt5058_fled_strbpin},
	{ .irq_name = "FLED_TORPIN", .irq_handler = rt5058_fled_torpin},
	{ .irq_name = "FLED_TX", .irq_handler = rt5058_fled_tx},
	{ .irq_name = "FLED_LVF", .irq_handler = rt5058_fled_lvf},
	{ .irq_name = "FLED_LED2_SHORT", .irq_handler = rt5058_fled2_short},
	{ .irq_name = "FLED_LED1_SHORT", .irq_handler = rt5058_fled1_short},
	{ .irq_name = "FLED_LED2_STRB", .irq_handler = rt5058_fled2_strobe},
	{ .irq_name = "FLED_LED1_STRB", .irq_handler = rt5058_fled1_strobe},
	{ .irq_name = "FLED_LED2_STRB_TO", .irq_handler = rt5058_fled2_strbto},
	{ .irq_name = "FLED_LED1_STRB_TO", .irq_handler = rt5058_fled1_strbto},
	{ .irq_name = "FLED_LED2_TOR", .irq_handler = rt5058_fled2_tor},
	{ .irq_name = "FLED_LED1_TOR", .irq_handler = rt5058_fled1_tor},
};

static int rt5058_fled_irqinit(struct platform_device *pdev)
{
	struct rt5058_fled_info *fi = platform_get_drvdata(pdev);
	int i = 0, ret = 0;
	const char *irq_name = NULL;

	pr_info("%s\n", __func__);
	rt5058_clr_bits(fi->client, RT5058_REG_IRQMSK, RT5058_FLED_IRQMASK);

	for (i = 0; i < ARRAY_SIZE(rt5058_irq_handler); i++) {
		irq_name = rt5058_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
			rt5058_irq_handler[i].irq_handler,
			IRQF_TRIGGER_NONE, irq_name, fi);
		if (ret < 0) {
			dev_err(&pdev->dev, "request %s fail\n", irq_name);
			goto out_irq_init;
		}
	}
	return 0;
out_irq_init:
	while (--i >= 0) {
		irq_name = rt5058_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		devm_free_irq(&pdev->dev, ret, fi);
	}
	return -EINTR;
}

static void rt5058_fled_irq_deinit(struct platform_device *pdev)
{
	struct rt5058_fled_info *fi = platform_get_drvdata(pdev);
	int i = 0, ret = 0;
	const char *irq_name = NULL;

	pr_info("%s\n", __func__);
	rt5058_set_bits(fi->client, RT5058_REG_IRQMSK, RT5058_FLED_IRQMASK);
	for (i = 0; i < ARRAY_SIZE(rt5058_irq_handler); i++) {
		irq_name = rt5058_irq_handler[i].irq_name;
		ret = platform_get_irq_byname(pdev, irq_name);
		if (ret < 0)
			continue;
		devm_free_irq(&pdev->dev, ret, fi);
	}
}
#else
static void rt5058_fledirq_handler(void *info, int eventno)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)info;

	dev_info(fi->dev, "%s: eventno = %d\n", __func__, eventno);

	switch (eventno) {
	case FLEDEVENT_STRBPIN:
		pr_info("IRQ: fled strobe pin\n");
		break;
	case FLEDEVENT_TORPIN:
		pr_info("IRQ: fled torch pin\n");
		break;
	case FLEDEVENT_TX:
		pr_info("IRQ: fled tx event\n");
		break;
	case FLEDEVENT_LVF:
		pr_info("IRQ: fled low VF\n");
		break;
	case FLEDEVENT_LED2SHORT:
		pr_info("IRQ: fled led2 short\n");
		break;
	case FLEDEVENT_LED1SHORT:
		pr_info("IRQ: fled led1 short\n");
		break;
	case FLEDEVENT_LED2STRB:
		pr_info("IRQ: fled led2 strobe\n");
		break;
	case FLEDEVENT_LED1STRB:
		pr_info("IRQ: fled led1 strobe\n");
		break;
	case FLEDEVENT_LED2STRBTO:
		pr_info("IRQ: fled led2 strobe timeout\n");
		break;
	case FLEDEVENT_LED1STRBTO:
		pr_info("IRQ: fled led1 strobe\n");
		break;
	case FLEDEVENT_LED2TOR:
		pr_info("IRQ: fled led2 torch\n");
		break;
	case FLEDEVENT_LED1TOR:
		pr_info("IRQ: fled led1 torch\n");
		break;
	}
}

static rt_irq_handler rt_fledirq_handler[FLEDEVENT_MAX] = {
	[FLEDEVENT_STRBPIN] = rt5058_fledirq_handler,
	[FLEDEVENT_TORPIN] = rt5058_fledirq_handler,
	[FLEDEVENT_TX] = rt5058_fledirq_handler,
	[FLEDEVENT_LVF] = rt5058_fledirq_handler,
	[FLEDEVENT_LED2SHORT] = rt5058_fledirq_handler,
	[FLEDEVENT_LED1SHORT] = rt5058_fledirq_handler,
	[FLEDEVENT_LED2STRB] = rt5058_fledirq_handler,
	[FLEDEVENT_LED1STRB] = rt5058_fledirq_handler,
	[FLEDEVENT_LED2STRBTO] = rt5058_fledirq_handler,
	[FLEDEVENT_LED1STRBTO] = rt5058_fledirq_handler,
	[FLEDEVENT_LED2TOR] = rt5058_fledirq_handler,
	[FLEDEVENT_LED1TOR] = rt5058_fledirq_handler,
};

static irqreturn_t rt5058_fled_irq_handler(int irqno, void *param)
{
	struct rt5058_fled_info *fi = (struct rt5058_fled_info *)param;
	u8 regval[2];
	int ret, i;

	RTINFO("%s\n", __func__);
	regval[0] = rt5058_reg_read(fi->client, RT5058_REG_FLED_IRQ1);
	regval[1] = rt5058_reg_read(fi->client, RT5058_REG_FLED_IRQ2);
	if (regval[0] < 0 || regval[1] < 0) {
		dev_err(fi->dev, "read fled irq fail\n");
		return IRQ_HANDLED;
	}

	ret = (regval[1] & ~(fi->pdata->irq_mask[1])) << 8 |
			(regval[0] & ~(fi->pdata->irq_mask[0]));

	RTINFO("irq_status & irq_mask = 0x%04x\n", ret);
	for (i = 0; i < FLEDEVENT_MAX; i++) {
		if ((ret & (1 << i)) && rt_fledirq_handler[i])
			rt_fledirq_handler[i](fi, i);
	}
	return IRQ_HANDLED;
}

static int rt5058_fled_irqinit(struct platform_device *pdev)
{
	struct rt5058_fled_info *fi = platform_get_drvdata(pdev);
	int ret;

	pr_info("%s\n", __func__);
	rt5058_clr_bits(fi->client, RT5058_REG_IRQMSK, RT5058_FLED_IRQMASK);
	ret = platform_get_irq_byname(pdev, "FLED_IRQ");
	if (ret < 0)
		return ret;
	ret = devm_request_threaded_irq(&pdev->dev, ret , NULL,
				rt5058_fled_irq_handler,
				IRQF_TRIGGER_NONE, "FLED_IRQ", fi);
	if (ret < 0) {
		dev_err(fi->dev, "request FLED_IRQ fail\n");
		return ret;
	}

	ret = rt5058_reg_write(fi->client,
			RT5058_REG_FLED_MASK1, fi->pdata->irq_mask[0]);
	ret = rt5058_reg_write(fi->client,
			RT5058_REG_FLED_MASK2, fi->pdata->irq_mask[1]);
	return ret;
}

static void rt5058_fled_irq_deinit(struct platform_device *pdev)
{
	struct rt5058_fled_info *fi = platform_get_drvdata(pdev);
	int ret;

	pr_info("%s\n", __func__);
	rt5058_set_bits(fi->client, RT5058_REG_IRQMSK, RT5058_FLED_IRQMASK);
	ret = platform_get_irq_byname(pdev, "FLED_IRQ");
	if (ret < 0)
		return;
	devm_free_irq(&pdev->dev, ret, fi);
}
#endif /* RT5058_FLED_USE_NESTED_IRQ */

static void rt5058_fled_platform_data_check(struct rt5058_fled_info *fi)
{
	pr_info("%s\n", __func__);
	RTINFO("control_mode = %s\n", fi->pdata->control_mode ? "PIN":"I2C");
	RTINFO("led in torch mode = <%d %d>\n",
				fi->pdata->torch1_en, fi->pdata->torch2_en);
	RTINFO("led in strobe mode = <%d %d>\n",
				fi->pdata->flash1_en, fi->pdata->flash2_en);
}

static int rt5058_fled_probe(struct platform_device *pdev)
{
	struct rt5058_mfd_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct rt5058_fled_platform_data *fdata;
	struct rt5058_fled_info *fi;
	bool use_dt = pdev->dev.of_node;
	int ret = 0;

	pr_info("%s\n", __func__);

	if (use_dt) {
		fdata = devm_kzalloc(&pdev->dev, sizeof(*fdata), GFP_KERNEL);
		if (!fdata) {
			dev_err(&pdev->dev, "fail to allocate memory\n");
			return -ENOMEM;
		}
		rt_parse_dt(&pdev->dev, fdata);
	} else {
		dev_err(&pdev->dev, "no dts node\n");
		return -ENODEV;
	}

	fi = devm_kzalloc(&pdev->dev, sizeof(*fi), GFP_KERNEL);
	if (!fi) {
		dev_err(&pdev->dev, "fail to allocate memory\n");
		return -ENOMEM;
	}

	fi->client = chip->client;
	fi->dev = &pdev->dev;
	fi->pdata = fdata;
	rt5058_fled_platform_data_check(fi);

	fi->ctrl[0] = &rt5058_fled_ctrls[0];
	fi->ctrl[1] = &rt5058_fled_ctrls[1];
	platform_set_drvdata(pdev, fi);
	chip->fled_info = fi;

	rt_fled_pdev.dev.parent = &pdev->dev;

	fi->base.hal = &rt5058_fled_hal;
	fi->base.init_props = &rt5058_fled_props;

	ret = platform_device_register(&rt_fled_pdev);
	if (ret < 0)
		goto out;

	ret = rt5058_fled_irqinit(pdev);
	if (ret < 0) {
		dev_err(fi->dev, "flashlight led irq init fail\n");
		goto irq_err;
	}

	#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_add_debugfs(chip->m_dev,
			"led_test", 0644, chip, &rt5058_fled_fops);
	if (ret < 0) {
		dev_err(fi->dev, "add led_test debugfs node fail\n");
		goto dbg_err;
	}
	#endif /* CONFIG_RT_REGMAP */

	dev_info(&pdev->dev, "rt5058 flash led successfully loaded\n");

	return 0;
#ifdef CONFIG_RT_REGMAP
dbg_err:
	rt5058_fled_irq_deinit(pdev);
#endif
irq_err:
	platform_device_unregister(&rt_fled_pdev);
out:
	dev_err(&pdev->dev, "flash led register fail\n");
	devm_kfree(&pdev->dev, fi);
	return ret;
}

static int rt5058_fled_remove(struct platform_device *pdev)
{
	struct rt5058_fled_info *fi = platform_get_drvdata(pdev);

	platform_device_unregister(&rt_fled_pdev);
	rt5058_fled_irq_deinit(pdev);
	if (fi)
		devm_kfree(&pdev->dev, fi);
	return 0;
}

static struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,rt5058-fled",},
};

static struct platform_driver rt5058_fled_driver = {
	.driver = {
		.name = RT5058_DEV_NAME "-fled",
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
	},
	.probe = rt5058_fled_probe,
	.remove = rt5058_fled_remove,
};

static int __init rt5058_fled_module_init(void)
{
	return platform_driver_register(&rt5058_fled_driver);
}

static void __exit rt5058_fled_module_exit(void)
{
	platform_driver_unregister(&rt5058_fled_driver);
}
device_initcall(rt5058_fled_module_init);
module_exit(rt5058_fled_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com");
MODULE_DESCRIPTION("Flashlight Led driver for RT5058");
MODULE_ALIAS("platform:" RT5058_DEVICE_NAME "-fled");
MODULE_VERSION("1.0.0_LG");
