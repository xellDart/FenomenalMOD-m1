/*
 * RT4832 MFD Core Driver
 *
 * Copyright 2015 LG Electronics Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rt4832.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

static struct rt4832 *rt4832_base;

/* TODO : Modify if regulator control is required
#define RT4832_DEV_LCD_BIAS(_id)		\
{						\
	.name = "rt4832-regulator",		\
	.id   = _id,				\
	.of_compatible = "richtek,rt4832-regulator",	\
}
*/

#define RT4832_DEV_BL				\
{						\
	.name = "rt4832-backlight",		\
	.of_compatible = "richtek,rt4832-backlight",	\
}


#define RT4832_DEV_FL				\
{						\
	.name = "rt4832-flash",		\
	.of_compatible = "richtek,rt4832-flash",	\
}

static struct mfd_cell rt4832_devs[] = {
	/* Backlight */
	RT4832_DEV_BL,

	/* Flash */
	RT4832_DEV_FL,
};

int rt4832_read_byte(struct rt4832 *rt4832, u8 reg, u8 *read)
{
	int ret;
	unsigned int val;

	ret = regmap_read(rt4832->regmap, reg, &val);
	if (ret < 0)
		return ret;

	*read = (u8)val;
	return 0;
}
EXPORT_SYMBOL_GPL(rt4832_read_byte);

int rt4832_write_byte(struct rt4832 *rt4832, u8 reg, u8 data)
{
	return regmap_write(rt4832->regmap, reg, data);
}
EXPORT_SYMBOL_GPL(rt4832_write_byte);

int rt4832_update_bits(struct rt4832 *rt4832, u8 reg, u8 mask, u8 data)
{
	int ret;
	ret = regmap_update_bits(rt4832->regmap, reg, mask, data);
	return ret;
}
EXPORT_SYMBOL_GPL(rt4832_update_bits);

/* rt4832_periodic_ctrl - is set DSV periodic mode
 * @enable : input data, 0:sleep, 1:resume
 */
int rt4832_periodic_ctrl(int enable)
{
	int ret;

	if(rt4832_base == NULL)
		return 0;

	if (enable){
		ret = regmap_write(rt4832_base->regmap, 0x08, 0x1C);
		ret = regmap_write(rt4832_base->regmap, 0x0D, 0x67);
		mdelay(1);
		ret = regmap_write(rt4832_base->regmap, 0x0E, 0x64);
		ret = regmap_write(rt4832_base->regmap, 0x0F, 0x64);
	}else {
		ret = regmap_write(rt4832_base->regmap, 0x08, 0x9C);
		ret = regmap_write(rt4832_base->regmap, 0x0D, 0x68);
		mdelay(1);
		ret = regmap_write(rt4832_base->regmap, 0x0E, 0x66);
		ret = regmap_write(rt4832_base->regmap, 0x0F, 0x68);
	}
	if (ret)
		pr_info("%s write fail!! : %d\n", __func__, enable);
	else
		pr_info("RT4832 Periodic mode is %s\n", enable?"OFF":"ON");

	mdelay(20);

	return ret;
}
EXPORT_SYMBOL_GPL(rt4832_periodic_ctrl);

#if 0
/* Currently GPIOs are controlled by Display Driver */
int rt4832_power_ctrl(struct rt4832 *rt4832, int on)
{
	struct rt4832_platform_data *pdata = rt4832->pdata;

	if (!gpio_is_valid(pdata->en_gpio)){
		pr_err("%s,en gpio is not valid\n", __func__);
		return -1;
	}

	if (pdata->power_state > 0) {
		pr_err("%s, rt4832 is working, power_state=0x%8x\n",
				__func__, pdata->power_state);
		return -1;
	}

	gpio_set_value((pdata->en_gpio), on);

	return 0;
}
EXPORT_SYMBOL_GPL(rt4832_power_ctrl);

int rt4832_dsv_ctrl(struct rt4832 *rt4832, int on)
{
	struct rt4832_platform_data *pdata = rt4832->pdata;

	if(pdata->ext_ctrl) {
		if (!gpio_is_valid(pdata->enp_gpio)){
			pr_err("%s,en_p gpio is not valid\n", __func__);
			return -1;
		}

		if (!gpio_is_valid(pdata->enn_gpio)){
			pr_err("%s,en_n gpio is not valid\n", __func__);
			return -1;
		}

		gpio_set_value((pdata->enp_gpio), on);
		mdelay(1);
		gpio_set_value((pdata->enn_gpio), on);

	} else {
		/* TODO : Add source code for control DSV by I2C */
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rt4832_dsv_ctrl);
#endif

static int rt4832_parse_dt(struct device *dev, struct rt4832 *rt4832)
{
//	struct device_node *node = dev->of_node;
	struct rt4832_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

#if 0
/* Currently Power Control is done by Display Driver */
	pdata->en_gpio = of_get_named_gpio(node, "richtek,bl-en-gpio", 0);
	if (!gpio_is_valid(pdata->en_gpio)){
		pr_err("rt4832 enable gpio not specified\n");
		return -EPROBE_DEFER;
	}

	pdata->ext_ctrl = of_property_read_bool(node,"richtek,ext-io-ctrl");
	if(pdata->ext_ctrl) {
		pr_info("rt4832, DSV is controlled by external IO\n");
		pdata->enp_gpio = of_get_named_gpio(node, "richtek,enp-gpio", 0);
		if (!gpio_is_valid(pdata->enp_gpio)){
			pr_err("rt4832 en_p gpio not specified\n");
			return -EPROBE_DEFER;
		}

		pdata->enn_gpio = of_get_named_gpio(node, "richtek,enn-gpio", 0);
		if (!gpio_is_valid(pdata->enn_gpio)) {
			pr_err("rt4832 en_n gpio not specified\n");
			return -EPROBE_DEFER;
		}
	} else {
		pr_info("rt4832, DSV is controlled by I2C\n");
	}
#endif

	rt4832->pdata = pdata;
	return 0;
}

static struct regmap_config rt4832_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RT4832_MAX_REGISTERS,
};

static int rt4832_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct rt4832 *rt4832;
	struct device *dev = &cl->dev;
	struct rt4832_platform_data *pdata = dev_get_platdata(dev);
	int ret;

	rt4832 = devm_kzalloc(dev, sizeof(*rt4832), GFP_KERNEL);
	if (!rt4832)
		return -ENOMEM;

	rt4832->pdata = pdata;

	if (!pdata) {
		if (IS_ENABLED(CONFIG_OF))
			ret = rt4832_parse_dt(dev, rt4832);
		else
			ret = -ENODEV;
	}

#if 0
/* Currently Power Control is done by Display Driver */
	if (pdata->en_gpio && gpio_request(pdata->en_gpio, "rt4832_en") != 0)
		return -ENODEV;

	if (pdata->enp_gpio && gpio_request(pdata->enp_gpio, "rt4832_dsv_p") != 0)
		return -ENODEV;

	if (pdata->enn_gpio && gpio_request(pdata->enn_gpio, "rt4832_dsv_n") != 0)
		return -ENODEV;
#endif

	rt4832->regmap = devm_regmap_init_i2c(cl, &rt4832_regmap_config);
	if (IS_ERR(rt4832->regmap))
		return PTR_ERR(rt4832->regmap);

	rt4832->dev = &cl->dev;
	i2c_set_clientdata(cl, rt4832);

	rt4832_base = rt4832;

	return mfd_add_devices(dev, -1, rt4832_devs, ARRAY_SIZE(rt4832_devs),
			NULL, 0, NULL);
}

static int rt4832_remove(struct i2c_client *cl)
{
	struct rt4832 *rt4832 = i2c_get_clientdata(cl);
#if 0
/* Currently Power Control is done by Display Driver */
	struct rt4832_platform_data *pdata = rt4832->pdata;
	if (gpio_is_valid(pdata->en_gpio))
		gpio_free(pdata->en_gpio);

	if (gpio_is_valid(pdata->en_gpio))
		gpio_free(pdata->enp_gpio);

	if (gpio_is_valid(pdata->en_gpio))
		gpio_free(pdata->enn_gpio);
#endif
	mfd_remove_devices(rt4832->dev);

	return 0;
}

static const struct i2c_device_id rt4832_ids[] = {
	{ "rt4832", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt4832_ids);

#ifdef CONFIG_OF
static const struct of_device_id rt4832_of_match[] = {
	{ .compatible = "richtek,rt4832", },
	{ }
};
MODULE_DEVICE_TABLE(of, rt4832_of_match);
#endif

static struct i2c_driver rt4832_driver = {
	.probe = rt4832_probe,
	.remove = rt4832_remove,
	.driver = {
		.name = "rt4832",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt4832_of_match),
	},
	.id_table = rt4832_ids,
};
module_i2c_driver(rt4832_driver);

MODULE_DESCRIPTION("Richtek RT4832 MFD Core");
MODULE_AUTHOR("YJ Kim");
MODULE_LICENSE("GPL");
