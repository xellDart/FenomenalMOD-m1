/*
 * RT8542 MFD Core Driver
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
#include <linux/mfd/rt8542.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#define RT8542_DEV_BL				\
{						\
	.name = "rt8542-backlight",		\
	.of_compatible = "richtek,rt8542-backlight",	\
}


#define RT8542_DEV_FL				\
{						\
	.name = "rt8542_flash",		\
	.of_compatible = "richtek,rt8542-flash",	\
}

static struct mfd_cell rt8542_devs[] = {
	/* Backlight */
	RT8542_DEV_BL,

	/* Flash */
	RT8542_DEV_FL,
};

int rt8542_read_byte(struct rt8542 *rt8542, u8 reg, u8 *read)
{
	int ret;
	unsigned int val;

	ret = regmap_read(rt8542->regmap, reg, &val);
	if (ret < 0)
		return ret;

	*read = (u8)val;
	return 0;
}
EXPORT_SYMBOL_GPL(rt8542_read_byte);

int rt8542_write_byte(struct rt8542 *rt8542, u8 reg, u8 data)
{
	return regmap_write(rt8542->regmap, reg, data);
}
EXPORT_SYMBOL_GPL(rt8542_write_byte);

int rt8542_update_bits(struct rt8542 *rt8542, u8 reg, u8 mask, u8 data)
{
	int ret;
	ret = regmap_update_bits(rt8542->regmap, reg, mask, data);
	return ret;
}
EXPORT_SYMBOL_GPL(rt8542_update_bits);

#if 0
/* Currently GPIOs are controlled by Display Driver */
int rt8542_power_ctrl(struct rt8542 *rt8542, int on)
{
	struct rt8542_platform_data *pdata = rt8542->pdata;

	if (!gpio_is_valid(pdata->en_gpio)){
		pr_err("%s,en gpio is not valid\n", __func__);
		return -1;
	}

	if (pdata->power_state > 0) {
		pr_err("%s, rt8542 is working, power_state=0x%8x\n",
				__func__, pdata->power_state);
		return -1;
	}

	gpio_set_value((pdata->en_gpio), on);

	return 0;
}
EXPORT_SYMBOL_GPL(rt8542_power_ctrl);
#endif

static int rt8542_parse_dt(struct device *dev, struct rt8542 *rt8542)
{
//	struct device_node *node = dev->of_node;
	struct rt8542_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

#if 0
/* Currently Power Control is done by Display Driver */
	pdata->en_gpio = of_get_named_gpio(node, "richtek,bl-en-gpio", 0);
	if (!gpio_is_valid(pdata->en_gpio)){
		pr_err("rt8542 enable gpio not specified\n");
		return -EPROBE_DEFER;
	}
#endif

	rt8542->pdata = pdata;
	return 0;
}

static struct regmap_config rt8542_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RT8542_MAX_REGISTERS,
};

static int rt8542_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct rt8542 *rt8542;
	struct device *dev = &cl->dev;
	struct rt8542_platform_data *pdata = dev_get_platdata(dev);
	int ret;

	rt8542 = devm_kzalloc(dev, sizeof(*rt8542), GFP_KERNEL);
	if (!rt8542)
		return -ENOMEM;

	rt8542->pdata = pdata;

	if (!pdata) {
		if (IS_ENABLED(CONFIG_OF))
			ret = rt8542_parse_dt(dev, rt8542);
		else
			ret = -ENODEV;
	}

#if 0
/* Currently Power Control is done by Display Driver */
	if (pdata->en_gpio && gpio_request(pdata->en_gpio, "rt8542_en") != 0)
		return -ENODEV;
#endif

	rt8542->regmap = devm_regmap_init_i2c(cl, &rt8542_regmap_config);
	if (IS_ERR(rt8542->regmap))
		return PTR_ERR(rt8542->regmap);

	rt8542->dev = &cl->dev;
	i2c_set_clientdata(cl, rt8542);

	return mfd_add_devices(dev, -1, rt8542_devs, ARRAY_SIZE(rt8542_devs),
			NULL, 0, NULL);
}

static int rt8542_remove(struct i2c_client *cl)
{
	struct rt8542 *rt8542 = i2c_get_clientdata(cl);
#if 0
/* Currently Power Control is done by Display Driver */
	struct rt8542_platform_data *pdata = rt8542->pdata;
	if (gpio_is_valid(pdata->en_gpio))
		gpio_free(pdata->en_gpio);
#endif
	mfd_remove_devices(rt8542->dev);

	return 0;
}

static const struct i2c_device_id rt8542_ids[] = {
	{ "rt8542", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt8542_ids);

#ifdef CONFIG_OF
static const struct of_device_id rt8542_of_match[] = {
	{ .compatible = "richtek,rt8542", },
	{ }
};
MODULE_DEVICE_TABLE(of, rt8542_of_match);
#endif

static struct i2c_driver rt8542_driver = {
	.probe = rt8542_probe,
	.remove = rt8542_remove,
	.driver = {
		.name = "rt8542",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt8542_of_match),
	},
	.id_table = rt8542_ids,
};
module_i2c_driver(rt8542_driver);

MODULE_DESCRIPTION("Richtek RT8542 MFD Core");
MODULE_AUTHOR("YJ Kim");
MODULE_LICENSE("GPL");
