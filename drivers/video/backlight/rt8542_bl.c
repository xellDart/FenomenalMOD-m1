/*
 * RT8542 Backlight Driver
 *
 * Copyright 2015 LG Electronics Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mfd/rt8542.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#define DEFAULT_BL_NAME			"lcd-backlight"

struct rt8542_bl {
	struct device *dev;
	struct backlight_device *bl_dev;

	struct rt8542 *rt8542;
	struct rt8542_backlight_platform_data *pdata;

	struct pwm_device *pwm;
};

#define BL_OFF 0
#define BL_ON 1
static int backlight_status = BL_OFF;

struct backlight_device *rt8542_device;
static int cur_main_lcd_level;

static int rt8542_bl_enable(struct rt8542_bl *rt8542_bl, int enable)
{
	backlight_status = enable;
	pr_info("%s:%s\n", __func__, enable?"on":"off");

	rt8542_update_bits(rt8542_bl->rt8542, RT8542_REG_LED_CTRL, RT8542_BL_EN_MASK, enable);

	if(enable)
		rt8542_bl->rt8542->pdata->power_state |= BIT(0);
	else
		rt8542_bl->rt8542->pdata->power_state &= ~BIT(0);

	return 0;
}

static inline int rt8542_bl_set_brightness(struct rt8542_bl *rt8542_bl, int val)
{
	int ret;

	cur_main_lcd_level = val;

	ret = rt8542_write_byte(rt8542_bl->rt8542, RT8542_REG_BL_LINEAR, (u8)val);

	return ret;
}

void rt8542_lcd_backlight_set_level(int level) {
	struct rt8542_bl *rt8542_bl = bl_get_data(rt8542_device);
	int ret = 0;
	struct rt8542_backlight_platform_data *pdata = rt8542_bl->pdata;
	int cal_level;

	pr_info("%s : %d\n",__func__, level);
	rt8542_bl->bl_dev->props.brightness = level;

	if (level == 0) {
		if (backlight_status == BL_ON)
			ret = rt8542_bl_enable(rt8542_bl, 0);
		return;
	} else{
		if (backlight_status == BL_OFF)
			ret = rt8542_bl_enable(rt8542_bl, 1);
	}
	if (ret) {
		pr_err("%s DEBUG error enable or disable backlight\n", __func__);
	}

	if (level >= pdata->blmap_size)
		level = pdata->blmap_size - 1;

	if(pdata->blmap)
		cal_level = pdata->blmap[level];
	else
		cal_level = level;

	if (cal_level == cur_main_lcd_level)
		return;

	ret = rt8542_bl_set_brightness(rt8542_bl, cal_level);
	if (ret) {
		pr_err("%s DEBUG error set backlight\n", __func__);
	}
};
EXPORT_SYMBOL(rt8542_lcd_backlight_set_level);

static int rt8542_bl_update_status(struct backlight_device *bl_dev)
{
	rt8542_lcd_backlight_set_level(bl_dev->props.brightness);

	return 0;
}

static int rt8542_bl_get_brightness(struct backlight_device *bl_dev)
{
	return bl_dev->props.brightness;
}

static void rt8542_lcd_backlight_set_level_nomapping(int level){
	struct rt8542_bl *rt8542_bl = bl_get_data(rt8542_device);
	int ret;
	pr_err("%s : level is %d\n",__func__, level);

	if(level > RT8542_MAX_BRIGHTNESS)
		level = RT8542_MAX_BRIGHTNESS;

	ret = rt8542_bl_set_brightness(rt8542_bl, level);
	if (ret){
		pr_err("%s DEBUG error set backlight\n",__func__);
	}
};

static const struct backlight_ops rt8542_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = rt8542_bl_update_status,
	.get_brightness = rt8542_bl_get_brightness,
};

static int rt8542_bl_register(struct rt8542_bl *rt8542_bl)
{
	struct backlight_device *bl_dev;
	struct backlight_properties props;
	struct rt8542_backlight_platform_data *pdata = rt8542_bl->pdata;
	char name[20];

	props.type = BACKLIGHT_PLATFORM;
	props.brightness = pdata ? pdata->init_brightness : RT8542_MAX_BRIGHTNESS;
	props.max_brightness = RT8542_MAX_BRIGHTNESS;

	if (!pdata || !pdata->name)
		snprintf(name, sizeof(name), "%s", DEFAULT_BL_NAME);
	else
		snprintf(name, sizeof(name), "%s", pdata->name);

	bl_dev = backlight_device_register(name, rt8542_bl->dev, rt8542_bl,
					   &rt8542_bl_ops, &props);
	rt8542_device = bl_dev;
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);

	rt8542_bl->bl_dev = bl_dev;

	return 0;
}

static void rt8542_bl_unregister(struct rt8542_bl *rt8542_bl)
{
	if (rt8542_bl->bl_dev)
		backlight_device_unregister(rt8542_bl->bl_dev);
}

static ssize_t lcd_backlight_show_on_off(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "LCD Backlight is : %s\n",
			backlight_status?"ON":"OFF");

	return r;
}

static void rt8542_lcd_power_direct(int onoff){
	struct rt8542_bl *rt8542_bl = bl_get_data(rt8542_device);
	int ret;

	ret = rt8542_bl_enable(rt8542_bl, onoff);
	if (ret){
		pr_err("%s DEBUG error backlight power ctrl\n",__func__);
	}
};

static ssize_t lcd_backlight_store_on_off(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int on_off;

	if (!count)
		return -EINVAL;

	on_off = simple_strtoul(buf, NULL, 10);
	rt8542_lcd_power_direct(on_off);

	return count;
}
DEVICE_ATTR(bl_power, 0644, lcd_backlight_show_on_off,
		lcd_backlight_store_on_off);

static ssize_t lcd_backlight_show_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n",
			cur_main_lcd_level);

	return r;
}

static ssize_t lcd_backlight_store_level(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int level;

	if(!count)
		return -EINVAL;

	level = simple_strtoul(buf, NULL, 10);
	rt8542_lcd_backlight_set_level_nomapping(level);

	return count;
}
DEVICE_ATTR(bl_level, 0644, lcd_backlight_show_level,
		lcd_backlight_store_level);

static int rt8542_bl_parse_dt(struct device *dev, struct rt8542_bl *rt8542_bl)
{
	struct device_node *node = dev->of_node;
	struct rt8542_backlight_platform_data *pdata;
	int i;
	u32* array;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	of_property_read_string(node, "bl-name", &pdata->name);
	of_property_read_u32(node, "initial-brightness",
			    (u32 *)&pdata->init_brightness);

	of_property_read_u32(node, "blmap_size", &pdata->blmap_size);

	if(pdata->blmap_size){
		array = kzalloc(sizeof(u32) * pdata->blmap_size, GFP_KERNEL);
		if (!array) {
			pr_err("no more mem for array\n");
			return -ENOMEM;
		}
		of_property_read_u32_array(node, "blmap", array, pdata->blmap_size);
		pdata->blmap = kzalloc(sizeof(u16) * pdata->blmap_size, GFP_KERNEL);
		if (!pdata->blmap){
			pr_err("no more mem for blmap\n");
			kfree(array);
			return -ENOMEM;
		}

		for (i = 0; i < pdata->blmap_size; i++)
			pdata->blmap[i] = (u16)array[i];

		kfree(array);
	}else{
		pr_err("not defined blmap_size");
	}
	rt8542_bl->pdata = pdata;

	return 0;
}

static int rt8542_bl_probe(struct platform_device *pdev)
{
	struct rt8542 *rt8542 = dev_get_drvdata(pdev->dev.parent);
	struct rt8542_backlight_platform_data *pdata = rt8542->pdata->bl_pdata;
	struct rt8542_bl *rt8542_bl;
	int ret;

	rt8542_bl = devm_kzalloc(&pdev->dev, sizeof(*rt8542_bl), GFP_KERNEL);
	if (!rt8542_bl)
		return -ENOMEM;

	rt8542_bl->pdata = pdata;
	if (!rt8542_bl->pdata) {
		if (IS_ENABLED(CONFIG_OF))
			ret = rt8542_bl_parse_dt(&pdev->dev, rt8542_bl);
		else
			return -ENODEV;

		if (ret)
			return ret;
	}

	rt8542_bl->dev = &pdev->dev;
	rt8542_bl->rt8542 = rt8542;
	platform_set_drvdata(pdev, rt8542_bl);

	ret = rt8542_bl_register(rt8542_bl);
	if (ret) {
		dev_err(&pdev->dev, "register backlight err: %d\n", ret);
		return ret;
	}

	device_create_file(&pdev->dev, &dev_attr_bl_level);
	device_create_file(&pdev->dev, &dev_attr_bl_power);

	backlight_status = BL_ON;
	backlight_update_status(rt8542_bl->bl_dev);

	return 0;
}

static int rt8542_bl_remove(struct platform_device *pdev)
{
	struct rt8542_bl *rt8542_bl = platform_get_drvdata(pdev);
	struct backlight_device *bl_dev = rt8542_bl->bl_dev;

	bl_dev->props.brightness = 0;
	backlight_update_status(bl_dev);
	rt8542_bl_unregister(rt8542_bl);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rt8542_bl_of_match[] = {
	{ .compatible = "richtek,rt8542-backlight", },
	{ }
};
MODULE_DEVICE_TABLE(of, rt8542_bl_of_match);
#endif

static struct platform_driver rt8542_bl_driver = {
	.probe = rt8542_bl_probe,
	.remove = rt8542_bl_remove,
	.driver = {
		.name = "rt8542-backlight",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt8542_bl_of_match),
	},
};
module_platform_driver(rt8542_bl_driver);

MODULE_DESCRIPTION("Richtek RT8542 Backlight Driver");
MODULE_AUTHOR("YJ Kim");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rt8542-backlight");
