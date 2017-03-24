/* drivers/mfd/rt5058-i2c.c
 * Source file for Richtek RT5058
 *
 * Copyright (C) 2015 Richtek Technology Corp.
 * Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/irqdomain.h>
#include <linux/delay.h>

#include <linux/mfd/rt5058/rt5058.h>
#include <linux/mfd/rt5058/rt5058-irq.h>
#include <linux/misc/rt-regmap.h>

char *rt5058_fuel_devname = "rt5058-fuelgauge";
char *rt5058_chg_devname = "rt5058-charger";
char *rt5058_batt_devname = "battery";
char *rt5058_usb_devname = "usb";

int rt5058_read_device(void *client, u32 reg, int len, void *dst)
{
	struct i2c_client *i2c = (struct i2c_client *)client;
	int ret = 0, count = 5;

	while (count) {
		if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(i2c, reg, len, dst);
			if (ret < 0)
				count--;
			else
				return ret;
		} else {
			ret = i2c_smbus_read_byte_data(i2c, reg);
			if (ret < 0)
				count--;
			else {
				*(u8 *)dst = (u8)ret;
				return ret;
			}
		}
		udelay(100);
	}
	return ret;
}

int rt5058_write_device(void *client, u32 reg, int len, const void *src)
{
	const u8 *data;
	struct i2c_client *i2c = (struct i2c_client *)client;
	int ret = 0, count = 5;

	while (count) {
		if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(i2c, reg, len, src);
			if (ret < 0)
				count--;
			else
				return ret;
		} else {
			data = src;
			ret = i2c_smbus_write_byte_data(i2c, reg, *data);
			if (ret < 0)
				count--;
			else
				return ret;
		}
		udelay(100);
	}
	return ret;
}

int rt5058_reg_read(struct i2c_client *i2c, u8 reg)
{
	struct rt5058_mfd_chip *chip = i2c_get_clientdata(i2c);
	u8 val = 0;
	int ret = 0;

	#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(chip->m_dev, reg, 1, &val);
	#else
	ret = rt5058_read_device(chip->client, reg, 1, &val);
	#endif /* CONFIG_RT_REGMAP */
	if (ret < 0) {
		dev_err(chip->dev, "rt5058 reg read fail\n");
		return ret;
	}
	return val;
}
EXPORT_SYMBOL(rt5058_reg_read);

int rt5058_block_read(struct i2c_client *i2c,
			u8 reg, int len, void *dst)
{
	struct rt5058_mfd_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(chip->m_dev, reg, len, dst);
#else
	ret = rt5058_read_device(chip->client, reg, len, dst);
#endif /* #ifdef CONFIG_RT_REGMAP */
	if (ret < 0)
		dev_err(chip->dev, "rt5058 block read fail\n");
	return ret;
}
EXPORT_SYMBOL(rt5058_block_read);

int rt5058_reg_write(struct i2c_client *i2c,
		u8 reg, const u8 data)
{
	struct rt5058_mfd_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(chip->m_dev, reg, 1, &data);
#else
	ret = rt5058_write_device(chip->client, reg, 1, &data);
#endif /* #ifdef CONFIG_RT_REGMAP */
	if (ret < 0)
		dev_err(chip->dev, "rt5058 reg write fail\n");
	return ret;
}
EXPORT_SYMBOL(rt5058_reg_write);

int rt5058_block_write(struct i2c_client *i2c,
			u8 reg, int len, const void *src)
{
	struct rt5058_mfd_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(chip->m_dev, reg, len, src);
#else
	ret = rt5058_write_device(chip->client, reg, len, src);
#endif /* #ifdef CONFIG_RT_REGMAP */
	if (ret < 0)
		dev_err(chip->dev, "rt5058 block write fail\n");
	return ret;
}
EXPORT_SYMBOL(rt5058_block_write);

int rt5058_assign_bits(struct i2c_client *i2c, u8 reg,
					u8 mask, const u8 data)
{
	struct rt5058_mfd_chip *chip = i2c_get_clientdata(i2c);
	u8 value = 0;
	int ret = 0;
#ifdef CONFIG_RT_REGMAP
	struct rt_reg_data rrd;

	ret = rt_regmap_update_bits(chip->m_dev, &rrd, reg, mask, data);
	value = 0;
#else
	down(&chip->semaphore);
	value = rt5058_reg_read(i2c, reg);
	if (value < 0) {
		up(&chip->semaphore);
		return value;
	}
	value &= ~mask;
	value |= data;
	ret = rt5058_reg_write(i2c, reg, value);
	up(&chip->semaphore);
#endif /* CONFIG_RT_REGMAP */
	return 0;
}
EXPORT_SYMBOL(rt5058_assign_bits);

int rt5058_set_bits(struct i2c_client *i2c,
					u8 reg, u8 mask)
{
	return rt5058_assign_bits(i2c, reg, mask, mask);
}
EXPORT_SYMBOL(rt5058_set_bits);

int rt5058_clr_bits(struct i2c_client *i2c,
					u8 reg, u8 mask)
{
	return rt5058_assign_bits(i2c, reg, mask, 0);
}
EXPORT_SYMBOL(rt5058_clr_bits);

int32_t rt5058_i2c_read_word(struct i2c_client *client,
				       uint8_t reg_addr)
{
	uint16_t data = 0;
	int ret;

	ret = rt5058_block_read(client, reg_addr, 2, &data);
	if (ret < 0)
		return ret;
	return (int32_t)be16_to_cpu(data);
}
EXPORT_SYMBOL(rt5058_i2c_read_word);

int32_t rt5058_i2c_write_word(struct i2c_client *client,
					uint8_t reg_addr, uint16_t data)
{
	int ret;

	data = cpu_to_be16(data);
	ret = rt5058_block_write(client, reg_addr, 2, (uint8_t *)&data);
	return ret;
}
EXPORT_SYMBOL(rt5058_i2c_write_word);

int rt5058_assign_bits16(struct i2c_client *client,
				uint8_t reg, uint16_t mask, uint16_t val)
{
	struct rt5058_mfd_chip *chip = i2c_get_clientdata(client);
	uint16_t temp;
	int ret;
#ifdef CONFIG_RT_REGMAP
	struct rt_reg_data rrd;

	ret = rt_regmap_update_bits(chip->m_dev, &rrd, reg, mask, val);
	temp = 0;
#else
	down(&chip->io_lock);
	ret = rt5058_block_read(client, reg, sizeof(temp), &temp);
	if (ret < 0) {
		up(&chip->io_lock);
		return ret;
	}
	temp = (temp & ~mask) | (val & mask);
	temp = be16_to_cpu(temp);
	ret = rt5058_block_write(client, reg, 2, &temp);
	up(&chip->io_lock);
#endif
	return ret;
}
EXPORT_SYMBOL(rt5058_assign_bits16);

static int rt5058_parse_dt(struct rt5058_mfd_chip *chip, struct device *dev)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	u32 val;
	int len = 0, ret;
	const char *bat_name = "battery";
	const char *usb_name = "usb";
	const char *fuel_name = "fuelgauge";
	const char *chg_name = "charger";

	ret = of_property_read_string(np, "rt,fuel_devname",
					(char const **)&fuel_name);
	if (ret >= 0) {
		len = strlen(fuel_name);
		rt5058_fuel_devname = devm_kzalloc(dev, len+1, GFP_KERNEL);
		if (!rt5058_fuel_devname)
			return -ENOMEM;
		strcpy(rt5058_fuel_devname, fuel_name);
	} else
		dev_info(dev, "use default fuel devname %s\n",
							rt5058_fuel_devname);

	ret = of_property_read_string(np, "rt,chg_devname",
					(char const **)&chg_name);
	if (ret >= 0) {
		len = strlen(chg_name);
		rt5058_chg_devname = devm_kzalloc(dev, len+1, GFP_KERNEL);
		if (!rt5058_chg_devname)
			return -ENOMEM;
		strcpy(rt5058_chg_devname, chg_name);
	} else
		dev_info(dev, "use default chg devname %s\n",
							rt5058_chg_devname);

	ret = of_property_read_string(np, "rt,batt_devname",
					(char const **)&bat_name);
	if (ret >= 0) {
		len = strlen(bat_name);
		rt5058_batt_devname = devm_kzalloc(dev, len+1, GFP_KERNEL);
		if (!rt5058_batt_devname)
			return -ENOMEM;
		strcpy(rt5058_batt_devname, bat_name);
	} else
		dev_info(dev, "use default batt devname %s\n",
							rt5058_batt_devname);

	ret = of_property_read_string(np, "rt,usb_devname",
					(char const **)&usb_name);
	if (ret >= 0) {
		len = strlen(usb_name);
		rt5058_usb_devname = devm_kzalloc(dev, len+1, GFP_KERNEL);
		if (!rt5058_usb_devname)
			return -ENOMEM;
		strcpy(rt5058_usb_devname, usb_name);
	} else
		dev_info(dev, "use default usb devname %s\n",
							rt5058_usb_devname);

	chip->irq_gpio = of_get_named_gpio(np, "rt,irq_pin", 0);
	if (of_property_read_u32(np, "rt,i2cstmr_rsttmr", &val) >= 0) {
		if (val > RT5058_I2CSTMR_2SEC)
			chip->i2cstmr_rsttmr = RT5058_I2CSTMR_2SEC;
		else
			chip->i2cstmr_rsttmr = val;
	} else {
		dev_info(dev, "use default i2c fast timer 0.5 sec\n");
		chip->i2cstmr_rsttmr = RT5058_I2CSTMR_0_5SEC;
	}
	RTINFO("fuel(%s), chg(%s), batt(%s), usb(%s)\n",
		rt5058_fuel_devname, rt5058_chg_devname, rt5058_batt_devname,
		rt5058_usb_devname);
	RTINFO("i2c safe timer = <%02x>\n", chip->i2cstmr_rsttmr);
#endif /* #ifdef CONFIG_OF */
	return 0;
}

static inline int rt5058_chip_reset(struct rt5058_mfd_chip *chip)
{
	uint8_t data = 0;

	data = rt5058_reg_read(chip->client, RT5058_REG_CORECTRL1);
	pr_err("%s: %02x:%02x\n", __func__, RT5058_REG_CORECTRL1, data);

	/* battery present, It is ok to set ALL_RST */
	if (data & RT5058_RESET_FLAG_MASK) {
		pr_err("%s: runing ALL_RST.\n", __func__);
		/* unlock reset function */
		rt5058_reg_write(chip->client, RT5058_REG_RSTPASCODE1, 0xa9);
		rt5058_reg_write(chip->client, RT5058_REG_RSTPASCODE2, 0x96);
		rt5058_set_bits(chip->client, RT5058_REG_CORECTRL2, 0x80);

		msleep(150);

		/* set CORECTRL2 bit0 */
		rt5058_assign_bits16(chip->client, RT5058_REG_CORECTRL1,
				RT5058_RESET_FLAG_MASK, RT5058_RESET_FLAG_MASK);
	}

#ifdef CONFIG_RT_REGMAP
	rt_regmap_cache_reload(chip->m_dev);
#endif /* CONFIG_RT_REGMAP */
	/* set i2c safe timer for SDA/SCL low active */
	rt5058_set_bits(chip->client, RT5058_REG_CORECTRL1,
					RT5058_I2CSTMR_RST_MASK);
	rt5058_assign_bits(chip->client, RT5058_REG_CORECTRL1,
		RT5058_I2CSTMR_RSTTMR_MASK,
		chip->i2cstmr_rsttmr << RT5058_I2CSTMR_RSTTMR_SHIFT);
	return 0;
}

static int rt5058_check_id(struct i2c_client *i2c)
{
	int ret;
	u8 data;

	ret = rt5058_read_device(i2c, RT5058_REG_DEVINFO, 1, &data);
	if (ret < 0) {
		dev_err(&i2c->dev, "read ID register fail\n");
		return ret;
	}
	RTINFO("DEV INFO = 0x%02x, REV = %d\n", data, data&RT5058_REV_MASK);
	return 0;
}

static int rt5058_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	struct rt5058_mfd_chip *chip;
	struct rt5058_mfd_platform_data *pdata;
	bool use_dt = i2c->dev.of_node;
	int ret = 0;

	dev_info(&i2c->dev, "%s\n", __func__);
	chip = devm_kzalloc(&i2c->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (use_dt) {
		pdata = devm_kzalloc(&i2c->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err_out;
		}
		rt5058_parse_dt(chip, &i2c->dev);
		i2c->dev.platform_data = pdata;
	} else {
		dev_err(&i2c->dev, "no dts node\n");
		return -ENODEV;
	}

	chip->client = i2c;
	chip->dev = &i2c->dev;
	i2c_set_clientdata(i2c, chip);

	/* check ID */
	ret = rt5058_check_id(i2c);
	if (ret < 0)
		goto err_out_id;

	sema_init(&chip->semaphore, 1);
	sema_init(&chip->io_lock, 1);
	sema_init(&chip->irq_lock, 1);
	sema_init(&chip->suspend_lock, 1);
	wake_lock_init(&chip->irq_wake, WAKE_LOCK_SUSPEND, "rt5058_irq_wake");
	ret = rt5058_regmap_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "rt5058 regmap init fail\n");
		goto err_init_regmap;
	}
	ret = rt5058_chip_reset(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Error chip reset\n");
		goto err_init_irq;
	}
	ret = rt5058_init_irq(chip);
	if (ret < 0) {
		dev_err(chip->dev,
			"Error : can't initialize RT5058 MFD irq\n");
		goto err_init_irq;
	}
	ret = rt5058_core_init(chip, pdata);
	if (ret < 0) {
		dev_err(chip->dev, "rt5058 core init fail\n");
		goto err_init_core;
	}

	dev_info(&i2c->dev, "driver successfully loaded\n");
	return 0;
err_init_core:
	rt5058_exit_irq(chip);
err_init_irq:
	rt5058_regmap_deinit(chip);
err_init_regmap:
err_out_id:
	if (use_dt)
		devm_kfree(&i2c->dev, pdata);
err_out:
	devm_kfree(&i2c->dev, chip);
	return ret;
}

static int rt5058_i2c_remove(struct i2c_client *i2c)
{
	struct rt5058_mfd_chip *chip = i2c_get_clientdata(i2c);
	bool use_dt = chip->dev->of_node;

	dev_info(&i2c->dev, "%s\n", __func__);

	if (!chip) {
		pr_err("%s(): chip is NULL\n", __func__);
		return 0;
	}
	BUG_ON(chip == NULL);
	rt5058_core_deinit(chip->dev);
	rt5058_exit_irq(chip);
	rt5058_regmap_deinit(chip);
	if (use_dt)
		devm_kfree(&i2c->dev, chip->dev->platform_data);
	devm_kfree(&i2c->dev, chip);
	return 0;
}

#ifdef CONFIG_PM
static int rt5058_i2c_suspend(struct device *dev)
{
	struct rt5058_mfd_chip *chip = dev_get_drvdata(dev);

	rt5058_irq_suspend(chip);
	down(&chip->suspend_lock);
	return 0;
}

static int rt5058_i2c_resume(struct device *dev)
{
	struct rt5058_mfd_chip *chip = dev_get_drvdata(dev);

	rt5058_irq_resume(chip);
	up(&chip->suspend_lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(rt5058_pm_ops, rt5058_i2c_suspend, rt5058_i2c_resume);
#endif /* #ifdef CONFIG_PM */

static const struct i2c_device_id rt5058_id_table[] = {
	{RT5058_DEV_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, rt5058_id_table);

static const struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,rt5058",},
	{},
};

static struct i2c_driver rt5058_i2c_driver = {
	.driver = {
		.name = RT5058_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
#ifdef CONFIG_PM
		.pm = &rt5058_pm_ops,
#endif /* #ifdef CONFIG_PM */
		   },
	.probe = rt5058_i2c_probe,
	.remove = rt5058_i2c_remove,
	.id_table = rt5058_id_table,
};


static int __init rt5058_i2c_init(void)
{
	return i2c_add_driver(&rt5058_i2c_driver);
}
subsys_initcall(rt5058_i2c_init);

static void __exit rt5058_i2c_exit(void)
{
	i2c_del_driver(&rt5058_i2c_driver);
}
module_exit(rt5058_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C Driver for Richtek RT5058");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_VERSION("1.0.0_LG");
