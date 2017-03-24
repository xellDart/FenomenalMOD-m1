/*
 * TI RT4832 MFD Driver
 *
 * Copyright 2013 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_RT4832_H__
#define __MFD_RT4832_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

/* LED Control Registers */
#define RT4832_REG_LED_CTRL		0x0A
#define RT4832_BL_EN_MASK		BIT(0)
#define RT4832_BLED1_EN_MASK	BIT(3)
#define RT4832_BLED2_EN_MASK	BIT(4)
#define RT4832_FL_EN_MASK		BIT(1)
#define RT4832_FL_EN_SHIFT		1

#define RT4832_REG_BRT_MSB		0x04
#define RT4832_BRT_MSB_MASK		(BIT(0)|BIT(1)|BIT(2))
#define RT4832_BRT_MSB_SHIFT	0x08

#define RT4832_REG_BRT_LSB		0x05

/* Flash Config Registers */
/* LGE_TODO :
 * Fill out this area if needed */

/* DSV Periodic Register */
#define RT4832_REG_DSV_PERIOD	0x08
#define RT4832_PERIOD_EN_MASK	BIT(7)

/* DSV Control Register */
#define RT4832_REG_DSV_CTRL		0x0C
#define RT4832_VPOS_EN_MASK		BIT(6)
#define RT4832_VNEG_EN_MASK		BIT(3)

#define RT4832_MAX_REGISTERS	0x10
#define RT4832_MAX_BRIGHTNESS	0xFF

/*
 * struct rt4832_bl_platform_data
 * @name: Backlight driver name
 * @init_brightness: Initial brightness value
 * @blmap_size: size of brightness table
 * @blmap : adress of brightness table
 */
struct rt4832_backlight_platform_data {
	const char *name;
	u32 init_brightness;
	int blmap_size;
	u16 *blmap;
};

/*
 * struct rt4832_platform_data
 * @power state : 2 bit flag for state of backlight(0 bit)/flash(1 bit)
 * @bl_pdata: Backlight platform data
 */
struct rt4832_platform_data {
#if 0
/* Currently Power Control is done by Display Driver */
	int en_gpio;

	int ext_ctrl;
	int enp_gpio;
	int enn_gpio;
#endif

	char power_state;
	struct rt4832_backlight_platform_data *bl_pdata;
};

/*
 * struct rt4832
 * @dev: Parent device pointer
 * @regmap: Used for i2c communcation on accessing registers
 * @pdata: LMU platform specific data
 */
struct rt4832 {
	struct device *dev;
	struct regmap *regmap;
	struct rt4832_platform_data *pdata;
};

static inline struct rt4832 *dev_to_rt4832(struct device *dev)
{
	    return dev_get_drvdata(dev);
}
int rt4832_read_byte(struct rt4832 *rt4832, u8 reg, u8 *read);
int rt4832_write_byte(struct rt4832 *rt4832, u8 reg, u8 data);
int rt4832_update_bits(struct rt4832 *rt4832, u8 reg, u8 mask, u8 data);
int rt4832_periodic_ctrl(int enable);
#if 0
/* Currently Power Control is done by Display Driver */
int rt4832_power_ctrl(struct rt4832 *rt4832, int on);
int rt4832_dsv_ctrl(struct rt4832 *rt4832, int on);
#endif
#endif

