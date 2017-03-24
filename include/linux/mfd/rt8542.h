/*
 * RT8542 MFD Driver
 *
 * Copyright 2015 LG Electronics Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_rt8542_H__
#define __MFD_rt8542_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

/* LED Control Registers */
#define RT8542_REG_BL_CTRL1		0x02
#define RT8542_BL_FULL_LOAD_MASK	(BIT(0)|BIT(1)|BIT(2))

#define RT8542_REG_BL_LINEAR	0x05

#define RT8542_REG_LED_CTRL		0x0A
#define RT8542_BL_EN_MASK		BIT(0)
#define RT8542_FL_EN_MASK		BIT(1)
#define RT8542_BLED1_EN_MASK	BIT(4)
#define RT8542_TORCH_FLASH_MASK BIT(2)
#define RT8542_BLED2_EN_MASK	BIT(3)
#define RT8542_FLED1_EN_MASK	BIT(6)
#define RT8542_FLED2_EN_MASK	BIT(5)

#define RT8542_MAX_REGISTERS	0x10
#define RT8542_MAX_BRIGHTNESS	0x7F

/*
 * struct rt8542_bl_platform_data
 * @name: Backlight driver name
 * @init_brightness: Initial brightness value
 * @blmap_size: size of brightness table
 * @blmap : adress of brightness table
 */
struct rt8542_backlight_platform_data {
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
struct rt8542_platform_data {
	char power_state;
	struct rt8542_backlight_platform_data *bl_pdata;
};

/*
 * struct rt8542
 * @dev: Parent device pointer
 * @regmap: Used for i2c communcation on accessing registers
 * @pdata: LMU platform specific data
 */
struct rt8542 {
	struct device *dev;
	struct regmap *regmap;
	struct rt8542_platform_data *pdata;
};

static inline struct rt8542 *dev_to_rt8542(struct device *dev)
{
	    return dev_get_drvdata(dev);
}
int rt8542_read_byte(struct rt8542 *rt8542, u8 reg, u8 *read);
int rt8542_write_byte(struct rt8542 *rt8542, u8 reg, u8 data);
int rt8542_update_bits(struct rt8542 *rt8542, u8 reg, u8 mask, u8 data);
#if 0
int rt4832_power_ctrl(struct rt4832 *rt4832, int on);
#endif
#endif
