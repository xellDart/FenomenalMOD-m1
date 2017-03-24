/*include/linux/mfd/rt5058/rt5058.h
 *  Include header file for Richtek RT5058 Core file
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _LINUX_MFD_RT5058_H
#define _LINUX_MFD_RT5058_H

#include <linux/irq.h>
#include <linux/semaphore.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>

#include <linux/misc/rt-regmap.h>

#define RT5058_FUEL_DEVNAME	"rt5058-fuelgauge"
#define RT5058_CHG_DEVNAME	"rt5058-charger"
#define RT5058_BATT_DEVNAME	"battery"
#define RT5058_AC_DEVNAME	"ac"
#define RT5058_USB_DEVNAME	"usb"
#define RT5058_DEV_NAME		"rt5058"

#define RT5058_VENDOR_ID	(0x8 << 4)
#define RT5058_VENDORID_MASK	0xf0
#define RT5058_REV_MASK		0x0f

/* RT5058 CORE */
#define RT5058_REG_DEVINFO	(0x00)
#define RT5058_REG_CORECTRL1	(0x01)
#define RT5058_REG_CORECTRL2	(0x02)
#define RT5058_REG_RSTPASCODE1	(0x03)
#define RT5058_REG_RSTPASCODE2	(0x04)
#define RT5058_REG_PASCODE1	(0x05)
#define RT5058_REG_PASCODE2	(0x06)
#define RT5058_REG_IRQIND	(0x0a)
#define RT5058_REG_IRQMSK	(0x0b)
#define RT5058_REG_IRQSET	(0x0c)

/* RT5058 MUIC */
#define RT5058_REG_MUICCTRL1	(0x10)
#define RT5058_REG_MUICADC	(0x11)
#define RT5058_REG_MUICTIMERSET	(0x12)
#define RT5058_REG_MUICDEV1	(0x13)
#define RT5058_REG_MUICDEV2	(0x14)
#define RT5058_REG_MUICDEV3	(0x15)
#define RT5058_REG_MUICBTN1	(0x16)
#define RT5058_REG_MUICBTN2	(0x17)
#define RT5058_REG_MUICMANSW1	(0x18)
#define RT5058_REG_MUICMANSW2	(0x19)
#define RT5058_REG_MUICSTBADC	(0x1a)
#define RT5058_REG_MUICCTRL2	(0x1b)
#define RT5058_REG_MUICCTRL3	(0x1c)
#define RT5058_REG_MUICCTRL4	(0x1d)
#define RT5058_REG_MUICUNSTBADC	(0x1e)
#define RT5058_REG_MUICHIDDEN1	(0x20)
#define RT5058_REG_MUICHIDDEN2	(0x21)

/* RT5058 CHARGER */
#define RT5058_REG_CHGHIDDENCTRL1	(0x25)
#define RT5058_REG_CHGHIDDENCTRL2	(0x26)
#define RT5058_REG_CHGHIDDENCTRL3	(0x27)
#define RT5058_REG_CHGHIDDENCTRL4	(0x28)
#define RT5058_REG_CHGHIDDENCTRL5	(0x29)
#define RT5058_REG_CHGHIDDENCTRL6	(0x2a)
#define RT5058_REG_CHGHIDDENCTRL7	(0x2b)
#define RT5058_REG_CHGHIDDENCTRL8	(0x2c)
#define RT5058_REG_CHGHIDDENCTRL9	(0x2d)
#define RT5058_REG_CHGSTAT		(0x30)
#define RT5058_REG_CHGCTRL1		(0x31)
#define RT5058_REG_CHGCTRL2		(0x32)
#define RT5058_REG_CHGCTRL3		(0x33)
#define RT5058_REG_CHGCTRL4		(0x34)
#define RT5058_REG_CHGCTRL5		(0x35)
#define RT5058_REG_CHGCTRL6		(0x36)
#define RT5058_REG_CHGCTRL7		(0x37)
#define RT5058_REG_CHGCTRL8		(0x38)
#define RT5058_REG_CHGCTRL9		(0x39)
#define RT5058_REG_CHGCTRL10		(0x3a)
#define RT5058_REG_CHGCTRL11		(0x3b)
#define RT5058_REG_CHGCTRL12		(0x3c)
#define RT5058_REG_CHGCTRL13		(0x3d)
#define RT5058_REG_CHGCTRL14		(0x3e)

/* RT5058 Flashlight LED */
#define RT5058_REG_FLEDCFG		(0x40)
#define RT5058_REG_STRBEN		(0x41)
#define RT5058_REG_FLED1CTRL		(0x42)
#define RT5058_REG_FLED1STRBCTRL1	(0x43)
#define RT5058_REG_FLED1STRBCTRL2	(0x44)
#define RT5058_REG_FLED1TORCTRL		(0x45)
#define RT5058_REG_FLED2CTRL		(0x46)
#define RT5058_REG_FLED2STRBCTRL1	(0x47)
#define RT5058_REG_FLED2STRBCTRL2	(0x48)
#define RT5058_REG_FLED2TORCTRL		(0x49)
#define RT5058_REG_FLEDVMIDTRK_CTRL1	(0x4a)
#define RT5058_REG_FLEDVMIDRTM		(0x4b)
#define RT5058_REG_FLEDVMIDTRK_CTRL2	(0x4c)
#define RT5058_REG_FLEDTASTAT		(0x4d)
#define RT5058_REG_FLEDHIDDEN		(0x4e)

/* RT5058 PMIC */
#define RT5058_REG_BUCK1CFG1		(0x50)
#define RT5058_REG_BUCK1CFG2		(0x51)
#define RT5058_REG_BUCK1VOUT		(0x52)
#define RT5058_REG_LDO1CFG		(0x53)
#define RT5058_REG_LDO1VOUT		(0x54)
#define RT5058_REG_LDO2CFG		(0x55)
#define RT5058_REG_LDO2VOUT		(0x56)
#define RT5058_REG_LDO3CFG		(0x57)
#define RT5058_REG_LDO3VOUT		(0x58)
#define RT5058_REG_SLDO1CFG		(0x59)
#define RT5058_REG_SLDO1VOUT		(0x5a)
#define RT5058_REG_SLDO2CFG		(0x5b)
#define RT5058_REG_SLDO2VOUT		(0x5c)
#define RT5058_REG_PMICSHDNCTRL1	(0x5d)
#define RT5058_REG_PMICSHDNCTRL2	(0x5e)
#define RT5058_REG_PMICOFFEVT		(0x5f)

/* RT5058 DIGITAL */
#define RT5058_REG_HIDDEN_PAS_CODE1	(0x60)
#define RT5058_REG_HIDDEN_PAS_CODE2	(0x61)
#define RT5058_REG_HIDDEN_PAS_CODE3	(0x62)
#define RT5058_REG_HIDDEN_PAS_CODE4	(0x63)

/* RT5058 IRQ MASK and Status */
#define RT5058_REG_MUIC_IRQ1		(0x70)
#define RT5058_REG_MUIC_IRQ2		(0x71)
#define RT5058_REG_MUIC_IRQ3		(0x72)
#define RT5058_REG_CHG_IRQ1		(0x74)
#define RT5058_REG_CHG_IRQ2		(0x75)
#define RT5058_REG_CHG_IRQ3		(0x76)
#define RT5058_REG_CHG_IRQ4		(0x77)
#define RT5058_REG_CHG_IRQ5		(0x78)
#define RT5058_REG_FLED_IRQ1		(0x79)
#define RT5058_REG_FLED_IRQ2		(0x7a)
#define RT5058_REG_PMIC_IRQ1		(0x7b)
#define RT5058_REG_PMIC_IRQ2		(0x7c)
#define RT5058_REG_MUIC_STAT1		(0x80)
#define RT5058_REG_MUIC_STAT2		(0x81)
#define RT5058_REG_MUIC_STAT3		(0x82)
#define RT5058_REG_CHG_STAT1		(0x84)
#define RT5058_REG_CHG_STAT2		(0x85)
#define RT5058_REG_CHG_STAT3		(0x86)
#define RT5058_REG_CHG_STAT4		(0x87)
#define RT5058_REG_CHG_STAT5		(0x88)
#define RT5058_REG_FLED_STAT1		(0x89)
#define RT5058_REG_FLED_STAT2		(0x8a)
#define RT5058_REG_PMIC_STAT1		(0x8b)
#define RT5058_REG_PMIC_STAT2		(0x8c)
#define RT5058_REG_MUIC_MASK1		(0x90)
#define RT5058_REG_MUIC_MASK2		(0x91)
#define RT5058_REG_MUIC_MASK3		(0x92)
#define RT5058_REG_CHG_MASK1		(0x94)
#define RT5058_REG_CHG_MASK2		(0x95)
#define RT5058_REG_CHG_MASK3		(0x96)
#define RT5058_REG_CHG_MASK4		(0x97)
#define RT5058_REG_CHG_MASK5		(0x98)
#define RT5058_REG_FLED_MASK1		(0x99)
#define RT5058_REG_FLED_MASK2		(0x9a)
#define RT5058_REG_PMIC_MASK1		(0x9b)
#define RT5058_REG_PMIC_MASK2		(0x9c)

/* RT5058 FUEL GAUGE */
#define RT5058_REG_FGOCV		(0xb0)
#define RT5058_REG_FGVBAT		(0xb1)
#define RT5058_REG_FGSOC		(0xb2)
#define RT5058_REG_FGCTRL		(0xb3)
#define RT5058_REG_FGVER		(0xb4)
#define RT5058_REG_FGCURR		(0xb5)
#define RT5058_REG_FGTEMP		(0xb6)
#define RT5058_REG_FGAI			(0xb7)
#define RT5058_REG_FGAV			(0xb8)
#define RT5058_REG_FGAT			(0xb9)
#define RT5058_REG_FGINTT		(0xba)
#define RT5058_REG_FGDSNCAP		(0xbb)
#define RT5058_REG_FGBATCAP		(0xbc)
#define RT5058_REG_FGCYCCNT		(0xbd)
#define RT5058_REG_FGCYCBUF		(0xbe)
#define RT5058_REG_FGBATDETV		(0xbf)
#define RT5058_REG_FGOPCFG1		(0xc0)
#define RT5058_REG_FGOPCFG2		(0xc1)
#define RT5058_REG_FGOPCFG3		(0xc2)
#define RT5058_REG_FGSOCCFG1		(0xc3)
#define RT5058_REG_FGAGEFCT		(0xc4)
#define RT5058_REG_FGVGCOMP1_2		(0xc5)
#define RT5058_REG_FGVGCOMP3_4		(0xc6)
#define RT5058_REG_FGCSCOMP1		(0xc7)
#define RT5058_REG_FGCSCOMP2		(0xc8)
#define RT5058_REG_FGCSCOMP3		(0xc9)
#define RT5058_REG_FGCSCOMP4		(0xca)
#define RT5058_REG_FGMFA		(0xcb)
#define RT5058_REG_FG_OTUTDET		(0xce)
#define RT5058_REG_FG_UVDET		(0xcf)
#define RT5058_REG_FG_OSDET		(0xd0)
#define RT5058_REG_FG_USDET		(0xd1)
#define RT5058_REG_FG_SLP_V		(0xd2)
#define RT5058_REG_FG_CURRCALI		(0xd3)
#define RT5058_REG_FG_VOLTCALI		(0xd4)
#define RT5058_REG_FG_OEPTH		(0xd5)
#define RT5058_REG_FG_DEADBAND		(0xd6)
#define RT5058_REG_FGSOCCFG2		(0xda)
#define RT5058_REG_FG_STATUS1		(0xe0)
#define RT5058_REG_FG_STATUS2		(0xe1)
#define RT5058_REG_FG_MASK1		(0xe2)
#define RT5058_REG_FG_MASK2		(0xe3)
#define RT5058_REG_FG_IRQ1		(0xe4)
#define RT5058_REG_FG_IRQ2		(0xe5)
#define RT5058_REG_FG_HIDDEN1		(0xe8)
#define RT5058_REG_FG_HIDDEN2		(0xe9)
#define RT5058_REG_FG_HIDDEN3		(0xea)
#define RT5058_REG_FG_HIDDEN4		(0xeb)
#define RT5058_REG_FG_HIDDEN5		(0xec)
#define RT5058_REG_FG_HIDDEN6		(0xed)
#define RT5058_REG_FG_HIDDEN7		(0xee)
#define RT5058_REG_FG_HIDDEN8		(0xef)

#define RT5058_RESET_FLAG_MASK		(0x01)

#define RT5058_MUIC_IRQ_REGS_NR	3
#define RT5058_CHG_IRQ_REGS_NR	5
#define RT5058_FLED_IRQ_REGS_NR	2
#define RT5058_PMIC_IRQ_REGS_NR	2
#define RT5058_FUEL_IRQ_REGS_NR 4

#define RT5058_IRQ_REGS_NR	\
	(RT5058_MUIC_IRQ_REGS_NR + \
	 RT5058_CHG_IRQ_REGS_NR + \
	 RT5058_FLED_IRQ_REGS_NR + \
	 RT5058_PMIC_IRQ_REGS_NR + \
	 RT5058_FUEL_IRQ_REGS_NR)

union rt5058_irq_maskstatus {
	struct {
		uint8_t muic_irq[RT5058_MUIC_IRQ_REGS_NR];
		uint8_t chg_irq[RT5058_CHG_IRQ_REGS_NR];
		uint8_t fled_irq[RT5058_FLED_IRQ_REGS_NR];
		uint8_t pmic_irq[RT5058_PMIC_IRQ_REGS_NR];
		uint8_t fuel_irq[RT5058_FUEL_IRQ_REGS_NR];
	};
	struct {
		uint8_t regs[RT5058_IRQ_REGS_NR];
	};
};

enum {
	RT5058_ID_BUCK1,
	RT5058_ID_LDO1,
	RT5058_ID_LDO2,
	RT5058_ID_LDO3,
	RT5058_ID_SLDO1,
	RT5058_ID_SLDO2,
	RT5058_MAX_REGULATOR,
};

enum { /* Temperature Status */
	RT_TEMP_COLD,
	RT_TEMP_COOL,
	RT_TEMP_NORMAL,
	RT_TEMP_WARM,
	RT_TEMP_HOT,
};

struct rt5058_irq_handler {
	const char *irq_name;
	irq_handler_t irq_handler;
};

struct rt5058_muic_info;
struct rt5058_fled_info;
struct rt5058_charger_info;

struct rt5058_mfd_chip {
	struct i2c_client *client;
	struct device *dev;
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *m_dev;
#endif /* CONFIG_RT_REGMAP */
	struct irq_domain *irq_domain;
	struct rt5058_mfd_platform_data *pdata;
	struct rt5058_fuelgauge_info *fuelgauge;
	struct rt5058_muic_info *muic_info;
	struct rt5058_fled_info *fled_info;
	struct rt5058_charger_info *chg_info;
	struct semaphore semaphore;
	struct semaphore io_lock;
	struct semaphore irq_lock;
	struct semaphore suspend_lock;
	struct wake_lock irq_wake;
	union rt5058_irq_maskstatus irq_mask;
	union rt5058_irq_maskstatus irq_event;
	int irq_gpio;
	int irq_base;
	int irq;
	int i2cstmr_rsttmr;
	u8 fuel_irq_bypass:1;
	u8 muic_irq_bypass:1;
	u8 fled_irq_bypass:1;
	u8 chg_irq_bypass:1;
	u8 pmic_irq_bypass:1;
	u8 irq_initialized:1;
	u8 irq_mask_changed:1;
};

struct rt5058_irq_enable_t {
	const char **irq_name;
	int irq_count;
};

struct rt5058_fuelgauge_platform_data;
struct rt5058_muic_platform_data;
struct rt5058_charger_platform_data;
struct rt5058_regulator_platform_data {
	struct regulator_init_data *regulator[RT5058_MAX_REGULATOR];
	struct rt5058_irq_enable_t *enable_irq[RT5058_MAX_REGULATOR];
};
struct rt5058_fled_platform_data;

struct rt5058_mfd_platform_data {
	struct rt5058_regulator_platform_data *regulator_platform_data;
	struct rt5058_fled_platform_data *fled_pdata;
	struct rt5058_charger_platform_data *chg_pdata;
	struct rt5058_muic_platform_data *muic_pdata;
	struct rt5058_fuelgauge_platform_data *fg_pdata;
	int irq_gpio;
	int irq_base;
};

extern char *rt5058_chg_devname;
extern char *rt5058_fuel_devname;
extern char *rt5058_batt_devname;
extern char *rt5058_usb_devname;
extern int rt5058_cv_mode;

extern int rt5058_read_device(void *client, u32 reg, int len, void *dst);
extern int rt5058_write_device(void *client,
				u32 reg, int len, const void *src);

extern int rt5058_reg_read(struct i2c_client *i2c, u8 reg);
extern int rt5058_block_read(struct i2c_client *i2c,
			u8 reg, int len, void *dst);
extern int rt5058_reg_write(struct i2c_client *i2c,
				u8 reg, const u8 data);
extern int rt5058_block_write(struct i2c_client *i2c,
			u8 reg, int len, const void *src);
extern int rt5058_assign_bits(struct i2c_client *i2c,
				u8 reg, u8 mask, u8 data);
extern int rt5058_assign_bits16(struct i2c_client *client,
			uint8_t reg, uint16_t mask, uint16_t val);
extern int rt5058_set_bits(struct i2c_client *i2c, u8 reg, u8 mask);
extern int rt5058_clr_bits(struct i2c_client *i2c, u8 reg, u8 mask);
extern int32_t rt5058_i2c_write_word(struct i2c_client *client,
					uint8_t reg_addr, uint16_t data);
extern int32_t rt5058_i2c_read_word(struct i2c_client *client,
							uint8_t reg_addr);
extern void rt5058_write_back_all_cache(struct rt5058_mfd_chip *chip);

extern int rt5058_core_init(struct rt5058_mfd_chip *chip,
			struct rt5058_mfd_platform_data *pdata);
extern int rt5058_core_deinit(struct device *dev);

extern int rt5058_init_irq(struct rt5058_mfd_chip *chip);
extern int rt5058_exit_irq(struct rt5058_mfd_chip *chip);
extern int rt5058_get_irq_index_byname(const char *);

extern int rt5058_regmap_init(struct rt5058_mfd_chip *chip);
extern int rt5058_regmap_deinit(struct rt5058_mfd_chip *chip);

#ifdef CONFIG_PM
extern int rt5058_irq_suspend(struct rt5058_mfd_chip *);
extern int rt5058_irq_resume(struct rt5058_mfd_chip *);
#endif /* CONFIG_PM */

#ifdef CONFIG_RT5058_SHOW_INFO
#define RTINFO(format, args...) \
	pr_info("%s() : " format,\
	__func__, ##args)
#else
#define RTINFO(format, args...)
#endif /* CONFIG_RT5058_SHOW_INFO */

#define RT5058_I2CSTMR_RST_MASK		(0x80)
#define RT5058_I2CSTMR_RSTTMR_MASK	(0x60)
#define RT5058_I2CSTMR_RSTTMR_SHIFT	5

enum { /* i2c safe timer for SDA/SCL low active */
	RT5058_I2CSTMR_0_5SEC,
	RT5058_I2CSTMR_0_75SEC,
	RT5058_I2CSTMR_1SEC,
	RT5058_I2CSTMR_2SEC,
};

typedef void (*rt_irq_handler)(void *info, int eventno);

/* Set & Get property Macro */
#define RT_DOPSY_FAIL	(-1987) /* Magic Error Number */
static inline struct power_supply *get_power_supply_by_name(char *name)
{
	if (!name)
		return (struct power_supply *)NULL;
	return power_supply_get_by_name(name);
}

#define psy_do_property(name, func, property, value) \
{	\
	struct power_supply *psy;	\
	int ret;	\
	psy = get_power_supply_by_name((name));	\
	if (!psy) {	\
		pr_err("%s: Fail to "#func" psy (%s)\n",	\
			__func__, (name));	\
		value.intval = RT_DOPSY_FAIL;	\
	} else {	\
		if (psy->func##_property != NULL) { \
			ret = psy->func##_property(psy,	\
						(property), &(value)); \
			if (ret < 0) {	\
				pr_err("Fail to "#name" "#func" (%d=>%d)\n", \
					(property), ret);	\
				value.intval = RT_DOPSY_FAIL;	\
			}	\
		}	\
	}	\
}

#endif  /* _LINUX_MFD_RT5058_H */
