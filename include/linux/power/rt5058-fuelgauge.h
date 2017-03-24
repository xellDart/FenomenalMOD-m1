/*
 * drivers/power/rt5058-fuelgauge.h
 *
 * Header of Richtek RT5058 Fuelgauge Driver
 *
 * Copyright (C) 2015 Richtek Technology Corp.
 * Author: Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef RT5058_FUELGAUGE_H
#define RT5058_FUELGAUGE_H

#include <linux/i2c.h>
#include <linux/power_supply.h>

#include <linux/mfd/rt5058/rt5058.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif /* #ifdef CONFIG_DEBUG_FS */
#include <linux/qpnp/qpnp-adc.h>

#define RT5058_FUELGAUGE_USE_NESTED_IRQ	(0)

#define VG_CS_BC_NR 9 /* 4 vgcomp, 4 cscomp, 1 battery capacity */

#define RT5058_FGBAT_PRES_MASK		(0x0080)
#define RT5058_FG_RI_MASK		(0x0001)
#define RT5058_FG_IRQMASK		(0x08)
#define RT5058_FGSOCCFG2_UNLOCK_MASK	(0x0001)
#define RT5058_FGOS_MASK		(0x1000)
#define RT5058_FGUS_MASK		(0x0800)
#define RT5058_FGUV_MASK		(0x2000)
#define RT5058_FGSHDN_EN_MASK		(0x0020)
#define RT5058_FGSLP_MASK		(0x0008)
#define RT5058_BATPRES_EN_MASK		(0x8000)
#define RT5058_FGOT_MASK		(0x8000)
#define RT5058_FGUT_MASK		(0x4000)
#define RT5058_ICCJEITA_MASK		(0x80)
#define RT5058_SCIRQACCEN_MASK		(0x0010)
#define RT5058_SCSTEP_MASK		(0x3f00)
#define RT5058_SCSTEP_SHIFT		8
#define RT5058_REG_FG_SC_MASK   (0x0100)
#define RT5058_FG_EXIT_SHDN     (0x6400)
#define RT5058_FG_ENTER_SHDN    (0x64AA)
#define RT5058_FG_SHDN_MASK     (0x0004)

struct vg_comp_data {
	int data[VG_CS_BC_NR];
};

struct data_point {
	union {
		int x;
		int voltage;
		int soc;
	};
	union {
		int y;
		int temperature;
	};
	union {
		int z;
		int curr;
	};
	union {
		int data[VG_CS_BC_NR];
		struct vg_comp_data vg_comp;
		int w;
		int offset;
	};
};

struct vg_comp_table {
	int voltNR;
	int tempNR;
	int currNR;
	struct data_point *vg_comp_data;
};

struct soc_offset_table {
	int soc_voltNR;
	int tempNR;
	int currNR;
	struct data_point *soc_offset_data;
};

struct fg_function_table {
	int enable;
	int data[8];
};

struct fg_soc_table {
	int data[8];
};

enum {
	SOC_OFFSET = 0,
	OFFSET_NR,
};

enum {
	FGCOMP = 0,
	FGCOMP_NR,
};

#ifdef CONFIG_DEBUG_FS
enum {
	RT5058FG_SOC_OFFSET_SIZE = 0,
	RT5058FG_SOC_OFFSET_DATA,
	RT5058FG_FG_COMP_SIZE,
	RT5058FG_FG_COMP_DATA,
	RT5058FG_PARAM_LOCK,
	RT5058FG_VGCOMP_IP_ORDER,
	RT5058FG_OFFSET_IP_ORDER,
	RT5058FG_FIND_TABLE_TEST,
	RT5058FG_FIND_OFFSET_TEST,
	RT5058FG_FULLSOC_CALI,
	RT5058FG_SET_ICC,
	RT5058FG_QUICK_SENSING,
	RT5058FG_DENTRY_NR,
};

#define RT5058_DBG_OUT_BUF_SIZE 2048
#endif /* CONFIG_DEBUG_FS */

struct rt5058_fuelgauge_info {
	struct mutex param_lock;
	struct device *dev;
	struct i2c_client *client;
	struct rt5058_fuelgauge_platform_data *fdata;
	struct power_supply psy;
	struct delayed_work update_polling_work;
	struct delayed_work dwork;
	struct wake_lock dwork_lock;
	struct qpnp_vadc_chip *vadc_dev;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dir_dentry;
	struct dentry *file_dentries[RT5058FG_DENTRY_NR];
	char dbg_out_buffer[RT5058_DBG_OUT_BUF_SIZE];
#endif /* #ifdef CONFIG_DEBUG_FS */
	int BC_FL;
	int32_t temperature;; /* 0.1 deg C*/
	int reg_addr;
	int batt_soc;
	int avg_vbat;
	int avg_curr;
	int os_us_th_base;
	int sc_step;
	int soc_old;
	int fg_ver;
	int cut_off_vbat;
	int32_t irq_mask[2]; /* masked irq bits*/
	int32_t irq_occured_flag[2]; /* occured irq bits */
	u8 reg_data[2];
	u8 flag_full_charge:1; /* 0 : no , 1 : yes*/
	u8 suspend:1;
	u8 online:1;
	u8 fullsoc_cali:1;
};

enum {
	FUELEVENT_DSG,
	FUELEVENT_RDY,
	FUELEVENT_QSDONE = 5,
	FUELEVENT_RI,
	FUELEVENT_SHDN = 18,
	FUELEVENT_SLP,
	FUELEVENT_PRESRDY,
	FUELEVENT_OEPPACT,
	FUELEVENT_BATTYPE,
	FUELEVENT_BATPRES,
	FUELEVENT_SC = 24,
	FUELEVENT_EOD,
	FUELEVENT_EOC,
	FUELEVENT_US,
	FUELEVENT_OS,
	FUELEVENT_UV,
	FUELEVENT_UT,
	FUELEVENT_OT,
	FUELEVENT_MAX,
};

struct rt5058_fuelgauge_platform_data {
	int vg_comp_interpolation_order[3];
	int offset_interpolation_order[3];
	struct vg_comp_table vg_comp;
	struct soc_offset_table soc_offset;
	struct fg_function_table function_table[5];
	struct fg_soc_table soc_table[10];
	int battery_type; /* 4200 or 4350 or 4400*/
	u32 oep_threshold;
	u32 otut_threshold;
	u32 uv_threshold;
	u32 os_threshold;
	u32 us_threshold;
	u32 slpvol_threshold;
	u32 temp_source; /* temp source table */
	u32 full_design; /* battery full capacity */
	u32 vol_cali;
	u32 cur_cali;
	u32 cyc_adj_rat;
	u32 cyc_adj_th;
	u32 fg_aging_factor;
	u32 fg_deadband;
	u32 capacity_max;
	u32 capacity_max_margin;
	u32 capacity_min;
	u32 sc_step;
	u32 dtsi_version[2];
	u32 op_config[3];
	u32 soc_config[2];
	u32 irq_times[2];
	u32 irq_mask[2]; /* masked irq bits*/
	u8 wr_bc_en:1; /* enable/disable write BC function */
	u8 use_sc_count:1;
};

#endif /* RT5058_FUELGAUGE_H */
