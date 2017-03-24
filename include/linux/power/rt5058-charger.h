/* include/linux/power/rt5058-charger.h
 * RT5058 Charger Driver Header file
 * Copyright (C) 2015
 * Author: Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_RT5058_CHARGER_H
#define __LINUX_RT5058_CHARGER_H

#define RT5058_CHG_USE_NESTED_IRQ	(0)

#define RT5058_CHGEN_MASK	(0x10)
#define RT5058_CHGAICR_MASK	(0Xfc)
#define RT5058_CHGAICR_SHIFT	2
#define RT5058_CHGICC_MASK	(0xf8)
#define RT5058_CHGICC_SHIFT	3
#define RT5058_OPAMODE_MASK	(0x08)
#define RT5058_CHGCV_MASK	(0xfc)
#define RT5058_CHGCV_SHIFT	2
#define RT5058_TEEN_MASK	(0x20)
#define RT5058_OTGVOLT_MASK	(0xfc)
#define RT5058_OTGVOLT_SHIFT	2
#define RT5058_ICCJEITA_MASK	(0x80)
#define RT5058_BATDET_MASK	(0x40)
#define RT5058_IEOC_MASK	(0xf8)
#define RT5058_IEOC_SHIFT	3
#define RT5058_MIVR_MASK	(0xf0)
#define RT5058_MIVR_SHIFT	4
#define RT5058_BSTOVP_STAT_MASK	(0x80)
#define RT5058_CHGHZ_MASK	(0x10)	/* high impedance mode */
#define RT5058_CHGVBATFC_MASK	(0x10)	/* 0:pre-charge level, 1:fast-charge*/
#define RT5058_CHGPWRRDY_STAT_MASK	(0x08)
#define RT5058_CHGBST_STAT_MASK	(0x20)
#define RT5058_CHG_IRQMASK	(0x80)
#define RT5058_VPREC_MASK	(0x0f)
#define RT5058_VPREC_SHIFT	0
#define RT5058_IPREC_MASK	(0x07)
#define RT5058_IPREC_SHIFT	0
#define RT5058_EOCTIMER_MASK	(0x03)
#define RT5058_EOCTIMER_SHIFT	0
#define RT5058_IINLMTSEL_MASK	(0x18)
#define RT5058_IINLMTSEL_SHIFT	3
#define RT5058_CHGLBP_MASK	(0x3c)
#define RT5058_CHGLBP_SHIFT	2
#define RT5058_SWFREQ_MASK	(0x80)
#define RT5058_FIXFREQ_MASK	(0x40)
#define RT5058_WTFC_MASK	(0xe0)
#define RT5058_WTFC_SHIFT	5
#define RT5058_WTPRC_MASK	(0x18)
#define RT5058_WTPRC_SHIFT	3
#define RT5058_SYSMIN_MASK	(0x60)
#define RT5058_SYSMIN_SHIFT	5
#define RT5058_CHG_IINMEAS_MASK	(0x80)
#define RT5058_BATABS_STAT_MASK	(0x01)
#define RT5058_CHGMIVR_MASK	(0x04)
#define RT5058_CHGIINVTH_MASK	(0x07)
#define RT5058_CHGIINVTH_SHIFT	0
#define RT5058_CHGHOCP_MASK	(0x40)
#define RT5058_IEOCSTAT_MASK	(0x08)
#define RT5058_MUICUL_MASK	(0x02)
#define RT5058_MUIC_IRQMASK	(0x20)
#define RT5058_FG_BDRTDET_EN	(0x1000)
#define RT5058_FG_PRESEN	(0x8000)
#define RT5058_FG_BAT_RMV_MASK (0x0004)

#define RT5058_MAX_AICR		2000
#define RT5058_MAX_ICC		2500
#define RT5058_MAX_CHGCV	4650
#define RT5058_MAX_BSTVOLT	5200
#define RT5058_MAX_IEOC		500
#define RT5058_MAX_MIVR		4800
#define RT5058_MAX_PRECHGVOLT	3800
#define RT5058_MAX_PRECHGCURR	550
#define RT5058_MAX_LBPVOLT	3800
#define RT5058_MAX_CHGIIN_VTH	4800

enum { /* MIVR */
	RT5058_MIVR_DISABLE = 0,
	RT5058_MIVR_3800MV,
	RT5058_MIVR_3900MV,
	RT5058_MIVR_4000MV,
	RT5058_MIVR_4100MV,
	RT5058_MIVR_4200MV,
	RT5058_MIVR_4300MV,
	RT5058_MIVR_4400MV,
	RT5058_MIVR_4500MV,
	RT5058_MIVR_4600MV,
	RT5058_MIVR_4700MV,
	RT5058_MIVR_4800MV,
};

enum { /* back-charging time */
	RT5058_BACKCHG_0MIN,
	RT5058_BACKCHG_30MIN,
	RT5058_BACKCHG_45MIN,
	RT5058_BACKCHG_60MIN,
};

enum { /* input current selection type */
	RT5058_IINLMT_CHGTYPE,
	RT5058_IINLMT_AICR,
	RT5058_IINLMT_MAX,
	RT5058_IINLMT_MIN,
};

enum { /* switching frequency */
	RT5058_SWFREQ_1_5MHZ,
	RT5058_SWFREQ_0_75MHZ,
};

enum { /* switching frequency type */
	RT5058_FIXFREQ_VARIED,
	RT5058_FIXFREQ_FIXED,
};

enum { /* The OCP level of buck mode selection bit */
	RT5058_HOCP_2_5A,
	RT5058_HOCP_3_5A,
};

enum { /* fast charge timer */
	RT5058_FCTIMER_4HRS,
	RT5058_FCTIMER_6HRS,
	RT5058_FCTIMER_8HRS,
	RT5058_FCTIMER_10HRS,
	RT5058_FCTIMER_12HRS,
	RT5058_FCTIMER_14HRS,
	RT5058_FCTIMER_16HRS,
	RT5058_FCTIMER_DISABLE,
};

enum { /* pre-charge timer */
	RT5058_PRCTIMER_30MIN,
	RT5058_PRCTIMER_45MIN,
	RT5058_PRCTIMER_60MIN,
	RT5058_PRCTIMER_DISABLE,
};

enum { /* system minimum regulation voltage */
	RT5058_SYSMIN_3_5V,
	RT5058_SYSMIN_3_6V,
	RT5058_SYSMIN_3_7V,
	RT5058_SYSMIN_3_8V,
};

struct rt5058_charger_platform_data {
	char *charger_name;
	int chg_volt;
	int aicr_ma;
	int icc;
	int bst_volt;
	int ieoc;
	int mivr;
	int prechg_volt;
	int prechg_curr;
	int lbp_volt;
	int chg_iin_vth;
	u32 temp_level[4];
	u8 irq_mask[5];
	u8 fc_timer:3;
	u8 sys_minvolt:2;
	u8 prc_timer:2;
	u8 iinlmt_sel:2;
	u8 backchg_time:2;
	u8 use_te:1;
	u8 use_mivr:1;
	u8 use_aicr:1;
	u8 use_batdet:1;
	u8 use_lbp:1;
	u8 use_iinmeas:1;
	u8 sel_swfreq:1;
	u8 fixfreq:1;
	u8 iinmeas_running:1;
	u8 hocp:1;
	u8 use_jeita:1; /* enable/disable jeita function */
};

enum {
	CHGEVENT_IINMEAS,
	CHGEVENT_ICCMEAS,
	CHGEVENT_MIVR,
	CHGEVENT_PWRRDY,
	CHGEVENT_BATABS = 8,
	CHGEVENT_SYSUV,
	CHGEVENT_TMR,
	CHGEVENT_BATOV = 12,
	CHGEVENT_BADADP,
	CHGEVENT_RVP,
	CHGEVENT_TSSHD,
	CHGEVENT_TREG,
	CHGEVENT_RCHG,
	CHGEVENT_TERMTMR,
	CHGEVENT_IEOC,
	CHGEVENT_BSTLV = 29,
	CHGEVENT_BSTOL,
	CHGEVENT_BSTOVP,
	MUIC_UL_EVENT,
	CHGEVENT_MAX,
};

#endif /* __LINUX_RT5058_CHARGER_H */
