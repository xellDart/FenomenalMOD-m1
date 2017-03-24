/* RT5058 Voltage Tracking Fuelgauge Driver
 *
 * Copyright (C) 2015
 * Modified by Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/math64.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

#include <linux/mfd/rt5058/rt5058.h>
#include <linux/power/rt5058-charger.h>
#include <linux/power/rt5058-fuelgauge.h>

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
#include <soc/qcom/lge/lge_battery_id_checker.h>
#endif

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER
#include <soc/qcom/lge/power/lge_power_class.h>
#include <soc/qcom/lge/lge_battery_id_checker.h>
#endif
#include <linux/qpnp/qpnp-adc.h>

#define MINVAL(a, b) ((a <= b) ? a : b)
#define PRECISION_ENHANCE	5
#define RT5058_FG_DEBUG 0

static struct rt5058_fuelgauge_info *the_fuel_info;

static int rt5058_fg_get_offset(struct rt5058_fuelgauge_info *fi,
							int soc_val, int temp);

static struct vg_comp_data rt5058_fg_get_vgcomp(
	struct rt5058_fuelgauge_info *fi,
	int volt, int temp, int curr);

struct submask_condition {
	int x, y, z;
	int order_x, order_y, order_z;
	int xNR, yNR, zNR;
	const struct data_point *mesh_src;
};

enum comp_offset_type {
	FG_COMP = 0,
	FG_SOC_OFFSET,
};

static inline const struct data_point *get_mesh_data(
	int i, int j, int k,
	const struct data_point *mesh, int xNR, int yNR)
{
	return mesh + k * yNR * xNR + j * xNR + i;
}

#define DEFAULT_TEMP 250
int rt5058_fg_get_batt_temp(struct rt5058_fuelgauge_info *fi)
{
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(fi->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("%s(): Unable to read batt temperature rc=%d\n", __func__, rc);
		pr_err("%s(): Report default_batt_temp %d again\n",
				__func__, DEFAULT_TEMP);
		return DEFAULT_TEMP;
	} else {
		pr_debug("%s(): get_batt_temp %d %lld\n", __func__,
				results.adc_code, results.physical);
		return (int)results.physical;
	}
#else
	pr_err("%s: CONFIG_SENSORS_QPNP_ADC_VOLTAGE is not defined.\n", __func__);
	return DEFAULT_TEMP;
#endif
}

static int rt5058_fg_is_charging(struct rt5058_fuelgauge_info *fi)
{
	int ret;
	ret = rt5058_reg_read(fi->client, RT5058_REG_CHGCTRL2);
	return (ret & RT5058_CHGEN_MASK) ? 1 : 0;
}

static int rt5058_fg_is_eoc(struct rt5058_fuelgauge_info *fi)
{
	int is_eoc = 0;
	union power_supply_propval value;
	struct power_supply *chg_psy;

	chg_psy = power_supply_get_by_name(rt5058_chg_devname);

	if (chg_psy) {
		chg_psy->get_property(chg_psy, POWER_SUPPLY_PROP_CHARGER_EOC, &value);
		is_eoc = value.intval;
	} else {
		pr_err("%s: chg_psy is not yet ready\n", __func__);
	}
	return is_eoc;
}

#define REG_SIZE 13
static void rt5058_fg_reginfo(struct rt5058_fuelgauge_info *fi)
{
	int debug_regs[REG_SIZE] = {0xB0, 0xB1, 0xB2, 0xB7, 0xB8, 0xC0, 0xC3,
		0xC5, 0xC6, 0xCF, 0xE0, 0xE1, 0xE2};
	int i = 0, j = 0;
	char buf[100] = {0,};

	for (i = 0; i < REG_SIZE; i++) {
		if (i == REG_SIZE -1) {
			j += scnprintf(buf + j, 6, "%04x",
					rt5058_i2c_read_word(fi->client, debug_regs[i]));
		} else {
			j += scnprintf(buf + j, 6, "%04x:",
					rt5058_i2c_read_word(fi->client, debug_regs[i]));
		}
	}
	pr_err("%s: %s\n", __func__, buf);
	return;
}

#if RT5058_FG_DEBUG
static void rt5058_dwork_show(struct rt5058_fuelgauge_info *fi)
{
	int volt;
	int ocv;
	int soc;
	int inow;
	int is_charging;
	int is_eoc;
	int temp;
	struct power_supply *psy;
	union power_supply_propval value = {0,};
	const char *chg_status[] = {
		"Unknown", "Charging", "DisCharging", "Not Charging", "Full",
	};

	if (!fi->suspend) {
		psy = power_supply_get_by_name(rt5058_chg_devname);
		if (psy) {
			psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
		}
		is_charging = rt5058_fg_is_charging(fi);
		is_eoc = rt5058_fg_is_eoc(fi);
		temp = rt5058_fg_get_batt_temp(fi);

		volt = rt5058_i2c_read_word(fi->client, RT5058_REG_FGAV);
		ocv = rt5058_i2c_read_word(fi->client, RT5058_REG_FGOCV);
		soc = rt5058_i2c_read_word(fi->client, RT5058_REG_FGSOC) * 10;
		inow = rt5058_i2c_read_word(fi->client, RT5058_REG_FGAI);
		if (inow > 32768)
			inow = inow - 65536;
		inow = inow * -1;
		pr_err("%s: %s, charigng=%d, EOC=%d, VBAT=%d, OCV=%d, SOC=%d, "
				"TEMP=%d, INOW=%d\n", __func__, chg_status[value.intval],
				is_charging, is_eoc, volt, ocv,	soc/256, temp, inow);
	}
	return;
}

static void rt5058_dwork_func(struct work_struct *work)
{
	struct rt5058_fuelgauge_info *fi;

	fi = container_of(work, struct rt5058_fuelgauge_info,
			dwork.work);
	if (fi->suspend) {
		pr_err("%s: run dwork after 20msec\n", __func__);
		schedule_delayed_work(&fi->dwork, msecs_to_jiffies(20));
		return;
	} else {
		wake_lock(&fi->dwork_lock);
		rt5058_dwork_show(fi);
		schedule_delayed_work(&fi->dwork, msecs_to_jiffies(10000));
		if (wake_lock_active(&fi->dwork_lock)) {
			wake_unlock(&fi->dwork_lock);
		}
	}
	return;
}
#endif

/* n-order 2D interpolation */
static int offset_li(int xNR, int yNR, const struct data_point *mesh,
								int x, int y)
{
	long long retval = 0;
	int i, j, k;
	long long wM, wD;
	const struct data_point *cache;

	for (i = 0 ; i < xNR; ++i) {
		for (j = 0; j < yNR; ++j) {
			wM = wD = 1;
			cache = get_mesh_data(i, j, 0, mesh, xNR, yNR);
			for (k = 0; k < xNR; ++k) {
				if (i != k) {
					wM *= (x - get_mesh_data(k, j, 0,
							mesh, xNR, yNR)->x);
					wD *= (cache->x - get_mesh_data(k,
						j, 0, mesh, xNR, yNR)->x);
				}
			}
			for (k = 0; k < yNR; ++k) {
				if (j != k) {
					wM *= (y - get_mesh_data(i, k, 0,
							mesh, xNR, yNR)->y);
					wD *= (cache->y - get_mesh_data(i,
						k, 0, mesh, xNR, yNR)->y);
				}
			}
			retval += div64_s64(((cache->w * wM) <<
						PRECISION_ENHANCE), wD);
		}
	}
	return (int)((retval + (1 << (PRECISION_ENHANCE - 1)))
						>> PRECISION_ENHANCE);
}

static int get_sub_mesh(int state, struct data_point *mesh_buffer,
			struct submask_condition *condition)
{
	int i, j, k = 0, x, y, z;

	x = condition->x;
	y = condition->y;
	z = condition->z;
	for (i = 0; i < condition->xNR; ++i) {
		if (get_mesh_data(i, 0, 0, condition->mesh_src,
				condition->xNR, condition->yNR)->x >= x)
			break;
	}
	for ( ; i >= 0 && i < condition->xNR; --i) {
		if (get_mesh_data(i, 0, 0, condition->mesh_src,
				condition->xNR, condition->yNR)->x <= x)
			break;
	}

	for (j = 0; j < condition->yNR; ++j) {
		if (get_mesh_data(0, j, 0, condition->mesh_src,
				condition->xNR, condition->yNR)->y >= y)
			break;
	}
	for ( ; j >= 0 && j < condition->yNR; --j) {
		if (get_mesh_data(0, j, 0, condition->mesh_src,
				condition->xNR, condition->yNR)->y <= y)
			break;
	}

	if (state == FG_COMP) {
		for (k = 0; k < condition->zNR; ++k) {
			if (get_mesh_data(0, 0, k, condition->mesh_src,
				condition->xNR, condition->yNR)->z >= z)
				break;
		}
		for ( ; k >= 0 && k < condition->zNR; --k) {
			if (get_mesh_data(0, 0, k, condition->mesh_src,
				condition->xNR, condition->yNR)->z <= z)
				break;
		}
	}

	i -= ((condition->order_x - 1) / 2);
	j -= ((condition->order_y - 1) / 2);
	k -= ((condition->order_z - 1) / 2);

	if (i <= 0)
		i = 0;
	if (j <= 0)
		j = 0;
	if (k <= 0)
		k = 0;
	if ((i + condition->order_x) > condition->xNR)
		i = condition->xNR - condition->order_x;
	if ((j + condition->order_y) > condition->yNR)
		j = condition->yNR - condition->order_y;
	if ((k + condition->order_z) > condition->zNR)
		k = condition->zNR - condition->order_z;
	if (state == FG_COMP) {
		for (z = 0; z < condition->order_z; ++z) {
			for (y = 0; y < condition->order_y; ++y) {
				for (x = 0; x < condition->order_x; ++x) {
					*(mesh_buffer + z * condition->order_y *
						condition->order_z +
						y * condition->order_x + x)
						= *get_mesh_data(i + x, j + y,
						k + z, condition->mesh_src,
						condition->xNR, condition->yNR);
				}
			}
		}
	} else {
		for (y = 0; y < condition->order_y; ++y) {
			for (x = 0; x < condition->order_x; ++x) {
				*(mesh_buffer + y * condition->order_x + x)
					= *get_mesh_data(i + x, j + y, 0,
						condition->mesh_src,
						condition->xNR,
						condition->yNR);
			}
		}
	}
	return 0;
}

static struct vg_comp_data vgcomp_li(int xNR, int yNR, int zNR,
			const struct data_point *mesh, int x, int y, int z)
{
	long long retval[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0};
	struct vg_comp_data ret;
	int i, j, l, k;
	long long wM, wD;
	const struct data_point *cache;

	for (i = 0 ; i < xNR; ++i) {
		for (j = 0; j < yNR; ++j) {
			for (l = 0; l < zNR; ++l) {
				wM = wD = 1;
				cache = get_mesh_data(i, j, l, mesh, xNR, yNR);
				for (k = 0; k < xNR; ++k) {
					if (i != k) {
						wM *= (x - get_mesh_data(k, j,
							l, mesh, xNR, yNR)->x);
						wD *= (cache->x - get_mesh_data(
						k, j, l, mesh, xNR, yNR)->x);
					}
				}
				for (k = 0; k < yNR; ++k) {
					if (j != k) {
						wM *= (y - get_mesh_data(i, k,
							l, mesh, xNR, yNR)->y);
						wD *= (cache->y - get_mesh_data(
						i, k, l, mesh, xNR, yNR)->y);
					}
				}
				for (k = 0; k < zNR; ++k) {
					if (l != k) {
						wM *= (z - get_mesh_data(i, j,
							k, mesh, xNR, yNR)->z);
						wD *= (cache->z - get_mesh_data(
						i, j, k, mesh, xNR, yNR)->z);
					}
				}
				for (k = 0; k < ARRAY_SIZE(retval); ++k)
					retval[k]  +=  div64_s64(
						(cache->vg_comp.data[k] * wM)
						<< PRECISION_ENHANCE, wD);
			}
		}
	}
	for (k = 0; k < ARRAY_SIZE(retval); ++k) {
		ret.data[k] = (int)((retval[k] +
				(1 << (PRECISION_ENHANCE - 1)))
						>> PRECISION_ENHANCE);
		if (k < 4) { /* VGCOMP */
			if (ret.data[k] < 0)
				ret.data[k]  = 0;
			else if (ret.data[k] > 255)
				ret.data[k] = 255;
		}
		/* else are CSCOMP data and Battery Capacity data */
	}
	return ret;
}

static unsigned int fg_get_avg_vbat(struct rt5058_fuelgauge_info *fi);
static unsigned int fg_get_ocv(struct rt5058_fuelgauge_info *fi);
static unsigned int fg_get_current(struct rt5058_fuelgauge_info *fi);
static unsigned int fg_get_cyccnt(struct rt5058_fuelgauge_info *fi);
static int rt5058_fg_init(struct rt5058_fuelgauge_info *fi);

#define DEFAULT_TEMP	250
#define CUTOFF_OFFSET_HIGH	150
#define CUTOFF_OFFSET_MID	100
#define CUTOFF_OFFSET_LOW	50
#define LOW_TEMP_TOL	-80
static int dsg_light_load_check_done = 0;
static int low_soc_check_done = 0;
unsigned int fg_get_soc(struct rt5058_fuelgauge_info *fi)
{
	struct vg_comp_data vgcomp;
	unsigned int ocv, avg_volt, curr, cyccnt, inow;
	int ret, soc_val = 660 /* default soc */, offset;
	int is_charging;
	int raw_soc = 0;
	int vg1_vg2 = 0;
	int vg1 = 0;
	int vg2 = 0;

	/* if F/G is shutdown mode, exit shutdown mode */
	if ((rt5058_i2c_read_word(fi->client, RT5058_REG_FG_STATUS1)) & RT5058_FG_SHDN_MASK) {
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGMFA, RT5058_FG_EXIT_SHDN);
	}

	ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FG_STATUS1);
	if (ret & RT5058_FG_RI_MASK)
		rt5058_fg_init(fi);

	ocv = fg_get_ocv(fi);
	avg_volt = fg_get_avg_vbat(fi);
	curr = fg_get_current(fi);
	cyccnt = fg_get_cyccnt(fi);

	fi->temperature = rt5058_fg_get_batt_temp(fi);
	inow = curr;
	if (inow > 32768)
		inow -= 65536;
	inow = inow * -1;

	vgcomp = rt5058_fg_get_vgcomp(fi, avg_volt, fi->temperature, curr);
	/* write cscomp parameters */
	rt5058_i2c_write_word(fi->client,
		RT5058_REG_FGCSCOMP1, vgcomp.data[4] << 8); /* high byte */
	rt5058_i2c_write_word(fi->client,
				RT5058_REG_FGCSCOMP2, vgcomp.data[5]);
	rt5058_i2c_write_word(fi->client,
				RT5058_REG_FGCSCOMP3, vgcomp.data[6]);
	rt5058_i2c_write_word(fi->client,
				RT5058_REG_FGCSCOMP4, vgcomp.data[7]);

	/* write vgcomp parameters */
	ret = (vgcomp.data[0] << 8) | vgcomp.data[1];
	rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP1_2, ret);
	ret = (vgcomp.data[2] << 8) | vgcomp.data[3];
	rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP3_4, ret);

	/* write battery capacity */
	if (fi->fdata->wr_bc_en) {
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGMFA, 0x7fcc);
		rt5058_i2c_write_word(fi->client,
				RT5058_REG_FGBATCAP, vgcomp.data[8]);
	}

	is_charging = rt5058_fg_is_charging(fi);
	if (inow > 400) {
		if (avg_volt <= (fi->cut_off_vbat - CUTOFF_OFFSET_HIGH)
				&& !is_charging && fi->temperature > LOW_TEMP_TOL) {
			pr_err("%s: prevent UVLO set\n", __func__);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x0000);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP1_2, 0x7f7f);
		}
	} else if (inow > 150) {
		if (avg_volt <= (fi->cut_off_vbat - CUTOFF_OFFSET_MID)
				&& !is_charging && fi->temperature > LOW_TEMP_TOL) {
			pr_err("%s: prevent UVLO set\n", __func__);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x0000);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP1_2, 0x7f7f);
		}
	} else {
		if (avg_volt <= (fi->cut_off_vbat - CUTOFF_OFFSET_LOW)
				&& !is_charging && fi->temperature > LOW_TEMP_TOL) {
			pr_err("%s: prevent UVLO set\n", __func__);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x0000);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP1_2, 0x7f7f);
		}
	}

	raw_soc = rt5058_i2c_read_word(fi->client, RT5058_REG_FGSOC);
	if (raw_soc < 0) {
		pr_err("%s: read soc fail\n", __func__);
		soc_val = 500;
	} else
		soc_val = (raw_soc * 10)/256;

	/* Fine Tune 11/12 */
	if (is_charging) {
		dsg_light_load_check_done = 0;
		fi->BC_FL = 2;

		if ((soc_val >= 0) && (soc_val < 500)) {
			ret = ((vgcomp.data[5] & 0xff00) | ((vgcomp.data[5] & 0x00ff) * 6/5));
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP2, ret);
		}
	}

	if(is_charging && (soc_val >=1000)){
		if(soc_val < 1005)
			fi->BC_FL=2;
		else if (soc_val < 1010 && soc_val >= 1005)
			fi->BC_FL=2;
		else if(soc_val < 1017 && soc_val >= 1010)
			fi->BC_FL=2;
		else if(soc_val < 1027 && soc_val >= 1017)
			fi->BC_FL=3;
		else if(soc_val < 1041 && soc_val >= 1027)
			fi->BC_FL=4;
		else if(soc_val >= 1041)
			fi->BC_FL=4;
	} else if(!is_charging){
		if((inow<=400)&&(!dsg_light_load_check_done)){
			fi->BC_FL++;
			dsg_light_load_check_done = 1;

		}else if((inow>400)&&(dsg_light_load_check_done)){
			fi->BC_FL--;
			dsg_light_load_check_done = 0;
		}
		if((soc_val<=400)&&(soc_val>100)){
			fi->BC_FL--;
			ret = (vgcomp.data[5] >> 8) * fi->BC_FL / 6;
			fi->BC_FL++;
		}else{
			ret = (vgcomp.data[5] >> 8) * fi->BC_FL / 6;
		}
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP2, ret << 8);
	}
	/* Fine Tuen End 11/12 */

	if(is_charging && (soc_val >=700)){
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x0000);
		mdelay(5);
		ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FGVGCOMP3_4);
		ret &= 0xff00;
		ret |= 0x0064;
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP3_4, ret);
		mdelay(5);
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x8000);
	} else {
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x0000);
		mdelay(5);
		ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FGVGCOMP3_4);
		ret &= 0xff00;
		ret |= 0x0032;
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP3_4, ret);
		mdelay(5);
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x8000);
	}

	if (!is_charging && (soc_val >= 1000)) {
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0xe300);
	} else if (!is_charging && (inow >= 100)) {
		vg1_vg2 = rt5058_i2c_read_word(fi->client, RT5058_REG_FGVGCOMP1_2);
		vg1 = vg1_vg2 >> 8;
		vg2 = vg1_vg2 & 0xff;
		if (vg2 != 0x7f) {
			if((vg2==50)&&(vg1>=200)){
				vg1_vg2 = (vg1<<8)|0x64;
			}else if((vg2>=100)&&(vg1>=200)){
				vg1_vg2 = (vg1<<8)|0x96;
			}else{
				vg1_vg2 = (vg1<<8)|0x32;
			}
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x0000);
			mdelay(5);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP1_2, vg1_vg2);
			mdelay(5);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x8000);
			mdelay(5);
		}
		if ((soc_val <= 200)) {
			if (inow < 200){
				vg1_vg2 = 0x4B4B;
			} else if (inow < 300) {
				vg1_vg2 = 0x6464;
			} else {
				vg1_vg2 = 0x7D7D;
			}
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x0000);
			mdelay(5);
			rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP1_2, vg1_vg2);
			mdelay(5);
			low_soc_check_done = 1;
		}
	}
	if(is_charging && low_soc_check_done){
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x0000);
		mdelay(5);
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGVGCOMP1_2, 0x3232);
		mdelay(5);
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0x8000);
		mdelay(5);
		low_soc_check_done = 0;
	}

	offset = rt5058_fg_get_offset(fi, soc_val, fi->temperature);
	soc_val = soc_val + offset;
	if (soc_val < 0)
		soc_val = 0;
	/* report FG_SOC for debugging. */
	pr_err("%s: VBAT=%d, OCV=%d, SOC_OFS=%d, SOC=%d, "
			"SOCCFG=0x%04x, temp=%d, INOW=%d, BC_FL=%d\n", __func__,
			avg_volt, ocv, offset, soc_val,
			rt5058_i2c_read_word(fi->client, RT5058_REG_FGSOCCFG1),
			fi->temperature, inow, fi->BC_FL);
	return soc_val;
}

static unsigned int fg_get_ocv(struct rt5058_fuelgauge_info *fi)
{
	int ret;
	unsigned int ocv = 0;

	ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FGOCV);
	if (ret < 0) {
		pr_err("%s: read soc reg fail", __func__);
		ocv = 3900;
	} else
		ocv = ret;
	return ocv;
}

static unsigned int fg_get_cyccnt(struct rt5058_fuelgauge_info *fi)
{
	int ret;
	unsigned int cyccnt = 0;

	ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FGCYCCNT);
	if (ret < 0) {
		pr_err("%s : read cycle count fail\n", __func__);
		cyccnt = 0;
	} else
		cyccnt = ret;
	if (cyccnt >= fi->fdata->cyc_adj_th)
		cyccnt = cyccnt * (100 - fi->fdata->cyc_adj_rat / 100)/100;
	return cyccnt;
}

static unsigned int fg_get_avg_vbat(struct rt5058_fuelgauge_info *fi)
{
	int ret;
	unsigned int avg_vbat;

	/* get average battery voltage */
	ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FGAV);
	if (ret < 0) {
		pr_err("%s: read vbat fail", __func__);
		avg_vbat = 4000;
	} else
		avg_vbat = ret;
	fi->avg_vbat = avg_vbat;
	return avg_vbat;
}

static unsigned int fg_get_current(struct rt5058_fuelgauge_info *fi)
{
	int ret;
	unsigned int avg_curr;

	ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FGAI);
	if (ret < 0) {
		pr_err("%s: read fg current fail\n", __func__);
		avg_curr = 0;
	} else
		avg_curr = ret;
	fi->avg_curr = avg_curr;
	return avg_curr;
}

static struct vg_comp_data rt5058_fg_get_vgcomp(
				struct rt5058_fuelgauge_info *fi,
				int volt, int temp, int curr)
{
	const int ip_x = fi->fdata->vg_comp_interpolation_order[0];
	const int ip_y = fi->fdata->vg_comp_interpolation_order[1];
	const int ip_z = fi->fdata->vg_comp_interpolation_order[2];
	struct data_point sub_mesh[ip_x * ip_y * ip_z];
	const struct vg_comp_data default_vgcomp = {
		.data = { 0x32, 0x32, 0x32, 0x32, 128, 256, 32100, 0, 2000},
	};
	int xNR, yNR, zNR;
	struct vg_comp_data retval;
	struct vg_comp_table *vgcomp_table = NULL;
	struct submask_condition condition = {
		.x = volt,
		.y = temp,
		.z = curr,
	};

	mutex_lock(&fi->param_lock);
	vgcomp_table = &fi->fdata->vg_comp;
	xNR = vgcomp_table->voltNR;
	yNR = vgcomp_table->tempNR;
	zNR = vgcomp_table->currNR;
	if (xNR == 0 || yNR == 0 || zNR == 0) {
		mutex_unlock(&fi->param_lock);
		return default_vgcomp;
	}
	condition.order_x = MINVAL(ip_x, xNR);
	condition.order_y = MINVAL(ip_y, yNR);
	condition.order_z = MINVAL(ip_z, zNR);
	condition.xNR = xNR;
	condition.yNR = yNR;
	condition.zNR = zNR;
	condition.mesh_src = vgcomp_table->vg_comp_data;
	get_sub_mesh(FG_COMP, sub_mesh, &condition);
	retval = vgcomp_li(condition.order_x, condition.order_y,
			condition.order_z, sub_mesh, volt, temp, curr);
	mutex_unlock(&fi->param_lock);
	return retval;
}

static int rt5058_fg_get_offset(struct rt5058_fuelgauge_info *fi,
							int soc_val, int temp)
{
	const int ip_x = fi->fdata->offset_interpolation_order[0];
	const int ip_y = fi->fdata->offset_interpolation_order[1];

	struct data_point sub_mesh[ip_x * ip_y];
	int xNR, yNR;
	int offset;
	struct soc_offset_table *offset_table = NULL;
	struct submask_condition condition = {
		.x = soc_val,
		.y = temp,
	};

	mutex_lock(&fi->param_lock);
	offset_table = &fi->fdata->soc_offset;
	xNR = offset_table->soc_voltNR;
	yNR = offset_table->tempNR;
	if (xNR == 0 || yNR == 0) {
		mutex_unlock(&fi->param_lock);
		return 0;
	}
	condition.order_x = MINVAL(ip_x, xNR);
	condition.order_y = MINVAL(ip_y, yNR);
	condition.xNR = xNR;
	condition.yNR = yNR;
	condition.mesh_src = offset_table->soc_offset_data;
	get_sub_mesh(FG_SOC_OFFSET, sub_mesh, &condition);
	offset = offset_li(condition.order_x, condition.order_y,
						sub_mesh, soc_val, temp);
	mutex_unlock(&fi->param_lock);
	return offset;
}

static int rt5058_fg_init(struct rt5058_fuelgauge_info *fi)
{
	int ret = 0, i, j;
	struct i2c_client *client = fi->client;

	int retry_times = 3;
	int count = 5;

	pr_info("%s\n", __func__);
	fi->BC_FL = 0;

	/* if F/G is in shutdown mode, exit shutdown mode */
	while (count) {
		rt5058_i2c_write_word(client, RT5058_REG_FGMFA, RT5058_FG_EXIT_SHDN);
		mdelay(20);
		if ((rt5058_i2c_read_word(client, RT5058_REG_FG_STATUS1)) & RT5058_FG_SHDN_MASK) {
			count--;
			if (count == 0)
				pr_err("%s: OOPS FG No active mode!\n", __func__);
		} else {
			pr_err("%s: F/G is active mode\n", __func__);
			break;
		}
	}

	/* if F/G is in sleep mode, exit sleep mode */
	ret = rt5058_i2c_read_word(client, RT5058_REG_FG_STATUS1);
	if (ret < 0) {
		pr_err("%s : fail to read fuelgauge flag status1\n", __func__);
		return ret;
	}
	if (ret & RT5058_FGSLP_MASK) {
		ret = rt5058_i2c_write_word(client,
						RT5058_REG_FGMFA, 0x7400);
		if (ret < 0) {
			pr_err("%s : fail to read fuelgauge flag status1\n",
								__func__);
			return ret;
		}
	}

	pr_info("%s: set FG init values\n", __func__);

	ret = rt5058_i2c_read_word(client, RT5058_REG_FGVER);
	fi->fg_ver = ret;
	pr_err("%s: fi->fg_ver = %d\n", __func__, fi->fg_ver);

	/* default soc */
	fi->batt_soc = 50;

	/* SC setting */
	if (fi->fdata->use_sc_count) {
		rt5058_assign_bits16(client, RT5058_REG_FGSOCCFG1,
					RT5058_SCIRQACCEN_MASK, 0);
		mdelay(10);
		rt5058_assign_bits16(client, RT5058_REG_FGSOCCFG1,
				RT5058_SCIRQACCEN_MASK, RT5058_SCIRQACCEN_MASK);
		ret = rt5058_i2c_read_word(client, RT5058_REG_FG_UVDET);
		fi->sc_step = ((ret&RT5058_SCSTEP_MASK)
						>> RT5058_SCSTEP_SHIFT) * 16;
		pr_info("fi->sc_step = %d\n", fi->sc_step);
		fi->soc_old = rt5058_i2c_read_word(client, RT5058_REG_FGSOC);
		pr_info("fi->soc_old = %d\n", fi->soc_old);
		/* init soc set */
		fi->batt_soc = fi->soc_old/256;
		pr_err("fi->batt_soc = %d\n", fi->batt_soc);
	}

	/* set fg age factor & fg deadband */
	rt5058_i2c_write_word(client, RT5058_REG_FGAGEFCT,
					fi->fdata->fg_aging_factor);
	rt5058_i2c_write_word(client, RT5058_REG_FG_DEADBAND,
						fi->fdata->fg_deadband);

	/* set rt5058 threshold */
	rt5058_i2c_write_word(client,
		RT5058_REG_FG_OEPTH, fi->fdata->oep_threshold);
	rt5058_assign_bits16(client,
		RT5058_REG_FG_UVDET, 0x00ff, fi->fdata->uv_threshold);
	rt5058_i2c_write_word(client,
		RT5058_REG_FG_OTUTDET, fi->fdata->otut_threshold);
	rt5058_i2c_write_word(client,
		RT5058_REG_FG_SLP_V, fi->fdata->slpvol_threshold);
	rt5058_i2c_write_word(client,
		RT5058_REG_FG_OSDET, fi->fdata->os_threshold);
	rt5058_i2c_write_word(client,
		RT5058_REG_FG_USDET, fi->fdata->us_threshold);

	/* set rt5058 calibration */
	rt5058_i2c_write_word(client, RT5058_REG_FG_CURRCALI,
						fi->fdata->cur_cali);
	rt5058_i2c_write_word(client, RT5058_REG_FG_VOLTCALI,
						fi->fdata->vol_cali);

	/* write design capacity */
	ret = rt5058_i2c_read_word(client, RT5058_REG_FGDSNCAP);
	if ((ret >= 0) && (ret != fi->fdata->full_design))
		rt5058_i2c_write_word(client, RT5058_REG_FGDSNCAP,
					fi->fdata->full_design);
	else
		pr_info("rt5058 design capacity no change\n");

	/* set op config 1 ~ 3 */
	rt5058_assign_bits16(client, RT5058_REG_FGOPCFG1,
			0xffe3, fi->fdata->op_config[0]);
	rt5058_i2c_write_word(client, RT5058_REG_FGOPCFG2,
					fi->fdata->op_config[1]);
	rt5058_i2c_write_word(client, RT5058_REG_FGOPCFG3,
					fi->fdata->op_config[2]);

	/* set soc config 1,2 */
	rt5058_i2c_write_word(client, RT5058_REG_FGSOCCFG1,
					fi->fdata->soc_config[0]);
	rt5058_assign_bits16(client, RT5058_REG_FGSOCCFG1,
				RT5058_FGSOCCFG2_UNLOCK_MASK,
				RT5058_FGSOCCFG2_UNLOCK_MASK);
	rt5058_i2c_write_word(client, RT5058_REG_FGSOCCFG2,
					fi->fdata->soc_config[1]);
	rt5058_assign_bits16(client, RT5058_REG_FGSOCCFG1,
				RT5058_FGSOCCFG2_UNLOCK_MASK, 0);

	/* set SC change step */
	rt5058_assign_bits16(client, RT5058_REG_FG_UVDET,
				0xff00, fi->fdata->sc_step << 8);

	/* set Battery Type */
	if (fi->fdata->battery_type == 4400) {
		pr_debug("%s: set 4400 Battery OCV Table\n", __func__);
		rt5058_i2c_write_word(client, RT5058_REG_FGMFA, 0x8085);
		mdelay(5);
		ret = rt5058_i2c_read_word(client, RT5058_REG_FGMFA);
		ret |= 0x0009;
		ret += 0x8500;
		rt5058_i2c_write_word(client, RT5058_REG_FGMFA, ret);
	} else if (fi->fdata->battery_type == 4200) {
		pr_debug("%s: set 4200 Battery OCV Table\n", __func__);
		rt5058_i2c_write_word(client, RT5058_REG_FGMFA, 0x8085);
		mdelay(5);
		ret = rt5058_i2c_read_word(client, RT5058_REG_FGMFA);
		ret &= ~0x0009;
		ret += 0x8500;
		rt5058_i2c_write_word(client, RT5058_REG_FGMFA, ret);
	} else {
		/* battery type = 4350 */
	}

	/* write fg function table and fg soc table */
	for (i = 0; i < 5; i++) {
		if (fi->fdata->function_table[i].enable) {
			rt5058_i2c_write_word(client,
					RT5058_REG_FGMFA, 0x6550 + i);
			for (j = 0; j < 8; j++)
				rt5058_i2c_write_word(client,
					RT5058_REG_FG_HIDDEN1 + j,
					fi-> fdata->function_table[i].data[j]);
		}
	}

	/*set OCV TABLE */
	ret = rt5058_i2c_read_word(client, RT5058_REG_FGOPCFG1);
	if ((fi->fdata->soc_table[0].data[0] == 0x13) && ((ret&0x0002) == 0)) {
		pr_debug("%s: Write New SOC Table\n", __func__);
		while (retry_times) {
			for (i = 0; i < 9; i++) {
				rt5058_i2c_write_word(client, RT5058_REG_FGMFA, 0xca50 + i);
				for (j = 0; j < 8; j++)
					rt5058_i2c_write_word(client, RT5058_REG_FG_HIDDEN1 + j,
							fi->fdata->soc_table[i].data[j]);
			}
			rt5058_i2c_write_word(client, RT5058_REG_FGMFA, 0xca59);
			for (j = 0; j < 5; j++) {
				rt5058_i2c_write_word(client, RT5058_REG_FG_HIDDEN1 + j,
						fi->fdata->soc_table[9].data[j]);
			}
			msleep(50);
			ret = rt5058_i2c_read_word(client, RT5058_REG_FGOPCFG1);
			if (ret & 0x0002) {
				pr_err("%s: Write SOC table successfully\n", __func__);
				break;
			}
			retry_times--;
		}
	}

	/* set cut_off_vbat */
#if defined CONFIG_LGE_PM_BATTERY_RT5058_CUT_OFF_UNDER_3P4_USED
	fi->cut_off_vbat = 3400;
#elif defined CONFIG_LGE_PM_BATTERY_RT5058_CUT_OFF_UNDER_3P5_USED
	fi->cut_off_vbat = 3500;
#else
	fi->cut_off_vbat = 3400;
#endif

	/* set FG_RI = 0  write 0x2bb2 to FGMFA register */
	rt5058_i2c_write_word(client, RT5058_REG_FGMFA, 0x2bb2);
	fi->online = 1;
	pr_err("%s: FG Init successfully\n", __func__);
	return 0;
}

static void new_vgcomp_soc_offset_data(struct device *dev, int type,
				struct rt5058_fuelgauge_platform_data *fdata,
				int size_x, int size_y, int size_z)
{
	switch (type) {
	case FG_COMP:
		if (fdata->vg_comp.vg_comp_data) {
			devm_kfree(dev,
				fdata->vg_comp.vg_comp_data);
			fdata->vg_comp.vg_comp_data = NULL;
		}
		if (size_x != 0 && size_y != 0 && size_z != 0)
			fdata->vg_comp.vg_comp_data =
				devm_kzalloc(dev, size_x * size_y * size_z *
						sizeof(struct data_point),
								GFP_KERNEL);
		if (fdata->vg_comp.vg_comp_data) {
			fdata->vg_comp.voltNR = size_x;
			fdata->vg_comp.tempNR = size_y;
			fdata->vg_comp.currNR = size_z;

		} else {
			fdata->vg_comp.voltNR = 0;
			fdata->vg_comp.tempNR = 0;
		}
		break;
	case FG_SOC_OFFSET:
		if (fdata->soc_offset.soc_offset_data) {
			devm_kfree(dev, fdata->soc_offset.soc_offset_data);
			fdata->soc_offset.soc_offset_data = NULL;
		}
		if (size_x != 0 && size_y != 0)
			fdata->soc_offset.soc_offset_data =
				devm_kzalloc(dev, size_x * size_y *
						sizeof(struct data_point),
								GFP_KERNEL);
		if (fdata->soc_offset.soc_offset_data) {
			fdata->soc_offset.soc_voltNR = size_x;
			fdata->soc_offset.tempNR = size_y;

		} else {
			fdata->soc_offset.soc_voltNR = 0;
			fdata->soc_offset.tempNR = 0;
		}
		break;
	default:
		BUG();
	}
}

#ifdef CONFIG_OF
struct dt_offset_params {
	int data[3];
};

static int rt5058_fg_parse_dt(struct device *dev,
			struct rt5058_fuelgauge_platform_data *fdata)
{
	struct device_node *np = dev->of_node;
	int sizes[3] = {0}, ret, j, i;
	struct dt_offset_params *offset_params;
	const char* fg_soc_table = "rt,fg_soc_table";
	const char* battery_profile = "rt,lgc_battery_fg_comp_data";
	int batt_id = 0;
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	batt_id = read_lge_battery_id();
#elif defined (CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER)
	struct lge_power *lge_batt_id_lpc;
	union lge_power_propval lge_val = {0, };

	lge_batt_id_lpc = lge_power_get_by_name("batt_id");
	if (!lge_batt_id_lpc) {
		pr_err("%s: lge_batt_id_lpc load failed\n", __func__);
	} else {
		lge_batt_id_lpc->get_property(lge_batt_id_lpc,
				LGE_POWER_PROP_BATTERY_ID_CHECKER, &lge_val);
		batt_id = lge_val.intval;
	}
#endif
	pr_err("%s(): batt_id = %d\n", __func__, batt_id);
	switch (batt_id) {
#if defined CONFIG_LGE_PM_BATTERY_RT5058_BL49JH_1940mAh
	case BATT_ID_SW3800_VC0:
		battery_profile = "rt,lgc_battery_fg_comp_data";
		pr_err("%s: SW3800_VC0, lgc_battery_fg_comp_data\n", __func__);
		break;
	case BATT_ID_RA4301_VC1:
		battery_profile = "rt,lgc_battery_fg_comp_data";
		pr_err("%s: RA4301_VC1, lgc_battery_fg_comp_data\n", __func__);
		break;
	case BATT_ID_RA4301_VC0:
		battery_profile ="rt,byd_battery_fg_comp_data";
		pr_err("%s: RA4301_VC0, byd_battery_fg_comp_data\n", __func__);
		break;
#elif defined CONFIG_LGE_PM_BATTERY_RT5058_BL46ZH_2125mAh
	case BATT_ID_SW3800_VC0:
	case BATT_ID_RA4301_VC1:
		battery_profile = "rt,lgc_battery_fg_comp_data";
		pr_err("%s: lgc_battery_fg_comp_data\n", __func__);
		break;
	case BATT_ID_SW3800_VC1:
	case BATT_ID_RA4301_VC0:
		battery_profile ="rt,tocad_battery_fg_comp_data";
		pr_err("%s: tocad_battery_fg_comp_data\n", __func__);
		break;
#endif
	default:
		battery_profile = "rt,lgc_battery_fg_comp_data";
		pr_err("%s: unknown, lgc_battery_fg_comp_data\n", __func__);
		break;
	}

	/* set fg_soc_table */
#if defined CONFIG_LGE_PM_BATTERY_RT5058_CUT_OFF_UNDER_3P4_USED
	fg_soc_table = "rt,fg_soc_table_3_4v";
#elif defined CONFIG_LGE_PM_BATTERY_RT5058_CUT_OFF_UNDER_3P5_USED
	fg_soc_table = "rt,fg_soc_table_3_5v";
#endif
	ret = of_property_read_u32_array(np, "rt,dtsi_version",
						fdata->dtsi_version, 2);
	if (ret < 0)
		fdata->dtsi_version[0] =
			fdata->dtsi_version[1] = 0;

	ret = of_property_read_u32_array(np, "rt,fg_comp_interpolation_order",
				fdata->vg_comp_interpolation_order, 3);
	if (ret < 0)
		fdata->vg_comp_interpolation_order[0] =
			fdata->vg_comp_interpolation_order[1] =
			fdata->vg_comp_interpolation_order[2] = 2;

	/*FG_COMP*/
	sizes[0] = sizes[1] = sizes[2] = 0;
	ret = of_property_read_u32_array(np, "rt,fg_comp_size", sizes, 3);
	if (ret < 0)
		pr_err("%s: Can't get prop fg_comp_size (%d)\n", __func__, ret);
	new_vgcomp_soc_offset_data(dev, FG_COMP,
			fdata, sizes[0], sizes[1], sizes[2]);
	/* Battery Profile */
	if (fdata->vg_comp.vg_comp_data) {
			of_property_read_u32_array(np, battery_profile,
					(u32 *)fdata->vg_comp.vg_comp_data,
					sizes[0] * sizes[1] * sizes[2] * 12);
	}
	pr_err("%s: battery_proflie = %s\n", __func__, battery_profile);

	ret = of_property_read_u32_array(np, "rt,offset_interpolation_order",
			fdata->offset_interpolation_order, 2);
	if (ret < 0)
		fdata->offset_interpolation_order[0] =
			fdata->offset_interpolation_order[1] = 2;

	sizes[0] = sizes[1] = 0;
	ret = of_property_read_u32_array(np, "rt,soc_offset_size", sizes, 2);
	if (ret < 0)
		pr_err("%s: Can't get prop soc_offset_size(%d)\n", __func__, ret);
	new_vgcomp_soc_offset_data(dev, FG_SOC_OFFSET,
						fdata, sizes[0], sizes[1], 0);
	if (fdata->soc_offset.soc_offset_data) {
		offset_params = devm_kzalloc(dev,
					     sizes[0] * sizes[1] *
					     sizeof(struct dt_offset_params),
					     GFP_KERNEL);
		if (offset_params == NULL)
			return -ENOMEM;

		of_property_read_u32_array(np, "rt,soc_offset_data",
				(u32 *)offset_params, sizes[0] * sizes[1] * 3);
		for (j = 0; j < sizes[0] * sizes[1];  ++j) {
			fdata->soc_offset.
				soc_offset_data[j].x = offset_params[j].data[0];
			fdata->soc_offset.
				soc_offset_data[j].y = offset_params[j].data[1];
			fdata->soc_offset.
				soc_offset_data[j].offset =
						offset_params[j].data[2];
		}
		devm_kfree(dev, offset_params);
	}

	fdata->wr_bc_en = of_property_read_bool(np, "rt,wr_bc_en") ? 1 : 0;
	fdata->use_sc_count =
			of_property_read_bool(np, "rt,use_sc_count") ? 1 : 0;

	ret = of_property_read_u32_array(np, "rt,battery_type",
					 &fdata->battery_type, 1);
	if (ret < 0) {
		dev_info(dev, "uset defualt battery_type 4350mV\n");
		fdata->battery_type = 4350;
	}

	ret = of_property_read_u32(np, "rt,temp_source_table",
					&fdata->temp_source);
	if (ret < 0)
		fdata->temp_source = 0;
	else {
		switch (fdata->temp_source) {
		case 1:
			fdata->temp_source = RT5058_REG_FGTEMP;
			break;
		case 2:
			fdata->temp_source = RT5058_REG_FGINTT;
			break;
		case 3:
			fdata->temp_source = RT5058_REG_FGAT;
			break;
		default:
			fdata->temp_source = 0;
			break;
		}
	}

	ret = of_property_read_u32(np, "rt,fg_oep_threshold",
					 &fdata->oep_threshold);
	if (ret < 0) {
		dev_info(dev, "no oep th property, use default 0x210a\n");
		fdata->oep_threshold = 0x210a;
	}

	ret = of_property_read_u32(np, "rt,fg_otut_threshold",
					 &fdata->otut_threshold);
	if (ret < 0) {
		dev_info(dev, "no otut th property, use default 0x7f80\n");
		fdata->otut_threshold = 0x7f80;
	}

	ret = of_property_read_u32(np, "rt,fg_uv_threshold",
					 &fdata->uv_threshold);
	if (ret < 0) {
		dev_info(dev, "no uv th property, use default 0x0000\n");
		fdata->uv_threshold = 0x0000;
	}

	ret = of_property_read_u32(np, "rt,fg_os_threshold",
					 &fdata->os_threshold);
	if (ret < 0) {
		dev_info(dev, "no os th property, use default 0xffff\n");
		fdata->os_threshold = 0xffff;
	}

	ret = of_property_read_u32(np, "rt,fg_us_threshold",
					 &fdata->us_threshold);
	if (ret < 0) {
		dev_info(dev, "no us th property, use default 0xffff\n");
		fdata->us_threshold = 0x0000;
	}

	ret = of_property_read_u32(np, "rt,fg_sc_step", &fdata->sc_step);
	if (ret < 0) {
		dev_info(dev, "no sc step property, use defualt 1 %%\n");
		fdata->sc_step = 16;
	}

	ret = of_property_read_u32(np, "rt,fg_slpvol_threshold",
					 &fdata->slpvol_threshold);
	if (ret < 0) {
		dev_info(dev, "no slpvol th property, use default 0x0096\n");
		fdata->slpvol_threshold = 0x0096;
	}

	ret = of_property_read_u32(np, "rt,fg_full_design",
					 &fdata->full_design);
	if (ret < 0) {
		dev_info(dev, "no design capacity property, use defaut 2100\n");
		fdata->full_design = 2100;
	}

	ret = of_property_read_u32_array(np, "rt,fg_function_table",
			(u32 *)fdata->function_table, 45);
	if (ret < 0) {
		dev_info(dev, "no fucntion tabale property\n");
		for (j = 0; j < 5; j++) {
			fdata->function_table[j].enable = 0;
			for (i = 0; i < 8; i++)
				fdata->function_table[j].data[i] = 0;
		}
	}

	ret = of_property_read_u32_array(np, fg_soc_table,
					(u32 *)fdata->soc_table, 80);
	if (ret < 0) {
		dev_info(dev, "no soc table property\n");
		for (j = 0; j < 10; j++)
			for (i = 0; i < 8; i++)
				fdata->soc_table[j].data[i] = 0;
	}

	ret = of_property_read_u32_array(np, "rt,fg_op_config",
					(u32 *)fdata->op_config, 3);
	if (ret < 0) {
		dev_info(dev, "no fg op config proeprty, use default\n");
		fdata->op_config[0] = 0x8400;
		fdata->op_config[1] = 0x0000;
		fdata->op_config[2] = 0x0000;
	}

	ret = of_property_read_u32_array(np, "rt,fg_irq_mask",
				(u32 *)fdata->irq_mask, 2);
	if (ret < 0) {
		dev_info(dev, "no irq mask property, use default 0xffff\n");
		fdata->irq_mask[0] = fdata->irq_mask[1] = 0xffff;
	}

	ret = of_property_read_u32_array(np, "rt,fg_soc_config",
					(u32 *)fdata->soc_config, 2);
	if (ret < 0) {
		dev_info(dev, "no soc config property, use defaunt 0x0000\n");
		fdata->soc_config[0] = fdata->soc_config[1] = 0;
	}

	ret = of_property_read_u32(np, "rt,fg_voltage_calibration",
					&fdata->vol_cali);
	if (ret < 0) {
		dev_info(dev, "no vol cali property, use default 0x0000\n");
		fdata->vol_cali = 0x0000;
	}

	ret = of_property_read_u32(np, "rt,fg_current_calibration",
					&fdata->cur_cali);
	if (ret < 0) {
		dev_info(dev, "no cur cali property, use default 0x8080\n");
		fdata->cur_cali = 0x8080;
	}

	ret = of_property_read_u32(np, "rt,cyc_adj_th",
					&fdata->cyc_adj_th);
	if (ret < 0) {
		dev_info(dev, "no cyc adj th property, use default 100\n");
		fdata->cyc_adj_th = 100;
	}

	ret = of_property_read_u32(np, "rt,cyc_adj_rat",
					&fdata->cyc_adj_rat);
	if (ret < 0) {
		dev_info(dev, "no cyc adj rat property, use default 0\n");
		fdata->cyc_adj_rat = 0;
	}

	ret = of_property_read_u32(np, "rt,fg_aging_factor",
					&fdata->fg_aging_factor);
	if (ret < 0) {
		dev_info(dev, "no aging factor property, use default 0x0032\n");
		fdata->fg_aging_factor = 0x0032;
	}

	ret = of_property_read_u32(np, "rt,fg_deadband",
					&fdata->fg_deadband);
	if (ret < 0) {
		dev_info(dev, "no deadband property, use default 0x0606\n");
		fdata->fg_deadband = 0x0606;
	}

	return 0;
}
#else
static int rt5058_fg_parse_dt(struct rt5058_fuelgauge_info *fuelgauge)
{
	return 0;
}
#endif

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
static void rt5058_fg_avg_quick_sensing(struct rt5058_fuelgauge_info *fi)
{
	int ret = 0;
	pr_info("%s\n", __func__);

	/* execute AVGQS Command */
	rt5058_i2c_write_word(fi->client, RT5058_REG_FGCTRL, 0xc000);
	mdelay(200);

	/* read average quick sensing source */
	ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FGOPCFG3);
	if ((ret&0x0006) >> 1 != 2)
		mdelay(100);
	if (ret < 0) {
		dev_err(fi->dev, "Quick Sensing fail\n");
		return;
	}

	fi->batt_soc = fg_get_soc(fi) / 10;
	/* call power_supply_changed */
	power_supply_changed(&fi->psy);
}
#endif

static void rt5058_handle_soc_changed(struct rt5058_fuelgauge_info *fi)
{
	int fg_soc = 0, soc_now_i, soc_old_i;
	int is_charging = 0;
	int is_eoc = 0;
	int regval = 0;
	int pre_soc;

	is_charging = rt5058_fg_is_charging(fi);
	is_eoc = rt5058_fg_is_eoc(fi);
	pre_soc = fi->soc_old;

	fg_soc = rt5058_i2c_read_word(fi->client, RT5058_REG_FGSOC);
	if (fi->fdata->use_sc_count) {
		soc_now_i = fg_soc / fi->sc_step;
		soc_now_i *= fi->sc_step;
		soc_old_i = fi->soc_old / fi->sc_step;
		soc_old_i *= fi->sc_step;

		pr_debug("%s: abs(soc_now_i - soc_old_i) = %d\n", __func__,
				(int)abs(soc_now_i - soc_old_i));
		if (abs(soc_now_i - soc_old_i) < (2*fi->sc_step)) {
			rt5058_assign_bits16(fi->client, RT5058_REG_FGSOCCFG1,
					RT5058_SCIRQACCEN_MASK, 0);
			mdelay(10);
			rt5058_assign_bits16(fi->client, RT5058_REG_FGSOCCFG1,
					RT5058_SCIRQACCEN_MASK, RT5058_SCIRQACCEN_MASK);
			fi->soc_old = fg_soc;
			pr_debug("%s: new soc_old = %d\n", __func__, fi->soc_old);
		} else {
			pr_err("%s: Ops soc skip!\n", __func__);
			pr_err("%s: fg_soc = %d, pre_soc = %d\n",
					__func__, fg_soc/256, pre_soc/256);
			fg_get_soc(fi);
			if (fg_soc > fi->soc_old) {
				fi->soc_old += fi->sc_step;
				fi->batt_soc = fi->batt_soc + 1;
			} else {
				fi->soc_old -= fi->sc_step;
				fi->batt_soc = fi->batt_soc - 1;
			}
			rt5058_i2c_write_word(fi->client,
					RT5058_REG_FGMFA, 0x8600 | fi->soc_old / 128);
			pr_err("%s: new batt_soc = %d\n", __func__, fi->batt_soc);
			power_supply_changed(&fi->psy);
			return;
		}
	}
	pr_err("%s: fg_soc = %d, pre_soc = %d\n", __func__, fg_soc/256, pre_soc/256);
	if (fg_soc > pre_soc && !is_charging && !is_eoc) {
		/* When TA not exist, keep SOC Value as pre_soc*/
		pr_err("%s: Do not update fi->batt_soc\n", __func__);
		pr_err("%s: is_charging=%d, is_eoc=%d\n", __func__, is_charging, is_eoc);
		regval = 0x8600 | pre_soc / 128;
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGMFA, regval);
		fi->soc_old = pre_soc;
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP1, 0xd000);
		rt5058_i2c_write_word(fi->client, RT5058_REG_FGCSCOMP3, 0x0000);
	} else { /* report new soc to Battery */
		fi->batt_soc = fg_get_soc(fi) / 10;
		pr_err("%s: new batt_soc = %d\n", __func__, fi->batt_soc);
		power_supply_changed(&fi->psy);
	}
}

static void rt5058_handle_under_voltage(struct rt5058_fuelgauge_info *fi)
{
	int ret;
	union power_supply_propval value;
	struct power_supply *chg_psy;

	ret = rt5058_i2c_read_word(fi->client, RT5058_REG_FG_STATUS1);
	if (ret < 0)
		return;

	pr_info("%s: IRQ: Under Voltage\n", __func__);
	chg_psy = power_supply_get_by_name(rt5058_chg_devname);

	if (chg_psy) {
		psy_do_property(rt5058_chg_devname, get,
				POWER_SUPPLY_PROP_CHARGER_EOC, value);
	}
	else {
		value.intval = 0;
		pr_err("%s: chg_psy is not yet ready.\n", __func__);
	}

	pr_info("%s: charger_eoc : %d\n", __func__, value.intval);
	if (ret & RT5058_FGUV_MASK) {
		if (value.intval) {
			pr_info("Force Recharge by S/W\n, __func__");
			psy_do_property(rt5058_chg_devname, set,
				POWER_SUPPLY_PROP_CHARGE_TYPE, value);
		}

		rt5058_assign_bits16(fi->client, RT5058_REG_FG_MASK1,
						RT5058_FGUV_MASK, RT5058_FGUV_MASK);
		rt5058_assign_bits16(fi->client, RT5058_REG_FG_MASK1,
						RT5058_FGUV_MASK, 0);
		rt5058_assign_bits16(fi->client,
			RT5058_REG_FG_UVDET, 0x00ff, fi->fdata->uv_threshold);
	}
}

static void rt5058_fuelirq_handler(void *info, int eventno)
{
	struct rt5058_fuelgauge_info *fi = info;

	pr_info("%s, eventno = %d\n", __func__, eventno);
	rt5058_fg_reginfo(fi);
	switch (eventno) {
	case FUELEVENT_DSG:
		pr_info("IRQ: fuelgauge Discharging irq\n");
		break;
	case FUELEVENT_RDY:
		pr_info("IRQ: fuelgauge Initialization Ready irq\n");
		break;
	case FUELEVENT_QSDONE:
		pr_info("IRQ: fuelgauge Quick Sensing Done irq\n");
		break;
	case FUELEVENT_RI:
		pr_info("IRQ: fuelgauge Reset irq\n");
		break;
	case FUELEVENT_SHDN:
		pr_info("IRQ: fuelgauge Shutdown irq\n");
		break;
	case FUELEVENT_SLP:
		pr_info("IRQ: fuelgauge Sleep Mode irq\n");
		break;
	case FUELEVENT_PRESRDY:
		pr_info("IRQ: fuelgauge Battery Presence ready irq\n");
		break;
	case FUELEVENT_OEPPACT:
		pr_info("IRQ: fuelgauge Over Engergy Protection irq\n");
		break;
	case FUELEVENT_BATTYPE:
		pr_info("IRQ: fuelgauge Battery Type irq\n");
		break;
	case FUELEVENT_BATPRES:
		pr_info("IRQ: fuelgauge Battery Presence irq\n");
		break;
	case FUELEVENT_SC:
		rt5058_handle_soc_changed(fi);
		break;
	case FUELEVENT_EOD:
		pr_info("IRQ: fuelgauge End-of-Discharge irq\n");
		break;
	case FUELEVENT_EOC:
		pr_info("IRQ: fuelgauge End-of-Charging irq\n");
		break;
	case FUELEVENT_US:
		pr_info("IRQ: fuelgauge under soc\n");
		break;
	case FUELEVENT_OS:
		pr_info("IRQ: fuelgauge over soc\n");
		break;
	case FUELEVENT_UV:
		rt5058_handle_under_voltage(fi);
		break;
	case FUELEVENT_UT:
	case FUELEVENT_OT:
		pr_info("IRQ: fuelgauge temp changed irq\n");
		break;
	default:
		break;
	}
}

static rt_irq_handler rt_fuelirq_handler[FUELEVENT_MAX] = {
	[FUELEVENT_DSG] = rt5058_fuelirq_handler,
	[FUELEVENT_RDY] = rt5058_fuelirq_handler,
	[FUELEVENT_QSDONE] = rt5058_fuelirq_handler,
	[FUELEVENT_RI] = rt5058_fuelirq_handler,
	[FUELEVENT_SHDN] = rt5058_fuelirq_handler,
	[FUELEVENT_SLP] = rt5058_fuelirq_handler,
	[FUELEVENT_PRESRDY] = rt5058_fuelirq_handler,
	[FUELEVENT_OEPPACT] = rt5058_fuelirq_handler,
	[FUELEVENT_BATTYPE] = rt5058_fuelirq_handler,
	[FUELEVENT_BATPRES] = rt5058_fuelirq_handler,
	[FUELEVENT_SC] = rt5058_fuelirq_handler,
	[FUELEVENT_EOD] = rt5058_fuelirq_handler,
	[FUELEVENT_EOC] = rt5058_fuelirq_handler,
	[FUELEVENT_US] = rt5058_fuelirq_handler,
	[FUELEVENT_OS] = rt5058_fuelirq_handler,
	[FUELEVENT_UV] = rt5058_fuelirq_handler,
	[FUELEVENT_UT] = rt5058_fuelirq_handler,
	[FUELEVENT_OT] = rt5058_fuelirq_handler,
};

static irqreturn_t rt5058_fuel_irq_handler(int irqno, void *param)
{
	struct rt5058_fuelgauge_info *fi =
				(struct rt5058_fuelgauge_info *)param;
	struct i2c_client *client = fi->client;
	int ret[2], regval, i;

	ret[0] = rt5058_i2c_read_word(client, RT5058_REG_FG_IRQ1);
	ret[0] = (ret[0] < 0) ? 0 : ret[0];
	ret[1] = rt5058_i2c_read_word(client, RT5058_REG_FG_IRQ2);
	ret[1] = (ret[1] < 0) ? 0 : ret[1];
	fi->irq_occured_flag[0] = ret[0];
	fi->irq_occured_flag[1] = ret[1];

	regval = (fi->irq_occured_flag[0] &
			~(fi->irq_mask[0]))<<16 |
		(fi->irq_occured_flag[1] &
			~(fi->irq_mask[1]));
	pr_info("%s: irq_staus & irq_mask = 0x%08x\n", __func__, regval);
	for (i = 0; i < FUELEVENT_MAX; i++) {
		if ((regval & (1 << i)) && rt_fuelirq_handler[i])
			rt_fuelirq_handler[i](fi, i);
	}
	return IRQ_HANDLED;
}

static int rt5058_fuelgauge_irqinit(struct platform_device *pdev)
{
	struct rt5058_fuelgauge_info *fi = platform_get_drvdata(pdev);
	int ret;

	pr_info("%s\n", __func__);
	pr_err("%s: clear soc changed bit\n", __func__);
	rt5058_assign_bits16(fi->client, RT5058_REG_FG_STATUS1, RT5058_REG_FG_SC_MASK, 0);
	rt5058_clr_bits(fi->client, RT5058_REG_IRQMSK, RT5058_FG_IRQMASK);
	ret = platform_get_irq_byname(pdev, "FUEL_IRQ");
	if (ret < 0)
		return ret;
	ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
			rt5058_fuel_irq_handler,
			IRQF_TRIGGER_NONE, "FUEL_IRQ", fi);
	if (ret < 0) {
		dev_err(fi->dev, "request FUEL_IRQ fail\n");
		return ret;
	}
	/* unmask fg irqs */
	ret = rt5058_i2c_write_word(fi->client, RT5058_REG_FG_MASK1,
					fi->fdata->irq_mask[0]);
	ret = rt5058_i2c_write_word(fi->client, RT5058_REG_FG_MASK2,
					fi->fdata->irq_mask[1]);
	return ret;
}

static void rt5058_fuelgauge_irq_deinit(struct platform_device *pdev)
{
	struct rt5058_fuelgauge_info *fi = platform_get_drvdata(pdev);
	int ret;

	pr_debug("%s\n", __func__);
	rt5058_set_bits(fi->client, RT5058_REG_IRQMSK, RT5058_FG_IRQMASK);
	ret = platform_get_irq_byname(pdev, "FUEL_IRQ");
	if (ret < 0)
		return;
	devm_free_irq(&pdev->dev, ret, fi);
}

static char *rt_fuelgauge_supply_to_list[] = {
	"battery",
	"rt5058-charger",
};

static enum power_supply_property rt5058_fuel_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
};

static int rt5058_fuel_get_property(struct power_supply *psy,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	struct rt5058_fuelgauge_info *fi = dev_get_drvdata(psy->dev->parent);
	int inow;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = fi->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: /* Battery average Voltage */
		if (fi->suspend) {
			val->intval = fi->avg_vbat * 1000;
		} else {
			val->intval = fg_get_avg_vbat(fi) * 1000;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: /* Battery average Current */
		if (fi->suspend) {
			inow = fi->avg_curr;
		} else {
			inow = fg_get_current(fi);
		}
		if (inow > 32768)
			inow -= 65536;
		val->intval = inow * (-1000);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = fi->fdata->full_design;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = fi->batt_soc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
			val->intval = fi->temperature;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int rt5058_fuel_set_property(struct power_supply *psy,
			     enum power_supply_property psp,
			     const union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
	default:
		break;
	}
	return 0;
}

#if RT5058_FG_DEBUG
static void rt5058_fg_platform_data_check(
				struct rt5058_fuelgauge_info *fi)
{
	int j;

	pr_info("%s\n", __func__);
	RTINFO("dtsi version = <%02d %04d>\n",
			fi->fdata->dtsi_version[0],
			fi->fdata->dtsi_version[1]);
	RTINFO("fg_comp interpolation order = <%d %d %d>\n",
	      fi->fdata->vg_comp_interpolation_order[0],
	      fi->fdata->vg_comp_interpolation_order[1],
	      fi->fdata->vg_comp_interpolation_order[2]);
	RTINFO("fg_comp_size = <%d %d %d>\n",
			fi->fdata->vg_comp.voltNR,
			fi->fdata->vg_comp.tempNR,
			fi->fdata->vg_comp.currNR);
	RTINFO("fg_comp_data\n");
	for (j = 0; j < fi->fdata->vg_comp.voltNR *
			fi->fdata->vg_comp.tempNR *
			fi->fdata->vg_comp.currNR; ++j) {
		RTINFO("<%d %d %d %d %d %d %d %d %d %d %d %d>\n",
		fi->fdata->vg_comp.vg_comp_data[j].x,
		fi->fdata->vg_comp.vg_comp_data[j].y,
		fi->fdata->vg_comp.vg_comp_data[j].z,
		fi->fdata->vg_comp.vg_comp_data[j].data[0],
		fi->fdata->vg_comp.vg_comp_data[j].data[1],
		fi->fdata->vg_comp.vg_comp_data[j].data[2],
		fi->fdata->vg_comp.vg_comp_data[j].data[3],
		fi->fdata->vg_comp.vg_comp_data[j].data[4],
		fi->fdata->vg_comp.vg_comp_data[j].data[5],
		fi->fdata->vg_comp.vg_comp_data[j].data[6],
		fi->fdata->vg_comp.vg_comp_data[j].data[7],
		fi->fdata->vg_comp.vg_comp_data[j].data[8]
		);
	}
	RTINFO("offset interpolation order = <%d %d>\n",
	      fi->fdata->offset_interpolation_order[0],
	      fi->fdata->offset_interpolation_order[1]);
	RTINFO("soc_offset_size = <%d %d>\n",
		fi->fdata->soc_offset.soc_voltNR,
		fi->fdata->soc_offset.tempNR);

	RTINFO("fg_soc_offset_data\n");
	for (j = 0; j < fi->fdata->soc_offset.soc_voltNR *
			fi->fdata->soc_offset.tempNR; ++j) {
		RTINFO("<%d %d %d>\n",
			fi->fdata->soc_offset.soc_offset_data[j].x,
			fi->fdata->soc_offset.soc_offset_data[j].y,
			fi->fdata->soc_offset.soc_offset_data[j].offset);
	}
	RTINFO("wr_bc_en  = <%d>\n", fi->fdata->wr_bc_en);
	RTINFO("use_sc_count = <%d>\n", fi->fdata->use_sc_count);
	RTINFO("battery_type = <%d>\n", fi->fdata->battery_type);
	RTINFO("temp_source = 0x%02x\n", fi->fdata->temp_source);
	RTINFO("oep_threshold = 0x%04x\n", fi->fdata->oep_threshold);
	RTINFO("otut_threshold = 0x%04x\n", fi->fdata->otut_threshold);
	RTINFO("uv_threshold = %dmV\n",
				(fi->fdata->uv_threshold&0x00ff)*20);
	RTINFO("sc_step = %d.%04d%%\n", fi->fdata->sc_step/16,
				(fi->fdata->sc_step%16)*10000/16);
	RTINFO("os_threshold = 0x%04x\n", fi->fdata->os_threshold);
	RTINFO("us_threshold = 0x%04x\n", fi->fdata->us_threshold);
	RTINFO("slpvol_threshold = 0x%04x\n",
					fi->fdata->slpvol_threshold);
	RTINFO("design capacity = %d\n", fi->fdata->full_design);
	RTINFO("fg_function_table\n");
	for (j = 0; j < 5; j++) {
		RTINFO("<%d 0x%04x 0x%04x 0x%04x 0x%04x ",
			fi->fdata->function_table[j].enable,
			fi->fdata->function_table[j].data[0],
			fi->fdata->function_table[j].data[1],
			fi->fdata->function_table[j].data[2],
			fi->fdata->function_table[j].data[3]);
		RTINFO("   0x%04x 0x%04x 0x%04x 0x%04x>\n",
			fi->fdata->function_table[j].data[4],
			fi->fdata->function_table[j].data[5],
			fi->fdata->function_table[j].data[6],
			fi->fdata->function_table[j].data[7]);
	}
	RTINFO("fg_soc_table\n");
	for (j = 0; j < 10; j++) {
		RTINFO("<0x%04x 0x%04x 0x%04x 0x%04x ",
			fi->fdata->soc_table[j].data[0],
			fi->fdata->soc_table[j].data[1],
			fi->fdata->soc_table[j].data[2],
			fi->fdata->soc_table[j].data[3]);
		RTINFO(" 0x%04x 0x%04x 0x%04x 0x%04x>\n",
			fi->fdata->soc_table[j].data[4],
			fi->fdata->soc_table[j].data[5],
			fi->fdata->soc_table[j].data[6],
			fi->fdata->soc_table[j].data[7]);
	}
	RTINFO("op_config = <0x%04x 0x%04x 0x%04x>\n",
				fi->fdata->op_config[0],
				fi->fdata->op_config[1],
				fi->fdata->op_config[2]);
	RTINFO("fg_irq_mask = <0x%04x 0x%04x>\n",
				fi->fdata->irq_mask[0],
				fi->fdata->irq_mask[1]);
	RTINFO("soc_config = <0x%04x 0x%04x>\n",
				fi->fdata->soc_config[0],
				fi->fdata->soc_config[1]);
	RTINFO("vol_cali = <0x%04x>\n", fi->fdata->vol_cali);
	RTINFO("cur_cali = <0x%04x>\n", fi->fdata->cur_cali);
	RTINFO("cyc_adj_th = <%d>\n", fi->fdata->cyc_adj_th);
	RTINFO("cyc_adj_rat = <%d>\n", fi->fdata->cyc_adj_rat);
	RTINFO("fg_aging_factor = <0x%04x>\n",
					fi->fdata->fg_aging_factor);
	RTINFO("fg_deadband = <0x%04x>\n", fi->fdata->fg_deadband);
}
#endif

static void rt5058_update_polling_worker(struct work_struct *work)
{
	struct rt5058_fuelgauge_info *fi;
	int new_soc;

	fi = container_of(work, struct rt5058_fuelgauge_info,
			update_polling_work.work);

	new_soc = fg_get_soc(fi)/10;
	if (new_soc != fi->batt_soc) {
		pr_err("old_soc = %d, new_soc = %d, change soc\n",
				fi->batt_soc, new_soc);
		fi->batt_soc = new_soc;
		power_supply_changed(&fi->psy);
	}

	schedule_delayed_work(&fi->update_polling_work, msecs_to_jiffies(40000));
	return;
}
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
static int at_fuel_gauge_reset_show(
		struct device *dev, struct device_attribute *attr, char *buf) {
	pr_err("%s: [AT_CMD][at_fuel_gauge_reset_show]\n", __func__);
	rt5058_fg_avg_quick_sensing(the_fuel_info);
	return snprintf(buf, PAGE_SIZE, "%d\n", true);
}

static int at_rt_chcomp_show(
		struct device *dev, struct device_attribute *attr, char *buf) {
	union power_supply_propval value;
	pr_err("%s: [AT_CMD][at_rt_chcomp_show]\n", __func__);
	psy_do_property(rt5058_batt_devname, get,
			POWER_SUPPLY_PROP_CAPACITY, value);
	if (value.intval >= 100) {
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	} else {
		return snprintf(buf, PAGE_SIZE, "%d\n", 1);
	}
}

DEVICE_ATTR(at_fuelrst, 0444, at_fuel_gauge_reset_show, NULL);
DEVICE_ATTR(at_rt_chcomp, 0444, at_rt_chcomp_show, NULL);
#endif

static int rt5058_fuelgauge_probe(struct platform_device *pdev)
{
	struct rt5058_mfd_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct rt5058_fuelgauge_info *fi;
	struct rt5058_fuelgauge_platform_data *fdata;
	bool use_dt = pdev->dev.of_node;
	int ret;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_BATTERY_ID_CHECKER
	struct lge_power *lge_batt_id_lpc;

	lge_batt_id_lpc = lge_power_get_by_name("batt_id");
	if (!lge_batt_id_lpc) {
		pr_err("%s(), lge_batt_id_lpc is not yet ready\n", __func__);
		return -EPROBE_DEFER;
	}
#endif
	pr_info("%s\n", __func__);

	if (use_dt) {
		fdata = devm_kzalloc(&pdev->dev, sizeof(*fdata), GFP_KERNEL);
		if (!fdata) {
			dev_err(&pdev->dev, "fail to allocate memory\n");
			return -ENOMEM;
		}
		ret = rt5058_fg_parse_dt(&pdev->dev, fdata);
		if (ret)
			return -ENOMEM;
	} else {
		dev_err(&pdev->dev, "no dts node\n");
		return -ENODEV;
	}

	fi = devm_kzalloc(&pdev->dev, sizeof(*fi), GFP_KERNEL);
	if (!fi) {
		pr_err("kzalloc() failed\n");
		return -ENOMEM;
	}
	fi->client = chip->client;
	fi->dev = chip->dev;
	fi->fdata = fdata;
#if RT5058_FG_DEBUG
	rt5058_fg_platform_data_check(fi);
#endif
	platform_set_drvdata(pdev, fi);

	mutex_init(&fi->param_lock);

	fi->vadc_dev = qpnp_get_vadc(&pdev->dev, "rt5058-fg");
	if (IS_ERR(fi->vadc_dev)) {
		ret = PTR_ERR(fi->vadc_dev);
		if (ret != -EPROBE_DEFER)
			pr_err("vadc property missing\n");
		else
			pr_err("probe defer due to not initializing vadc\n");
		goto out_init;
	}

	ret = rt5058_fg_init(fi);
	if (ret < 0) {
		dev_err(fi->dev, "rt5058 fuelgauge init fail\n");
		goto out_init;
	}

	fi->suspend = false;
	fi->fullsoc_cali = 0;

	fi->psy.name = rt5058_fuel_devname;
	fi->psy.type = POWER_SUPPLY_TYPE_FUELGAUGE;
	fi->psy.set_property = rt5058_fuel_set_property;
	fi->psy.get_property = rt5058_fuel_get_property;
	fi->psy.supplied_to = rt_fuelgauge_supply_to_list;
	fi->psy.num_supplicants =
		ARRAY_SIZE(rt_fuelgauge_supply_to_list);
	fi->psy.properties = rt5058_fuel_props;
	fi->psy.num_properties = ARRAY_SIZE(rt5058_fuel_props);

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	ret = device_create_file(&pdev->dev, &dev_attr_at_fuelrst);
	if (ret < 0) {
		pr_err("file device creation failed\n");
		goto out_dev;
	}
	ret = device_create_file(&pdev->dev, &dev_attr_at_rt_chcomp);
	if (ret < 0) {
		pr_err("file device creation failed\n");
		goto out_dev;
	}
#endif

	ret = power_supply_register(&pdev->dev, &fi->psy);
	if (ret < 0) {
		dev_err(&pdev->dev, "rt5058 fuelgauge supply register fail\n");
		goto out_dev;
	}
	/* set fg_parameter */
	fg_get_soc(fi);

	/* for rt5052 polling soc update */
	if (fi->fg_ver == 1) {
		fi->fdata->use_sc_count = 0;
		INIT_DELAYED_WORK(&fi->update_polling_work,
				rt5058_update_polling_worker);
		pr_err("start soc update polling work\n");
		schedule_delayed_work(&fi->update_polling_work,
				msecs_to_jiffies(40000));
	}

	ret = rt5058_fuelgauge_irqinit(pdev);
	if (ret < 0) {
		dev_err(fi->dev, "rt5058 fg irq int fail\n");
		goto out_irq;
	}

	chip->fuelgauge = fi;
#if RT5058_FG_DEBUG
	wake_lock_init(&fi->dwork_lock, WAKE_LOCK_SUSPEND, "rt5058fg_dwork_lock");
	INIT_DELAYED_WORK(&fi->dwork, rt5058_dwork_func);
	schedule_delayed_work(&fi->dwork, msecs_to_jiffies(10000));
#endif
	the_fuel_info = fi;
	rt5058_fg_reginfo(chip->fuelgauge);

	pr_info("rt5058 fuelgauge probe OK\n");
	return 0;

out_irq:
	power_supply_unregister(&fi->psy);
out_init:
out_dev:
	devm_kfree(&pdev->dev, fi);
	return ret;
}

static void rt5058_fuelgauge_shutdown(struct platform_device *pdev)
{
	struct rt5058_fuelgauge_info *fi = platform_get_drvdata(pdev);
	int ret;

	/* F/G shutdown */
	ret = rt5058_i2c_write_word(fi->client, RT5058_REG_FGMFA, RT5058_FG_ENTER_SHDN);
	if (ret < 0)
		pr_err("%s: shutdown fuel-gauge failed\n", __func__);
}

static int rt5058_fuelgauge_remove(struct platform_device *pdev)
{
	struct rt5058_fuelgauge_info *fi = platform_get_drvdata(pdev);

	if (!fi) {
		return 0;
	} else {
		if(fi->fg_ver == 1)
			cancel_delayed_work_sync(&fi->update_polling_work);
		rt5058_fuelgauge_irq_deinit(pdev);
		power_supply_unregister(&fi->psy);
		devm_kfree(&pdev->dev, fi);
	}
	return 0;
}

static int rt5058_fuelgauge_suspend(
			struct platform_device *pdev, pm_message_t mesg)
{
	struct rt5058_fuelgauge_info *fi = platform_get_drvdata(pdev);
	fi->suspend = true;
	return 0;
}

static int rt5058_fuelgauge_resume(struct platform_device *pdev)
{
	struct rt5058_fuelgauge_info *fi = platform_get_drvdata(pdev);

	fi->suspend = false;
#if RT5058_FG_DEBUG
	rt5058_dwork_show(fi);
#endif
	return 0;
}

static const struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,rt5058-fuelgauge",},
	{},
};

static struct platform_driver rt5058_fuel_driver = {
	.driver = {
		   .name = RT5058_FUEL_DEVNAME,
		   .owner = THIS_MODULE,
		   .of_match_table = rt_match_table,
		   },
	.probe = rt5058_fuelgauge_probe,
	.remove = rt5058_fuelgauge_remove,
	.suspend = rt5058_fuelgauge_suspend,
	.resume = rt5058_fuelgauge_resume,
	.shutdown = rt5058_fuelgauge_shutdown,
};

static int __init rt5058_fuel_init(void)
{
	return platform_driver_register(&rt5058_fuel_driver);
}
module_init(rt5058_fuel_init);

static void __exit rt5058_fuel_exit(void)
{
	platform_driver_unregister(&rt5058_fuel_driver);
}

module_exit(rt5058_fuel_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_DESCRIPTION("RT5058 FUELGAUGE Driver");
MODULE_ALIAS(rt5058_fuel_devname);
MODULE_VERSION("1.0.0_LG");
