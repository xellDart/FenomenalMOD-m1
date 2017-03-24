/* linux/driver/input/drivers/sensor/pas230.c
 * Copyright (C) 2014 Partron Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "pas230.h"

/* For debugging */
//#undef DEBUG
#define DEBUG

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
};

struct sensor_dt_to_pdata_map {
	const char					*dt_name;
	void						*ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type	type;
	int							default_val;
};
#endif

#define	VENDOR		"PARTRON"
#define	CHIP_ID		"PAS230"

#define LGE_PROXIMITY_NAME	"lge_proximity"
#define LGE_LIGHT_NAME		"lge_light"
#define PAS230_DRV_NAME		"pas230"
#define PROX_CAL_DATA_PATH	"/sns/prox_calibration.dat"
#define PROX_READ_NUM		1	/*40*/

enum {
	MAIN_CTRL=0x00,
	PS_LED,
	PS_PULSES,
	PS_MEAS_RATE,
	ALS_CS_MEAS_RATE,
	ALS_CS_GAIN,
	PART_ID,
	MAIN_STATUS,
	PS_DATA=0x08,
	CLEAR_DATA=0x0a,
	GREEN_DATA=0x0d,
	BLUE_DATA=0x10,
	RED_DATA=0x13,
	COMP_DATA=0x16,
	INT_CFG=0x19,
	INT_PST,
	PS_THRES_UP=0x1b,
	PS_THRES_LOW=0x1d,
	PS_CAN=0x1f,
	ALS_THRES_UP=0x21,
	ALS_THRES_LOW=0x24,
	ALS_THRES_VAR=0x27
};

static u8 reg_defaults[40] = {
 	0x03, /* 0x00_0 : MAIN_CTRL */
	0x36, /* 0x01_1 : PS_LED */
	0x08, /* 0x02_2 : PS_PULSES */
	0x45, /* 0x03_3 : PS_MEAS_RATE */
	0x22, /* 0x04_4 : ALS_CS_MEAS_RATE */
	0x01, /* 0x05_5 : ALS_CS_GAIN */
	0xb0, /* 0x06_6 : PART_ID */
	0x00, /* 0x07_7 : MAIN_STATUS */
	0x00, 0x00, /* 0x08_8 : PS_DATA */
	0x00, 0x00, 0x00, /* 0x0a_10 : CLEAR_DATA */
	0x00, 0x00, 0x00, /* 0x0d_13 : GREEN_DATA */
	0x00, 0x00, 0x00, /* 0x10_16 : BLUE_DATA */
	0x00, 0x00, 0x00, /* 0x13_19 : RED_DATA */
	0x00, 0x00, 0x00, /* 0x16_22 : COMP_DATA */	
	0x12, /* 0x19_25 : INT_CFG */
	0x00, /* 0x1a_26 : INT_PST */
	0x14, 0x00, /* 0x1b_27 : PS_THRES_UP, 2047_80 */
	0x0a, 0x00, /* 0x1d_29 : PS_THRES_LOW, 0_65 */
	0x00, 0x00, /* 0x1f_31 : PS_CAN, 2047_0 */
	0xff, 0xff, 0x0f, /* 0x21_33 : ALS_THRES_UP */
	0x00, 0x00, 0x00, /* 0x24_36 : ALS_THRES_LOW */
	0x00, /* 0x27_39 : ALS_THRES_VAR */
};

#define PS_ON		(reg_defaults[0]&0x01)
#define PS_OFF		(reg_defaults[0]&(0x01^0xff))
#define ALS_CS_ON	(reg_defaults[0]&0x02)
#define ALS_CS_OFF	(reg_defaults[0]&(0x02^0xff))
#define ALL_ON		(reg_defaults[0]&0x03)
#define ALL_OFF		(reg_defaults[0]&(0x03^0xff))

enum {
	LIGHT_ENABLED = BIT(0),
	PROXIMITY_ENABLED = BIT(1),
};

/* driver data */
struct pas230_data {
	struct input_dev *proximity_input_dev;
	struct input_dev *light_input_dev;
	struct i2c_client *i2c_client;
	struct work_struct work_light;
	struct work_struct work_prox;
	struct hrtimer light_timer;
	struct hrtimer prox_timer;
	struct mutex power_lock;
	struct wake_lock prx_wake_lock;
	struct workqueue_struct *light_wq;
	struct workqueue_struct *prox_wq;
	struct class *lightsensor_class;
	struct class *proximity_class;
	struct device *lightsensor_dev;
	struct device *proximity_dev;
	struct pas230_platform_data *pdata;
	int irq;
	int avg[3];
	int crosstalk;
	int cal_result;
	int near_threshold;
	int far_threshold;
	
	ktime_t light_poll_delay;
	ktime_t prox_poll_delay;
	u8 power_state;
};

#ifdef CONFIG_OF
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
	       regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int sensor_regulator_configure(struct pas230_data *data, bool on)
{
	struct i2c_client *client = data->i2c_client;
	struct pas230_platform_data *pdata = data->pdata;
	int rc;

	if (on == false)
		goto hw_shutdown;

	pdata->vcc_ana = regulator_get(&client->dev, "Partron,vdd_ana");
	if (IS_ERR(pdata->vcc_ana)) {
		rc = PTR_ERR(pdata->vcc_ana);
		dev_err(&client->dev,
			"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->vcc_ana) > 0) {
		rc = regulator_set_voltage(pdata->vcc_ana, pdata->vdd_ana_supply_min,
					   pdata->vdd_ana_supply_max);

		if (rc) {
			dev_err(&client->dev,
				"regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
	if (pdata->digital_pwr_regulator) {
		pdata->vcc_dig = regulator_get(&client->dev, "Partron,vddio_dig");
		if (IS_ERR(pdata->vcc_dig)) {
			rc = PTR_ERR(pdata->vcc_dig);
			dev_err(&client->dev,
				"Regulator get dig failed rc=%d\n", rc);
			goto error_get_vtg_vcc_dig;
		}

		if (regulator_count_voltages(pdata->vcc_dig) > 0) {
			rc = regulator_set_voltage(pdata->vcc_dig,
						   pdata->vddio_dig_supply_min, pdata->vddio_dig_supply_max);
			if (rc) {
				dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_vcc_dig;
			}
		}
	}
	if (pdata->i2c_pull_up) {
		pdata->vcc_i2c = regulator_get(&client->dev, "Partron,vddio_i2c");
		if (IS_ERR(pdata->vcc_i2c)) {
			rc = PTR_ERR(pdata->vcc_i2c);
			dev_err(&client->dev,
				"Regulator get failed rc=%d\n", rc);
			goto error_get_vtg_i2c;
		}
		if (regulator_count_voltages(pdata->vcc_i2c) > 0) {
			rc = regulator_set_voltage(pdata->vcc_i2c,
						   pdata->vddio_i2c_supply_min, pdata->vddio_i2c_supply_max);
			if (rc) {
				dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_i2c;
			}
		}
	}

	return 0;

error_set_vtg_i2c:
	regulator_put(pdata->vcc_i2c);
error_get_vtg_i2c:
	if (pdata->digital_pwr_regulator)
		if (regulator_count_voltages(pdata->vcc_dig) > 0)
			regulator_set_voltage(pdata->vcc_dig, 0,
					      pdata->vddio_dig_supply_max);
error_set_vtg_vcc_dig:
	if (pdata->digital_pwr_regulator)
		regulator_put(pdata->vcc_dig);
error_get_vtg_vcc_dig:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);
error_set_vtg_vcc_ana:
	regulator_put(pdata->vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);
	regulator_put(pdata->vcc_ana);
	if (pdata->digital_pwr_regulator) {
		if (regulator_count_voltages(pdata->vcc_dig) > 0)
			regulator_set_voltage(pdata->vcc_dig, 0,
					      pdata->vddio_dig_supply_max);

		regulator_put(pdata->vcc_dig);
	}
	if (pdata->i2c_pull_up) {
		if (regulator_count_voltages(pdata->vcc_i2c) > 0)
			regulator_set_voltage(pdata->vcc_i2c, 0,
					      pdata->vddio_i2c_supply_max);
		regulator_put(pdata->vcc_i2c);
	}
	return 0;
}

static int sensor_regulator_power_on(struct pas230_data *data, bool on)
{
	struct i2c_client *client = data->i2c_client;
	struct pas230_platform_data *pdata = data->pdata;

	int rc;

	if (on == false)
		goto power_off;

	rc = reg_set_optimum_mode_check(pdata->vcc_ana, pdata->vdd_ana_load_ua);
	if (rc < 0) {
		dev_err(&client->dev,
			"Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(pdata->vcc_ana);
	if (rc) {
		dev_err(&client->dev,
			"Regulator vcc_ana enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_ana;
	}

	if (pdata->digital_pwr_regulator) {
		rc = reg_set_optimum_mode_check(pdata->vcc_dig,
						pdata->vddio_dig_load_ua);
		if (rc < 0) {
			dev_err(&client->dev,
				"Regulator vcc_dig set_opt failed rc=%d\n",
				rc);
			goto error_reg_opt_vcc_dig;
		}

		rc = regulator_enable(pdata->vcc_dig);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_dig enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_dig;
		}
	}

	if (pdata->i2c_pull_up) {
		rc = reg_set_optimum_mode_check(pdata->vcc_i2c, pdata->vddio_i2c_load_ua);
		if (rc < 0) {
			dev_err(&client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto error_reg_opt_i2c;
		}

		rc = regulator_enable(pdata->vcc_i2c);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	}

	msleep(10);

	return 0;

error_reg_en_vcc_i2c:
	if (pdata->i2c_pull_up)
		reg_set_optimum_mode_check(pdata->vcc_i2c, 0);
error_reg_opt_i2c:
	if (pdata->digital_pwr_regulator)
		regulator_disable(pdata->vcc_dig);
error_reg_en_vcc_dig:
	if (pdata->digital_pwr_regulator)
		reg_set_optimum_mode_check(pdata->vcc_dig, 0);
error_reg_opt_vcc_dig:
	regulator_disable(pdata->vcc_ana);
error_reg_en_vcc_ana:
	reg_set_optimum_mode_check(pdata->vcc_ana, 0);
	return rc;

power_off:
	reg_set_optimum_mode_check(pdata->vcc_ana, 0);
	regulator_disable(pdata->vcc_ana);
	if (pdata->digital_pwr_regulator) {
		reg_set_optimum_mode_check(pdata->vcc_dig, 0);
		regulator_disable(pdata->vcc_dig);
	}
	if (pdata->i2c_pull_up) {
		reg_set_optimum_mode_check(pdata->vcc_i2c, 0);
		regulator_disable(pdata->vcc_i2c);
	}
	msleep(50);
	return 0;
}

static int sensor_platform_hw_power_on(struct i2c_client *client, bool on)
{
	sensor_regulator_power_on(i2c_get_clientdata(client), on);
	return 0;
}

static int sensor_platform_hw_init(struct i2c_client *client)
{
	struct pas230_data *data = i2c_get_clientdata(client);
	int error;

	error = sensor_regulator_configure(data, true);

	if (gpio_is_valid(data->pdata->irq_gpio)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(data->pdata->irq_gpio, "pas230_irq_gpio");
		if (error) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
				data->pdata->irq_gpio);
		}
		error = gpio_direction_input(data->pdata->irq_gpio);
		if (error) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				data->pdata->irq_gpio);
		}
		data->irq = client->irq = gpio_to_irq(data->pdata->irq_gpio);
	} else {
		dev_err(&client->dev, "irq gpio not provided");
	}
	return 0;
}

static void sensor_platform_hw_exit(struct i2c_client *client)
{
	struct pas230_data *data = i2c_get_clientdata(client);;

	sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

}

static int sensor_parse_dt(struct device *dev,
			   struct pas230_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int ret, err = 0;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
		{"Partron,i2c-pull-up",		&pdata->i2c_pull_up,		DT_REQUIRED,	DT_BOOL,	0},
		//{"Partron,dig-reg-support",	&pdata->digital_pwr_regulator,	DT_REQUIRED,	DT_BOOL,	0},
		{"Partron,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
		{"Partron,vdd_ana_supply_min",	&pdata->vdd_ana_supply_min,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,vdd_ana_supply_max",	&pdata->vdd_ana_supply_max,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,vdd_ana_load_ua",	&pdata->vdd_ana_load_ua,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,vddio_dig_supply_min",	&pdata->vddio_dig_supply_min,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,vddio_dig_supply_max",	&pdata->vddio_dig_supply_max,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,vddio_dig_load_ua",	&pdata->vddio_dig_load_ua,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,vddio_i2c_supply_min",	&pdata->vddio_i2c_supply_min,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,vddio_i2c_supply_max",	&pdata->vddio_i2c_supply_max,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,vddio_i2c_load_ua",	&pdata->vddio_i2c_load_ua,	DT_SUGGESTED,	DT_U32,		0},
		{"Partron,near_offset",		&pdata->near_offset,		DT_SUGGESTED,	DT_U32,		0},
		{"Partron,far_offset",		&pdata->far_offset,		DT_SUGGESTED,	DT_U32,		0},
		{"Partron,crosstalk_max",		&pdata->crosstalk_max,		DT_SUGGESTED,	DT_U32,		0},
		{NULL,				NULL,				0,		0,		0},
	};

	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
		case DT_GPIO:
			ret = of_get_named_gpio(np, itr->dt_name, 0);
			if (ret >= 0) {
				*((int *) itr->ptr_data) = ret;
				ret = 0;
			}
			break;
		case DT_U32:
			ret = of_property_read_u32(np, itr->dt_name, (u32 *) itr->ptr_data);
			break;
		case DT_BOOL:
			*((bool *) itr->ptr_data) = of_property_read_bool(np, itr->dt_name);
			ret = 0;
			break;
		default:
			pr_info("[pas230]_%s: %d is an unknown DT entry type\n", __func__, itr->type);
			ret = -EBADE;
		}

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				pr_info("[pas230]_%s: Missing '%s' DT entry\n", __func__, itr->dt_name);
				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->proximity_power = sensor_platform_hw_power_on;

	reg_defaults[PS_THRES_UP] = (u8)pdata->near_offset;
	reg_defaults[PS_THRES_UP+1] = (u8)((pdata->near_offset>>8) & 0x07);
	reg_defaults[PS_THRES_LOW] = (u8)pdata->far_offset;
	reg_defaults[PS_THRES_LOW+1] = (u8)((pdata->far_offset>>8) & 0x07);

	return err;

}
#endif

static int pas230_i2c_read(struct pas230_data *pas230, u8 cmd, u8 *val)
{
	int err = 0;
	int retry = 3;
	struct i2c_client *client = pas230->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	while (retry--) {
		err = i2c_smbus_read_i2c_block_data(client, cmd, 1, val);
		if (err >= 0)
			return err;
	}

	return err;
}

static int pas230_i2c_readn(struct pas230_data *pas230, u8 cmd, u8 *val, u8 cnt)
{
	int err = 0;
	int retry = 3;
	struct i2c_client *client = pas230->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	while (retry--) {
		err = i2c_smbus_read_i2c_block_data(client, cmd, cnt, val);
		if (err >= 0)
			return err;
	}

	return err;
}

static int pas230_i2c_write(struct pas230_data *pas230, u8 cmd, u8 val)
{
	u8 data[2]={0, };
	int err = 0;
	int retry = 3;
	struct i2c_msg msg[1];
	struct i2c_client *client = pas230->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	data[0]=cmd;
	data[1]=val;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	while (retry--) {
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0)
			return err;
	}

	return err;
}


static int pas230_write_crosstalk_data_to_fs(unsigned int crosstalk)
{
	int fd;
	int ret = 0;
	char crosstalk_buf[50];
	mm_segment_t old_fs = get_fs();

	memset(crosstalk_buf, 0, sizeof(crosstalk_buf));
	sprintf(crosstalk_buf, "%d", crosstalk);

	set_fs(KERNEL_DS);
	fd = sys_open(PROX_CAL_DATA_PATH, O_WRONLY | O_CREAT, 0664);

	if (fd >= 0) {
		sys_write(fd, crosstalk_buf, sizeof(crosstalk_buf));
		sys_fsync(fd); /*ensure calibration data write to file system*/
		sys_close(fd);
		sys_chmod(PROX_CAL_DATA_PATH, 0664);
		set_fs(old_fs);
	} else {
		ret++;
		sys_close(fd);
		set_fs(old_fs);
		return ret	;
	}

	return ret;
}

static int pas230_read_crosstalk_data_from_fs(struct pas230_data *pas230)
{
	int fd;
	int ret = 0;
	int len = 0;
	unsigned int crosstalk = 0;
	char read_buf[50];
	mm_segment_t old_fs = get_fs();
	memset(read_buf, 0, sizeof(read_buf));
	set_fs(KERNEL_DS);

	fd = sys_open(PROX_CAL_DATA_PATH, O_RDONLY, 0);
	if (fd >= 0) {
		pr_info("[pas230]_%s: Success read Prox Cross-talk from FS\n", __func__);
		len = sys_read(fd, read_buf, sizeof(read_buf));
		if (len <= 0) {
			ret = -1;
			sys_close(fd);
			set_fs(old_fs);
			return ret;
		}
		sys_close(fd);
		set_fs(old_fs);
	} else {
		pr_info("[pas230]_%s: Fail read Prox Cross-talk FS (err:%d)\n", __func__, fd);
		ret = -1;
		sys_close(fd);
		set_fs(old_fs);
		return ret;
	}

	// save crosstalk value to default register setting (PS_CAN)
	crosstalk = (simple_strtol(read_buf, NULL, 10));
	//reg_defaults[PS_CAN] = (u8)crosstalk;
	//reg_defaults[PS_CAN+1] = (u8)((crosstalk>>8) & 0x07);
	pas230->near_threshold = pas230->pdata->near_offset + crosstalk;
	pas230->far_threshold = pas230->near_threshold - pas230->pdata->far_offset;

	reg_defaults[PS_THRES_UP] = (u8)(pas230->near_threshold);
	reg_defaults[PS_THRES_UP+1] = (u8)((pas230->near_threshold>>8) & 0x07);
	reg_defaults[PS_THRES_LOW] = (u8)pas230->far_threshold;
	reg_defaults[PS_THRES_LOW+1] = (u8)((pas230->far_threshold>>8) & 0x07);

	return crosstalk;
}

void pas230_swap(int *x, int *y)
{
	int temp = *x;
	*x = *y;
	*y = temp;
}

static int pas230_run_calibration(struct pas230_data *pas230)
{
	u8 ps_value[2] = {0, };
	unsigned int sum_of_pdata, temp_pdata[20];
	unsigned int ret = 0, i = 0, j = 0, ArySize = 20, cal_check_flag = 0;
	unsigned int old_enable = 0;

RE_CALIBRATION:
	sum_of_pdata = 0;
	old_enable = (pas230->power_state & PROXIMITY_ENABLED)? 1:0;
	
	pas230->pdata->proximity_power(pas230->i2c_client, 1);
	pas230->power_state |= PROXIMITY_ENABLED;


	for (i = 0; i < 20; i++)	{
		msleep(101);
		pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
		temp_pdata[i] = ((ps_value[1]&0x07)<<8) | ps_value[0];
	}

	for (i = 0; i < ArySize - 1; i++)
		for (j = i + 1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				pas230_swap(temp_pdata + i, temp_pdata + j);

	for (i = 5; i < 15; i++)
		sum_of_pdata = sum_of_pdata + temp_pdata[i];

	pas230->crosstalk = sum_of_pdata / 10;
	if (pas230->crosstalk > pas230->pdata->crosstalk_max) {
		if (cal_check_flag == 0) {
			cal_check_flag = 1;
			goto RE_CALIBRATION;
		} else {
			pas230->pdata->proximity_power(pas230->i2c_client, old_enable);
			if(old_enable == 1)
				pas230->power_state |= PROXIMITY_ENABLED;
			else
				pas230->power_state &= ~PROXIMITY_ENABLED;

			return -1;
		}
	}

	ret = pas230_write_crosstalk_data_to_fs(pas230->crosstalk);
	pas230->pdata->proximity_power(pas230->i2c_client, old_enable);
	if(old_enable == 1)
		pas230->power_state |= PROXIMITY_ENABLED;
	else
		pas230->power_state &= ~PROXIMITY_ENABLED;

	return pas230->crosstalk;
}

static void pas230_light_enable(struct pas230_data *pas230)
{
	u8 tmp;
	int64_t temp_time = 0;
	
	temp_time = ktime_to_ns(pas230->light_poll_delay) + 100000000;
	pas230_i2c_read(pas230, PART_ID, &tmp);
//	pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
	pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
	pas230_i2c_write(pas230, MAIN_CTRL, tmp | ALS_CS_ON);

#ifdef DEBUG
	pr_err("[pas230]_%s:  temp_time=%lld\n", __func__, temp_time);
#endif

	hrtimer_start(&pas230->light_timer, ns_to_ktime(temp_time),
						HRTIMER_MODE_REL);
}

static void pas230_light_disable(struct pas230_data *pas230)
{
	u8 tmp;
	
	pas230_i2c_read(pas230, MAIN_CTRL, &tmp);	
	pas230_i2c_write(pas230, MAIN_CTRL, tmp & ALS_CS_OFF);

#ifdef DEBUG
	pr_err("[pas230]_%s: MAIN_CTRL=0x%02x\n", __func__, tmp & ALS_CS_OFF);
#endif

	hrtimer_cancel(&pas230->light_timer);
	cancel_work_sync(&pas230->work_light);
}

static int lightsensor_get_alsvalue(struct pas230_data *pas230)
{
	int value = 0;
	u8 als_value[3] = {0, };

	/* get ALS */
	pas230_i2c_readn(pas230, GREEN_DATA, als_value, 3);
	value = ((als_value[2]<<16) | (als_value[1]<<8) | als_value[0]);

	/* get ALS sensitivity(18bit, Gain 3) = 2.4/(2^(18-16)*3) = 0.2 */
	/* value /= 5; */
	/* glass conpensation = x5 */
	/* value *= 5; */

	pr_info("[pas230]_%s: als value=%d\n", __func__, value);

	return value;
}

static void proxsensor_get_avgvalue(struct pas230_data *pas230)
{
	int min = 0, max = 0, avg = 0;
	int i;
	int value = 0;
	u8 ps_value[2] = {0, };

	for (i = 0; i < PROX_READ_NUM; i++) {
		msleep(101);
		pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
		value = ((ps_value[1]&0x07)<<8) | ps_value[0];
		avg += value;

		if (!i)
			min = value;
		else if (value < min)
			min = value;

		if (value > max)
			max = value;
	}
	avg /= PROX_READ_NUM;

	pas230->avg[0] = min;
	pas230->avg[1] = avg;
	pas230->avg[2] = max;

#ifdef DEBUG
	pr_err("[pas230]_%s:  min(%3d),avg(%3d),max(%3d)\n", __func__, min, avg, max);
#endif
}

static ssize_t pas230_proximity_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp;
	int value = 0;
	u8 ps_value[2] = {0, };
	
	if (!(pas230->power_state & PROXIMITY_ENABLED)) {
		mutex_lock(&pas230->power_lock);
		pas230->pdata->proximity_power(pas230->i2c_client, 1);
//		pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp | PS_ON);
		mutex_unlock(&pas230->power_lock);
	}

	msleep(101);
	pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
	value = ((ps_value[1]&0x07)<<8) | ps_value[0];

	if (!(pas230->power_state & PROXIMITY_ENABLED)) {
		mutex_lock(&pas230->power_lock);
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp | PS_OFF);
		pas230->pdata->proximity_power(pas230->i2c_client, 0);
		mutex_unlock(&pas230->power_lock);
	}
	
#ifdef DEBUG
	pr_err("[pas230]_%s:  value=%d\n", __func__, value);
#endif

	return sprintf(buf, "%d", value);
}

static ssize_t pas230_alsdata_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int als = 0;

	if (!(pas230->power_state & LIGHT_ENABLED))
		pas230_light_enable(pas230);

	als = lightsensor_get_alsvalue(pas230);
	
#ifdef DEBUG
	pr_err("[pas230]_%s:  als=%d\n", __func__, als);
#endif

	if (!(pas230->power_state & LIGHT_ENABLED))
		pas230_light_disable(pas230);

	return sprintf(buf, "%d\n", als);
}

static ssize_t poll_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);

#ifdef DEBUG
	pr_err("[pas230]_%s:  poll_delay=%lld\n", __func__, ktime_to_ns(pas230->light_poll_delay));
#endif

	return sprintf(buf, "%lld\n", ktime_to_ns(pas230->light_poll_delay));
}

static ssize_t poll_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

#ifdef DEBUG
	pr_err("[pas230]_%s:  new_delay=%lld\n", __func__, new_delay);
#endif

	mutex_lock(&pas230->power_lock);
	if (new_delay != ktime_to_ns(pas230->light_poll_delay)) {
		pas230->light_poll_delay = ns_to_ktime(new_delay);
		if (pas230->power_state & LIGHT_ENABLED) {
			pas230_light_disable(pas230);
			pas230_light_enable(pas230);
		}
	}
	mutex_unlock(&pas230->power_lock);

	return size;
}

static ssize_t pas230_light_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",
		       (pas230->power_state & LIGHT_ENABLED) ? 1 : 0);
}

static ssize_t pas230_proximity_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",
		       (pas230->power_state & PROXIMITY_ENABLED) ? 1 : 0);
}

static ssize_t pas230_light_enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("[pas230]_%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

#ifdef DEBUG
	pr_err("[pas230]_%s:  new_value=%d\n", __func__, new_value);
#endif

	mutex_lock(&pas230->power_lock);
	if (new_value && !(pas230->power_state & LIGHT_ENABLED)) {
		pas230->power_state |= LIGHT_ENABLED;
		pas230_light_enable(pas230);
	} else if (!new_value && (pas230->power_state & LIGHT_ENABLED)) {
		pas230_light_disable(pas230);
		pas230->power_state &= ~LIGHT_ENABLED;
	}
	mutex_unlock(&pas230->power_lock);
	return size;
}

static ssize_t pas230_proximity_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	bool new_value;
	u8 tmp;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("[pas230]_%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

#ifdef DEBUG
	pr_err("[pas230]_%s:  new_value=%d\n", __func__, new_value);
#endif

	mutex_lock(&pas230->power_lock);
	if (new_value && !(pas230->power_state & PROXIMITY_ENABLED)) {
		/* read crosstalk value from fs */
		pas230->crosstalk = pas230_read_crosstalk_data_from_fs(pas230);
	
		pas230->pdata->proximity_power(pas230->i2c_client, 1);
		pas230->power_state |= PROXIMITY_ENABLED;
		pas230_i2c_read(pas230, PART_ID, &tmp);
//		pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
		pas230_i2c_write(pas230, PS_THRES_UP, reg_defaults[PS_THRES_UP]);
		pas230_i2c_write(pas230, PS_THRES_UP+1, reg_defaults[PS_THRES_UP+1]);
		pas230_i2c_write(pas230, PS_THRES_LOW, reg_defaults[PS_THRES_LOW]);
		pas230_i2c_write(pas230, PS_THRES_LOW+1, reg_defaults[PS_THRES_LOW+1]);
		pas230_i2c_write(pas230, PS_CAN, reg_defaults[PS_CAN]);
		pas230_i2c_write(pas230, PS_CAN+1, reg_defaults[PS_CAN+1]);
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp | PS_ON);
		enable_irq(pas230->irq);
		enable_irq_wake(pas230->irq);
	} else if (!new_value && (pas230->power_state & PROXIMITY_ENABLED)) {
		pas230->power_state &= ~PROXIMITY_ENABLED;
		disable_irq_wake(pas230->irq);
		disable_irq(pas230->irq);
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp & PS_OFF);
		pas230->pdata->proximity_power(pas230->i2c_client, 0);
	}
	mutex_unlock(&pas230->power_lock);
	return size;
}

static ssize_t proximity_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	proxsensor_get_avgvalue(pas230);
	return sprintf(buf, "%d\n", pas230->avg[1]);
}

/*
static ssize_t pas230_proximity_avg_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	bool new_value;
	u8 tmp;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	
#ifdef DEBUG
	pr_err("[pas230]_%s:  new_value=%d\n", __func__, new_value);
#endif

	mutex_lock(&pas230->power_lock);
	if (new_value) {
		if (!(pas230->power_state & PROXIMITY_ENABLED)) {
			pas230->pdata->proximity_power(pas230->i2c_client, 1);
//			pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
			pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
			pas230_i2c_write(pas230, MAIN_CTRL, tmp | PS_ON);
		}
		hrtimer_start(&pas230->prox_timer, pas230->prox_poll_delay,
							HRTIMER_MODE_REL);
	} else if (!new_value) {
		hrtimer_cancel(&pas230->prox_timer);
		cancel_work_sync(&pas230->work_prox);
		if (!(pas230->power_state & PROXIMITY_ENABLED)) {
			pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
			pas230_i2c_write(pas230, MAIN_CTRL, tmp & PS_OFF);
			pas230->pdata->proximity_power(pas230->i2c_client, 0);
		}
	}
	mutex_unlock(&pas230->power_lock);

	return size;
}
*/
#ifdef DEBUG
static ssize_t proximity_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 dat[40]={0, };

	pas230_i2c_readn(pas230, MAIN_CTRL, dat, 40);

	return sprintf(buf, "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n"
						"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n"
						"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n"
						"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
						dat[0],dat[1],dat[2],dat[3],dat[4],dat[5],dat[6],dat[7],dat[8],dat[9],
						dat[10],dat[11],dat[12],dat[13],dat[14],dat[15],dat[16],dat[17],dat[18],dat[19],
						dat[20],dat[21],dat[22],dat[23],dat[24],dat[25],dat[26],dat[27],dat[28],dat[29],
						dat[30],dat[31],dat[32],dat[33],dat[34],dat[35],dat[36],dat[37],dat[38],dat[39]);
}

static ssize_t pas230_proximity_reg_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned int reg, val;

	sscanf(buf,"%x%x\n", &reg, &val);

	reg_defaults[(u8)reg] = (u8)val;

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, reg, reg_defaults[reg]);
	mutex_unlock(&pas230->power_lock);

	pr_err("[pas230]_%s:  reg=0x%x, write val=0x%x\n", __func__, reg, val);

	return size;
}

/*
static ssize_t test_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
//	struct pas230_data *pas230 = dev_get_drvdata(dev);

	return sprintf(buf, "%d,%d,%d,%d,%d,%d\n",
		0, 1,  2, 3, 4, 5);
}

static ssize_t pas230_test_reg_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
//	struct pas230_data *pas230 = dev_get_drvdata(dev);
//	unsigned int reg, val;

//	sscanf(buf,"%d%d\n", &reg, &val);

//	als_comp[(u8)reg] = val;

//	pr_err("[pas230]_%s:  als_comp[%d] = %d\n", __func__, reg, val);

	return size;
}
*/
#endif

/* sysfs for vendor & name */
/*
static ssize_t pas230_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}

static ssize_t pas230_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}
*/

// sysfs for far_threshold
static ssize_t pas230_ps_thres_low_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);	
	int err=0, value=0;
	u8 ps_thres_low[2] = {0, };

	err = pas230_i2c_readn(pas230, PS_THRES_LOW, ps_thres_low, 2);
	value = ((ps_thres_low[1]&0x07)<<8) | ps_thres_low[0];

	if (err < 0) {
		pr_err("[pas230]_%s: read ps_thres_low failed\n", __func__);
		err = -EIO;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t pas230_ps_thres_low_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 ps_thres_low[2] = {0, };
	unsigned int val;
	sscanf(buf,"%d\n", &val);
	ps_thres_low[0] = (u8)val;
	ps_thres_low[1] = (u8)((val>>8) & 0x07);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, PS_THRES_LOW, ps_thres_low[0]);
	pas230_i2c_write(pas230, PS_THRES_LOW+1, ps_thres_low[1]);
	mutex_unlock(&pas230->power_lock);

	pas230->near_threshold = val;
	return size;
}

// sysfs for near_threshold
static ssize_t pas230_ps_thres_up_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);	
	int err=0, value=0;
	u8 ps_thres_up[2] = {0, };

	err = pas230_i2c_readn(pas230, PS_THRES_UP, ps_thres_up, 2);
	value = ((ps_thres_up[1]&0x07)<<8) | ps_thres_up[0];

	if (err < 0) {
		pr_err("[pas230]_%s: read ps_thres_high failed\n", __func__);
		err = -EIO;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t pas230_ps_thres_up_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 ps_thres_up[2] = {0, };
	unsigned int val;
	sscanf(buf,"%d\n", &val);
	ps_thres_up[0] = (u8)val;
	ps_thres_up[1] = (u8)((val>>8) & 0x07);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, PS_THRES_UP, ps_thres_up[0]);
	pas230_i2c_write(pas230, PS_THRES_UP+1, ps_thres_up[1]);
	mutex_unlock(&pas230->power_lock);

	pas230->far_threshold = val;
	
	return size;
}

// sysfs for crosstalk_max 
static ssize_t pas230_crosstalk_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", pas230->pdata->crosstalk_max);
}

static ssize_t pas230_crosstalk_max_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);	
	unsigned long val = simple_strtoul(buf, NULL, 10);
	pas230->pdata->crosstalk_max = val;
	return size;
}

// sysfs for prox_cal_data 
static ssize_t pas230_prox_cal_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);	
	int ret=0;
	if((ret = pas230_read_crosstalk_data_from_fs(pas230)) < 0) 
		return sprintf(buf, "fail to read crosstalk data\n");
	return sprintf(buf, "%d\n", ret);
}

static ssize_t pas230_prox_cal_data_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	int ret = 0;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if ((ret = pas230_write_crosstalk_data_to_fs(val)) != 0)
		return pr_info("[pas230]_%s: fail to write crosstalk data to fs (err:%d)\n", __func__, ret);

	pr_info("[pas230]_%s: crosstalk data %d is written on fs\n", __func__, (int)val);
	return size;
}

// sysfs for run_calibration 
static ssize_t pas230_run_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pas230->cal_result);
}

static ssize_t pas230_run_calibration_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int ret=0;
	pas230->cal_result=0;
	
	if((ret = pas230_run_calibration(pas230)) < 0){
		pr_info("[pas230]_%s: fail proximity calibration (err:%d)\n", __func__, ret);
	}
	else{
		pas230->cal_result=1;
		pr_info("[pas230]_%s: success proximity calibration. crosstalk=%d\n", __func__, ret);
	}
	return size;
}

/*
static DEVICE_ATTR(proximity_avg, 0644, proximity_avg_show, pas230_proximity_avg_store);
static DEVICE_ATTR(proximity_state, 0644, pas230_proximity_state_show, NULL);
*/
#ifdef DEBUG
static DEVICE_ATTR(proximity_reg, 0644, proximity_reg_show, pas230_proximity_reg_store);
//static DEVICE_ATTR(test_reg, 0644, test_reg_show, pas230_test_reg_store);
#endif
/*
static DEVICE_ATTR(vendor, 0644, pas230_vendor_show, NULL);
static DEVICE_ATTR(name, 0644, pas230_name_show, NULL);
*/
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP, poll_delay_show, poll_delay_store);

// ssoon.lee@lge.com
static DEVICE_ATTR(pdata, S_IRUGO | S_IWUSR | S_IWGRP, proximity_avg_show, NULL);
static DEVICE_ATTR(value, S_IRUGO | S_IWUSR | S_IWGRP, pas230_proximity_state_show, NULL);
static DEVICE_ATTR(alsdata, S_IRUGO | S_IWUSR | S_IWGRP, pas230_alsdata_show, NULL);
static DEVICE_ATTR(far_offset, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_thres_low_show, pas230_ps_thres_low_store);
static DEVICE_ATTR(near_offset, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_thres_up_show, pas230_ps_thres_up_store);
static DEVICE_ATTR(prox_cal_data, S_IRUGO | S_IWUSR | S_IWGRP, pas230_prox_cal_data_show, pas230_prox_cal_data_store);
static DEVICE_ATTR(crosstalk_max, S_IRUGO | S_IWUSR | S_IWGRP, pas230_crosstalk_max_show, pas230_crosstalk_max_store);
static DEVICE_ATTR(run_calibration, S_IRUGO | S_IWUSR | S_IWGRP, pas230_run_calibration_show, pas230_run_calibration_store);

static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       pas230_light_enable_show, pas230_light_enable_store);

static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       pas230_proximity_enable_show, pas230_proximity_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_alsdata.attr,
	&dev_attr_proximity_reg.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_reg.attr,
	&dev_attr_proximity_enable.attr,
	&dev_attr_pdata.attr,
	&dev_attr_value.attr,
	&dev_attr_near_offset.attr,
	&dev_attr_far_offset.attr,
	&dev_attr_prox_cal_data.attr,
	&dev_attr_crosstalk_max.attr,
	&dev_attr_run_calibration.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

static void pas230_work_func_light(struct work_struct *work)
{
	int als;
	struct pas230_data *pas230 = container_of(work, struct pas230_data,
					      work_light);

	als = lightsensor_get_alsvalue(pas230);

	input_report_abs(pas230->light_input_dev, ABS_MISC, als);
	input_sync(pas230->light_input_dev);
}

static void pas230_work_func_prox(struct work_struct *work)
{
	struct pas230_data *pas230 = container_of(work, struct pas230_data,
					      work_prox);
	
	proxsensor_get_avgvalue(pas230);
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart pas230_light_timer_func(struct hrtimer *timer)
{
	struct pas230_data *pas230
			= container_of(timer, struct pas230_data, light_timer);
	queue_work(pas230->light_wq, &pas230->work_light);
	hrtimer_forward_now(&pas230->light_timer, pas230->light_poll_delay);
	return HRTIMER_RESTART;
}

static enum hrtimer_restart pas230_prox_timer_func(struct hrtimer *timer)
{
	struct pas230_data *pas230
			= container_of(timer, struct pas230_data, prox_timer);
	queue_work(pas230->prox_wq, &pas230->work_prox);
	hrtimer_forward_now(&pas230->prox_timer, pas230->prox_poll_delay);
	return HRTIMER_RESTART;
}

/* interrupt happened due to transition/change of near/far proximity state */
irqreturn_t pas230_irq_thread_fn(int irq, void *data)
{
	struct pas230_data *pas230 = data;
	u8 val = 1;
	int value = 0;
	u8 ps_value[2] = {0, };
	val = gpio_get_value(pas230->pdata->irq_gpio);
	if (val < 0) {
		pr_err("[pas230]_%s: gpio_get_value error %d\n", __func__, val);
		return IRQ_HANDLED;
	}

	/* for debugging : going to be removed */
	pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
	value = ((ps_value[1]&0x07)<<8) | ps_value[0];
	
#ifdef DEBUG
	//pr_err("[pas230]_%s:  proximity value=%d, val=%d\n", __func__, value, val);
#endif

	if (val == 0){	// FAR -> NEAR
		pr_info("[pas230]_%s: proximity status is changed from FAR -> NEAR, ps_data=%d\n", __func__, value);
	}
	else if (val == 1) {	// NEAR -> FAR
		pr_info("[pas230]_%s: proximity status is changed from NEAR -> FAR, ps_data=%d\n", __func__, value);
	}

	/* 0 is close, 1 is far */
	input_report_abs(pas230->proximity_input_dev, ABS_DISTANCE, val);
	input_sync(pas230->proximity_input_dev);
	wake_lock_timeout(&pas230->prx_wake_lock, 3*HZ);

	return IRQ_HANDLED;
}

static int pas230_setup_irq(struct pas230_data *pas230)
{
	int rc = -EIO;
	int irq;
	
	if (gpio_is_valid(pas230->pdata->irq_gpio)) {
		irq = gpio_to_irq(pas230->pdata->irq_gpio);
		rc = request_threaded_irq(irq, NULL, pas230_irq_thread_fn,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				PAS230_DRV_NAME, (void*)pas230);
		if (rc < 0) {
			pr_err("[pas230]_%s: request_irq(%d) failed for gpio %d (%d)\n",
					__func__, irq, irq, rc);
			return rc;
		}

		/* start with interrupts disabled */
		disable_irq(irq);
		pas230->irq = irq;
	}
	else {
		pr_err("[pas230]_%s: irq gpio not provided",__func__);
	}
	return rc;
}

static int pas230_setup_reg(struct pas230_data *pas230)
{
	int err = 0;
	u8 tmp;
	int value = 0;
	u8 ps_value[2] = {0, };

	/* initializing the proximity and light sensor registers */
	mutex_lock(&pas230->power_lock);
	pas230->pdata->proximity_power(pas230->i2c_client, 1);
	pas230_i2c_read(pas230, PART_ID, &tmp);
	pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
	pas230_i2c_write(pas230, PS_LED, reg_defaults[PS_LED]);
	pas230_i2c_write(pas230, PS_PULSES, reg_defaults[PS_PULSES]);
	pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);
	pas230_i2c_write(pas230, ALS_CS_MEAS_RATE, reg_defaults[ALS_CS_MEAS_RATE]);
	pas230_i2c_write(pas230, ALS_CS_GAIN, reg_defaults[ALS_CS_GAIN]);
	pas230_i2c_write(pas230, INT_CFG, reg_defaults[INT_CFG]);
	pas230_i2c_write(pas230, INT_PST, reg_defaults[INT_PST]);
	pas230_i2c_write(pas230, PS_THRES_UP, reg_defaults[PS_THRES_UP]);
	pas230_i2c_write(pas230, PS_THRES_UP+1, reg_defaults[PS_THRES_UP+1]);
	pas230_i2c_write(pas230, PS_THRES_LOW, reg_defaults[PS_THRES_LOW]);
	pas230_i2c_write(pas230, PS_THRES_LOW+1, reg_defaults[PS_THRES_LOW+1]);
	pas230_i2c_write(pas230, PS_CAN, reg_defaults[PS_CAN]);
	pas230_i2c_write(pas230, PS_CAN+1, reg_defaults[PS_CAN+1]);
	pas230_i2c_write(pas230, ALS_THRES_UP, reg_defaults[ALS_THRES_UP]);
	pas230_i2c_write(pas230, ALS_THRES_UP+1, reg_defaults[ALS_THRES_UP+1]);
	pas230_i2c_write(pas230, ALS_THRES_UP+2, reg_defaults[ALS_THRES_UP+2]);
	pas230_i2c_write(pas230, ALS_THRES_LOW, reg_defaults[ALS_THRES_LOW]);
	pas230_i2c_write(pas230, ALS_THRES_LOW+1, reg_defaults[ALS_THRES_LOW+1]);
	pas230_i2c_write(pas230, ALS_THRES_LOW+2, reg_defaults[ALS_THRES_LOW+2]);
	pas230_i2c_write(pas230, ALS_THRES_VAR, reg_defaults[ALS_THRES_VAR]);
	pas230_i2c_write(pas230, MAIN_CTRL, ALL_ON);
	mutex_unlock(&pas230->power_lock);

	/* printing the inital proximity value with no contact */
	msleep(101);
	err = pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
	value = ((ps_value[1]&0x07)<<8) | ps_value[0];
	if (err < 0) {
		pr_err("[pas230]_%s: read ps_data failed\n", __func__);
		err = -EIO;
	}
	
#ifdef DEBUG
	pr_err("[pas230]_%s:  proximity value=%d\n", __func__, value);
#endif

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, MAIN_CTRL, ALL_OFF);
	pas230->pdata->proximity_power(pas230->i2c_client, 0);
	mutex_unlock(&pas230->power_lock);

	return err;
}

static int pas230_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct input_dev *input_dev;
	struct pas230_data *pas230;
#ifdef CONFIG_OF
	struct pas230_platform_data *platform_data;
#endif

	pr_info("[pas230]_%s: probe start\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[pas230]_%s: i2c functionality check failed!\n", __func__);
		return ret;
	}

	pas230 = kzalloc(sizeof(struct pas230_data), GFP_KERNEL);
	if (!pas230) {
		pr_err("[pas230]_%s: failed to alloc memory for module data\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		platform_data = kzalloc(sizeof(struct pas230_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		pas230->pdata = platform_data;
		client->dev.platform_data = platform_data;
	
		if ((ret = sensor_parse_dt(&client->dev, pas230->pdata))) {
			pr_err("[pas230]_%s: sensor_parse_dt error\n", __func__);
			return ret;
		}
	}
#else 
	pas230->pdata = client->dev.platform_data;
#endif	
	pas230->i2c_client = client;
	
	i2c_set_clientdata(client, pas230);

#ifdef CONFIG_OF
	/* h/w initialization */
	if (platform_data->init) {
		pr_info("[pas230]_%s: platform_data->init called\n", __func__);
		ret = platform_data->init(client);
	}

	if (platform_data->proximity_power) {
		pr_info("[pas230]_%s: platform_data->proximity_power called\n", __func__);
		ret = platform_data->proximity_power(client, true);
	}
#endif

	/* wake lock init */
	wake_lock_init(&pas230->prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");
	mutex_init(&pas230->power_lock);

	/* setup initial registers */
	ret = pas230_setup_reg(pas230);
	if (ret < 0) {
		pr_err("[pas230]_%s: could not setup regs\n", __func__);
		goto err_setup_reg;
	}

	ret = pas230_setup_irq(pas230);
	if (ret) {
		pr_err("[pas230]_%s: could not setup irq\n", __func__);
		goto err_setup_irq;
	}

	/* allocate proximity input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("[pas230]_%s: could not allocate input device\n", __func__);
		goto err_input_allocate_device_proximity;
	}
	pas230->proximity_input_dev = input_dev;
	input_set_drvdata(input_dev, pas230);
	input_dev->name = "proximity";
	input_dev->uniq = PAS230_DRV_NAME;
	input_dev->dev.init_name = LGE_PROXIMITY_NAME;
	
	input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("[pas230]_%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_proximity;
	}
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &proximity_attribute_group);
	if (ret) {
		pr_err("[pas230]_%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_proximity;
	}

	/* light_timer settings. we poll for light values using a timer. */
	hrtimer_init(&pas230->light_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pas230->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	pas230->light_timer.function = pas230_light_timer_func;

	/* prox_timer settings. we poll for light values using a timer. */
	hrtimer_init(&pas230->prox_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pas230->prox_poll_delay = ns_to_ktime(2000 * NSEC_PER_MSEC);
	pas230->prox_timer.function = pas230_prox_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	pas230->light_wq = create_singlethread_workqueue("pas230_light_wq");
	if (!pas230->light_wq) {
		ret = -ENOMEM;
		pr_err("[pas230]_%s: could not create light workqueue\n", __func__);
		goto err_create_light_workqueue;
	}
	pas230->prox_wq = create_singlethread_workqueue("pas230_prox_wq");
	if (!pas230->prox_wq) {
		ret = -ENOMEM;
		pr_err("[pas230]_%s: could not create prox workqueue\n", __func__);
		goto err_create_prox_workqueue;
	}

	/* this is the thread function we run on the work queue */
	INIT_WORK(&pas230->work_light, pas230_work_func_light);
	INIT_WORK(&pas230->work_prox, pas230_work_func_prox);

	/* allocate lightsensor-level input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("[pas230]_%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(input_dev, pas230);
	input_dev->name = "light";
	input_dev->uniq = PAS230_DRV_NAME;
	input_dev->dev.init_name = LGE_LIGHT_NAME;

	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);

	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("[pas230]_%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	pas230->light_input_dev = input_dev;
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &light_attribute_group);
	if (ret) {
		pr_err("[pas230]_%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

#if 0
	/* set sysfs for proximity sensor */
	pas230->proximity_class = class_create(THIS_MODULE, "proximity");
	if (IS_ERR(pas230->proximity_class)) {
		pr_err("%s: could not create proximity_class\n", __func__);
		goto err_proximity_class_create;
	}

	pas230->proximity_dev = device_create(pas230->proximity_class,
						NULL, 0, NULL, "proximity");
	if (IS_ERR(pas230->proximity_dev)) {
		pr_err("%s: could not create proximity_dev\n", __func__);
		goto err_proximity_device_create;
	}

	if (device_create_file(pas230->proximity_dev,
		&dev_attr_proximity_state) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			dev_attr_proximity_state.attr.name);
		goto err_proximity_device_create_file1;
	}

	if (device_create_file(pas230->proximity_dev,
		&dev_attr_proximity_avg) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			dev_attr_proximity_avg.attr.name);
		goto err_proximity_device_create_file2;
	}
#ifdef DEBUG
	if (device_create_file(pas230->proximity_dev,
		&dev_attr_proximity_reg) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			dev_attr_proximity_reg.attr.name);
		goto err_proximity_device_create_file3;
	}

	if (device_create_file(pas230->proximity_dev,
		&dev_attr_test_reg) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			dev_attr_test_reg.attr.name);
		goto err_test_device_create_file4;
	}
#endif
	if (device_create_file(pas230->proximity_dev,
		&dev_attr_vendor) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			   dev_attr_vendor.attr.name);
		goto err_proximity_device_create_file5;
	}

	if (device_create_file(pas230->proximity_dev,
		&dev_attr_name) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			   dev_attr_name.attr.name);
		goto err_proximity_device_create_file6;
	}
	
	dev_set_drvdata(pas230->proximity_dev, pas230);

	/* set sysfs for light sensor */
	pas230->lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(pas230->lightsensor_class)) {
		pr_err("%s: could not create lightsensor_class\n", __func__);
		goto err_light_class_create;
	}

	pas230->lightsensor_dev = device_create(pas230->lightsensor_class,
						NULL, 0, NULL, "lightsensor");
	if (IS_ERR(pas230->lightsensor_dev)) {
		pr_err("%s: could not create lightsensor_dev\n", __func__);
		goto err_light_device_create;
	}

	if (device_create_file(pas230->lightsensor_dev,
		&dev_attr_lightsensor_file_state) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			dev_attr_lightsensor_file_state.attr.name);
		goto err_light_device_create_file1;
	}

	if (device_create_file(pas230->lightsensor_dev, &dev_attr_vendor) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
		       dev_attr_vendor.attr.name);
		goto err_light_device_create_file2;
	}

	if (device_create_file(pas230->lightsensor_dev, &dev_attr_name) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
		       dev_attr_name.attr.name);
		goto err_light_device_create_file3;
	}

	dev_set_drvdata(pas230->lightsensor_dev, pas230);
#endif

	/* set initial proximity value as 1. ahn_KT */
	input_report_abs(pas230->proximity_input_dev, ABS_DISTANCE, 1);
	input_sync(pas230->proximity_input_dev);

	pr_info("[pas230]_%s: proximity & light sensor probed successfully \n", __func__);
	
	goto done;

#if 0
/* error, unwind it all */
err_light_device_create_file3:
	device_remove_file(pas230->lightsensor_dev, &dev_attr_name);
err_light_device_create_file2:
	device_remove_file(pas230->lightsensor_dev, &dev_attr_vendor);
err_light_device_create_file1:
	device_remove_file(pas230->lightsensor_dev, &dev_attr_lightsensor_file_state);
err_light_device_create:
	device_destroy(pas230->lightsensor_class, 0);
err_light_class_create:
	class_destroy(pas230->lightsensor_class);
err_proximity_device_create_file6:
	device_remove_file(pas230->proximity_dev, &dev_attr_name);
err_proximity_device_create_file5:
	device_remove_file(pas230->proximity_dev, &dev_attr_vendor);
#ifdef DEBUG
err_test_device_create_file4:
	device_remove_file(pas230->proximity_dev, &dev_attr_test_reg);
err_proximity_device_create_file3:
	device_remove_file(pas230->proximity_dev, &dev_attr_proximity_reg);
#endif	
err_proximity_device_create_file2:
	device_remove_file(pas230->proximity_dev, &dev_attr_proximity_avg);
err_proximity_device_create_file1:
	device_remove_file(pas230->proximity_dev, &dev_attr_proximity_state);
err_proximity_device_create:
	device_destroy(pas230->proximity_class, 0);
err_proximity_class_create:
	class_destroy(pas230->proximity_class);
#endif
err_sysfs_create_group_light:
	sysfs_remove_group(&input_dev->dev.kobj, &light_attribute_group);
err_input_register_device_light:
	input_unregister_device(pas230->light_input_dev);	
err_input_allocate_device_light:

err_create_prox_workqueue:
	destroy_workqueue(pas230->prox_wq);
err_create_light_workqueue:
	destroy_workqueue(pas230->light_wq);
err_sysfs_create_group_proximity:
	sysfs_remove_group(&pas230->proximity_input_dev->dev.kobj,
			   &proximity_attribute_group);
err_input_register_device_proximity:
	input_unregister_device(pas230->proximity_input_dev);	
err_input_allocate_device_proximity:

err_setup_irq:
	free_irq(pas230->irq, NULL);
err_setup_reg:
	wake_lock_destroy(&pas230->prx_wake_lock);
	mutex_destroy(&pas230->power_lock);
	kfree(pas230);
done:
	return ret;
}

static int pas230_suspend(struct device *dev)
{
	/* We disable power only if proximity is disabled.  If proximity
	   is enabled, we leave power on because proximity is allowed
	   to wake up device.  We remove power without changing
	   pas230->power_state because we use that state in resume.
	*/
	struct i2c_client *client = to_i2c_client(dev);
	struct pas230_data *pas230 = i2c_get_clientdata(client);
	
	if (pas230->power_state & LIGHT_ENABLED)
		pas230_light_disable(pas230);

	return 0;
}

static int pas230_resume(struct device *dev)
{
	/* Turn power back on if we were before suspend. */
	struct i2c_client *client = to_i2c_client(dev);
	struct pas230_data *pas230 = i2c_get_clientdata(client);

	if (pas230->power_state & LIGHT_ENABLED)
		pas230_light_enable(pas230);
	
	return 0;
}

static int pas230_i2c_remove(struct i2c_client *client)
{
	struct pas230_data *pas230 = i2c_get_clientdata(client);

	/* free irq */
	free_irq(pas230->irq, NULL);

	/* device off */
	if (pas230->power_state) {
		if (pas230->power_state & LIGHT_ENABLED)
			pas230_light_disable(pas230);
		if (pas230->power_state & PROXIMITY_ENABLED) {
			pas230_i2c_write(pas230, MAIN_CTRL, PS_OFF);
			pas230->pdata->proximity_power(pas230->i2c_client, 0);
		}
	}

	/* destroy workqueue */
	destroy_workqueue(pas230->light_wq);
	destroy_workqueue(pas230->prox_wq);

#if 0
	/* sysfs destroy */
	device_remove_file(pas230->lightsensor_dev, &dev_attr_name);
	device_remove_file(pas230->lightsensor_dev, &dev_attr_vendor);
	device_remove_file(pas230->lightsensor_dev, &dev_attr_lightsensor_file_state);
	device_destroy(pas230->lightsensor_class, 0);
	class_destroy(pas230->lightsensor_class);
#ifdef DEBUG
	device_remove_file(pas230->proximity_dev, &dev_attr_proximity_reg);
	device_remove_file(pas230->proximity_dev, &dev_attr_test_reg);
#endif
	device_remove_file(pas230->proximity_dev, &dev_attr_name);
	device_remove_file(pas230->proximity_dev, &dev_attr_vendor);
	device_remove_file(pas230->proximity_dev, &dev_attr_proximity_avg);
	device_remove_file(pas230->proximity_dev, &dev_attr_proximity_state);
	device_destroy(pas230->proximity_class, 0);
	class_destroy(pas230->proximity_class);
#endif

	/* input device destroy */
	sysfs_remove_group(&pas230->light_input_dev->dev.kobj, &light_attribute_group);
	input_unregister_device(pas230->light_input_dev);
	sysfs_remove_group(&pas230->proximity_input_dev->dev.kobj, &proximity_attribute_group);
	input_unregister_device(pas230->proximity_input_dev);
	
	/* lock destroy */
	mutex_destroy(&pas230->power_lock);
	wake_lock_destroy(&pas230->prx_wake_lock);
	
	kfree(pas230);
	
	return 0;
}

static const struct i2c_device_id pas230_device_id[] = {
	{"pas230", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pas230_device_id);

static const struct dev_pm_ops pas230_pm_ops = {
	.suspend = pas230_suspend,
	.resume = pas230_resume
};

#ifdef CONFIG_OF
static struct of_device_id pas230_match_table[] = {
	{ .compatible = "partron,pas230",},
	{ },
};
#else
#define pas230_match_table NULL
#endif

static struct i2c_driver pas230_i2c_driver = {
	.driver = {
		.name = "pas230",
		.owner = THIS_MODULE,
		.of_match_table = pas230_match_table,
		.pm = &pas230_pm_ops
	},
	.probe		= pas230_i2c_probe,
	.remove		= pas230_i2c_remove,
	.id_table	= pas230_device_id,
};


static int __init pas230_init(void)
{
	pr_info("[pas230]_%s: PAS230 driver initialized\n", __func__);
	return i2c_add_driver(&pas230_i2c_driver);
}

static void __exit pas230_exit(void)
{
	i2c_del_driver(&pas230_i2c_driver);
}

module_init(pas230_init);
module_exit(pas230_exit);

MODULE_AUTHOR("partron@partron.co.kr");
MODULE_DESCRIPTION("Optical Sensor driver for pas230");
MODULE_LICENSE("GPL");
