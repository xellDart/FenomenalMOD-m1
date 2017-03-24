/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include <linux/mfd/rt4832.h>

#define FLASH_NAME "rt4832-flash"

#undef CDBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)

extern int32_t msm_led_torch_create_classdev(
				struct platform_device *pdev, void *data);

static enum flash_type flashtype;
static struct msm_led_flash_ctrl_t fctrl;

typedef struct{
	struct rt4832 *rt4832;
}rt4832_flash;

unsigned char flash_ctrl = 0;
unsigned char strobe_ctrl = 0;
unsigned char torch_toggle = 0;


static struct msm_camera_i2c_client rt4832_flash_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static int32_t rt4832_flash_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}
	*subdev_id = fctrl->pdev->id;
	CDBG("%s:%d subdev_id %d\n", __func__, __LINE__, *subdev_id);
	return 0;
}

int rt4832_flash_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	CDBG("%s:%d called\n", __func__, __LINE__);

	torch_toggle = 0;
	CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);

	return rc;
}


int rt4832_flash_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	u8 status=0;
	rt4832_flash *rt4832_fl = (rt4832_flash*)fctrl->data;
	CDBG("%s:%d called\n", __func__, __LINE__);

	status = 0;
	rc = rt4832_read_byte(rt4832_fl->rt4832, 0x0A, &status);
	pr_err("flash release read 0x0A status : %d \n", status);
	if (rc < 0) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
			}
			if(status && 0x04 == 0x04) {
				status &= 0x1D;// torch/flash = flash, flash enable = enable
			}
			else {
				status &= 0x19;// torch/flash = torch, flash enable = off
			}

	rc = rt4832_write_byte(rt4832_fl->rt4832, 0x0A, status);
	if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			}

	torch_toggle = 0;
	CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);

	return 0;
}


int rt4832_flash_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc		= 0;
	u8 status = 0;

	rt4832_flash *rt4832_fl = (rt4832_flash*)fctrl->data;
	//struct rt4832 *rt4832 = rt4832_fl->rt4832;

	CDBG("%s:%d called\n", __func__, __LINE__);

       if(rt4832_fl == NULL || rt4832_fl->rt4832 == NULL)
       {
       	pr_err(" %s : Null exception return \n",__func__);
       	return rc;
       }

	if (rt4832_fl->rt4832) {
		rc = rt4832_read_byte(rt4832_fl->rt4832, 0x07, &status);
		pr_err("flash read 0x07 status : %d \n", status);
		status |= 0x2F; // set strobe timeout
		pr_err("flash set to 0x07 status : %d \n", status);
		rc = rt4832_write_byte(rt4832_fl->rt4832, 0x07, status);
		if (rc < 0) {
				pr_err("%s:%d write to 0x07 failed\n", __func__, __LINE__);
				}


		rc = rt4832_read_byte(rt4832_fl->rt4832, 0x06, &status);
		pr_err("flash read 0x06 status : %d \n", status); // 3E
		status &= 0x0F; // clear 0x06[7:4] to 0
		status |= 0x30; // set torch current 100mA
		pr_err("flash set to 0x06 status : %d \n", status);
		rc = rt4832_write_byte(rt4832_fl->rt4832, 0x06, status);
		if (rc < 0) {
					pr_err("%s:%d write to 0x06 failed\n", __func__, __LINE__);
					}
	}

	strobe_ctrl = 0;
	rc = rt4832_read_byte(rt4832_fl->rt4832, 0x09, &strobe_ctrl); // 58
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	pr_err("flash low read 0x09 status : %d \n", strobe_ctrl);

	strobe_ctrl &= 0xEF;
	rc = rt4832_write_byte(rt4832_fl->rt4832, 0x09, strobe_ctrl);
	if (rc < 0) {
		pr_err("%s:%d write to 0x09 failed\n", __func__, __LINE__);
	}

	status = 0;
	rc = rt4832_read_byte(rt4832_fl->rt4832, 0x0A, &status);
	pr_err("flash read 0x0A status : %d \n", status);
	status &= 0xFB; // for torch mode
	status |= 0x02; // Flash Enable
	pr_err("flash set to 0x0A status : %d \n", status);
	rc = rt4832_write_byte(rt4832_fl->rt4832, 0x0A, status);
	if (rc < 0) {
		pr_err("%s:%d write to 0x0A failed\n", __func__, __LINE__);
	}

	return rc;
}


int rt4832_flash_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc		= 0;
	u8 status = 0;

	rt4832_flash *rt4832_fl = (rt4832_flash*)fctrl->data;
	//struct rt4832 *rt4832 = rt4832_fl->rt4832;

	CDBG("%s:%d called\n", __func__, __LINE__);

       if(rt4832_fl == NULL || rt4832_fl->rt4832 == NULL)
       {
       	pr_err(" %s : Null exception return \n",__func__);
       	return rc;
       }

	if (rt4832_fl->rt4832) {
		rc = rt4832_read_byte(rt4832_fl->rt4832, 0x07, &status);
		pr_err("flash read 0x07 status : %d \n", status);
		status |= 0x2F; // set strobe timeout
		pr_err("flash set to 0x07 status : %d \n", status);
		rc = rt4832_write_byte(rt4832_fl->rt4832, 0x07, status);
		if (rc < 0) {
				pr_err("%s:%d write to 0x07 failed\n", __func__, __LINE__);
				}


		rc = rt4832_read_byte(rt4832_fl->rt4832, 0x06, &status);
		pr_err("flash read 0x06 status : %d \n", status); // 3E
		status &= 0xF0; // clear 0x06[3:0] to 0
#if defined(CONFIG_MACH_MSM8909_M1_TMO_US) || defined(CONFIG_MACH_MSM8909_M1_MPCS_US)
		status |= 0x08; // set strobe current 900mA
#else
		status |= 0x07; // set strobe current 800mA
#endif
		pr_err("flash set to 0x06 status : %d \n", status);
		rc = rt4832_write_byte(rt4832_fl->rt4832, 0x06, status);
		if (rc < 0) {
					pr_err("%s:%d write to 0x06 failed\n", __func__, __LINE__);
					}
	}

	strobe_ctrl = 0;
	rc = rt4832_read_byte(rt4832_fl->rt4832, 0x09, &strobe_ctrl); // 58
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	pr_err("flash low read 0x09 status : %d \n", strobe_ctrl);

	strobe_ctrl &= 0xEF;
	rc = rt4832_write_byte(rt4832_fl->rt4832, 0x09, strobe_ctrl);
	if (rc < 0) {
		pr_err("%s:%d write to 0x09 failed\n", __func__, __LINE__);
	}

	status = 0;
	rc = rt4832_read_byte(rt4832_fl->rt4832, 0x0A, &status);
	pr_err("flash read 0x0A status : %d \n", status);
	status |= 0x06; // Flash Enable
	pr_err("flash set to 0x0A status : %d \n", status);
	rc = rt4832_write_byte(rt4832_fl->rt4832, 0x0A, status);
	if (rc < 0) {
		pr_err("%s:%d write to 0x0A failed\n", __func__, __LINE__);
	}

	return rc;
}


int rt4832_flash_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	u8 status=0;
	rt4832_flash *rt4832_fl = (rt4832_flash*)fctrl->data;
	CDBG("%s:%d called\n", __func__, __LINE__);

	status = 0;
	rc = rt4832_read_byte(rt4832_fl->rt4832, 0x0A, &status);
	pr_err("flash release read 0x0A status : %d \n", status);
	if (rc < 0) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
			}
			if(status && 0x04 == 0x04) {
				status &= 0x1D;// torch/flash = flash, flash enable = enable
			}
			else {
				status &= 0x19;// torch/flash = torch, flash enable = off
			}

	rc = rt4832_write_byte(rt4832_fl->rt4832, 0x0A, status);
	if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			}

	torch_toggle = 0;
	CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);

	return 0;
}

static int32_t rt4832_flash_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	uint32_t i;
	//uint32_t curr_l, max_curr_l;
	CDBG("called led_state %d\n", cfg->cfgtype);

	if (!fctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_OFF:
	if (fctrl->func_tbl->flash_led_off)
			rc = fctrl->func_tbl->flash_led_off(fctrl);
		break;

	case MSM_CAMERA_LED_LOW:
	for (i = 0; i < fctrl->torch_num_sources; i++) {
			if (fctrl->torch_max_current[i] > 0) {
				fctrl->torch_op_current[i] =
					(cfg->torch_current[i] < fctrl->torch_max_current[i]) ?
					cfg->torch_current[i] : fctrl->torch_max_current[i];
				CDBG("torch source%d: op_current %d max_current %d\n",
					i, fctrl->torch_op_current[i], fctrl->torch_max_current[i]);
			}
		}
		if (fctrl->func_tbl->flash_led_low)
			rc = fctrl->func_tbl->flash_led_low(fctrl);
		break;

	case MSM_CAMERA_LED_HIGH:
	if (fctrl->func_tbl->flash_led_high)
			rc = fctrl->func_tbl->flash_led_high(fctrl);
		break;

	case MSM_CAMERA_LED_INIT:
		if (fctrl->func_tbl->flash_led_init)
			rc = fctrl->func_tbl->flash_led_init(fctrl);
		break;

	case MSM_CAMERA_LED_RELEASE:
		if (fctrl->func_tbl->flash_led_release)
			rc = fctrl->func_tbl->
				flash_led_release(fctrl);
		break;

	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}

static const struct of_device_id rt4832_flash_dt_match[] = {
	{.compatible = "richtek,rt4832-flash"},
	{}
};

MODULE_DEVICE_TABLE(of, rt4832_flash_dt_match);

static struct platform_driver rt4832_flash_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rt4832_flash_dt_match,
	},
};

static int32_t rt4832_flash_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct device_node *of_node = pdev->dev.of_node;

	struct rt4832 *rt4832 = dev_get_drvdata(pdev->dev.parent);
	rt4832_flash *rt4832_fl;

	CDBG("called\n");

	rt4832_fl = devm_kzalloc(&pdev->dev, sizeof(rt4832_flash), GFP_KERNEL);
	if (!rt4832_fl)
		return -ENOMEM;

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl.pdev = pdev;
	fctrl.flash_num_sources = 0;
	fctrl.torch_num_sources = 0;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}
	CDBG("pdev id %d\n", pdev->id);

	rc = of_property_read_u32(of_node,
			"qcom,flash-type", &flashtype);
	if (rc < 0) {
		pr_err("flash-type: read failed\n");
		return -EINVAL;
	}
	pr_err("rt4832 probe...1\n");

	rt4832_fl->rt4832 = rt4832;
	platform_set_drvdata(pdev, rt4832_fl);

		pr_err("rt4832 probe...2\n");
	rc = msm_led_flash_create_v4lsubdev(pdev, &fctrl);
	if (!rc)
		msm_led_torch_create_classdev(pdev, &fctrl);
		pr_err("rt4832 probe...3\n");

		fctrl.data = (void*)rt4832_fl;
	return rc;
}

static int __init rt4832_flash_add_driver(void)
{
	CDBG("called\n");
	return platform_driver_probe(&rt4832_flash_driver,
		rt4832_flash_probe);
}

static struct msm_flash_fn_t rt4832_flash_func_tbl = {
	.flash_get_subdev_id = rt4832_flash_get_subdev_id,
	.flash_led_config = rt4832_flash_config,
	.flash_led_init = rt4832_flash_init,
	.flash_led_release = rt4832_flash_release,
	.flash_led_off = rt4832_flash_off,
	.flash_led_low = rt4832_flash_low,
	.flash_led_high = rt4832_flash_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &rt4832_flash_i2c_client,
	.func_tbl = &rt4832_flash_func_tbl,
};

module_init(rt4832_flash_add_driver);
MODULE_DESCRIPTION("LED TRIGGER FLASH");
MODULE_LICENSE("GPL v2");
