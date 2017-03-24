/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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
#include <linux/mfd/rt8542.h>
#include <linux/module.h>
#include "msm_led_flash.h"
#include "msm_camera_io_util.h"
#include <mach/gpiomux.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#define FLASH_NAME "rt8542_flash"

typedef struct {
	struct rt8542 *rt8542;
}rt8542_flash;

#define ENABLE_FL 0xFF
#define DISABLE_FL 0

#define FLASH_CURRENT_REG 0x06
#define FLASH_CTR1_REG 0x07
#define FLASH_CTR2_REG 0x09

#define FLED_STORBE_TIMEOUT 0x1F
#define CURRENT 0x18


unsigned char flash_ctrl;
unsigned char strobe_ctrl;
u8 torch_toggle = 0;

#undef CDBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)

extern int32_t msm_led_torch_create_classdev(
				struct platform_device *pdev, void *data);

static enum flash_type flashtype;
static struct msm_led_flash_ctrl_t fctrl;

static int32_t rt8542_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
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

int rt8542_release(struct rt8542 *rt8542){
	int rc =0;
	CDBG("%s:%d called\n", __func__, __LINE__);
	flash_ctrl = 0;
	rc = rt8542_read_byte(rt8542, RT8542_REG_LED_CTRL, &flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	flash_ctrl &= 0x1D;
	rc =rt8542_write_byte(rt8542, RT8542_REG_LED_CTRL, flash_ctrl); //clear bit1,5,6
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}


	return rc;
}

int rt8542_off(struct rt8542 *rt8542){
	int rc = 0;

	CDBG("%s:%d called\n", __func__, __LINE__);

	flash_ctrl = 0;
	rc = rt8542_read_byte(rt8542, RT8542_REG_LED_CTRL, &flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	flash_ctrl &= 0x1D;
	rc =rt8542_write_byte(rt8542, RT8542_REG_LED_CTRL, flash_ctrl); //clear bit1,5,6
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	return rc;
}

int rt8542_low(struct rt8542 *rt8542){
	int rc = 0;
	u8 status;
	CDBG("%s:%d called\n", __func__, __LINE__);

	//LGE_CHANGE_S, fix live-shot make flicker issue when video recording with torch on, jongkwon.chae@lge.com, 2014-07-18
    //Check if current status is strobe on -> then skip
    rc = rt8542_read_byte(rt8542, FLASH_CTR2_REG, &status);
    if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
    }
    //LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
    else if (torch_toggle == 0) {
        //torch field in Addr 0x0A has reset-value of 1, so we cannot check bit field if LCD backlight is not set yet.
        //this else-if part is defensive code for torch-request before lcd is on case.
        CDBG("[CHECK] Do not skip torch. (if torch_toggle is disabled.)");
    }
    else {
			status = 0;
			rc = rt8542_read_byte(rt8542, RT8542_REG_LED_CTRL, &status);
			if (rc < 0) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
			}
			if ((status & 0x42) == 0x42) {
				pr_err("[CHECK] already strobe-on! -> skip strobe-on request\n");
				return rc;
			}
    }
    //LGE_CHANGE_E, fix live-shot make flicker issue when video recording with torch on, jongkwon.chae@lge.com, 2014-07-18
	/* Configuration of frequency, current limit and timeout, default 2Mhz, 1.7A, 1024ms */
	rc =rt8542_write_byte(rt8542, FLASH_CTR1_REG, 0x0F);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	/* Configuration of current, torch : 84.375mAx2, strobe : 103.125mAx2 */
	rc = rt8542_write_byte(rt8542, 0x06, 0x21);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	flash_ctrl = 0;
	rc = rt8542_read_byte(rt8542, RT8542_REG_LED_CTRL, &flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	//LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
	if ( rt8542->pdata->power_state & BIT(0)) {
		flash_ctrl &= 0x1D;
	}
	else {
		CDBG("[CHECK] Currently, Backlight is Off (status: 0x%X)", rt8542->pdata->power_state);
		flash_ctrl &= 0x04;
	}

	rc =rt8542_write_byte(rt8542, RT8542_REG_LED_CTRL, flash_ctrl); //clear bit1,5,6
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	strobe_ctrl=0;
	rc = rt8542_read_byte(rt8542, FLASH_CTR2_REG, &strobe_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	strobe_ctrl &= 0xDF; /* 1101 1111 */
	strobe_ctrl |= 0x10; /* 0001 0000 */
	rc =rt8542_write_byte(rt8542,	FLASH_CTR2_REG, strobe_ctrl);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	/* Enable */
	flash_ctrl=0;
	rc = rt8542_read_byte(rt8542, RT8542_REG_LED_CTRL, &flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	flash_ctrl &= 0xFB; /* 1111 1011 */
	flash_ctrl |= 0x62; /* 0110 0010 */
	// we must set the third bit 0 to set TORCH mode.
	rc =rt8542_write_byte(rt8542, RT8542_REG_LED_CTRL, flash_ctrl);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	
	CDBG("[CHECK] [Addr: 0x0A] Write Data: 0x%X\n", flash_ctrl);

	//LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
	torch_toggle = 1;
	CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);

	return rc;
}

int rt8542_high(struct rt8542 *rt8542)
{
	int rc = 0;
	CDBG("%s:%d called\n", __func__, __LINE__);

	/* Configuration of frequency, current limit and timeout, default 2Mhz, 2.5A, 1024ms */
	//rc =flash_write_reg(fctrl->flash_i2c_client,	0x07, 0x1F);
	rc = rt8542_write_byte(rt8542,FLASH_CTR1_REG, FLED_STORBE_TIMEOUT);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	/* Configuration of current, torch : 50mA, strobe :900mA */
	//rc =flash_write_reg(fctrl->flash_i2c_client,	0x06, 0x18);
	rc = rt8542_write_byte(rt8542,FLASH_CURRENT_REG, CURRENT);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	/* Control of I/O register */

	flash_ctrl = 0;
	//	rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &flash_ctrl);
	rc = rt8542_update_bits(rt8542, RT8542_REG_LED_CTRL, RT8542_TORCH_FLASH_MASK, ENABLE_FL);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	
	strobe_ctrl &= 0xDF; /* 1101 1111 */
	strobe_ctrl |= 0x10; /* 0001 0000 */

//	rc =flash_write_reg(fctrl->flash_i2c_client,	0x09, strobe_ctrl);

	rc =rt8542_write_byte(rt8542, FLASH_CTR2_REG, strobe_ctrl);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	/* Enable */
	flash_ctrl=0;
	rc = rt8542_read_byte(rt8542, RT8542_REG_LED_CTRL, &flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	flash_ctrl &= 0xFD; /* clear bit1*/
	flash_ctrl |= 0x04; /* set bit2*/

	rc =rt8542_write_byte(rt8542, RT8542_REG_LED_CTRL, flash_ctrl);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	flash_ctrl |= 0x66; /* 0110 0110*/
	rc =rt8542_write_byte(rt8542, RT8542_REG_LED_CTRL, flash_ctrl);
	if (rc < 0){
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}
	
	return rc;
}

static int32_t rt8542_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	rt8542_flash *rt8542_fl = NULL;
	struct rt8542 *rt8542 = NULL;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	CDBG("%s called led_state %d\n",__func__, cfg->cfgtype);

	if (fctrl != NULL) {
		rt8542_fl = (rt8542_flash*)fctrl->data;
		rt8542 = rt8542_fl->rt8542;
	}
	else{
		pr_err("%s:%d fctrl is null\n", __func__, __LINE__);
		return -EINVAL;
	}

	if(rt8542 == NULL) {
        	pr_err("%s:%d rt8542\n", __func__, __LINE__);
		return -EINVAL;
	}

	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_OFF:		
		rc = rt8542_off(rt8542);
		break;
	case MSM_CAMERA_LED_LOW:
		rc = rt8542_low(rt8542);
		break;

	case MSM_CAMERA_LED_HIGH:
		rc = rt8542_high(rt8542);
		break;

	case MSM_CAMERA_LED_INIT:
//		rc = rt8542_flash_init(fctrl);
		torch_toggle = 0;
		CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);
		break;
		
	case MSM_CAMERA_LED_RELEASE:
		rc = rt8542_release(rt8542);
		break;
	default:
		pr_err("%s invalid param\n", __func__);
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}

static const struct of_device_id rt8542_dt_match[] = {
	{.compatible = "richtek,rt8542-flash"},
	{}
};

MODULE_DEVICE_TABLE(of, rt8542_dt_match);

static struct platform_driver rt8542_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rt8542_dt_match,
	},
};

static int32_t rt8542_probe(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	int32_t rc = 0;
	struct rt8542 *rt8542 = dev_get_drvdata(pdev->dev.parent);
	rt8542_flash *rt8542_fl;

	CDBG("%s called\n", __func__);
	rt8542_fl = devm_kzalloc(&pdev->dev, sizeof(rt8542_flash), GFP_KERNEL);
	if (!rt8542_fl)
		return -ENOMEM;

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl.pdev = pdev;
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

	rt8542_fl->rt8542 = rt8542;
	platform_set_drvdata(pdev, rt8542_fl);

	rc = msm_led_flash_create_v4lsubdev(pdev, &fctrl);
	if (!rc)
		msm_led_torch_create_classdev(pdev, &fctrl);
	fctrl.data = (void*)rt8542_fl;
	
	return rc;
}


static int __init rt8542_init(void)
{
	CDBG("%s called\n", __func__);
	return platform_driver_probe(&rt8542_driver,
		rt8542_probe);
}

static struct msm_flash_fn_t rt8542_func_tbl = {
	.flash_get_subdev_id = rt8542_get_subdev_id,
	.flash_led_config = rt8542_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
//	.flash_i2c_client = &rt8542_flash_i2c_client,
	.func_tbl = &rt8542_func_tbl,
};

module_init(rt8542_init);
MODULE_DESCRIPTION("rt8542 FLASH");
MODULE_LICENSE("GPL v2");
