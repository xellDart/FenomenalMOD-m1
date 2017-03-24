/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#define XO_CLK_RATE	19200000

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"

int lge_mdss_dsi_lane_config(struct mdss_panel_data *pdata, int enable)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mipi = &pdata->panel_info.mipi;

	if (mipi->force_clk_lane_hs) {
		u32 tmp;

		tmp = MIPI_INP((ctrl_pdata->ctrl_base) + 0xac);
		if (enable) {
			tmp |= BIT(28);
		} else {
			tmp &= ~BIT(28);
		}
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0xac, tmp);
		wmb();
		pr_info("%s: current mode=%s dsi_lane_ctrl=0x%08x\n",
			__func__, (enable ? "hs" : "lp"), MIPI_INP((ctrl_pdata->ctrl_base) + 0xac));
	}
	return 0;
}

int tianma_hvga_cmd_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int ret = 0;

	if (!enable) {
		lge_mdss_dsi_lane_config(pdata, enable);

		mdelay(10);

		ret = lge_mdss_dsi_panel_reset(pdata, 0);
		if (ret) {
			pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
			ret = 0;
		}
	}
	return ret;
}

int tianma_hvga_cmd_post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	if (IS_ENABLED(CONFIG_LGE_PM_PARALLEL_CHARGING))
		smbchg_fb_notify_update_cb(enable);

	return 0;
}

int lgd_fhd_video_mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int tianma_hvga_cmd_dsi_panel_device_register(struct device_node *pan_node,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

#if defined(CONFIG_LGD_SSD2068_FWVGA_VIDEO_PANEL)
int lgd_fwvga_video_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int ret = 0;

	if (!enable) {
		lge_mdss_dsi_lane_config(pdata, enable);

		mdelay(10);

		ret = lge_mdss_dsi_panel_reset(pdata, 0);
		if (ret) {
			pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
			ret = 0;
		}
	}
	return ret;
}

int lgd_fwvga_video_post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	if (IS_ENABLED(CONFIG_LGE_PM_PARALLEL_CHARGING))
		smbchg_fb_notify_update_cb(enable);

	return 0;
}

int lgd_fwvga_video_dsi_panel_device_register(struct device_node *pan_node,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int lgd_fwvga_video_msm_mdss_enable_vreg(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int i = 0;
	int ret = 0;

	if (enable) {
		regulator_set_voltage(ctrl_pdata->power_data[2].vreg_config[0].vreg, 3000000, 3000000);
		mdelay(1);

		for (i = 0; i < DSI_MAX_PM; i++) {
			if (!ctrl_pdata->panel_data.panel_info.cont_splash_enabled)
				if (DSI_PANEL_PM == i)
					continue;

			if (DSI_CORE_PM == i)
				continue;

			ret = msm_dss_enable_vreg(
					ctrl_pdata->power_data[i].vreg_config,
					ctrl_pdata->power_data[i].num_vreg, 1);

			if (ret) {
				pr_err("%s: failed to enable vregs for %s\n",
						__func__, __mdss_dsi_pm_name(i));
				return ret;
			}
		}
	} else {
		regulator_set_voltage(ctrl_pdata->power_data[2].vreg_config[0].vreg, 3300000, 3300000);
		mdelay(1);

		for (i = DSI_MAX_PM - 1; i >= 0; i--) {
			if (DSI_PANEL_PM == i)
				continue;

			if (DSI_CORE_PM == i)
				continue;

			ret = msm_dss_enable_vreg(
					ctrl_pdata->power_data[i].vreg_config,
					ctrl_pdata->power_data[i].num_vreg, 0);

			if (ret) {
				pr_err("%s: failed to disable vregs for %s\n",
						__func__, __mdss_dsi_pm_name(i));
				return ret;
			}

		}
	}

	return ret;
}

#elif defined (CONFIG_LGD_DB7400_HD_VIDEO_PANEL)
int lgd_db7400_hd_video_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int ret = 0;

	if (!enable) {
		lge_mdss_dsi_lane_config(pdata, enable);
	}

	return ret;
}

int lgd_db7400_hd_video_post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	if (IS_ENABLED(CONFIG_LGE_PM_PARALLEL_CHARGING))
		smbchg_fb_notify_update_cb(enable);

	return 0;
}

int lgd_db7400_hd_video_dsi_panel_device_register(struct device_node *pan_node,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int lgd_db7400_hd_video_msm_mdss_enable_vreg(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int i = 0;
	int ret = 0;

	if (enable) {
		for (i = 0; i < DSI_MAX_PM; i++) {
			if (!ctrl_pdata->panel_data.panel_info.cont_splash_enabled)
				if (DSI_PANEL_PM == i)
					continue;

			if (DSI_CORE_PM == i)
				continue;

			ret = msm_dss_enable_vreg(
					ctrl_pdata->power_data[i].vreg_config,
					ctrl_pdata->power_data[i].num_vreg, 1);

			if (ret) {
				pr_err("%s: failed to enable vregs for %s\n",
					__func__, __mdss_dsi_pm_name(i));
				return ret;
			}
		}
	} else {
		for (i = DSI_MAX_PM - 1; i >= 0; i--) {
			if (DSI_PANEL_PM == i)
				continue;

			if (DSI_CORE_PM == i)
				continue;
			ret = msm_dss_enable_vreg(
					ctrl_pdata->power_data[i].vreg_config,
					ctrl_pdata->power_data[i].num_vreg, 0);

			if (ret) {
				pr_err("%s: failed to disable vregs for %s\n",
					__func__, __mdss_dsi_pm_name(i));
				return ret;
			}

		}

	}

	return ret;
}
#elif defined(CONFIG_TCL_ILI9806E_FWVGA_VIDEO_PANEL)
int tcl_ili9806e_fwvga_video_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int ret = 0;

	if (!enable) {
		lge_mdss_dsi_lane_config(pdata, enable);

		mdelay(10);

		ret = lge_mdss_dsi_panel_reset(pdata, 0);
		if (ret) {
			pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
			ret = 0;
		}
	}
	return ret;
}

int tcl_ili9806e_fwvga_video_post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	if (IS_ENABLED(CONFIG_LGE_PM_PARALLEL_CHARGING))
		smbchg_fb_notify_update_cb(enable);

	return 0;
}

int tcl_ili9806e_fwvga_video_mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int tcl_ili9806e_fwvga_video_dsi_panel_device_register(struct device_node *pan_node,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}
#endif
