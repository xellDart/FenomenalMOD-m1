/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  	: lgtp_device_dummy.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[MIT200]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_device_mit200_proc4.h>

#include <soc/qcom/lge/lge_board_revision.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
#if defined( TOUCH_MODEL_P1C)
static const char defaultFirmware[] = "melfas/mit200/p1c/melfas_mip4.bin";
#endif

#if defined( TOUCH_MODEL_M1 )
#if defined( CONFIG_MACH_MSM8909_M1_SPR_US ) || defined( CONFIG_MACH_MSM8909_M1_TMO_US ) || defined( CONFIG_MACH_MSM8909_M1_TRF_US_VZW ) || defined( CONFIG_MACH_MSM8909_M1_TRF_US ) || defined( CONFIG_MACH_MSM8909_M1_MPCS_US )
static const char defaultFirmware[] = "melfas/m1_spr_us/fw.bin";
#elif defined (CONFIG_MACH_MSM8909_M1DS_GLOBAL_COM)
static const char defaultFirmware[] = "melfas/m1ds_global_com/fw.bin";
static const char defaultFirmwareRevA[] = "melfas/m1ds_global_com/fw_a.bin";
#endif
#endif

static struct melfas_ts_data *ts = NULL;

#if defined(ENABLE_SWIPE_MODE)
static int get_swipe_mode = 1;
/*
static int wakeup_by_swipe = 0;
*/
extern int lockscreen_stat;
#endif
int palm_state;

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
static void MIT200_WriteFile(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (fd >= 0) {
		if (time > 0) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, 64, "\n%02d-%02d %02d:%02d:%02d.%03lu \n\n\n", my_date.tm_mon + 1,my_date.tm_mday, my_date.tm_hour, my_date.tm_min, my_date.tm_sec, (unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);
}


/****************************************************************************
* Device Specific Functions
****************************************************************************/
int mip_i2c_dummy(struct i2c_client* client,  char *write_buf, unsigned int write_len)
{
	int retry = 3;
	int res = 0;

	while (retry--) {
		TOUCH_FUNC();
		res = Mit200_proc4_I2C_Write(client, write_buf, write_len);
		if(res < 0) {
			TOUCH_ERR("i2c_transfer - errno[%d]\n", res);
			return TOUCH_FAIL;
		}
	}

	return TOUCH_SUCCESS;
}


int mip_lpwg_config(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_IDLE_REPORTRATE;
	wbuf[2] = 20;							// LPWG_IDLE_REPORTRATE
	wbuf[3] = 60;							// LPWG_ACTIVE_REPORTRATE
	wbuf[4] = 20;							// LPWG_SENSITIVITY
	wbuf[5] = 23 & 0xFF;						// LPWG_ACTIVE_AREA (horizontal start low byte)
	wbuf[6] = 23 >> 8 & 0xFF; 				// LPWG_ACTIVE_AREA (horizontal start high byte)
	wbuf[7] = 23 & 0xFF;						// LPWG_ACTIVE_AREA (vertical start low byte)
	wbuf[8] = 23 >> 8 & 0xFF;				// LPWG_ACTIVE_AREA (vertical start high byte)
	wbuf[9] = 457 & 0xFF;					// LPWG_ACTIVE_AREA (horizontal end low byte)
	wbuf[10] = 457 >> 8 & 0xFF;				// LPWG_ACTIVE_AREA (horizontal end high byte)
	wbuf[11] = 831 & 0xFF; 				// LPWG_ACTIVE_AREA (vertical end low byte)
	wbuf[12] = 831 >> 8 & 0xFF;			// LPWG_ACTIVE_AREA (vertical end high byte)
	wbuf[13] = 1;							// LPWG_FAIL_REASON

	printk("\n");printk("\n");
	TOUCH_LOG("========================== LPWG setting ==========================\n");
	TOUCH_LOG("[Hz] : idle(%4d), active(%4d)\n", wbuf[2], wbuf[3]);
	TOUCH_LOG("sensitivity(%4d)\n", wbuf[4]);
	TOUCH_LOG("[area start] hori.(%4d) vert.(%4d)\n", (wbuf[6]<<8)|wbuf[5], (wbuf[8]<<8)|wbuf[7]);
	TOUCH_LOG("[area end  ] hori.(%4d) vert.(%4d)\n", (wbuf[10]<<8)|wbuf[9], (wbuf[12]<<8)|wbuf[11]);
	TOUCH_LOG("fail reason(%d)\n", wbuf[13]);

	if( Mit200_proc4_I2C_Write(client, wbuf, 14) )
	{
		TOUCH_ERR("mip_lpwg_config failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_config_knock_on(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE;
	wbuf[2] = 1;							// LPWG_ENABLE
	wbuf[3] = 2;							// LPWG_TAP_COUNT
	wbuf[4] = 10 & 0xFF;					// LPWG_TOUCH_SLOP (low byte)
	wbuf[5] = 10 >> 8 & 0xFF;				// LPWG_TOUCH_SLOP (high byte)
	wbuf[6] = 0 & 0xFF;						// LPWG_MIN_DISTANCE (low byte)
	wbuf[7] = 0 >> 8 & 0xFF;				// LPWG_MIN_DISTANCE (high byte)
	wbuf[8] = 10 & 0xFF;					// LPWG_MAX_DISTANCE (low byte)
	wbuf[9] = 10 >> 8 & 0xFF;				// LPWG_MAX_DISTANCE (high byte)
	wbuf[10] = 0 & 0xFF;					// LPWG_MIN_INTERTAP_TIME (low byte)
	wbuf[11] = 0 >> 8 & 0xFF;				// LPWG_MIN_INTERTAP_TIME (high byte)
	wbuf[12] = 700 & 0xFF;					// LPWG_MAX_INTERTAP_TIME (low byte)
	wbuf[13] = 700 >> 8 & 0xFF;				// LPWG_MAX_INTERTAP_TIME (high byte)
	wbuf[14] = (ts->lpwgSetting.isFirstTwoTapSame ? KNOCKON_DELAY : 0) & 0xFF;         // LPWG_INTERTAP_DELAY (low byte)
	wbuf[15] = ((ts->lpwgSetting.isFirstTwoTapSame ? KNOCKON_DELAY : 0) >> 8) & 0xFF;  // LPWG_INTERTAP_DELAY (high byte)

	TOUCH_LOG("enable(%4d), tap_count(%4d) touch_slope(%4d)\n", wbuf[2], wbuf[3], (wbuf[5]<<8)|wbuf[4]);
	TOUCH_LOG("[intertap distance] min(%4d), max(%4d)\n", (wbuf[7]<<8)|wbuf[6], (wbuf[9]<<8)|wbuf[8]);
	TOUCH_LOG("[intertap time    ] min(%4d), max(%4d)\n", (wbuf[11]<<8)|wbuf[10], (wbuf[13]<<8)|wbuf[12]);
	TOUCH_LOG("interrupt_delay(%4d)\n", (wbuf[15]<<8)|wbuf[14]);

	if( Mit200_proc4_I2C_Write(client, wbuf, 16) )
	{
		TOUCH_ERR("Knock on Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_config_knock_code(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE2;
	wbuf[2] = 1;							// LPWG_ENABLE2
	wbuf[3] = ts->lpwgSetting.tapCount;		// LPWG_TAP_COUNT2
	wbuf[4] = 10 & 0xFF;					// LPWG_TOUCH_SLOP2 (low byte)
	wbuf[5] = 10 >> 8 & 0xFF;				// LPWG_TOUCH_SLOP2 (high byte)
	wbuf[6] = 0 & 0xFF;						// LPWG_MIN_DISTANCE2 (low byte)
	wbuf[7] = 0 >> 8 & 0xFF;				// LPWG_MIN_DISTANCE2 (high byte)
	wbuf[8] = 65535 & 0xFF;					// LPWG_MAX_DISTANCE2 (low byte)
	wbuf[9] = 65535 >>8 & 0xFF;				// LPWG_MAX_DISTANCE2 (high byte)
	wbuf[10] = 0 & 0xFF;					// LPWG_MIN_INTERTAP_TIME2 (low byte)
	wbuf[11] = 0 >> 8 & 0xFF;				// LPWG_MIN_INTERTAP_TIME2 (high byte)
	wbuf[12] = 700 & 0xFF;					// LPWG_MAX_INTERTAP_TIME2 (low byte)
	wbuf[13] = 700 >> 8 & 0xFF;				// LPWG_MAX_INTERTAP_TIME2 (high byte)
	wbuf[14] = 700 & 0xFF;					// LPWG_INTERTAP_DELAY2 (low byte)
	wbuf[15] = 700 >> 8 & 0xFF;				// LPWG_INTERTAP_DELAY2 (high byte)

	TOUCH_LOG("enable(%4d), tap_count(%4d) touch_slope(%4d)\n", wbuf[2], wbuf[3], (wbuf[5]<<8)|wbuf[4]);
	TOUCH_LOG("[intertap distance] min(%4d), max(%4d)\n", (wbuf[7]<<8)|wbuf[6], (wbuf[9]<<8)|wbuf[8]);
	TOUCH_LOG("[intertap time    ] min(%4d), max(%4d)\n", (wbuf[11]<<8)|wbuf[10], (wbuf[13]<<8)|wbuf[12]);
	TOUCH_LOG("interrupt_delay(%4d)\n", (wbuf[15]<<8)|wbuf[14]);

	if( Mit200_proc4_I2C_Write(client, wbuf, 16) )
	{
		TOUCH_ERR("Knock code Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_debug_enable(struct i2c_client* client)
{
	u8 wbuf[4];

	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
		return TOUCH_FAIL;
	}

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_LPWG_DEBUG_ENABLE;

	if(pDriverData->lpwg_debug_enable == 0){
		wbuf[2] = 0;
	}else{
		wbuf[2] = 1;
	}

	if( Mit200_proc4_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("LPWG debug Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_enable_sensing(struct i2c_client* client, bool enable)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE_SENSING;
	wbuf[2] = enable;

	TOUCH_FUNC();
	if( Mit200_proc4_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("mip_lpwg_enable_sensing failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_start(struct i2c_client* client)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_START;
	wbuf[2] = 1;

	TOUCH_FUNC();

	TOUCH_LOG("[%s] lpwg start %d, %d, %d\n",__func__,wbuf[0],wbuf[1],wbuf[2]);
	if( Mit200_proc4_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("mip_lpwg_start failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
		return TOUCH_FAIL;
	}

	TOUCH_FUNC();

/*	TOUCH_LOG("==============  LPWG state  ==============\n");
	TOUCH_LOG("STATE_UNKNOWN = 0\n"); 
	TOUCH_LOG("STATE_BOOT = 1\n"); 
	TOUCH_LOG("STATE_NORMAL = 2\n"); 
	TOUCH_LOG("STATE_OFF = 3\n");
	TOUCH_LOG("STATE_KNOCK_ON_ONLY = 4\n"); 
	TOUCH_LOG("STATE_KNOCK_ON_CODE = 5\n"); 
	TOUCH_LOG("STATE_NORMAL_HOVER = 6\n");
	TOUCH_LOG("STATE_HOVER = 7\n");
	TOUCH_LOG("STATE_UPDATE_FIRMWARE = 8\n"); 
	TOUCH_LOG("STATE_SELF_DIAGNOSIS = 9\n");
	TOUCH_LOG("==============  LPWG state  ==============\n");
*/
	TOUCH_LOG("currState = [%d], newState = [%d]\n", ts->currState, newState);

	switch( newState )
	{
		case STATE_NORMAL:
			break;

		case STATE_KNOCK_ON_ONLY :
			TOUCH_LOG("case STATE_KNOCK_ON_ONLY\n");
			if( ts->currState == STATE_OFF ){
				TOUCH_LOG("No STATE_KNOCK_ON_ONLY setting!!\n");

			}else{
			//mip_lpwg_config(client);
			//mip_lpwg_config_knock_on(client);
			//if( LPWG_DEBUG_ENABLE )
			if(pDriverData->lpwg_debug_enable == 1){
				//TOUCH_LOG("[%s] lpwg_debug_enable status = %d\n",__func__,pDriverData->lpwg_debug_enable);
				mip_lpwg_debug_enable(client);
			}
			//mip_lpwg_start(client);
			//f( ts->currState == STATE_OFF )
				//TOUCH_LOG("LPWG mode change STATE_OFF -> STATE_KNOCK_ON_ONLY, Touch driver do nothing\n");
			//mip_lpwg_enable_sensing(client, 1);
			}//end of else
#if defined(SENSOR_DETECT)
		        if(pDriverData->lpwgSetting.lcdState == 0 && ts->currState != STATE_NORMAL){
		                if(pDriverData->enable_sensor_interlock)
				{
						TOUCH_LOG("LPWG mode change STATE_OFF -> STATE_KNOCK_ON_ONLY, Touch driver do nothing\n");
	                                MIT200_mcu_sleep_control(client);
		                }
			 }
#endif
			//mip_lpwg_start(client);
			mip_lpwg_config(client);
			mip_lpwg_config_knock_on(client);
			mip_lpwg_start(client);
			break;

		case STATE_KNOCK_ON_CODE:
			TOUCH_LOG("case STATE_KNOCK_ON_CODE\n");
			if( ts->currState == STATE_OFF ){
				TOUCH_LOG("No STATE_KNOCK_ON_ONLY setting!!\n");
			}else{
			//mip_lpwg_config(client);
			//mip_lpwg_config_knock_on(client);
			//mip_lpwg_config_knock_code(client);
			//if( LPWG_DEBUG_ENABLE )
			if(pDriverData->lpwg_debug_enable == 1){
				TOUCH_LOG("[%s] lpwg_debug_enable status = %d\n",__func__,pDriverData->lpwg_debug_enable);
				mip_lpwg_debug_enable(client);
			}
		//mip_lpwg_start(client);
			//if( ts->currState == STATE_OFF )
			//mip_lpwg_enable_sensing(client, 1);

			//mip_lpwg_start(client);
			}//end of else
#if defined(SENSOR_DETECT)
			if(pDriverData->lpwgSetting.lcdState == 0 && ts->currState != STATE_NORMAL){
				if(pDriverData->enable_sensor_interlock)
				{
					MIT200_mcu_sleep_control(client);
				}
			}
#endif
			//mip_lpwg_start(client);
			mip_lpwg_config(client);
			mip_lpwg_config_knock_on(client);
			mip_lpwg_config_knock_code(client);
			mip_lpwg_start(client);
			break;

		case STATE_OFF:
			TOUCH_LOG("case STATE_OFF\n");
			if( ts->currState == STATE_NORMAL || ts->currState == STATE_NORMAL_HOVER ){
				TOUCH_LOG("LPWG mode change STATE_NORMAL -> STATE_OFF, Touch driver set to lpwg setting\n");
				mip_lpwg_config(client);
				mip_lpwg_start(client);
			//mip_lpwg_enable_sensing(client, 0);
			}
#if defined(SENSOR_DETECT)
			if(pDriverData->lpwgSetting.lcdState == 0){
				if(pDriverData->enable_sensor_interlock)
				{
					MIT200_mcu_sleep_control(client);
				}
			}
#endif
			//mip_lpwg_start(client);
			break;

		default:
			TOUCH_ERR("invalid touch state ( %d )\n", newState);
			break;
	}

	return TOUCH_SUCCESS;
}


int mip_palm_rejection_enable(struct i2c_client* client, bool enable)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_PALM_REJECTION;
	wbuf[2] = enable;

	if( Mit200_proc4_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("Palm Rejection Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

#if defined(ENABLE_SWIPE_MODE)
static ssize_t show_swipe_mode(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u\n", get_swipe_mode);

	return ret;
}

static ssize_t store_swipe_mode(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	get_swipe_mode = value;

	return count;
}
#endif

static int MIT200_show_product_id(struct i2c_client *client, char *buf)
{
	u8 wbuf[2] = {0, };
	u8 product_code[16] = {0, };
	int ret = 0;

	TOUCH_FUNC();

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;

	ret = Mit200_proc4_I2C_Read(client, wbuf, 2, product_code, 16);
	if( ret == TOUCH_FAIL )
	{
		TOUCH_ERR("Product ID Read fail!\n");
		return TOUCH_FAIL;
	}

	memcpy(ts->product_code,product_code,sizeof(product_code));
	sprintf(buf,product_code);
	return TOUCH_SUCCESS;
}

static int show_product_id(struct i2c_client *client, char *buf)
{
	u8 wbuf[2] = {0, };
	u8 product_code[16] = {0, };
	int ret = 0;

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;

	ret = Mit200_proc4_I2C_Read(client, wbuf, 2, product_code, 16);
	if( ret == TOUCH_FAIL )
	{
		TOUCH_ERR("Product ID Read fail!\n");
		return TOUCH_FAIL;
	}

	memcpy(ts->product_code,product_code,sizeof(product_code));
	sprintf(buf,product_code);
	return TOUCH_SUCCESS;
}

static ssize_t show_jitter(struct i2c_client *client, char *buf)
{
		int ret = 0;
		int intensityStatus = 0;

		TouchDriverData *pDriverData = i2c_get_clientdata(client);
		if (pDriverData == NULL){
				TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
		}

		TOUCH_FUNC();

		// intensity check
		ret = MIT200_GetTestResult(client, buf, &intensityStatus, JITTER_SHOW);
		if (ret < 0) {
				TOUCH_ERR("failed to get jitter data\n");
				ret = sprintf(buf, "failed to get jitter data\n");
		}

		return ret;
}

static ssize_t show_intensity(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int intensityStatus = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	TOUCH_FUNC();

	// intensity check
	ret = MIT200_GetTestResult(client, buf, &intensityStatus, INTENSITY_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get intensity data\n");
		ret = sprintf(buf, "failed to get intensity data\n");
	}

	return ret;
}

/*This function used in Normal boot mode*/
static ssize_t show_lpwg_raw_data(struct i2c_client *client, char *buf)
{
	char *sd_path = "/sdcard/touch_self_test.txt";
	int ret = 0;
	int pRawStatus = 0;
	int bufSize = 4*1024;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
		return TOUCH_FAIL;
	}

	MIT200_WriteFile(sd_path, buf, 1);
	memset(buf, 0, bufSize);

/*
	if(pDriverData->lpwgSetting.lcdState == 0 || pDriverData->lpwg_debug_enable == 0){
		ret = snprintf(buf,PAGE_SIZE,"If you want to check raw data, please turn on the LCD.\n");
		TOUCH_LOG("[%s]  LCD state = %d, lpwg debug state = %d\n",__func__,pDriverData->lpwgSetting.lcdState,pDriverData->lpwg_debug_enable);
		return 0;
	}else{	
		ret = MIT200_GetTestResult(client, buf, &pRawStatus, RAW_DATA_SHOW_IMG);
		if( ret < 0 ) {
			TOUCH_ERR("failed to get raw data\n");
			ret = sprintf(buf, "failed to get intensity data\n");
			pRawStatus = TOUCH_FAIL;
		}
*/
	ret = MIT200_GetTestResult(client, buf, &pRawStatus,RAW_DATA_SHOW_IMG);
	if( ret < 0 ) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(buf, "failed to get rawdata\n");

		pRawStatus = TOUCH_FAIL;
	}else{
		MIT200_WriteFile(sd_path, buf, 0);
	}
		msleep(30);

		//lpwg_debug_enable_set_status(client,0);
	//}//end of if(pDriverData->lpwgSetting.lcdState == 0 || pDriverData->lpwg_debug_enable == 0){

	return ret;
}

/*This function used in MiniOS only*/
static ssize_t MIT200_lpwg_raw_data_show(struct i2c_client *client, char *buf)
{
	char *sd_path = "/sdcard/touch_self_test.txt";
	int ret = 0;
	int pRawStatus = 0;
	int bufSize = 4*1024;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	//lpwg_debug_enable_set_status(client,1);
	mdelay(2000);
	TOUCH_FUNC();
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	MIT200_WriteFile(sd_path, buf, 1);
	memset(buf, 0, bufSize);
	sprintf(buf,"%s\n","[LPWG RAWDATA]\n");
	if(pDriverData->lpwgSetting.lcdState != 0 || pDriverData->lpwg_debug_enable == 0){
		ret = snprintf(buf,PAGE_SIZE,"If you want to check raw data, please turn off the LCD.\n");
		TOUCH_LOG("[%s]  LCD state = %d, lpwg debug state = %d\n",__func__,pDriverData->lpwgSetting.lcdState,pDriverData->lpwg_debug_enable);
		ret = TOUCH_FAIL;
	}else{
		TOUCH_LOG("[%s]  LCD state = %d, lpwg debug state = %d\n",__func__,pDriverData->lpwgSetting.lcdState,pDriverData->lpwg_debug_enable);
		ret = MIT200_GetTestResult(client, buf, &pRawStatus, RAW_DATA_SHOW);
		MIT200_WriteFile(sd_path, buf, 0);
		msleep(30);
	}

	if( ret < 0 ) {
		TOUCH_ERR("failed to get raw data\n");
		pRawStatus = TOUCH_FAIL;
	}

	if(pRawStatus==TOUCH_SUCCESS){
		ret = TOUCH_SUCCESS;
	}else{
		ret = TOUCH_FAIL;
	}
	//lpwg_debug_enable_set_status(client,0);

	return ret;
}


static ssize_t show_sensing_status(struct i2c_client *client, char *buf)
{
	u8 wbuf[2] = {0, };
	u8 sensing_status = -1;
	int ret = 0;

	TOUCH_FUNC();

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = SENSING_ON_OFF;

	TOUCH_LOG("store value wbuf[0] = %x, [1] = %x\n",wbuf[0],wbuf[1]);

	ret = Mit200_proc4_I2C_Read(client, wbuf, 2, &sensing_status, 1);
	if( ret == TOUCH_FAIL )
	{
		TOUCH_ERR("Sensing status read fail!\n");
		return TOUCH_FAIL;
	}

	TOUCH_LOG("============ sensing_status = %d ============ n",sensing_status);

	ret = sprintf(buf, "%u\n", sensing_status);
	return ret;
}

static ssize_t store_sensing_status(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;
	u8 wbuf[4];

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = SENSING_ON_OFF;
	wbuf[2] = 1;

	sscanf(buf, "%d", &value);

	switch (value) {
	case 0:
		wbuf[2] = 0;
		TOUCH_DBG("touch sensing set to 0\n");
		break;
	case 1:
		wbuf[2] = 1;
		TOUCH_DBG("touch sensing set to 1\n");
		break;
	default:
	    break;
	}

	TOUCH_LOG("============================================================\n");
	TOUCH_LOG("store value wbuf[0] = %x, [1] = %x, [2] = %x\n",wbuf[0],wbuf[1],wbuf[2]);
	TOUCH_LOG("============================================================\n");

	if( Mit200_proc4_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("store_sensing_status failed\n");
		return TOUCH_FAIL;
	}

	return count;
}

static ssize_t show_lpwg_debug_enable_state(struct i2c_client *client, char *buf)
{
	int ret = 0;
	u8 lpwg_state = -1;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	if(pDriverData->lpwg_debug_enable == 0) lpwg_state = 0;
	else lpwg_state = 1;

	ret = sprintf(buf, "%u\n", lpwg_state);

	return ret;
}

static ssize_t store_lpwg_debug_enable_state(struct i2c_client *client, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	switch (value) {
	case 0:
		lpwg_debug_enable_set_status(client,0);
		TOUCH_DBG("lpwg debug enable set to 0\n");
		break;
	case 1:
		lpwg_debug_enable_set_status(client,1);
		TOUCH_DBG("lpwg debug enable set to 1\n");
		break;
	default:
		    break;
	}
	return count;
}

static ssize_t show_lcd_status(struct i2c_client *client, char *buf)
{
	u8 wbuf[2] = {0, };
	u8 lcd_in_sleep_status = -1;
	int ret = 0;

	TOUCH_FUNC();

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R0_LCD_IN_SLEEP;

	TOUCH_LOG("wbuf[0] = %x, [1] = %x\n",wbuf[0],wbuf[1]);

	ret = Mit200_proc4_I2C_Read(client, wbuf, 2, &lcd_in_sleep_status, 1);
	if( ret == TOUCH_FAIL )
	{
		TOUCH_ERR("Sensing status read fail!\n");
		return TOUCH_FAIL;
	}

	TOUCH_LOG("============ lcd_in_sleep_status read = %d ============ n",lcd_in_sleep_status);

	ret = sprintf(buf, "%u\n", lcd_in_sleep_status);
	return ret;
}


#if defined(ENABLE_SWIPE_MODE)
static LGE_TOUCH_ATTR(swipe_mode, S_IRUGO | S_IWUSR, show_swipe_mode, store_swipe_mode);
#endif
static LGE_TOUCH_ATTR(product_id, S_IRUGO | S_IWUSR, show_product_id, NULL);
static LGE_TOUCH_ATTR(intensity, S_IRUGO | S_IWUSR, show_intensity, NULL);
static LGE_TOUCH_ATTR(show_rawdata, S_IRUGO | S_IWUSR, show_lpwg_raw_data, NULL);
static LGE_TOUCH_ATTR(jitter, S_IRUGO | S_IWUSR, show_jitter, NULL);
static LGE_TOUCH_ATTR(sensing_status, S_IRUGO | S_IWUSR, show_sensing_status, store_sensing_status);
static LGE_TOUCH_ATTR(lpwg_debug, S_IRUGO | S_IWUSR, show_lpwg_debug_enable_state, store_lpwg_debug_enable_state);
static LGE_TOUCH_ATTR(lcd_status, S_IRUGO | S_IWUSR, show_lcd_status, NULL);

static struct attribute *MIT200_attribute_list[] = {
#if defined(ENABLE_SWIPE_MODE)
	&lge_touch_attr_swipe_mode.attr,
#endif
	&lge_touch_attr_product_id.attr,
	&lge_touch_attr_intensity.attr,
	&lge_touch_attr_show_rawdata.attr,
	&lge_touch_attr_jitter.attr,
	&lge_touch_attr_sensing_status.attr,
	&lge_touch_attr_lpwg_debug.attr,
	&lge_touch_attr_lcd_status.attr,
	NULL,
};

#if MIP_USE_DEV
/**
* Dev node output to user
*/
static ssize_t mip_dev_fs_read(struct file *fp, char *rbuf, size_t cnt, loff_t *fpos)
{
	struct melfas_ts_data *ts = fp->private_data;
	int ret = 0;

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	ret = copy_to_user(rbuf, ts->dev_fs_buf, cnt);

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return ret;
}

/**
* Dev node input from user
*/
static ssize_t mip_dev_fs_write(struct file *fp, const char *wbuf, size_t cnt, loff_t *fpos)
{
	struct melfas_ts_data *ts = fp->private_data;
	u8 *buf;
	int ret = 0;
	int cmd = 0;

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	buf = kzalloc(cnt + 1, GFP_KERNEL);

	if ((buf == NULL) || copy_from_user(buf, wbuf, cnt)) {
		TOUCH_ERR("copy_from_user\n");
		ret = -EIO;
		goto EXIT;
	}

	cmd = buf[cnt - 1];

	if(cmd == 1){
		//dev_dbg(&info->client->dev, "%s - cmd[%d] w_len[%d] r_len[%d]\n", __func__, cmd, (cnt - 2), buf[cnt - 2]);

		if( Mit200_proc4_I2C_Read(ts->client, buf, (cnt - 2), ts->dev_fs_buf, buf[cnt - 2]) ){
			TOUCH_ERR("Mit200_I2C_Read\n");
		}
		//print_hex_dump(KERN_ERR, MIP_DEVICE_NAME" : input ", DUMP_PREFIX_OFFSET, 16, 1, wbuf, cnt, false);
		//print_hex_dump(KERN_ERR, MIP_DEVICE_NAME" : output ", DUMP_PREFIX_OFFSET, 16, 1, info->dev_fs_buf, buf[cnt - 2], false);
	}
	else if(cmd == 2){
		//dev_dbg(&info->client->dev, "%s - cmd[%d] w_len[%d]\n", __func__, cmd, (cnt - 1));
		if( Mit200_proc4_I2C_Write(ts->client, buf, (cnt - 1)) ){
			TOUCH_ERR("Mit200_I2C_Write\n");
		}
		//print_hex_dump(KERN_ERR, MIP_DEVICE_NAME" : input ", DUMP_PREFIX_OFFSET, 16, 1, wbuf, cnt, false);
	}
	else{
		goto EXIT;
	}

EXIT:
	kfree(buf);

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return ret;
}

/**
* Open dev node
*/
static int mip_dev_fs_open(struct inode *node, struct file *fp)
{
	struct melfas_ts_data *ts = container_of(node->i_cdev, struct melfas_ts_data, cdev);

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	fp->private_data = ts;

	ts->dev_fs_buf = kzalloc(1024 * 4, GFP_KERNEL);

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return 0;
}

/**
* Close dev node
*/
static int mip_dev_fs_release(struct inode *node, struct file *fp)
{
	struct melfas_ts_data *ts = fp->private_data;

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	kfree(ts->dev_fs_buf);

	//dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return 0;
}

/**
* Dev node info
*/
static struct file_operations mip_dev_fops = {
	.owner	= THIS_MODULE,
	.open	= mip_dev_fs_open,
	.release= mip_dev_fs_release,
	.read	= mip_dev_fs_read,
	.write	= mip_dev_fs_write,
};

/**
* Create dev node
*/
int mip_dev_create(struct melfas_ts_data *ts)
{
	int ret = 0;

	TOUCH_FUNC();

	if (alloc_chrdev_region(&ts->mip_dev, 0, 1, "lge_touch")) {
		TOUCH_ERR("alloc_chrdev_region\n");
		ret = -ENOMEM;
		goto ERROR;
	}

	cdev_init(&ts->cdev, &mip_dev_fops);
	ts->cdev.owner = THIS_MODULE;

	if (cdev_add(&ts->cdev, ts->mip_dev, 1)) {
		TOUCH_ERR("cdev_add\n");
		ret = -EIO;
		goto ERROR;
	}

	return 0;

ERROR:
	TOUCH_ERR("mip_dev_create\n");
	return 0;
}

#endif
static int MIT200_Initialize(struct i2c_client *client)
{
	int ret = 0;
	TOUCH_FUNC();

	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR("failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->client = client;

#if MIP_USE_DEV
	//Create dev node (optional)
	if(mip_dev_create(ts)){
		TOUCH_ERR("mip_dev_create\n");
		ret = -EAGAIN;
	}

	//Create dev
	ts->class = class_create(THIS_MODULE, "lge_touch");
	device_create(ts->class, NULL, ts->mip_dev, NULL, "lge_touch");
#endif
	return TOUCH_SUCCESS;
}

static void MIT200_Reset(struct i2c_client *client)
{
	TouchResetCtrl(0);
	msleep(10);
	TouchResetCtrl(1);
	msleep(100);

	TOUCH_LOG("Device was reset\n");
}

static int MIT200_Connect(void)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Device detection function */

	return TOUCH_SUCCESS;
}

static int MIT200_InitRegister(struct i2c_client *client)
{
	/* IMPLEMENT : Register initialization after reset */
	if(PALM_REJECTION_ENABLE)
		mip_palm_rejection_enable(client,1);

	return TOUCH_SUCCESS;
}

static int MIT200_InterruptHandler(struct i2c_client *client, TouchReadData *pData)
{
	TouchFingerData *pFingerData = NULL;
	u8 i = 0;
	u8 wbuf[8] = {0};
	u8 rbuf[256] = {0};
	u32 packet_size = 0;
	u8 packet_type = 0;
	u8 alert_type = 0;
	u8 index = 0;
	u8 state = 0;

	pData->type = DATA_UNKNOWN;
	pData->count = 0;
/*
	if (LPWG_DEBUG_ENABLE == 0 && (ts->currState == STATE_KNOCK_ON_ONLY || ts->currState == STATE_KNOCK_ON_CODE)) {
		if(mip_i2c_dummy(client, wbuf, 2) == TOUCH_FAIL){
			TOUCH_ERR("Fail to send dummy packet\n");
			return TOUCH_FAIL;
		}
	}
*/
	//Read packet info
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if( Mit200_proc4_I2C_Read(client, wbuf, 2, rbuf, 1) )
	{
		TOUCH_ERR("Read packet info\n");
		return TOUCH_FAIL;
	}

	packet_size = (rbuf[0] & 0x7F);
	packet_type = ((rbuf[0] >> 7) & 0x1);

	if( packet_size == 0 )
		return TOUCH_SUCCESS;

	//Read packet data
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
	if( Mit200_proc4_I2C_Read(client, wbuf, 2, rbuf, packet_size) )
	{
		TOUCH_ERR("Read packet data\n");
		return TOUCH_FAIL;
	}

	//Event handler
	if( packet_type == 0 )	/* Touch event */
	{
		for( i = 0 ; i < packet_size ; i += 6 )
		{
			u8 *tmp = &rbuf[i];

			if( (tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0 )
			{
				TOUCH_LOG("use sofrware key\n");
				continue;
			}

			index = (tmp[0] & 0xf) - 1;
			state = (tmp[0] & 0x80) ? 1 : 0;

			if( (index < 0) || (index > MAX_NUM_OF_FINGERS - 1) )
			{
				TOUCH_ERR("invalid touch index (%d)\n", index);
				return TOUCH_FAIL;
			}

			pData->type = DATA_FINGER;

			if( (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4 )
			{
				if( state ){
					TOUCH_LOG("Palm detected : %d \n", tmp[4]);
					palm_state = 1;
				}else{
				        TOUCH_LOG("Palm released : %d \n", tmp[4]);
					palm_state = 0;
				}
				if(!PALM_REJECTION_ENABLE)
				        return TOUCH_SUCCESS;
			}

			if( state )
			{
				pFingerData = &pData->fingerData[index];
				pFingerData->id = index ;
				pFingerData->x = tmp[2] | ((tmp[1] & 0x0f) << 8);
				pFingerData->y = tmp[3] | ((tmp[1] & 0xf0) << 4);
				pFingerData->width_major = tmp[5];
				pFingerData->width_minor = 0;
				pFingerData->orientation = 0;
				pFingerData->pressure = tmp[4];
				if( tmp[4] < 1 )
					pFingerData->pressure = 1;
				else if( tmp[4] > 255 - 1 )
					pFingerData->pressure = 255 - 1;
				if(PALM_REJECTION_ENABLE &&(tmp[0] & MIP_EVENT_INPUT_PALM) >> 4 )
					pFingerData->pressure = 255;
				pData->count++;
				pFingerData->status = FINGER_PRESSED;
			}else{
				pFingerData = &pData->fingerData[index];
				pFingerData->id = index ;
				pFingerData->status = 0;
			}
		}
	}
	else	/* Alert event */
	{
		alert_type = rbuf[0];

		if( alert_type == MIP_ALERT_ESD )	//ESD detection
		{
			TOUCH_LOG("ESD Detected!\n");
			return TOUCH_FAIL;
		}
		else if( alert_type == MIP_ALERT_WAKEUP )	//Wake-up gesture
		{
			if( rbuf[1] == MIP_EVENT_GESTURE_DOUBLE_TAP )
			{
				TOUCH_LOG("Knock-on Detected\n");
				pData->type = DATA_KNOCK_ON;
			}
			else if( rbuf[1] == MIP_EVENT_GESTURE_MULTI_TAP )
			{
				TOUCH_LOG("Knock-code Detected\n");
				pData->type = DATA_KNOCK_CODE;

				for( i = 2 ; i < packet_size ; i += 3 )
				{
					u8 *tmp = &rbuf[i];
					pData->knockData[((i + 1) / 3) - 1].x = tmp[1] | ((tmp[0] & 0xf) << 8);
					pData->knockData[((i + 1) / 3) - 1].y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);
					pData->count++;
				}
			}
			else
			{
				//Re-enter tap mode
				wbuf[0] = MIP_R0_CTRL;
				wbuf[1] = MIP_R1_CTRL_POWER_STATE;
				wbuf[2] = MIP_CTRL_POWER_LOW;
				if( Mit200_proc4_I2C_Write(client, wbuf, 3) )
				{
					TOUCH_ERR("mip_i2c_write failed\n");
					return TOUCH_FAIL;
				}
			}
		}
		else if( alert_type == MIP_ALERT_F1 )	//Gesture Fail Reason
		{
			if( rbuf[1] == MIP_LPWG_EVENT_TYPE_FAIL )
			{
				switch( rbuf[2] )
				{
					case OUT_OF_AREA:
						TOUCH_LOG("LPWG FAIL REASON = Out of Area\n");
						break;
					case PALM_DETECTED:
						TOUCH_LOG("LPWG FAIL REASON = Palm\n");
						break;
					case DELAY_TIME:
						TOUCH_LOG("LPWG FAIL REASON = Delay Time\n");
						break;
					case TAP_TIME:
						TOUCH_LOG("LPWG FAIL REASON = Tap Time\n");
						break;
					case TAP_DISTACE:
						TOUCH_LOG("LPWG FAIL REASON = Tap Distance\n");
						break;
					case TOUCH_SLOPE:
						TOUCH_LOG("LPWG FAIL REASON = Touch Slope\n");
						break;
					case MULTI_TOUCH:
						TOUCH_LOG("LPWG FAIL REASON = Multi Touch\n");
						break;
					case LONG_PRESS:
						TOUCH_LOG("LPWG FAIL REASON = Long Press\n");
						break;
					default:
						TOUCH_LOG("LPWG FAIL REASON = Unknown Reason\n");
						break;
				}
			}
			else
			{
				//Re-enter tap mode
				wbuf[0] = MIP_R0_CTRL;
				wbuf[1] = MIP_R1_CTRL_POWER_STATE;
				wbuf[2] = MIP_CTRL_POWER_LOW;
				if( Mit200_proc4_I2C_Write(client, wbuf, 3) )
				{
					TOUCH_ERR("mip_i2c_write failed\n");
					return TOUCH_FAIL;
				}
			}
		}
		else
		{
			TOUCH_LOG("Unknown alert type [%d]\n", alert_type);
		}
	}

	return TOUCH_SUCCESS;

}

static int MIT200_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	u8 wbuf[2] = {0, };
	u8 version[2] = {0, };
	int ret = 0;

	TOUCH_FUNC();

	/* IMPLEMENT : read IC firmware information function */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_CUSTOM;

	ret = Mit200_proc4_I2C_Read(client, wbuf, 2, version, 2);
	if( ret == TOUCH_FAIL )
		return TOUCH_FAIL;

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = version[1];
	pFwInfo->version = version[0];

	TOUCH_LOG("IC F/W Version = v%X.%02X ( %s )\n", version[1], version[0], pFwInfo->isOfficial ? "Official Release" : "Test Release");

	return TOUCH_SUCCESS;
}

static int MIT200_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 version[2] = {0, };
	u8 *pFwFilename = NULL;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
#if defined (CONFIG_MACH_MSM8909_M1DS_GLOBAL_COM)
        if (lge_get_board_revno() > HW_REV_A) {
		    pFwFilename = (char *)defaultFirmware;
        } else {
            pFwFilename = (char *)defaultFirmwareRevA;
        }
#else
        pFwFilename = (char *)defaultFirmware;
#endif
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	mip_bin_fw_version(ts, fw->data, fw->size, version);

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = version[0] ;
	pFwInfo->version = version[1];

	/* Free firmware image buffer */
	release_firmware(fw);

	TOUCH_LOG("BIN F/W Version = v%X.%02X ( %s )\n", version[0], version[1], pFwInfo->isOfficial ? "Official Release" : "Test Release");

	return TOUCH_SUCCESS;		
}

static int MIT200_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
	int ret = 0;
	u8 *pFwFilename = NULL;
	const struct firmware *fw = NULL;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
        #if defined (CONFIG_MACH_MSM8909_M1DS_GLOBAL_COM)
        if (lge_get_board_revno() > HW_REV_A) {
		    pFwFilename = (char *)defaultFirmware;
        } else {
             pFwFilename = (char *)defaultFirmwareRevA;
        }
        #else
        pFwFilename = (char *)defaultFirmware;
        #endif
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	ret = mip_flash_fw(ts, fw->data, fw->size, false, true);
	if( ret < fw_err_none ) {
		return TOUCH_FAIL;
	}

	release_firmware(fw);

	return TOUCH_SUCCESS;
}

static int MIT200_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;

	TOUCH_FUNC();

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LpwgSetting));

	if( ts->currState == newState ) {
		TOUCH_LOG("device state is same as driver requested\n");
		return TOUCH_SUCCESS;
	}

	if( ( newState < STATE_NORMAL ) && ( newState > STATE_KNOCK_ON_CODE ) ) {
		TOUCH_LOG("invalid request state ( state = %d )\n", newState);
		return TOUCH_FAIL;
	}

	ret = lpwg_control(client, newState);
	if( ret == TOUCH_FAIL ) {
		TOUCH_ERR("failed to set lpwg mode in device\n");
		return TOUCH_FAIL;
	}

	if( ret == TOUCH_SUCCESS ) {
		ts->currState = newState;
	}

	switch( newState )
	{
		case STATE_NORMAL:
			TOUCH_LOG("device was set to NORMAL\n");
			break;
		case STATE_OFF:
			TOUCH_LOG("device was set to OFF\n");
			break;
		case STATE_KNOCK_ON_ONLY:
			TOUCH_LOG("device was set to KNOCK_ON_ONLY\n");
			break;
		case STATE_KNOCK_ON_CODE:
			TOUCH_LOG("device was set to KNOCK_ON_CODE\n");
			break;
		default:
			TOUCH_LOG("impossilbe state ( state = %d )\n", newState);
			ret = TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;
}

static int MIT200_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	//int dataLen = 0;
	char *sd_path = "/sdcard/touch_self_test.txt";
	int ret = 0;
	//int deltaStatus = 0;
	int jitterStatus = 0;
	int openStatus = 0;
	int short1Status = 0;
	int short2Status = 0;

	memset(pBuf, 0, bufSize);
	*pDataLen = 0;

	TOUCH_FUNC();
	/* CAUTION : be careful not to exceed buffer size */

	//Result Varibles
	*pRawStatus = TOUCH_SUCCESS;
	*pChannelStatus = TOUCH_SUCCESS;

	/* IMPLEMENT : self-diagnosis function */
	// write date
	MIT200_WriteFile(sd_path, pBuf, 1);
	msleep(30);

	// raw data check
	//ret = MIT200_GetTestResult(client, pBuf, pRawStatus, RAW_DATA_SHOW);
	ret = MIT200_GetTestResult(client, pBuf, pRawStatus, RAW_DATA_SHOW);
	if( ret < 0 ) {
		TOUCH_ERR("failed to get raw data\n");
		memset(pBuf, 0, bufSize);
		*pRawStatus = TOUCH_FAIL;
		goto error;
	}
	MIT200_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// open data check
	ret = MIT200_GetTestResult(client, pBuf, &openStatus, OPEN_SHOW);
	if( ret < 0 ) {
		TOUCH_ERR("failed to get raw data\n");
		memset(pBuf, 0, bufSize);
		openStatus = TOUCH_FAIL;
		goto error;
	}
	MIT200_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// open short check
	ret = MIT200_GetTestResult(client, pBuf, &short1Status, OPENSHORT_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get open short data\n");
		memset(pBuf, 0, bufSize);
		short1Status = TOUCH_FAIL;
		goto error;
	}
	MIT200_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// open short 2 check (MUX)
	ret = MIT200_GetTestResult(client, pBuf, &short2Status, OPENSHORT2_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get open short (mux) data\n");
		memset(pBuf, 0, bufSize);
		short2Status = TOUCH_FAIL;
		goto error;
	}
	MIT200_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

/*
	// cm_delta check
	ret = MIT200_GetTestResult(client, pBuf, &deltaStatus, DELTA_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get intensity data\n");
		memset(pBuf, 0, bufSize);
		deltaStatus = TOUCH_FAIL;
		goto error;
	}
	MIT200_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);
*/
	// cm_jitter chec
	ret = MIT200_GetTestResult(client, pBuf, &jitterStatus, JITTER_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get intensity data\n");
		memset(pBuf, 0, bufSize);
		jitterStatus = TOUCH_FAIL;
		goto error;
	}
	MIT200_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	if(openStatus==TOUCH_SUCCESS && short1Status==TOUCH_SUCCESS && short2Status==TOUCH_SUCCESS){
		*pChannelStatus = TOUCH_SUCCESS;
	}else{
		*pChannelStatus = TOUCH_FAIL;
	}
	ret = sprintf(pBuf, "%s", "======ADDITIONAL======\n");
	ret += sprintf(pBuf+ret, "Open    Test: %s", (openStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	ret += sprintf(pBuf+ret, "Short1  Test: %s", (short1Status==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	ret += sprintf(pBuf+ret, "Short2  Test: %s", (short2Status==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	ret += sprintf(pBuf+ret, "RawData Test: %s", (*pRawStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	/* Delta and Jitter test is not supported in M1 models
	ret += sprintf(pBuf+ret, "Delta Test: %s", (deltaStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	*/
	ret += sprintf(pBuf+ret, "Jitter Test: %s", (jitterStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	MIT200_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);
	*pDataLen = ret;

	return TOUCH_SUCCESS;
error :
	TOUCH_LOG("[%s] error occured\n",__func__);
	return TOUCH_FAIL;	
}

static int MIT200_AccessRegister(struct i2c_client *client, int cmd, int reg1, int reg2, int *pValue)
{
	int ret = 0;
	u8 wbuf[3] = {0, };
	u8 result[2] = {0, };

	wbuf[0] = reg1;
	wbuf[1] = reg2;
	wbuf[2] = *pValue;

	switch( cmd )
	{
		case READ_IC_REG:
			ret = Mit200_proc4_I2C_Read(client, wbuf, 2, result, 2);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			*pValue = result[0];
			break;
		case WRITE_IC_REG:
			Mit200_proc4_I2C_Write(client, wbuf, 3);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;
		default:
			TOUCH_ERR("Invalid access command ( cmd = %d )\n", cmd);
			return TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;

}

void lpwg_debug_enable_set_status(struct i2c_client* client, int value)
{
	int ret = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	if(value == 0){
		pDriverData->lpwg_debug_enable = 0;
	}else{
		pDriverData->lpwg_debug_enable = 1;
	}
	ret = mip_lpwg_debug_enable(client);
	TOUCH_LOG("[%s] lpwg_debug_enable status = %d\n",__func__,pDriverData->lpwg_debug_enable);
}

static void MIT200_lpwg_debug_enable_set_status(struct i2c_client* client, int value)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	if(value == 0){
		pDriverData->lpwg_debug_enable = 0;
	}else{
		pDriverData->lpwg_debug_enable = 1;
	}
	mip_lpwg_debug_enable(client);
	TOUCH_LOG("[%s] lpwg_debug_enable status = %d\n",__func__,pDriverData->lpwg_debug_enable);
}

#if defined(SENSOR_DETECT)
void MIT200_mcu_sleep_control(struct i2c_client *client)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	//gp93 high in normal
	if(pDriverData->sensor_value){//sensor near
		//gpio_direction_output(TOUCH_GPIO_CONTROL, 0);//gp93 set to row
        TouchSleepCtrl(0);
		TOUCH_LOG("gp93 set to row, sensor near\n");
	}else{//sensor far
		//gpio_direction_output(TOUCH_GPIO_CONTROL, 1);//gp93 set to high
        TouchSleepCtrl(1);
		TOUCH_LOG("gp93 set to high, sensor far\n");
		TouchResetCtrl(0);
		msleep(10);
		TouchResetCtrl(1);
		msleep(20);
	}
	//TOUCH_LOG("MCU Sleep control onoff : %d\n",pDriverData->sensor_value);
}
#endif

TouchDeviceSpecificFunction MIT200_Func = {

	.Initialize 			= MIT200_Initialize,
	.Reset 					= MIT200_Reset,
	.Connect 				= MIT200_Connect,
	.InitRegister 			= MIT200_InitRegister,
	.InterruptHandler 		= MIT200_InterruptHandler,
	.ReadIcFirmwareInfo 	= MIT200_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo 	= MIT200_GetBinFirmwareInfo,
	.UpdateFirmware 		= MIT200_UpdateFirmware,
	.SetLpwgMode 			= MIT200_SetLpwgMode,
	.DoSelfDiagnosis 		= MIT200_DoSelfDiagnosis,
	.AccessRegister 		= MIT200_AccessRegister,
	.device_attribute_list 	= MIT200_attribute_list,
	.GetProduct_id 			= MIT200_show_product_id,
	.Getraw_data_result		= MIT200_lpwg_raw_data_show,
	.SetLpwwgDebug_enable	= MIT200_lpwg_debug_enable_set_status,
#if defined(SENSOR_DETECT)
	.mcu_sleep_control 	= MIT200_mcu_sleep_control,
#endif
};


/* End Of File */


