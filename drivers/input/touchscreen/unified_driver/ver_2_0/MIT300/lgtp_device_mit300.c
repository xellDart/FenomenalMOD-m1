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
#define LGTP_MODULE "[MIT300]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_device_mit300.h>


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
#if defined( TOUCH_MODEL_P1C )
static const char defaultFirmware[] = "melfas/mit300/p1c/melfas_mip4.bin";
#elif defined ( TOUCH_MODEL_M209 )
static const char defaultFirmware[] = "melfas/mit300/m209_trf_us/melfas_mip4.img";
#elif defined ( TOUCH_MODEL_C90NAS )
static const char defaultFirmware[] = "melfas/mit300/p1c/melfas_mip4.bin";
#endif

static struct melfas_ts_data *ts = NULL;

#if defined(ENABLE_SWIPE_MODE)
static int get_swipe_mode = 1;
/*
static int wakeup_by_swipe = 0;
*/
extern int lockscreen_stat;
#endif


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
static void MIT300_WriteFile(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "\n%02d-%02d %02d:%02d:%02d.%03lu \n\n\n",
		my_date.tm_mon + 1,my_date.tm_mday,
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
		(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (fd >= 0) {
		if (time > 0)
			sys_write(fd, time_string, strlen(time_string));
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);
}


/****************************************************************************
* Device Specific Functions
****************************************************************************/


static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	TOUCH_FUNC();

	switch( newState )
	{
		case STATE_NORMAL:
			break;

		case STATE_KNOCK_ON_ONLY:
			break;

		case STATE_KNOCK_ON_CODE:
			break;

		case STATE_OFF:
			break;

		default:
			TOUCH_ERR("invalid touch state ( %d )\n", newState);
			break;
	}

	return TOUCH_SUCCESS;
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

#if defined(ENABLE_SWIPE_MODE)
static LGE_TOUCH_ATTR(swipe_mode, S_IRUGO | S_IWUSR, show_swipe_mode, store_swipe_mode);
#endif

static struct attribute *MIT300_attribute_list[] = {
#if defined(ENABLE_SWIPE_MODE)
	&lge_touch_attr_swipe_mode.attr,
#endif
	NULL,
};

static int MIT300_Initialize(struct i2c_client *client)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR("failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->client = client;

	return TOUCH_SUCCESS;
}

static void MIT300_Reset(struct i2c_client *client)
{
	TouchResetCtrl(0);
	msleep(10);
	TouchResetCtrl(1);
	msleep(100);

	TOUCH_LOG("Device was reset\n");
}

static int MIT300_Connect(void)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Device detection function */

	return TOUCH_SUCCESS;
}

static int MIT300_InitRegister(struct i2c_client *client)
{
	/* IMPLEMENT : Register initialization after reset */

	return TOUCH_SUCCESS;
}

static int MIT300_InterruptHandler(struct i2c_client *client, TouchReadData *pData)
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

	//Read packet info
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if( Mit300_I2C_Read(client, wbuf, 2, rbuf, 1) )
	{
		TOUCH_ERR("Read packet info\n");
		return TOUCH_FAIL;
	}

	packet_size = (rbuf[0] & 0x7F);
	packet_type = ((rbuf[0] >> 7) & 0x1);

	//Read packet data
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
	if( Mit300_I2C_Read(client, wbuf, 2, rbuf, packet_size) )
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

			if( index < 0 || index > MAX_NUM_OF_FINGERS )
			{
				TOUCH_ERR("invalid touch index (%d)\n", index);
				return TOUCH_FAIL;
			}

			pData->type = DATA_FINGER;

			if( (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4 )
			{
				if( state )
					TOUCH_LOG("Palm detected : %d \n", tmp[5]);
				else
					TOUCH_LOG("Palm released : %d \n", tmp[5]);

				return TOUCH_SUCCESS;
			}

			if( state )
			{
				pFingerData = &pData->fingerData[pData->count];
				pFingerData->id = index ;
				pFingerData->x = tmp[2] | ((tmp[1] & 0x0f) << 8);
				pFingerData->y = tmp[3] | ((tmp[1] & 0xf0) << 4);
				pFingerData->width_major = tmp[5];
				pFingerData->width_minor = 0;
				pFingerData->orientation = 0;
				pFingerData->pressure = tmp[4];

				pData->count++;
			}
		}
	}
	else	/* Alert event */
	{
		alert_type = rbuf[0];

		if( alert_type == MIP_ALERT_ESD )	//ESD detection
		{
			TOUCH_LOG("ESD Detected!\n");
		}
		else if( alert_type == MIP_ALERT_WAKEUP )	//Wake-up gesture
		{
			if( rbuf[1] == MIP_EVENT_GESTURE_DOUBLE_TAP )
			{
				TOUCH_LOG( "Knock-on Detected\n" );
				pData->type = DATA_KNOCK_ON;
			}
			else if( rbuf[1] == MIP_EVENT_GESTURE_MULTI_TAP )
			{
				TOUCH_LOG( "Knock-code Detected\n" );
				pData->type = DATA_KNOCK_CODE;
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
		}
		else
		{
			TOUCH_LOG("Unknown alert type [%d]\n", alert_type);
		}
	}

	return TOUCH_SUCCESS;

}

static int MIT300_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	u8 wbuf[2] = {0, };
	u8 version[2] = {0, };
	int ret = 0;

	TOUCH_FUNC();

	/* IMPLEMENT : read IC firmware information function */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_CUSTOM;

	ret = Mit300_I2C_Read(client, wbuf, 2, version, 2);
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

static int MIT300_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 version[2] = {0, };
	u8 *pFwFilename = NULL;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
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

static int MIT300_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
	int ret = 0;
	char *pFwFilename = NULL;
	const struct firmware *fw = NULL;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
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

static int MIT300_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
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

static int MIT300_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int dataLen = 0;
	char *sd_path = "/sdcard/touch_self_test.txt";
	int ret = 0;
	TOUCH_FUNC();
	/* CAUTION : be careful not to exceed buffer size */

	/* IMPLEMENT : self-diagnosis function */

	*pRawStatus = TOUCH_SUCCESS;
	*pChannelStatus = TOUCH_SUCCESS;

	MIT300_WriteFile(sd_path, pBuf, 1);
	msleep(30);

	ret = MIT300_GetTestResult(client, pBuf, pRawStatus, RAW_DATA_SHOW);
		if( ret < 0 ) {
			TOUCH_ERR("failed to get raw data\n");
			memset(pBuf, 0, PAGE_SIZE);
			pRawStatus = 0;
			ret = snprintf(pBuf, PAGE_SIZE, "failed to get raw data\n\n");
	}
	/*pChannelStatus does not supported yet. */

/*
	dataLen += sprintf(pBuf, "%s", "========= Additional Information =========\n");
	dataLen += sprintf(pBuf+dataLen, "%s", "Device Name = Dummy\n");
*/
	*pDataLen = dataLen;

	return TOUCH_SUCCESS;	
}

static int MIT300_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;

	switch( cmd )
	{
		case READ_IC_REG:
			ret = Touch_I2C_Read_Byte(client, (u8)reg, (u8 *)pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		case WRITE_IC_REG:
			ret = Touch_I2C_Write_Byte(client, (u8)reg, (u8)*pValue);
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

TouchDeviceSpecificFunction MIT300_Func = {

	.Initialize 			= MIT300_Initialize,
	.Reset 					= MIT300_Reset,
	.Connect 				= MIT300_Connect,
	.InitRegister 			= MIT300_InitRegister,
	.InterruptHandler 		= MIT300_InterruptHandler,
	.ReadIcFirmwareInfo 	= MIT300_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo 	= MIT300_GetBinFirmwareInfo,
	.UpdateFirmware 		= MIT300_UpdateFirmware,
	.SetLpwgMode 			= MIT300_SetLpwgMode,
	.DoSelfDiagnosis 		= MIT300_DoSelfDiagnosis,
	.AccessRegister 		= MIT300_AccessRegister,
	.device_attribute_list 	= MIT300_attribute_list,

};


/* End Of File */


