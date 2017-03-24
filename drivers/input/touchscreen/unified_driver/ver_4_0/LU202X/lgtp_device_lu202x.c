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
 *    File  	: lgtp_device_lu202x.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[LU202X]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include <linux/input/unified_driver_4/lgtp_device_lu202x.h>

#include <linux/input/unified_driver_4/lgtp_common.h>
#include <linux/input/unified_driver_4/lgtp_common_driver.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_i2c.h>
#include <linux/input/unified_driver_4/lgtp_model_config_i2c.h>
#include <linux/input/unified_driver_4/lgtp_model_config_misc.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_misc.h>

#include <TestLimits_lu202x.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define TPD_I2C_ADDRESS				0x0E
#define I2C_DEVICE_ADDRESS_LEN		2
#define MAX_TRANSACTION_LENGTH		8
#define MAX_I2C_TRANSFER_SIZE		(MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)

#define LU202X_MAX_KEY 4
#define MAX_FINGER_NUM 2
#define MAX_CHANNEL 36

/* LeadingUI Firmware */
#define FW_SIZE 		        30*1024
#define CFG_SIZE 		        1*1024
#define FAC_SIZE		        1*1024

#define FAC_POS			        0xFC00
#define FW_POS			        0x8000

/* Firmware File Path */
#define FW_FILE_PATH "/mnt/sdcard/touch_fw_dump.img"

/* SD File Path */
#define SD_FILE_PATH "/mnt/sdcard/touch_self_test.txt"

/* Knock On/Code */
#define KNOCK_ON_STATUS			0x0082
#define KNOCK_TAP_COUNT			0x0083
#define KNOCK_STATUS			0x00C0
#define KNOCK_TAP_THON			0x00C1
#define KNOCK_EXCEPT_PALM_ONCH	0x00C5
#define KNOCK_WAKEUP_INTERVAL	0x00C9
#define KNOCK_TAPOFF_TIMEOUT	0x00D2
#define KNOCK_ON_TAP_COUNT		0x00D4
#define KNOCK_ON_REPORT_DELAY   0x00D5

#define KNOCK_CODE_TAPOFF_TIMEOUT	0x00DB
#define KNOCK_CODE_TAP_COUNT		0x00DD

#define MAX_KNOCK_CODE_POS		12
#define KNOCK_CODE_DATA			4

/* Touch Event Type */
#define TYPE_PRESS			0x01
#define TYPE_MOVE			0x02
#define TYPE_RELEASE		0x03

/* Key Event Type */
#define KEY_PRESSED			1
#define KEY_RELEASED		0
#define CANCEL_KEY			0xFF

#define SCREEN_MAX_X    	1280
#define SCREEN_MAX_Y    	800
#define PRESS_MAX       	255

#define EVENT_NONE			0x00
#define EVENT_ABS			0x01
#define EVENT_KEY	    	0x02
/*TO DO - NSM
EVENT_LPWG Saperate EVENT_KNOCK / EVENT_KNOCK_ONCODE*/
#define EVENT_KNOCK_ON          0x03
#define EVENT_KNOCK_CODE        0x04
#define EVENT_KNOCK_OVER        0x05
#define EVENT_PALM		0x06

#define FWSTATUS_NORMAL		    0x00
#define FWSTATUS_INITREQ	    0xFF
#define FWSTATUS_CHFAIL	    	0xfe
#define FWSTATUS_CALFAIL	    0xfd

#define I2C_DEVICE_ADDRESS_LEN	2
#define MAX_TRANSACTION_LENGTH	8

#define FW_STATUS               0x0000
#define INT_INFORM              0x0001
#define TOUCH_VALID             0x0002
#define TOUCH_KEY               0x0003
#define TOUCH_FINGER            0x0005

#define FW_VERSION_REG		    0x0080

#define LU202x_MODE_ADDR		0x00E0
#define LU202x_CMDACK_ADDR		0x00ED

#define LU202x_DEVICEID_ADDR	0x10FD
#define LU202x_I2CDONE_ADDR		0x10FF
#define LU202x_CMDReply_ADDR	0x0100
#define LU202x_RAWCAP_ADDR LU202x_CMDReply_ADDR+2
#define LU202x_JITTERCAP_ADDR LU202x_RAWCAP_ADDR+(MAX_CHANNEL * 2)
#define LU202x_AUTOCYCLE_ADDR LU202x_JITTERCAP_ADDR+(MAX_CHANNEL * 2)

#define CMD_I2C_DONE		    0x01
#define CMD_LU202x_CHANGEMODE	0xA3
#define CMD_LU202x_DEVINFO		0xAA
#define CMD_LU202x_CHCAPTEST	0xB6
#define LU202x_CHCAPTEST_Reply	0xC1

/* Command */
#define LU202x_CMD_FW_CHECKSUM_ADDR	0x0158

/* Major Mode */
#define CMD_LU202x_NORMODE		0x00
#define CMD_LU202x_PDN			0x01
#define CMD_LU202x_DEBUG		0x02
#define CMD_LU202x_KNOCK_ON_ONLY	0x11
#define CMD_LU202x_KNOCK_ON_CODE	0x11
#define CMD_LU202x_DIAL_MODE		0x13

/* Minor Mode */
#define CMD_LU202x_NONE		0x0000
#define CMD_LU202x_SUMHISTO	0x0002

/* USERMODE CHANGE */
#define ACCESS_CTRL         0x1000
#define USER_SPACE          0x1001
#define USER_PASSWORD       0x1003
#define EFLSH_CTRL			0x1000
#define TSP_INFORM          0x0038

/* Wakeup mode*/
#define TOGGLEWAKEUP	0x01
#define WAKEUP			0x00
/****************************************************************************
 * Macros
 ****************************************************************************/
#define COMB16(x, i) *((u16 *)x + i)

/****************************************************************************
* Type Definitions
****************************************************************************/
typedef struct Lu202xDriverDataTag {

	TouchState deviceState;

} Lu202xDriverData;

/****************************************************************************
* Variables
****************************************************************************/
typedef struct {
	u8 FWStatus;		// 0x0000
	u8 EventType;		// 0x0001
	u8 VPCount; 		// 0x0002
	u8 KeyData[2];		// 0x0003
	u8 Point[8];		// 0x0005 X0 position
} lu202x_tpd;

typedef struct {
	u8 status[MAX_FINGER_NUM];
	u8 id[MAX_FINGER_NUM];
	u16 x[MAX_FINGER_NUM];
	u16 y[MAX_FINGER_NUM];
} touch_info;

static int pressed_key = 0;
int palm_state;

static TouchDriverData *gDriverData = NULL;
static Lu202xDriverData gDeviceData = { STATE_NORMAL };
static const char defaultFirmware[] = "leadingUI/E1_GF_OFFICIAL_01_07_151121.img";

u32 touch_debug_mask = BASE_INFO;
/* Debug mask value
 * usage: echo [debug_mask] > /sys/module/lgtp_device_lu202x/parameters/debug_mask
 */
module_param_named(debug_mask, touch_debug_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static int Lu202X_command_send(u8 comm, u16 mode, u16 sub_mode, u8 *value);
void Lu202x_file_write( char *fname, char *data, int size );

/****************************************************************************
* Local Functions
****************************************************************************/
static int SleepInLu202x( void )
{
	u8 i2c_done = CMD_I2C_DONE;
	Lu20xx_I2C_Write(LU202x_I2CDONE_ADDR, &i2c_done, 1);

	return TOUCH_SUCCESS;
}

static int WakeUpLu202x( int toggle )
{
	int result = TOUCH_SUCCESS;
	int loopCount = 1000;

	if(toggle ==TOGGLEWAKEUP){
		TouchToggleGpioInterrupt();
	}
	while (TouchReadGpioInterrupt() && loopCount) {
		/* don't need a context switching */
		//msleep ( 1 );
		mdelay(1);
		loopCount--;
	}

	if (TouchReadGpioInterrupt() == 1) {
		LU202X_ERR("Failed to wakeup Lu202x\n");
		result = TOUCH_FAIL;
	}

	return result;
}

void Lu202x_Reset(void)
{
	/* Didn't reset during calling state. */
	if (atomic_read(&gDriverData->incoming_call) == INCOMING_CALL_IDLE) {
		TouchSetGpioReset(0);
		msleep(10);
		TouchSetGpioReset(1);
		msleep(300);
	}
	gDeviceData.deviceState = STATE_NORMAL;
}

static int LU202x_FlashReadyCheck(void)
{
	u8 status = 0;
	u8 rty_cnt = 100;

	LU202X_FUNC(FW_UPGRADE);
	/* minimum check time 100ms */
	while (rty_cnt-- > 0) {
		/* Flash Ready Check */
		if (Lu20xx_I2C_Read(EFLSH_CTRL, &status, 1) == TOUCH_FAIL) {
			LU202X_ERR("Read status operation failed\n" );
			return TOUCH_FAIL;
		}

		if ((status & 0x40) == 0x40) {
			break;
		}
		// 1ms check.
		msleep(1);
	}
	msleep(30);
	
	return TOUCH_SUCCESS;
}

static int LU202x_PageErase(int addr)
{
	u8 Cmd[2] = {0, 0};
	u8 pagenum = addr/1024;
	LU202X_FUNC(FW_UPGRADE);

	/* Erase */
	/* main Block Select */
	Cmd[0] = 0x02;
	if (Lu20xx_I2C_Write(0x1001, Cmd, 1) == TOUCH_FAIL)  {
		LU202X_ERR("Main Block Select command operation failed\n" );
		goto ERASE_FAIL;
	}

	/* Erase PageNum Write */
	Cmd[0] = pagenum;
	if (Lu20xx_I2C_Write(0x1006, Cmd, 1) == TOUCH_FAIL) {
		LU202X_ERR(" Erage Page Write command operation failed(page:%d)\n", pagenum);
		goto ERASE_FAIL;
	}

	/* Erase Function SelectEra */
	Cmd[0] = 0x82;
	if (Lu20xx_I2C_Write(EFLSH_CTRL, Cmd, 1) == TOUCH_FAIL) {
		LU202X_ERR("Page Erase Function Select command operation failed\n" );
		goto ERASE_FAIL;
	}

	if (LU202x_FlashReadyCheck() == TOUCH_FAIL) {
		LU202X_ERR("Flash Ready failed\n" );
		goto ERASE_FAIL;
	}

	return TOUCH_SUCCESS;

ERASE_FAIL:
	return TOUCH_FAIL;
}

static int LU202x_PageWrite(u8 *pBuf, int addr, int size)
{
	u8 Cmd[2] = {0, 0};
	u8 i = 0;
	LU202X_FUNC(FW_UPGRADE);

	/* I2C E-Flash Program Enable (Program Enable Fuction Select) */
	Cmd[0] = 0x88;
	if (Lu20xx_I2C_Write(EFLSH_CTRL, Cmd, 1) == TOUCH_FAIL) {
		LU202X_ERR("Program Fuction Select operation failed\n" );
		return TOUCH_FAIL;
	}

	for (i = 0; i < 2; i++) {
		/* Data Write */
		if (Lu20xx_I2C_Write(addr, pBuf, size) == TOUCH_FAIL) {
			LU202X_ERR("Data Write operation failed(addr:%d)\n", addr);
		} else {
			return TOUCH_SUCCESS;
		}
	}
	return TOUCH_FAIL;
}

static int LU202x_PageRead(u8 *pBuf, int addr, int size)
{
	u8 Cmd[2] = {0, 0};

	/* Read Function Select */
	Cmd[0] = 0x81;
	if (Lu20xx_I2C_Write(EFLSH_CTRL, Cmd, 1) == TOUCH_FAIL) {
		LU202X_ERR("Read Function operation failed\n" );
		return TOUCH_FAIL;
	}

	/* Data Read */
	if (Lu20xx_I2C_Read(addr, pBuf, size) == TOUCH_FAIL) {
		LU202X_ERR("Data Read operation failed\n" );
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int Lu202x_programFW(u8 *pBuf, int addr, int size)
{
	int i = 0;
	int W_addr = addr;
	int R_addr = addr;
	LU202X_FUNC(FW_UPGRADE);

	/* Erase sequence. */
	for (i=0; i<size; i+= 1024, W_addr += 1024) {
		/* 1K Erase */
		if (LU202x_PageErase(W_addr) == TOUCH_FAIL) {
			LU202X_ERR("Data Page Erase failed \n");
			return TOUCH_FAIL;
		}
	}
	/* Write sequence. */
	W_addr = addr;
	for (i=0; i<size; i+= 256, W_addr += 256) {
		/* 256Bytes Write * 4 */
		if (LU202x_PageWrite((pBuf+i), W_addr , 256) == TOUCH_FAIL){
			LU202X_ERR("Data Write failed \n");
			return TOUCH_FAIL;
		}
	}

	LU202X_LOG(BASE_INFO, "%s Writing (%d/%d) bytes\n", \
			(size == FAC_SIZE) ? "FACTORY" : "FIRMWARE", i, size);

	/* Data Check */
	if (0) {
		for (i = 0; i < size+FAC_SIZE; i += 1024, R_addr += 1024) {
			LU202X_LOG(BASE_INFO, "R_ADDRESS = 0x%x", R_addr);
			if (LU202x_PageRead((pBuf + i), R_addr, 1024) == TOUCH_FAIL) {
				LU202X_ERR("Data Read for Check operation failed\n" );
				return TOUCH_FAIL;
			} else {
				LU202X_LOG(BASE_INFO, "DATA CHECK SUCCESS [%d] page", i);
			}
		}
	}

	return TOUCH_SUCCESS;
}

static void Lu202x_bin_checksum(const struct firmware *fw, u8 *bin_checksum)
{
	unsigned int fw_sum = 0;
	unsigned short param_sum = 0;
	unsigned int temp = 0;
	int i = 0;

	/* Read Binary Checksum - firmware */
	/* Read 30kbytes by 4bytes unit */
	/* 30k = 30 * 1024 / 4 */
	for (i = 0; i < (FW_SIZE / sizeof(fw_sum)); i++) {
		memcpy(&temp, (fw->data + i*sizeof(fw_sum)), sizeof(fw_sum));
		fw_sum += temp;
	}
	memcpy(bin_checksum, &fw_sum, sizeof(fw_sum));

	/* Read Binary Checksum - parameter */
	/* Read 1kbytes by 2bytes unit */
	/* [ Header 16Bytes | Firmware Data 30720Bytes | Not include checksum data 12Bytes | Parameter Data 1011Bytes | not use 1bytes]*/
	/* 30k = 1 * 1024 / 2 */
	//30720 + 12
	for (i = 0; i < 505; i++) {
		memcpy(&temp, (fw->data + i*sizeof(param_sum) + FW_SIZE + 13), sizeof(param_sum));
		param_sum += temp;
	}
	temp = *(fw->data + i*sizeof(param_sum) + FW_SIZE + 13);
	param_sum += temp;

	memcpy(bin_checksum + sizeof(fw_sum), &param_sum ,sizeof(param_sum));
}

/****************************************************************************
* Global Functions
****************************************************************************/
static ssize_t show_Model_Info(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "======== Model info ========\n");
	if (TouchReadGpioMakerId() == 0) {
		LU202X_LOG(BASE_INFO, "Touch IC : LeadingUI\n");
		ret += sprintf(buf+ret, "Maker ID PIN: 0\n");
		ret += sprintf(buf+ret, "Module Product : SUNTEL\n");
		ret += sprintf(buf+ret, "Touch IC : LeadingUI\n");
	} else if (TouchReadGpioMakerId() == 1) {
		ret += sprintf(buf+ret, "Maker ID PIN: 1\n");
		ret += sprintf(buf+ret, "Module Product : LGIT\n");
		ret += sprintf(buf+ret, "Touch IC : Focaltech\n");
	}
	return ret;
}
static LGE_TOUCH_ATTR(Model_Info, S_IRUGO | S_IWUSR, show_Model_Info, NULL);

static ssize_t show_fw_dump(TouchDriverData *pDriverData, char *buf)
{
	int ret=0;
	int i=0;
	u8 *pBuf = NULL;
	u8 *temp = NULL;
	u8 Cmd[2] = {0, 0};

	TouchDisableIrq();

	TOUCH_LOG("Firmware Dumping...\n");
	ret += sprintf(buf, "====Firmware dump===\n");

	/* Set Password for userspace read mode */
	Cmd[0] = 0x75;
	Cmd[1] = 0x6C;
	if(Lu20xx_I2C_Write(USER_PASSWORD, Cmd, 2) == TOUCH_FAIL) {
	LU202X_ERR("USER_PASSWORD Mode Change failed\n");
		goto fail;
	}

	/* Set Read Command */
	Cmd[1] = 0x81;
	if(Lu20xx_I2C_Write(ACCESS_CTRL, &Cmd[1], 1) == TOUCH_FAIL) {
	LU202X_ERR("ACCESS_CTRL Mode Change failed\n");
		goto fail;
	}

	pBuf = kzalloc(FW_SIZE+CFG_SIZE, GFP_KERNEL);
	temp = kzalloc(sizeof(u8)*256, GFP_KERNEL);

	/* Read IC Firmware data */
	for(i = 0; i < FW_SIZE + CFG_SIZE; i += 256) {
		if (Lu20xx_I2C_Read(FW_POS+i, temp, 256) == TOUCH_FAIL) {
			LU202X_ERR("Read F/W failed\n");
				goto fail;
		}
		memcpy(&pBuf[i], temp, 256);
		TOUCH_LOG("Dump %d / %d byte\n", i+256, FW_SIZE+CFG_SIZE);
	}

	Lu202x_file_write(FW_FILE_PATH, pBuf, FW_SIZE+CFG_SIZE);
	kfree(pBuf);
	kfree(temp);

	TOUCH_LOG("Dumping is done\n");

	TouchEnableIrq();
	Lu202x_Reset();

	return ret;

fail:
	SleepInLu202x();
	return TOUCH_FAIL;
}
static LGE_TOUCH_ATTR(fw_dump, S_IRUGO | S_IWUSR, show_fw_dump, NULL);

static ssize_t store_i2c_control(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	int cmd = 0;
	u16 reg = 0;
	u8 pValue[2] = {0,};
	int ret = 0;

	sscanf(buf, "%d %x %d", &cmd, (u32 *)&reg, (int *)pValue);
	LU202X_LOG(BASE_INFO, "%s reg(0x%02X) value(%d)\n",
			(cmd == READ_IC_REG) ? "Read" : "Write",
			reg, pValue[0] | (pValue[1] << 8));

	switch (cmd) {
		case READ_IC_REG:
			ret = Lu20xx_I2C_Read((u16)reg, (u8 *)pValue, 2);
			if (ret == TOUCH_FAIL) {
				goto i2c_fail;
			}
			break;

		case WRITE_IC_REG:
			ret = Lu20xx_I2C_Write((u16)reg, (u8 *)pValue, 2);
			if (ret == TOUCH_FAIL) {
				goto i2c_fail;
			}
			break;

		default:
			LU202X_ERR("Invalid access command ( cmd = %d )\n", cmd);
			return TOUCH_FAIL;
			break;
	}
	SleepInLu202x();

	LU202X_LOG(BASE_INFO, "%s Success(%d %d)",
			(cmd == READ_IC_REG) ? "Read" : "Write", pValue[0], pValue[1]);

	return count;
i2c_fail:
	LU202X_ERR("i2c fail ( cmd = %d )\n", cmd);
	return count;
}
static LGE_TOUCH_ATTR(i2c_control, S_IRUGO | S_IWUSR, NULL, store_i2c_control);

static ssize_t show_rawdata(TouchDriverData *pDriverData, char *buf)
{
	u8 max_raw_data[MAX_CHANNEL * 4 + 4] = {0, };
	u8 Raw_Cap_Value[MAX_CHANNEL * 2] = {0, };
	u8 Jitter_Value[MAX_CHANNEL * 2] = {0, };
	int dataLen = 0;
	int ret = 0;
	int retry_cnt = 3;
	u8 i = 0;

	TouchDisableIrq();
	/* Read Raw Cap value(Read data size = MAX_CHANNEL * 2, channel number 36) */
	do {
		ret = Lu202X_command_send(CMD_LU202x_CHCAPTEST, LU202x_RAWCAP_ADDR,
				sizeof(max_raw_data), max_raw_data);
		if (ret == TOUCH_SUCCESS) {
			memcpy(Raw_Cap_Value, max_raw_data, sizeof(Raw_Cap_Value));
			memcpy(Jitter_Value, &max_raw_data[sizeof(Raw_Cap_Value)], sizeof(Jitter_Value));
			break;
		} else {
			LU202X_ERR("Read Raw Cap value fail %d\n", retry_cnt);
			if (retry_cnt == 1) {
				goto fail;
			}
		}
	} while (retry_cnt-- > 0);

	for (i = 0; i < MAX_CHANNEL; i++) {
		LU202X_LOG(BASE_INFO, "Raw cap value [%d] = %d , Jitter value [%d] = %d\n", \
			i, COMB16(Raw_Cap_Value, i), i, COMB16(Jitter_Value, i));
		dataLen += sprintf(buf + dataLen, "Raw cap value [%d] = %d , Jitter value [%d] = %d\n", \
			i, COMB16(Raw_Cap_Value, i), i, COMB16(Jitter_Value, i));
	}
	TouchEnableIrq();

	return dataLen;
fail:
	return TOUCH_FAIL;
}
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUSR, show_rawdata, NULL);

static ssize_t show_intensity(TouchDriverData *pDriverData, char *buf)
{
	u8 Raw_Cap_Value[MAX_CHANNEL * 2] = {0, };
	u8 i = 0;
	int dataLen = 0;
	int ret = 0;
	int retry_cnt = 3;

	/* Read Raw Cap value(Read data size = MAX_CHANNEL * 2, channel number 36) */
	TouchDisableIrq();
	do {
		ret = Lu202X_command_send(CMD_LU202x_CHANGEMODE, CMD_LU202x_DEBUG,
				CMD_LU202x_SUMHISTO, Raw_Cap_Value);
		if (ret == TOUCH_SUCCESS) {
			break;
		} else {
			LU202X_ERR("Read Raw Cap value fail %d\n", retry_cnt);
			if (retry_cnt == 1) {
				goto fail;
			}
		}
	} while (retry_cnt-- > 0);

	for (i = 0; i < MAX_CHANNEL / 2; i++) {
		LU202X_LOG(BASE_INFO, "Raw cap value [%d] = %d \n",
				i, COMB16(Raw_Cap_Value, i));
		dataLen += sprintf(buf + dataLen, "Raw cap value [%d] = %d \n",
				i, COMB16(Raw_Cap_Value, i));
	}
	TouchEnableIrq();
	//Lu202x_Reset();

	return dataLen;
fail:
	return TOUCH_FAIL;
}
static LGE_TOUCH_ATTR(intensity, S_IRUGO | S_IWUSR, show_intensity, NULL);

static struct attribute *lu202x_attribute_list[] = {
	&lge_touch_attr_Model_Info.attr,
	&lge_touch_attr_fw_dump.attr,
	&lge_touch_attr_i2c_control.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_intensity.attr,
	NULL,
};

int Lu202x_Initialize(TouchDriverData *pDriverData)
{
	LU202X_FUNC(BASE_INFO);
	gDriverData = pDriverData;
	Lu202x_Reset();
	return TOUCH_SUCCESS;
}

int Lu202x_InitRegister(void)
{
	LU202X_FUNC(BASE_INFO);

	return TOUCH_SUCCESS;
}

/* LGE_BSP_COMMON : branden.you@lge.com_20141106 : */
static int get_lpwg_data(TouchReadData *pData)
{
	u8 i = 0;
	u8 tap_count = 0;
	u8 buffer[MAX_KNOCK_CODE_POS * KNOCK_CODE_DATA] = {0,};

	if (Lu20xx_I2C_Read(KNOCK_TAP_COUNT, &tap_count, sizeof(u8)) == TOUCH_SUCCESS) {
		pData->count = tap_count;
	} else {
		LU202X_ERR("KNOCK_TAP_COUNT Read Fail\n");
		goto error;
	}

	if (!tap_count || tap_count > MAX_KNOCK_CODE_POS) {
		LU202X_LOG(BASE_INFO, "TAP COUNT = %d",tap_count);
		goto error;
	}

	if (Lu20xx_I2C_Read(KNOCK_TAP_COUNT + 1, buffer, KNOCK_CODE_DATA * tap_count) != TOUCH_SUCCESS) {
		LU202X_ERR("LPWG Data Read Fail\n");
		goto error;
	}

	for (i = 0; i < tap_count; i++)	{
		pData->knockData[i].x = (buffer[KNOCK_CODE_DATA*i + 1] << 8 | \
					buffer[KNOCK_CODE_DATA*i]);
		pData->knockData[i].y = (buffer[KNOCK_CODE_DATA*i + 3] << 8 | \
					buffer[KNOCK_CODE_DATA*i + 2]);
		/*This code is only debugging*/
		LU202X_LOG(LPWG_COORDINATES, "LPWG data [%d, %d]\n", pData->knockData[i].x, pData->knockData[i].y);
	}

	return TOUCH_SUCCESS;
error:
	return TOUCH_FAIL;
}

static void Lu202x_ClearInterrupt(void)
{
	int ret = 0;
	u8 regStatus = 0;

	ret = Lu20xx_I2C_Read(INT_INFORM, &regStatus, 1);
	if (ret == TOUCH_FAIL) {
		LU202X_ERR("failed to read interrupt status reg\n");
	}
	/* To Do - NSM */
	// Test & Checking
	return;
}

int Lu202x_InterruptHandler(TouchReadData *pData)
{
	int i=0;
	TouchFingerData *pFingerData = NULL;
	TouchKeyData *pKeyData = NULL;
	lu202x_tpd touch_data;
	touch_info info;
	static u8 pressure_temp = 0;
	u8 valid = 0;

	memset(&info, 0x0, sizeof(touch_info));
	memset(&touch_data, 0x0, sizeof(lu202x_tpd));

	pressure_temp ^= 1;

	pData->type = DATA_UNKNOWN;
	pData->count = 0;

	if (Lu20xx_I2C_Read(INT_INFORM, (u8 *)&touch_data.EventType, 1) != TOUCH_SUCCESS) {
		LU202X_ERR("Read Interrupt Status Fail. (0x0000)\n");
		goto fail;
	}

	if (Lu20xx_I2C_Read(TOUCH_VALID, &valid, 1) != TOUCH_SUCCESS) {
		LU202X_ERR("Read Touch valid count Fail. (0x0002)\n");
		goto fail;
	}

	switch (touch_data.EventType) {
		case EVENT_ABS :
		if (Lu20xx_I2C_Read(TOUCH_FINGER, (u8 *)&touch_data.Point, 8) != TOUCH_SUCCESS) {
			LU202X_ERR("Read touch data Fail. (0x0005)\n");
			goto fail;
		}
		SleepInLu202x();
		pData->type = DATA_FINGER;
		for (i = 0 ; i < valid; i++) {
			info.x[i] = ((touch_data.Point[i*4+1] & 0x07) << 8) | touch_data.Point[i*4];
			info.y[i] = ((touch_data.Point[i*4+2] & 0x38) << 5) | \
				((touch_data.Point[i*4+2] & 0x07) << 5) | \
				((touch_data.Point[i*4+1] >> 3) & 0x1f);
			info.id[i] = ((touch_data.Point[i*4+3] & 0x07) << 3) | \
				((touch_data.Point[i*4+2] >> 6) & 0x03);
			info.status[i] = ( touch_data.Point[i*4+3] >> 3 ) & 0x03;

			if(info.status[i] == TYPE_PRESS) {
				pFingerData = &pData->fingerData[info.id[i]];
				pFingerData->id = info.id[i];
				pFingerData->x  = info.x[i];
				pFingerData->y  = info.y[i];
				pFingerData->width_major = 15;
				pFingerData->width_minor = 10;
				pFingerData->orientation = 1;
				pFingerData->pressure = 20 + pressure_temp;
				pFingerData->status = FINGER_PRESSED;
				pData->count++;
			}
			if(info.status[i] == TYPE_RELEASE) {
				pFingerData = &pData->fingerData[info.id[i]];
				pFingerData->id = info.id[i];
				pFingerData->status = FINGER_RELEASED;
			}
		}
		break;

        case EVENT_KEY :
		if (Lu20xx_I2C_Read(TOUCH_KEY, (u8 *)&touch_data.KeyData, 2) != TOUCH_SUCCESS) {
			LU202X_ERR("Read touch key data Fail. (0x0003)\n");
			goto fail;
		}
		SleepInLu202x();
		pData->type = DATA_KEY;
		pData->count++;
		pKeyData = &pData->keyData;

		if (touch_data.KeyData[0] == 0)	{
			pKeyData->index = pressed_key;
			pKeyData->pressed = KEY_RELEASED;
			LU202X_LOG(BASE_INFO, "Touch Key[%d] was Released\n", pKeyData->index);
			pressed_key = 0;
		} else {
			pressed_key = touch_data.KeyData[0];
			pKeyData->index = pressed_key;
			pKeyData->pressed = KEY_PRESSED;
			LU202X_LOG(BASE_INFO, "Touch Key[%d] was Pressed\n", pKeyData->index);
		}
		break;

        case EVENT_KNOCK_ON :
		pData->type = DATA_KNOCK_ON;
		LU202X_LOG(BASE_INFO, "[KNOCK ON] Event Type = %d\n", touch_data.EventType);
		SleepInLu202x();
		break;

        case EVENT_KNOCK_CODE :
		pData->type = DATA_KNOCK_CODE;
		get_lpwg_data(pData);
		LU202X_LOG(BASE_INFO, "[KNOCK CODE]\n");
		SleepInLu202x();
		break;

        case EVENT_KNOCK_OVER :
		pData->type = DATA_KNOCK_CODE;
		pData->knockData[0].x = 1;
		pData->knockData[0].y = 1;
		pData->knockData[1].x = -1;
		pData->knockData[1].y = -1;
		LU202X_LOG(BASE_INFO, "[KNOCK CODE OVER] Event Type = %d\n", touch_data.EventType);
		SleepInLu202x();
		break;

	case EVENT_PALM :
		if (valid) {
			pData->type = DATA_FINGER;
			for (i = 0; i < MAX_FINGER_NUM; i++) {
				pFingerData = &pData->fingerData[i];
				if ((gDriverData->reportData.finger >> i) & FINGER_PRESSED) {
					pFingerData->id = i;
					pFingerData->pressure = 255;
					pFingerData->status = FINGER_PRESSED;
					pData->count++;
				}
			}
			palm_state = 1;
			LU202X_LOG(BASE_INFO, "[PALM] detected\n");
		} else {
			palm_state = 0;
			LU202X_LOG(BASE_INFO, "[PALM] released.\n");
		}
		SleepInLu202x();
		break;

        default:
		LU202X_LOG(BASE_INFO, "[Unknown] Event Type = %d\n",touch_data.EventType);
		SleepInLu202x();
		break;
	}
	return TOUCH_SUCCESS;
fail:
	/* To Do - NSM */
	//If occur fail, what to do.
	SleepInLu202x();
	return TOUCH_FAIL;
}

int Lu202x_ReadIcFirmwareInfo(TouchFirmwareInfo *pFwInfo)
{
	int result = TOUCH_SUCCESS;
	u8 readData[4] = {0};

	LU202X_FUNC(FW_UPGRADE);

	WakeUpLu202x(TOGGLEWAKEUP);
	if (Lu20xx_I2C_Read(FW_VERSION_REG-2, &readData[0], 4) != TOUCH_SUCCESS) {
		LU202X_ERR("Firmware version read fail (0x0080)\n");
		result = TOUCH_FAIL;
	}
	SleepInLu202x();

	pFwInfo->isOfficial = readData[2];
	pFwInfo->version = readData[3]; /* release version */

	LU202X_LOG(BASE_INFO, "IC Firmware Official = %d\n", pFwInfo->isOfficial);
	LU202X_LOG(BASE_INFO, "IC Firmware Version = 0x%02X\n", pFwInfo->version);

	return result;
}

int Lu202x_GetBinFirmwareInfo(char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	const struct firmware *fw = NULL;
	int result = TOUCH_SUCCESS;
	char *pFwFilename = NULL;
	u8 *pFw = NULL;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	LU202X_FUNC(FW_UPGRADE);

	if (pFilename == NULL) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}
	LU202X_LOG(BASE_INFO, "Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	result = request_firmware(&fw, pFwFilename, &client->dev);
	if (result) {
		LU202X_ERR("Failed at request_firmware() ( error = %d )\n", result);
		result = TOUCH_FAIL;
		goto earlyReturn;
	}

	pFw = (u8 *)(fw->data);
	pFwInfo->isOfficial = pFw[0x79FE];
	pFwInfo->version = pFw[0x79FF]; /* release version */

	LU202X_LOG(BASE_INFO, "BIN Firmware Official = %d\n", pFwInfo->isOfficial);
	LU202X_LOG(BASE_INFO, "BIN Firmware Version = 0x%02X\n", pFwInfo->version);

	/* Free firmware image buffer */
	release_firmware(fw);

earlyReturn:
	return result;
}


int Lu202x_UpdateFirmware(char *pFilename )
{
	int result = TOUCH_SUCCESS;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	u8 Cmd[3] = {0,};
	u8 bin_fw_checksum[6]= {0,};
	char *pFwFilename = NULL;
	char checksum[6] = {0};
	struct i2c_client *client = Touch_Get_I2C_Handle();
	u8 retry_cnt = 3;

	LU202X_FUNC(FW_UPGRADE);
	if (pFilename == NULL) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}
	LU202X_LOG(BASE_INFO, "Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	result = request_firmware(&fw, pFwFilename, &client->dev);
	if (result) {
		LU202X_ERR("Failed at request_firmware() ( error = %d )\n", result);
		result = TOUCH_FAIL;
		return result;
	} else {
		pBin = (u8 *)(fw->data); /* header size is 16bytes */
	}

	/* Check the firmware size for updating to IC */
	if (fw->size % 256) {
		LU202X_ERR("Touch Firmware Binary size is not 31KB(%d)\n", fw->size);
		return TOUCH_FAIL;
	}

	/*TO DO - Firmware update problem - NSM*/
	/*Complete : It need to enter usermode*/
	WakeUpLu202x(TOGGLEWAKEUP);
	Cmd[0] = 0x80;
	if (Lu20xx_I2C_Write(0x1000, Cmd, 1) == TOUCH_FAIL) {
		LU202X_ERR("Set mode operation failed\n" );
		goto earlyReturn;
	}

	Cmd[0] = 0x75;
	Cmd[1] = 0x6C;
	if (Lu20xx_I2C_Write(USER_PASSWORD, Cmd, 2) == TOUCH_FAIL) {
		LU202X_ERR("Set password operation failed\n" );
		goto earlyReturn;
	}
	//Update stable timing control(IC)
	Cmd[0] = 0x01;
	if (Lu20xx_I2C_Write(0x1020, Cmd, 1) == TOUCH_FAIL) {
		LU202X_ERR("Timing control operation failed\n" );
		goto earlyReturn;
	}
	Cmd[0] = 0xB0;
	Cmd[1] = 0x28;
	Cmd[2] = 0x09;
	if (Lu20xx_I2C_Write(0x102B, Cmd, 3) == TOUCH_FAIL) {
		LU202X_ERR("Timing control set failed\n" );
		goto earlyReturn;
	}

	Cmd[0] = 0xD0;
	Cmd[1] = 0x02;
	if (Lu20xx_I2C_Write(0x1027, Cmd, 2) == TOUCH_FAIL) {
		LU202X_ERR("Timing control set failed\n" );
		goto earlyReturn;
	}

	/*This part is LU2020 firmware update. */
	if (Lu202x_programFW(pBin, FW_POS, FW_SIZE + CFG_SIZE) == TOUCH_FAIL) {
		LU202X_ERR("Failed to program firmware\n");
		result = TOUCH_FAIL;
		goto earlyReturn;
	}

	/* Reset to read checksum */
	Lu202x_Reset();

	/* Calculate BIN Checksum */
	Lu202x_bin_checksum(fw, bin_fw_checksum);

	/* Read IC Checksum */
	do {
		result = Lu202X_command_send(CMD_LU202x_DEVINFO, LU202x_CMD_FW_CHECKSUM_ADDR, sizeof(checksum), checksum);
		if (result == TOUCH_SUCCESS) {
			break;
		} else {
			LU202X_ERR("Failed at read checksum %d\n", retry_cnt);
			if (retry_cnt == 1) {
				result = TOUCH_FAIL;
				goto earlyReturn;
			}
		}
	} while (retry_cnt-- > 0);

	/* Calculated Checksum*/
	LU202X_LOG(BASE_INFO, "FW binary checksum = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", \
		bin_fw_checksum[0], bin_fw_checksum[1], bin_fw_checksum[2], bin_fw_checksum[3], \
		bin_fw_checksum[4], bin_fw_checksum[5]);

	/* Read Checksum */
	LU202X_LOG(BASE_INFO, "I2C read checksum = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", \
		checksum[0], checksum[1], checksum[2], checksum[3], checksum[4], checksum[5]);

	/* Compare checksum */
	/* To Do - NSM*/
	// Must modify when use non header binary
	if (memcmp((u8 *)(bin_fw_checksum), checksum, 6)) {
		LU202X_ERR("Checksum value is not same. Failed to program firmware\n");
		result = TOUCH_FAIL;
	}

earlyReturn:
	SleepInLu202x();
	/* Free firmware image buffer */
	release_firmware(fw);

	/* Reset ??? */
	Lu202x_Reset();
	return result;
}

static int Lu202X_reply_check( void )
{
	u8 reply[2] = {0,};

	if (Lu20xx_I2C_Read(LU202x_CMDReply_ADDR, reply, 2) == TOUCH_FAIL) {
		LU202X_ERR("Reply Data Read fail.\n");
		goto fail;
	} else {
		if (reply[0] != LU202x_CHCAPTEST_Reply) {
			LU202X_ERR("Cap Test reply fail\n");
			/* Recovery I2C mode when Slave I2C timeout */
			if (TouchReadGpioInterrupt() == 1) {
				LU202X_LOG(BASE_INFO, "Try captest recovery.");
				if(WakeUpLu202x(TOGGLEWAKEUP) == TOUCH_FAIL) {
					goto fail;
				}
			} else {
				goto fail;
			}
		}
	}

	return TOUCH_SUCCESS;
fail :
	return TOUCH_FAIL;
}

int Lu202x_knock_on_code_setting(TouchState newState)
{
	u8 temp = 0;
	u8 dly_temp[2] = {0,};
	u16 tap_dlytime = 0;

	if (newState == STATE_KNOCK_ON_ONLY) {
		temp = 1;
		if (Lu20xx_I2C_Write(KNOCK_STATUS, &temp, 1) != 0) {
			LU202X_ERR(" Write KNOCK_STATUS fail.\n");
			goto fail;
		}
		temp = 2;
		if (Lu20xx_I2C_Write(KNOCK_ON_TAP_COUNT, &temp, 1) !=0) {
			LU202X_ERR(" Mode KNOCK_ON_TAP_COUNT fail.\n");
			goto fail;
		}
		temp = 0;
		if (Lu20xx_I2C_Write(KNOCK_ON_REPORT_DELAY, &temp, 1) !=0) {
			LU202X_ERR(" Write KNOCK_ON_REPORT_DELAY fail.\n");
			goto fail;
		}
	} else if (newState == STATE_KNOCK_ON_CODE) {
		temp = 3;
		if (Lu20xx_I2C_Write(KNOCK_STATUS, &temp, 1) != 0) {
			LU202X_ERR(" Write KNOCK_STATUS fail.\n");
			goto fail;
		}
		if(gDriverData->lpwgSetting.isFirstTwoTapSame)
			tap_dlytime = 60;
		else
			tap_dlytime = 35;
		dly_temp[0] = tap_dlytime & 0x00FF;
		dly_temp[1] = (tap_dlytime & 0xFF00) >> 8;
		if (Lu20xx_I2C_Write(KNOCK_ON_REPORT_DELAY, dly_temp, 2) !=0) {
			LU202X_ERR(" Write KNOCK_ON_REPORT_DELAY fail.\n");
			goto fail;
		}

		temp = (u8)(gDriverData->lpwgSetting.tapCount);
		if (Lu20xx_I2C_Write(KNOCK_CODE_TAP_COUNT, &temp, 1) != 0) {
			LU202X_ERR(" Write KNOCK_CODE_TAP_COUNT fail.\n");
			goto fail;
		}
	}
	return TOUCH_SUCCESS;
fail:
	return TOUCH_FAIL;
}

static int Lu202X_command_send(u8 comm, u16 mode, u16 sub_mode, u8 *value)
{
	TouchState newState = gDeviceData.deviceState;
	u8 buf[5]={0,};
	u8 size = 0;
	int ret = 0;

	buf[0] = comm;
	if (comm == CMD_LU202x_CHANGEMODE) {
		buf[1] = (u8)(mode & 0xFF);
		buf[3] = (u8)(sub_mode & 0xFF);
		size = 5;
	} else {
		size = 3;
	}

	WakeUpLu202x(TOGGLEWAKEUP);
	if (Lu20xx_I2C_Write(LU202x_MODE_ADDR, buf, size) == TOUCH_FAIL) {
		LU202X_ERR("Command send to mode reg fail.\n");
		goto fail;
	}

	/* Each mode setting data */
	ret = Lu202x_knock_on_code_setting(newState);
	if (ret == TOUCH_FAIL) {
		goto fail;
	}
	SleepInLu202x();

	/* Mode check */
	WakeUpLu202x(WAKEUP);
	if (Lu20xx_I2C_Read(LU202x_CMDACK_ADDR, buf, 3) == TOUCH_FAIL) {
		LU202X_ERR("Read Cmd ACK fail.\n");
		goto fail;
	} else {
		if (buf[2] != comm) {
			LU202X_ERR("Mode Change fail(%d)-%d.\n", buf[2], mode);
			goto fail;
		}
	}

	/* use value variable if you want to get data. */
	if (comm != CMD_LU202x_CHANGEMODE || value != NULL) {
		mdelay(2);
		/* only check reply data to cap test mode */
		if (comm == CMD_LU202x_CHCAPTEST) {
			if (Lu202X_reply_check() == TOUCH_FAIL) {
				goto fail;
			}
		} else if (mode == CMD_LU202x_DEBUG) {
			msleep(100);
			sub_mode = MAX_CHANNEL;
			mode = 0x0104;
		}
		/* sub_mode is using as size */
		ret = Lu20xx_I2C_Read(mode, value, sub_mode);
		if (ret == TOUCH_FAIL) {
			LU202X_ERR("Failed to read command data\n");
			goto fail;
		}
	}
	SleepInLu202x();

	/* Add 10ms to verify Mode change */
	if (newState == STATE_KNOCK_ON_ONLY || newState == STATE_KNOCK_ON_CODE) {
		msleep(10);
	}

	return TOUCH_SUCCESS;

fail:
	SleepInLu202x();
	return TOUCH_FAIL;
}

int Lu202x_SetLpwgMode(TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int result = TOUCH_SUCCESS;
	int call_state = 0;
	int i = 0;

	LU202X_FUNC(BASE_INFO);

	gDeviceData.deviceState = newState;
	call_state = atomic_read(&gDriverData->incoming_call);

	if (gpio_get_value(TOUCH_GPIO_RESET) == 0) {
		TOUCH_LOG("Reset pin is low %d", gpio_get_value(TOUCH_GPIO_RESET));
		Lu202x_Reset();
	}

	TouchDisableIrq();
	for (i = 0; i < 3; i++) {
		if (newState == STATE_NORMAL && call_state != INCOMING_CALL_IDLE) {
			result = Lu202X_command_send(CMD_LU202x_CHANGEMODE,
					CMD_LU202x_DIAL_MODE, 0x0, NULL);
			LU202X_LOG(BASE_INFO, "Lu202X was changed to CALLING\n");
		} else if (newState == STATE_NORMAL) {
			LU202X_LOG(BASE_INFO, "LU202X was changed to NORMAL\n");
		} else if (newState == STATE_KNOCK_ON_ONLY) {
			result = Lu202X_command_send(CMD_LU202x_CHANGEMODE,
					CMD_LU202x_KNOCK_ON_ONLY, 0x0, NULL);
			LU202X_LOG(BASE_INFO, "LPWG Mode Changed to KNOCK_ON_ONLY\n");
		} else if (newState == STATE_KNOCK_ON_CODE) {
			result = Lu202X_command_send(CMD_LU202x_CHANGEMODE,
					CMD_LU202x_KNOCK_ON_CODE, 0x0, NULL);
			LU202X_LOG(BASE_INFO, "LPWG Mode Changed to KNOCK_ON_CODE.\n");
		} else if (newState == STATE_OFF) {
			result = Lu202X_command_send(CMD_LU202x_CHANGEMODE,
					CMD_LU202x_PDN, 0x0, NULL);
			LU202X_LOG(BASE_INFO, "LPWG Mode Changed to PDN mode.\n");
		} else {
			LU202X_ERR("Unknown State %d", newState);
		}

		if (result == TOUCH_SUCCESS) {
			break;
		} else {
			Lu202x_Reset();
		}
	}
	TouchEnableIrq();

	return result;
}

static int LU202x_ChCap_Test(char* pBuf ,int* pRawStatus, int* pChannelStatus, int* pDataLen)
{
	u8 max_raw_data[MAX_CHANNEL * 4 + 4] = {0, };
	u8 Raw_Cap_Value[MAX_CHANNEL * 2] = {0, };
	u8 Jitter_Value[MAX_CHANNEL * 2] = {0, };
	u8 AutoCycle[4] = {0,};
	u8 i = 0;
	u8 fw_status = 0;
	int dataLen = 0;
	int ret = 0;
	int retry_cnt = 3;

	*pRawStatus = TOUCH_SUCCESS;
	*pChannelStatus = TOUCH_SUCCESS;

	dataLen += sprintf(pBuf + dataLen, "=====Test Start=====\n");
	dataLen += sprintf(pBuf + dataLen, "    -RawCap Value-                 -Jitter Value-    \n");

	LU202X_LOG(BASE_INFO, "=====Test Start=====\n");
	LU202X_LOG(BASE_INFO, "    -RawCap Value-                 -Jitter Value-    \n");

	/* Check the channel status. */
	do {
		ret = Lu202X_command_send(CMD_LU202x_DEVINFO, LU202x_CMDReply_ADDR + 2, sizeof(fw_status), &fw_status);
		if (ret == TOUCH_SUCCESS) {
			LU202X_LOG(BASE_INFO, "Firmware status check(%d)\n", fw_status);
			break;
		} else {
			LU202X_ERR("Firmware status check fail %d\n", retry_cnt);
			if (retry_cnt == 1) {
				goto fail;
			}
		}
	} while (retry_cnt-- > 0);

	if (fw_status == 1) {
		LU202X_LOG(BASE_INFO, "!!!Error_Open/Short!!! (%d)\n", fw_status);
		dataLen += sprintf(pBuf + dataLen, "!!!Error_Open/Short!!! %d\n", fw_status);
		*pChannelStatus = TOUCH_FAIL;
	}

	/* Read Raw Cap value(Read data size = MAX_CHANNEL * 2, channel number 36) */
	retry_cnt = 3;
	do {
		ret = Lu202X_command_send(CMD_LU202x_CHCAPTEST, LU202x_RAWCAP_ADDR, sizeof(max_raw_data), max_raw_data);
		if (ret == TOUCH_SUCCESS) {
			memcpy(Raw_Cap_Value, max_raw_data, sizeof(Raw_Cap_Value));
			memcpy(Jitter_Value, &max_raw_data[sizeof(Raw_Cap_Value)], sizeof(Jitter_Value));
			memcpy(AutoCycle, &max_raw_data[sizeof(Raw_Cap_Value) + sizeof(Jitter_Value)], sizeof(AutoCycle));
			break;
		} else {
			LU202X_ERR("Read Raw Cap value fail %d\n", retry_cnt);
			if (retry_cnt == 1) {
				goto fail;
			}
		}
	} while (retry_cnt-- > 0);

	for (i = 0; i < MAX_CHANNEL; i++) {
		/* compare test limits with readed rawcap */
		if (COMB16(Raw_Cap_Value, i) <= Lu202x_LowerLimit[i] || \
			COMB16(Raw_Cap_Value, i) >= Lu202x_UpperLimit[i]) {
			dataLen += sprintf(pBuf + dataLen, "!!!Raw data spec over and lower ch:%d[%d]\n", \
					i, COMB16(Raw_Cap_Value, i));
			*pRawStatus = TOUCH_FAIL;
		}

		LU202X_LOG(BASE_INFO, "Raw cap value [%d] = %d , Jitter value [%d] = %d\n", \
			i, COMB16(Raw_Cap_Value, i), i, COMB16(Jitter_Value, i));
		dataLen += sprintf(pBuf + dataLen, "Raw cap value [%d] = %d , Jitter value [%d] = %d\n", \
			i, COMB16(Raw_Cap_Value, i), i, COMB16(Jitter_Value, i));
	}
	LU202X_LOG(BASE_INFO, "View area cycle = %d , Button area cycle = %d\n", \
		COMB16(AutoCycle, 0), COMB16(AutoCycle, 1));

	dataLen += sprintf(pBuf + dataLen, "View area cycle = %d , Button area cycle = %d\n", \
		COMB16(AutoCycle, 0), COMB16(AutoCycle, 1));

	*pDataLen  += dataLen;

	return TOUCH_SUCCESS;
fail: //I2c Fail
	*pRawStatus = TOUCH_FAIL;
	*pChannelStatus = TOUCH_FAIL;
	return TOUCH_FAIL;
}

static int LU202x_Lpwg_ChCap_Test(char* pBuf, int* lpwgStatus, int* pDataLen)
{
	u8 max_raw_data[MAX_CHANNEL * 4 + 4] = {0, };
	u8 Raw_Cap_Value[MAX_CHANNEL * 2] = {0, };
	u8 Jitter_Value[MAX_CHANNEL * 2] = {0, };
	u8 AutoCycle[4] = {0,};
	u8 i = 0;
	int dataLen = 0;
	int ret = 0;
	int retry_cnt = 3;

	*lpwgStatus = TOUCH_SUCCESS;

	dataLen += sprintf(pBuf + dataLen, "=====Test Start(LPWG)=====\n");
	dataLen += sprintf(pBuf + dataLen, "    -RawCap Value-                 -Jitter Value-    \n");

	LU202X_LOG(BASE_INFO, "=====Test Start(LPWG)=====\n");
	LU202X_LOG(BASE_INFO, "    -RawCap Value-                 -Jitter Value-    \n");

	/* IC stable time for lpwg mode to read cap data */
	mdelay(2000);

	/* Read Raw Cap value(Read data size = MAX_CHANNEL * 2, channel number 36) */
	do {
		ret = Lu202X_command_send(CMD_LU202x_CHCAPTEST, LU202x_RAWCAP_ADDR, sizeof(max_raw_data), max_raw_data);
		if (ret == TOUCH_SUCCESS) {
			memcpy(Raw_Cap_Value, max_raw_data, sizeof(Raw_Cap_Value));
			memcpy(Jitter_Value, &max_raw_data[sizeof(Raw_Cap_Value)], sizeof(Jitter_Value));
			memcpy(AutoCycle, &max_raw_data[sizeof(Raw_Cap_Value) + sizeof(Jitter_Value)], sizeof(AutoCycle));
			break;
		} else {
			LU202X_ERR("Read Raw Cap value fail %d\n", retry_cnt);
			if (retry_cnt == 1) {
				goto fail;
			}
		}
	} while (retry_cnt-- > 0);

	for (i = 0; i < MAX_CHANNEL; i++) {
		/* compare test limits with readed rawcap */
		if (COMB16(Raw_Cap_Value, i) == 0) {
			dataLen += sprintf(pBuf + dataLen, "!!!Error_Open/Short ch:%d[%d]\n", \
					i, COMB16(Raw_Cap_Value, i));
			*lpwgStatus = TOUCH_FAIL;
		}

		LU202X_LOG(BASE_INFO, "Raw cap value [%d] = %d , Jitter value [%d] = %d\n", \
			i, COMB16(Raw_Cap_Value, i), i, COMB16(Jitter_Value, i));
		dataLen += sprintf(pBuf + dataLen, "Raw cap value [%d] = %d , Jitter value [%d] = %d\n", \
			i, COMB16(Raw_Cap_Value, i), i, COMB16(Jitter_Value, i));
	}
	LU202X_LOG(BASE_INFO, "View area cycle = %d , Button area cycle = %d\n", \
		COMB16(AutoCycle, 0), COMB16(AutoCycle, 1));

	dataLen += sprintf(pBuf + dataLen, "View area cycle = %d , Button area cycle = %d\n", \
		COMB16(AutoCycle, 0), COMB16(AutoCycle, 1));
	*pDataLen  += dataLen;

	return TOUCH_SUCCESS;
fail: //I2c Fail
	*lpwgStatus = TOUCH_FAIL;
	return TOUCH_FAIL;
}

void Lu202x_file_write ( char *fname, char *data, int fsize )
{
	int fd;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	if(!strcmp(fname, SD_FILE_PATH)) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND|O_SYNC, 0644);
	} /* SD */

	else {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_SYNC, 0666);
	} /* FW Dump */

	if (fd >= 0) {
		sys_write(fd, data, fsize);
		sys_close(fd);
	}

	set_fs(old_fs);
}

int Lu202x_DoSelfDiagnosis(int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int dataLen = 0;

	/* CAUTION : be careful not to exceed buffer size */

	/* do implementation for self-diagnosis */
	LU202x_ChCap_Test(pBuf, pRawStatus, pChannelStatus, &dataLen);
	Lu202x_file_write(SD_FILE_PATH, pBuf, strlen(pBuf));
	memset(pBuf, 0x00, bufSize);
	dataLen = 0;

	dataLen += sprintf(pBuf+dataLen, "======== RESULT File =======\n");
	dataLen += sprintf(pBuf+dataLen, "Channel Status : %s\n", (*pRawStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	dataLen += sprintf(pBuf+dataLen, "Raw Data : %s\n", (*pChannelStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	Lu202x_file_write(SD_FILE_PATH, pBuf, strlen(pBuf));

	LU202X_LOG(BASE_INFO, "======== RESULT File =======\n");
	LU202X_LOG(BASE_INFO, "Channel Status : %s\n", (*pRawStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	LU202X_LOG(BASE_INFO, "Raw Data : %s\n", (*pChannelStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	LU202X_LOG(BASE_INFO, "Channel Status : %d\n", *pRawStatus);
	LU202X_LOG(BASE_INFO, "Raw Data : %d\n", *pChannelStatus);
	/* TO DO - NSM */
	//SD log file write.
	//*pDataLen = dataLen;

	return TOUCH_SUCCESS;
}

static void Lu202x_PowerOn(int isOn)
{
	TOUCH_FUNC();
	
	if (isOn) {
		TouchVddPowerModel(isOn);
		TouchVioPowerModel(isOn);
	} else {
		TouchVddPowerModel(isOn);
		TouchVioPowerModel(isOn);
		TouchSetGpioReset(0);
	}
}

int Lu202x_DoSelfDiagnosis_Lpwg(int *lpwgStatus, char *pBuf, int bufSize, int *pDataLen)
{
	int dataLen = 0;

	/* CAUTION : be careful not to exceed buffer size */

	/* do implementation for self-diagnosis */
	LU202x_Lpwg_ChCap_Test(pBuf, lpwgStatus, &dataLen);
	Lu202x_file_write(SD_FILE_PATH, pBuf, strlen(pBuf));
	memset(pBuf, 0x00, bufSize);
	dataLen = 0;

	dataLen += sprintf(pBuf+dataLen, "======== RESULT File(LPWG) =======\n");
	dataLen += sprintf(pBuf+dataLen, "lpwgStatus : %s\n", (*lpwgStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	Lu202x_file_write(SD_FILE_PATH, pBuf, strlen(pBuf));

	LU202X_LOG(BASE_INFO, "======== RESULT File(LPWG) =======\n");
	LU202X_LOG(BASE_INFO, "lpwgStatus : %s\n", (*lpwgStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	LU202X_LOG(BASE_INFO, "lpwgStatus : %d\n", *lpwgStatus);

	return TOUCH_SUCCESS;
}

static void Lu202x_NotifyHandler(TouchNotify nofify, int data)
{
	int ret = 0;
	u8 retry_cnt = 3;

	if (nofify == NOTIFY_CALL) {
		do {
			if (data == INCOMING_CALL_IDLE) {
				/* call off */
				ret = Lu202X_command_send(CMD_LU202x_CHANGEMODE,
						CMD_LU202x_NORMODE, 0x0, NULL);
				LU202X_LOG(BASE_INFO, "INCOMING_CALL_IDLE\n");
			} else if (data == INCOMING_CALL_RINGING ||
					data == INCOMING_CALL_OFFHOOK) {
				ret = Lu202X_command_send(CMD_LU202x_CHANGEMODE,
						CMD_LU202x_DIAL_MODE, 0x0, NULL);
				LU202X_LOG(BASE_INFO, "%s\n", (data == INCOMING_CALL_RINGING) ?
									"INCOMING_CALL_RINGING" : "INCOMING_CALL_OFFHOOK");
			}

			if (ret == TOUCH_SUCCESS) {
				break;
			} else {
				if (retry_cnt == 1) {
					Lu202x_Reset();
				}
			}
		} while (retry_cnt-- > 0);
	}
}
/* This code is TestCode */


TouchDeviceControlFunction Lu202x_Func = {
	.Power = Lu202x_PowerOn,
	.Reset = Lu202x_Reset,
	.Initialize = Lu202x_Initialize,
	.InitRegister = Lu202x_InitRegister,
	.ClearInterrupt = Lu202x_ClearInterrupt,
	.InterruptHandler = Lu202x_InterruptHandler,
	.ReadIcFirmwareInfo = Lu202x_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = Lu202x_GetBinFirmwareInfo,
	.UpdateFirmware = Lu202x_UpdateFirmware,
	.SetLpwgMode = Lu202x_SetLpwgMode,
	.DoSelfDiagnosis = Lu202x_DoSelfDiagnosis,
	.DoSelfDiagnosis_Lpwg = Lu202x_DoSelfDiagnosis_Lpwg,
	.NotifyHandler = Lu202x_NotifyHandler,
	.device_attribute_list = lu202x_attribute_list,
};


/* End Of File */


