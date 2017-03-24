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
 *    File  	: lgtp_device_sn280h.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[SN280H]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include <linux/input/unified_driver_4/lgtp_device_sn280h.h>

#include <linux/input/unified_driver_4/lgtp_common.h>
#include <linux/input/unified_driver_4/lgtp_common_driver.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_i2c.h>
#include <linux/input/unified_driver_4/lgtp_model_config_i2c.h>
#include <linux/input/unified_driver_4/lgtp_model_config_misc.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_misc.h>

/* get revision information */
#include <soc/qcom/lge/lge_board_revision.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define TPD_I2C_ADDRESS			0x3C

#if defined ( TOUCH_PLATFORM_MTK )
#define I2C_DEVICE_ADDRESS_LEN		2
#define MAX_TRANSACTION_LENGTH		8
#define MAX_I2C_TRANSFER_SIZE		(MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)
#endif

/* Key Event Type */
#define KEY_PRESSED		1
#define KEY_RELEASED		0
#define CANCEL_KEY		0xFF

#define LPWG_STATUS_REG		0x4004
#define TAP_DATA_REG		0x4006

#define BUSY_CHECK_RETRY_COUNT	3
#define MAX_FINGER_NUM		2
#define MAX_BUTTONS		4
#define TSC_EEPROM_PAGE_SIZE	64

/*Touch Raw Data */
#define CAP_REFERENCE	0x4110
#define CAP_DELTA	0x40A0

/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/
typedef struct Sn280hDriverDataTag {

	TouchState deviceState;

} Sn280hDriverData;

/****************************************************************************
* Variables
****************************************************************************/
static int temp_hover_status = 0;
static int knock_flag = 0;
static int tap_count = 0;

static Sn280hDriverData gDeviceData = { STATE_NORMAL };
static const char defaultFirmware[] = "semisense/S212_E1_NA_LGE_4P5_HS_FW_V01_150916.img";
static const char defaultFirmwareRevB[] = "semisense/S212_E1_NA_LGE_4P5_HS_FW_V06_151014.img";
int g_miscInitialize = 0;
/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
void Sn280h_Reset(void);

void TouchSetGpioReset(int isHigh);
int TouchReadGpioMakerId(void);


/****************************************************************************
* Local Functions
****************************************************************************/



/****************************************************************************
* Global Functions
****************************************************************************/
static ssize_t show_Model_Info(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;

	WRITE_SYSBUF(buf, ret, "======== Model info ========\n");

	if (TouchReadGpioMakerId() == 0) {
		TOUCH_LOG("Touch IC : LeadingUI\n");
		WRITE_SYSBUF(buf, ret, "Maker ID PIN: 0\n");
		WRITE_SYSBUF(buf, ret, "Module Product : SUNTEL\n");
		WRITE_SYSBUF(buf, ret, "Touch IC : LeadingUI\n");
	} else if (TouchReadGpioMakerId() == 1) {
		WRITE_SYSBUF(buf, ret, "Maker ID PIN: 1\n");
		WRITE_SYSBUF(buf, ret, "Module Product : LGIT\n");
		WRITE_SYSBUF(buf, ret, "Touch IC : Focaltech\n");
	}

	return ret;
}
static LGE_TOUCH_ATTR(Model_Info, S_IRUGO | S_IWUSR, show_Model_Info, NULL);


static ssize_t show_version_read(TouchDriverData *pDriverData, char *buf)
{
	int result = 0;
	int ret = 0;
	u16 addr = 0;
	u16 rdata = 0;
	u8 ic_code[8] = {0,};

	/* Factory data & firmware version check 0x1e --> 30 -->*/
	addr = REG_FIRMWARE_VERSION;
	result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata,
			sizeof(addr), sizeof(rdata));
	if (result < 0) {
		goto i2c_fail;
	} else {
		WRITE_SYSBUF(buf, ret, "[TEST]Firmware version = 0x%x\n", rdata);
		TOUCH_LOG("[TEST]Firmware version = 0x%x\n", rdata);
	}

	addr = REG_IC_CODE;
	result = Semisense_I2C_Read((u8 *)&addr, (u8 *)ic_code,
			sizeof(addr), sizeof(ic_code));
	if (result < 0) {
		goto i2c_fail;
	} else {
		WRITE_SYSBUF(buf, ret, "[TEST]IC CODE = %d,%d,%d,%d,%d,%d,%d,%d \n",
				ic_code[0],ic_code[1],ic_code[2],ic_code[3],
				ic_code[4],ic_code[5],ic_code[6],ic_code[7]);
		TOUCH_LOG("[TEST]IC CODE = %d,%d,%d,%d,%d,%d,%d,%d \n",
				ic_code[0],ic_code[1],ic_code[2],ic_code[3],
				ic_code[4],ic_code[5],ic_code[6],ic_code[7]);
	}

	addr = TSC_FLASH_FW_VER_POS;
	result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata,
			sizeof(addr), sizeof(rdata));
	if (result < 0) {
		goto i2c_fail;
	} else {
		WRITE_SYSBUF(buf, ret, "[TEST]TSC_FLASH_FW_VER_POS = 0x%x\n", rdata);
		TOUCH_LOG("[TEST]TSC_FLASH_FW_VER_POS = 0x%x\n", rdata);
	}

	addr = TSC_PANEL_TEST_VER;
	result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata,
			sizeof(addr), sizeof(rdata));
	if (result < 0) {
		goto i2c_fail;
	} else {
		WRITE_SYSBUF(buf, ret, "[TEST]TSC_PANEL_TEST_VER = 0x%x\n", rdata);
		TOUCH_LOG("[TEST]TSC_PANEL_TEST_VER = 0x%x\n", rdata);
	}

	addr = REG_DEVIATION_CODE;
	result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata,
			sizeof(addr), sizeof(rdata));
	if (result < 0) {
		goto i2c_fail;
	} else {
		WRITE_SYSBUF(buf, ret, "[TEST]REG_DEVIATION_CODE  = 0x%x\n", rdata);
		TOUCH_LOG("[TEST]REG_DEVIATION_CODE  = 0x%x\n", rdata);
	}

	addr = REG_PROJECT_CODE;
	result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata,
			sizeof(addr), sizeof(rdata));
	if (result < 0) {
		goto i2c_fail;
	} else {
		WRITE_SYSBUF(buf, ret, "[TEST]REG_PROJECT_CODE  = 0x%x\n", rdata);
		TOUCH_LOG("[TEST]REG_PROJECT_CODE  = 0x%x\n", rdata);
	}

	addr = REG_MP_FIELD;
	result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata,
			sizeof(addr), sizeof(rdata));
	if (result < 0) {
		goto i2c_fail;
	} else {
		WRITE_SYSBUF(buf, ret, "[TEST]REG_MP_FIELD  = 0x%x\n", rdata);
		TOUCH_LOG("[TEST]REG_MP_FIELD  = 0x%x\n", rdata);
	}

	return ret;
i2c_fail:
	WRITE_SYSBUF(buf, ret, "I2C fail with addr(0x%x) \n", addr);
	TOUCH_ERR("I2C fail with addr(0x%x)\n", addr );
	return ret;
}
static LGE_TOUCH_ATTR(version_read, S_IRUGO | S_IWUSR, show_version_read, NULL);


static ssize_t show_Touch_Reset(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	Sn280h_Reset();
	WRITE_SYSBUF(buf, ret, "[touch reset]\n");
	return ret;
}
static LGE_TOUCH_ATTR(Touch_Reset, S_IRUGO | S_IWUSR, show_Touch_Reset, NULL);


static ssize_t show_Set_vp(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	WRITE_SYSBUF(buf, ret, "[%s] \n", temp_hover_status ? "1" : "0");

	return ret;
}


static ssize_t store_Set_vp(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	u16 Addr = GESTURE_MODE_REG;
	u16 wData = 0;
	int cmd = 0;
	int ret = 0;

	sscanf(buf, "%d", &cmd);
	if (cmd == 1) {
		wData = 0x0404;
		ret = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData,
				sizeof(Addr), sizeof(wData));
		if (ret < 0)
			goto i2c_fail;
		TOUCH_LOG("Set hover on mode.\n");
		temp_hover_status = 1;
	} else if(cmd == 0) {
		wData = 0x0505;
		if(Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData)) < 0)
			goto i2c_fail;
		TOUCH_LOG("Set hover off mode. \n");
		temp_hover_status = 0;
	} else {
		TOUCH_LOG("Invalid HOVER command\n");
	}

	return count;

i2c_fail:    
    TOUCH_LOG("I2C fail with addr(0x%x), write data(0x%x)\n", Addr, wData);
    return count;
}
static LGE_TOUCH_ATTR(Set_vp, S_IRUGO | S_IWUSR, show_Set_vp, store_Set_vp);


int print_channel(char *buf, int ret, int tx)
{
	int rx = 0;
	int i = 0;

	for (i = 0; i < 2; i++) {
		WRITE_SYSBUF(buf, ret, "[D%d] |   ",
			(tx == 0 || tx == 1) ? (i == 0) ? 0 : 3 : (i == 1) ? 2 : 1);
		rx = (tx % 2 == 0) ? 8 : 7;
		for ( ; rx > 0; rx -= 2) {
			WRITE_SYSBUF(buf, ret, "[S%d]  ", rx - 1);
		}
		if (i == 0)
			WRITE_SYSBUF(buf, ret, "\t| ");
	}
	WRITE_SYSBUF(buf, ret, "\n");

	return ret;
}

#define TOUCH_TX_CHANNEL	4
#define TOUCH_RX_CHANNEL	8
int read_channel_data(char *buf, u16 Addr)
{
	u16 rData[TOUCH_TX_CHANNEL * TOUCH_RX_CHANNEL] = {0,};
	int rx = 0, tx = 0;
	int ret = 0;

	ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)rData,
			sizeof(Addr), sizeof(rData));
	if (ret < 0) {
		goto i2c_fail;
	}
	ret = 0;

	WRITE_SYSBUF(buf, ret, "=====================================================\n");
	WRITE_SYSBUF(buf, ret, "               %s\n", (Addr == CAP_DELTA) ?\
						"DELTA VALUE" : "RAW DATA VALUE");
	WRITE_SYSBUF(buf, ret, "=====================================================\n");
	for (tx = 0; tx < TOUCH_TX_CHANNEL; tx++) {
		if (tx % 2 == 0)
			ret = print_channel(buf, ret, tx);
		WRITE_SYSBUF(buf, ret, "\t");

		for (rx = 0; rx < TOUCH_RX_CHANNEL; rx++) {
			if (rx == 4) {
				WRITE_SYSBUF(buf, ret, "\t|\t  ");
			}
			WRITE_SYSBUF(buf, ret, "%05d ", rData[tx * TOUCH_RX_CHANNEL + rx]);
		}
		WRITE_SYSBUF(buf, ret, "\n");

		if (tx % 2 == 1)
			ret = print_channel(buf, ret, tx);
	}

	return ret;

i2c_fail:
	TOUCH_LOG("I2C fail with addr(0x%x)-(%x)\n", Addr, ret);
	return TOUCH_FAIL;
}

static ssize_t show_delta(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;

	ret = read_channel_data(buf, CAP_DELTA);
	if (ret == TOUCH_FAIL) {
		TOUCH_LOG("I2C fail with addr(0x%x)\n", CAP_DELTA);
	}

	return ret;
}
static LGE_TOUCH_ATTR(delta, S_IRUGO, show_delta, NULL);


static ssize_t show_raw_data(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;

	ret = read_channel_data(buf, CAP_REFERENCE);
	if (ret == TOUCH_FAIL) {
		TOUCH_LOG("I2C fail with addr(0x%x)\n", CAP_REFERENCE);
	}

	return ret;
}
static LGE_TOUCH_ATTR(raw_data, S_IRUGO, show_raw_data, NULL);


/* debugging code */
static u16 Addr = 0;
static u16 rData = 0;
static u16 wData = 0;
static ssize_t store_i2c_control(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	u16 cmd[3] ={0,};

	int ret = 0;

	sscanf(buf, "%x %x %x",
		(unsigned int*)&cmd[0],
		(unsigned int*)&cmd[1],
		(unsigned int*)&cmd[2]);
	TOUCH_LOG("cmd : 0x%x 0x%x 0x%x\n", cmd[0], cmd[1], cmd[2]);

	Addr = cmd[1];
	wData = cmd[2];

	TouchDisableIrq();
	switch (cmd[0]) {
	case 1 :
		ret = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData,
				sizeof(Addr), sizeof(wData));
		if (ret < 0)
			goto i2c_fail;

		TOUCH_LOG("Success write : Addr[0x%x] Data[0x%x]\n", Addr, wData);
		break;

	case 2 :
		ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&rData,
				sizeof(Addr), sizeof(rData));
		if (ret < 0)
			goto i2c_fail;

		TOUCH_LOG("Success read : Addr[0x%x] Data[0x%x]\n", Addr, rData);
		break;

	default :
		TOUCH_LOG("===============================================\n");
		TOUCH_LOG(" if you want to use this sysfs as follows \n");
		TOUCH_LOG(" echo [1(Write)/2(Read)] [Address] [Write Value] \n");
		TOUCH_LOG("===============================================\n");
		break;
	}
	TouchEnableIrq();

	return count;

i2c_fail:
	TouchEnableIrq();
	TOUCH_LOG("I2C fail with addr(0x%x)\n", Addr);
	return count;
}

static ssize_t show_i2c_control(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;

	ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&rData,
				sizeof(Addr), sizeof(rData));
	if (ret < 0)
		goto i2c_fail;

	WRITE_SYSBUF(buf, ret, "Address[0x%x] Value[0x%x]\n", Addr, rData);

i2c_fail:
	TOUCH_LOG("I2C fail with addr(0x%x)\n", Addr);
	return ret;
}

static LGE_TOUCH_ATTR(i2c_control, S_IRUGO | S_IWUSR, show_i2c_control, store_i2c_control);


static struct attribute *sn280h_attribute_list[] = {
	&lge_touch_attr_Model_Info.attr,
	&lge_touch_attr_version_read.attr,
	&lge_touch_attr_Touch_Reset.attr,
	&lge_touch_attr_Set_vp.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_raw_data.attr,
	&lge_touch_attr_i2c_control.attr,
	NULL,
};


int Sn280h_Initialize(TouchDriverData *pDriverData)
{
	TOUCH_FUNC();
	Sn280h_Reset();
	return TOUCH_SUCCESS;
}

void Sn280h_Reset(void)
{
	TouchSetGpioReset(0);
	mdelay(1);
	TouchSetGpioReset(1);
	mdelay(2);
	TouchSetGpioReset(0);
	mdelay(1);
	TouchSetGpioReset(1);
	mdelay(20);
	TOUCH_LOG("SN280H was reset\n");
	temp_hover_status = 0;
	knock_flag = 0;
	TouchEnableIrq();
}


int Sn280h_InitRegister(void)
{
	TOUCH_FUNC();
	return TOUCH_SUCCESS;
}

/* LGE_BSP_COMMON : branden.you@lge.com_20141106 : */
static void Sn280h_ClearInterrupt(void)
{
	return;
}

static int get_lpwg_data(TouchReadData *pData, u8 tap_count)
{
	int ret = TOUCH_SUCCESS;
	u16 rData[2] = {0,};
	u16 Addr = 0;
	int i = 0;

	for (i = 0; i < tap_count; i++) {
		Addr = REG_KNOCK_DATA(i);
		ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)rData,
				sizeof(Addr), sizeof(rData));
		if (ret < 0) {
			TOUCH_ERR("KNOCK ON/CODE(%x) Data read Fail!\n", Addr);
			goto i2c_fail;
		}

		pData->knockData[i].x = rData[0];
		pData->knockData[i].y = rData[1];
		TOUCH_LOG("TAB DATA TEST = [%d %d]\n", rData[0], rData[1]);
	}

	return ret;

i2c_fail:
	ret = TOUCH_FAIL;
	return  ret;
}

int Sn280h_InterruptHandler(TouchReadData *pData)
{
	TouchFingerData *pFingerData = NULL;
	status_reg_u ts_status;
	data_reg_t ts_data;
	finger_t info[MAX_FINGER_NUM];
	u16 Addr = 0;
	u16 rData = 0;
	int ret = 0;

	pData->type = DATA_UNKNOWN;
	pData->count = 0;

	memset(info, 0x00, sizeof(info));
	memset(&ts_data, 0x00, sizeof(data_reg_t));

	if (g_miscInitialize == 0) {
		if (sys_chmod((const char __user *)"/dev/sn310m_dist", 666) < 0) {
			TOUCH_LOG("failed to change the \"/dev/sn310m_dist\" permission.\n");
		} else {
			TOUCH_LOG("succeeded to change the \"/dev/sn310m_dist\" permission.\n");
			g_miscInitialize = 1;
		}
	}

	Addr = REG_TS_STATUS;
	ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&ts_status.uint,
			sizeof(Addr), sizeof(status_reg_u));
	if (ret < 0)
		goto i2c_fail;

	if (ts_status.bits.ts_cnt <= MAX_FINGER_NUM && !knock_flag) {
		/* Touch Data */
		static u8 finger_status[MAX_FINGER_NUM] = {0,};
		static u8 prev_cnt = 0;
		u8 cnt = 0;
		pData->type = DATA_FINGER;

		/* Touch Press */
		for (cnt = 0; cnt < ts_status.bits.ts_cnt; cnt++) {
			Addr = REG_TS_DATA(cnt);
			ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&ts_data.packet0,
					sizeof(Addr), sizeof(data_reg_t));
			if (ret < 0) {
				goto i2c_fail;
			}

			info[cnt].id = ts_data.packet0 >> 12;
			info[cnt].x = ts_data.packet0 & 0xfff;
			info[cnt].y = ts_data.packet1 & 0xfff;
			info[cnt].area = ts_data.packet2 & 0xfff;
			info[cnt].pressure = ((ts_data.packet1 >> 8) & 0x00f0) + (ts_data.packet2 >> 12);
			if(info[cnt].pressure == 255) {
				info[cnt].pressure = 254;
			}

			pFingerData = &pData->fingerData[pData->count];
			pFingerData->status = FINGER_PRESSED;
			pFingerData->id = info[cnt].id;
			pFingerData->x = info[cnt].x;
			pFingerData->y = info[cnt].y;
			pFingerData->width_major = 15;
			pFingerData->width_minor = 10;
			pFingerData->orientation = 1;
			pFingerData->pressure = info[cnt].pressure;
			pData->count++;

			finger_status[info[cnt].id] = 1;
			/* check the released finger */
			if (prev_cnt == 2 && ts_status.bits.ts_cnt == 1) {
				finger_status[1 - info[cnt].id] = 0;
			}
		}

		prev_cnt = cnt;
		if (ts_status.bits.ts_cnt == 0) {
			memset(finger_status, 0x00, sizeof(finger_status));
		}

		/* Touch Release */
		for (cnt = 0; cnt < MAX_FINGER_NUM; cnt++) {
			if (finger_status[cnt] == 0) {
				pFingerData = &pData->fingerData[cnt];
				pFingerData->id = info[cnt].id;
				pFingerData->status = FINGER_RELEASED;
			}
		}

	} else if (knock_flag) {
		/* Knock on/code data - knock on,code */
		Addr = LPWG_STATUS_REG;
		ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&rData,
				sizeof(Addr), sizeof(rData));
		if (ret < 0)
			goto i2c_fail;

		TOUCH_LOG("rData = %x\n", rData);
		rData >>= 8;

		if (rData == 1) {
			pData->type = DATA_KNOCK_ON;
			/* need to know knock on Position */
			ret = get_lpwg_data(pData, 2);
			TOUCH_LOG("Knock on Occur\n");
			return TOUCH_SUCCESS;
		} else if (rData == 2) {
			pData->type = DATA_KNOCK_CODE;
			ret = get_lpwg_data(pData, tap_count);
			pData->count = tap_count;
			TOUCH_LOG("Knock code Occur\n");
			return TOUCH_SUCCESS;
		} else {
			TOUCH_LOG("Invalid lpwg status\n");
		}
	}
	return TOUCH_SUCCESS;

i2c_fail:
	TOUCH_LOG("I2C fail with addr(0x%x)\n", Addr);
	ret = TOUCH_FAIL;
	return ret;
}

int Sn280h_ReadIcFirmwareInfo(TouchFirmwareInfo *pFwInfo)
{
	int ret = TOUCH_SUCCESS;
	u16 addr = 0;
	u16 rData = 0;

	addr = TSC_FLASH_FW_VER_POS;
	ret = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rData,
			sizeof(addr), sizeof(rData));
	if (ret < 0)
		goto i2c_fail;
	TOUCH_LOG("IC FIRMWARE VERSION %x\n", rData);

	/* NSM - Temporary code */
	pFwInfo->isOfficial = 1;

	/* Test Firmware version didn't update */
	if (0xa0 <= rData && rData <= 0xFF) {
		pFwInfo->isOfficial = 0;
	}
	pFwInfo->version = rData;
//	pFwInfo->modelID = 0; ex)????
//	pFwInfo->moduleMakerID = 0; ex) suntel, tovis, lgit
//	pFwInfo->moduleVersion = 0; ex) product module version

	return ret;

i2c_fail:
	TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, rData);
	ret = TOUCH_FAIL;
	return ret;
}

int Sn280h_GetBinFirmwareInfo(char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	struct i2c_client *client = Touch_Get_I2C_Handle();
	const struct firmware *fw = NULL;
	int ret = TOUCH_SUCCESS;
	char *pFwFilename = NULL;
	u8 *pFw = NULL;
	
	TOUCH_FUNC();

	if( pFilename == NULL ) {
		if (lge_get_board_revno() > HW_REV_A) {
			pFwFilename = (char *)defaultFirmwareRevB;
		} else {
			pFwFilename = (char *)defaultFirmware;
		}
		TOUCH_LOG("Revision : %d %s\n",
			lge_get_board_revno(), lge_get_board_rev());
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if (ret < 0) {
		TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", ret);
		ret = TOUCH_FAIL;
		goto earlyReturn;
	}
	pFw = (u8 *)(fw->data);
	pFwInfo->isOfficial = 1;
	pFwInfo->version = pFw[TSC_FLASH_FW_VER_POS]; /* release version */

	TOUCH_LOG("BIN Firmware Official = %d\n", pFwInfo->isOfficial);
	TOUCH_LOG("BIN Firmware Version = 0x%04X\n", pFwInfo->version);

	/* Free firmware image buffer */
	release_firmware(fw);

earlyReturn:
	return ret;
}
static int FlashStartSequnce(u8 flashMemType, u8 OpMode)
{
    int result = TOUCH_SUCCESS;
    u16 addr = 0;
    u16 wdata = 0;

    if(flashMemType == E_MEM_TYPE_EEPROM)
    {
        addr = REG_CMD_EER_PDOWN;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_RESET;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_CSCON;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
    }
    else
    {
        addr = REG_CMD_FLASH_CON_EN;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

    }

    if(flashMemType == E_MEM_TYPE_EEPROM)
    {
        if(OpMode == E_FLASH_OPMODE_READ)
        {
            addr = REG_CMD_EER_MODE;
            wdata = 0x0000;
            if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
                goto fail;
        }
    }
    else
    {
        if(OpMode != E_FLASH_OPMODE_READ)
        {
            addr = REG_CMD_FLASH_AUTH;
            wdata = 0x0000;
            if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
                goto fail;
        }
    }
    return result;
fail:
    TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, wdata);
    result = TOUCH_FAIL;
    return result;
}
static int FlashEndSequence(u8 flashType)
{
    u16 addr = 0;
    u16 wdata = 0;
    if(flashType == E_MEM_TYPE_EEPROM)
    {
        addr = REG_CMD_EER_CSCON;
        wdata = 0x0000;
    }
    else
    {
        addr = REG_CMD_FLASH_AUTH;
        wdata = 0x0001;
    }
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
        return TOUCH_FAIL;

    return TOUCH_SUCCESS;
}

/* To do : function rename */
static int SetISPMode(void)
{
    int result = TOUCH_SUCCESS;
    u16 addr = 0;
    u16 wdata = 0;

    addr = REG_ISP_MODE;
    wdata = 0x0001;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
        goto fail;

    addr = REG_ISP_MODE;
    wdata = 0x0002;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0) 
        goto fail;

    addr = REG_ISP_MODE_BUS;
    wdata = 0x0000;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
        goto fail;

    addr = REG_ISP_MODE_ENABLE;
    wdata = 0xFFFF;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
        goto fail;

    return TOUCH_SUCCESS;
fail:
    TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, wdata);
    result = TOUCH_FAIL;
    return result;
}

static int FirmwareUpgradeCheck(u8 *flashType)
{
    int result = TOUCH_SUCCESS;
    u16 rdata = 0;
    u16 flashAddr = 0;
    //u16 fileVer = 0;
    u8 flashMemType = E_MEM_TYPE_EFLASH;

    /*is it need check;*/
    //mutex_lock();

    /* Set ISP mode */
    if(SetISPMode() != TOUCH_SUCCESS)
    {
        TOUCH_LOG("Set ISP Mode fail. \n");
         goto fail;
    }
    
    /* Check flash type. */
    flashAddr = REG_ISP_MEM_TYPE;
    if( Semisense_I2C_Read((u8 *)&flashAddr, (u8 *)&rdata, sizeof(flashAddr), sizeof(rdata)) < 0)
    {
        TOUCH_LOG("I2C read fail (flashAddr 0x%x)\n", flashAddr);
        goto fail;
    }

    if( rdata == REG_ISP_VAL_ERROR )
    {
        TOUCH_LOG("ISP memory type error(0xDEAD)\n");
        goto fail;
    }
    else
    {
        flashMemType = (rdata == REG_ISP_VAL_EEPROM) ? E_MEM_TYPE_EEPROM : E_MEM_TYPE_EFLASH;
        TOUCH_LOG("ISP memory type [%s]\n", (flashMemType == E_MEM_TYPE_EEPROM) ? "EEPROM" : "EFLASH");
        *flashType = flashMemType;
    }

    if(FlashStartSequnce(flashMemType, E_FLASH_OPMODE_READ) != TOUCH_SUCCESS)
    {
        TOUCH_LOG("Flash sequence fail(operation mode = E_FLASH_OPMODE_READ)\n");
        goto fail;
    }
    /* Read operation. */
    
    return result;
fail : 
    result = TOUCH_FAIL;
    //Sn280h_Reset();
    return result;
}

static int FirmwareErase(u8 flashType)
{
    int result = TOUCH_SUCCESS;
    u16 addr = 0;
    u16 rdata = 0;
    u16 wdata = 0;
    u8 retry = 0;

    /* Start of Erase Operation */
    TOUCH_LOG("[ERASE] START. \n");
	if(FlashStartSequnce(flashType, E_FLASH_OPMODE_ERASE) != TOUCH_SUCCESS)
    {
        TOUCH_LOG("Flash start sequnce fail.\n");
        return TOUCH_FAIL;
    }
    
    if(flashType == E_MEM_TYPE_EFLASH)
    {
        addr = REG_CMD_FLASH_COMMAND;
        wdata = 0x0002;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
            
        addr = 0x0000;
        wdata = 0xFFFF;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
        
        addr = REG_CMD_FLASH_BUSY;
        wdata = 0x8000;
        retry = 0;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
        do
        {
            mdelay(10);
            if(Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata)) < 0)
                TOUCH_LOG("Busy check I2C read fail (retry = %d)\n", retry);
            if(retry)
                TOUCH_LOG("[retry_%d] Busy flag (read data_0x%x) at (addr_0x%x)\n", retry, rdata, addr);
            retry++;
        }while((rdata & 0x8000) && (retry < BUSY_CHECK_RETRY_COUNT ));
    }
    else
    {
    TOUCH_LOG("[ERASE] START eeprom. \n");
        addr = REG_CMD_EER_XPROT;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
        
        addr = REG_CMD_EER_MODE;
        wdata = 0x0007;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = 0x0000;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_XEN;
        wdata = 0x0001;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = 0x0000;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_STATE;
        wdata = 0x0000;
        retry = 0;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        do
        {
            mdelay(10);
            if(Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata)) < 0)
                TOUCH_LOG("Busy check I2C read fail (retry = %d)\n", retry);
            if(retry)
                TOUCH_LOG("[retry_%d] Busy flag (read data_0x%x) at (addr_0x%x)\n", retry, rdata, addr);
            retry++;
        }while(((rdata & 0x0004) == 0) && (retry < BUSY_CHECK_RETRY_COUNT ));
    
        addr = REG_CMD_EER_XEN;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_XPROT;
        wdata = 0x0001;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_MODE;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
    TOUCH_LOG("[ERASE] eeprom. \n");
    }

   if(retry >= BUSY_CHECK_RETRY_COUNT)
    {
        TOUCH_LOG("FW upgrade erase busy check fail.\n");
        goto fail;
    }
    
    if(FlashEndSequence(flashType) != TOUCH_SUCCESS) 
    {
        TOUCH_LOG("Flash end sequnce fail.\n");
        return TOUCH_FAIL;
    }
    /* End of Erase Operation */
    TOUCH_LOG("[ERASE] END. \n");
    
    return TOUCH_SUCCESS;
fail:
    TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, wdata);
    result = TOUCH_FAIL;
    return result;
}

static int FirmwareProgram(const u8 *fwData, int fwSize, u8 flashType)
{
	int result = TOUCH_SUCCESS;
	u16 wdata = 0;
	u16 rdata = 0;
	u16 addr = 0;
	u16 retry = 0;
	u16 flashAddr = 0;

	/* Start of Program Operation */
	TOUCH_LOG("[PROGRAM] Start.\n");

	if (FlashStartSequnce(flashType, E_FLASH_OPMODE_WRITE) != TOUCH_SUCCESS) {
		TOUCH_LOG("START Flash sequence fail(operation mode = E_FLASH_OPMODE_READ)\n");
		goto fail;
	}

	if (flashType == E_MEM_TYPE_EFLASH) {
		addr = REG_CMD_FLASH_COMMAND;
		wdata = 0x0000;
		if (Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
			goto fail;

		for (flashAddr = 0; flashAddr < fwSize; flashAddr += 2) {
			u16 wAddr = flashAddr;
			if(Semisense_I2C_Write((u8 *)&wAddr, (u8 *)&fwData[flashAddr], sizeof(wAddr), 2) < 0)
				goto i2c_fail;
			addr = REG_CMD_FLASH_BUSY;
			rdata = 0x8000;
			retry = 0;

			do {
				/* if it failed once then, waiting time will be increased */
				if (retry)
					mdelay(100);
				if (Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata)) < 0)
					TOUCH_LOG("Busy check I2C read fail retry(%d)\n", retry);
				if (retry)
					TOUCH_LOG("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, addr);    
				retry++;
			} while((rdata & 0x8000) && (retry < BUSY_CHECK_RETRY_COUNT));
		
			if(retry >= BUSY_CHECK_RETRY_COUNT)
			{
			TOUCH_LOG("FW Upgrade Program Busy Check Fail.\n");
			goto fail;
			}
		}
	} else if (flashType == E_MEM_TYPE_EEPROM) {
		u16 pageIndex = 0;
		u16 pageOffset = 0;
		u16 numOfPage = (fwSize + TSC_EEPROM_PAGE_SIZE - 1) / TSC_EEPROM_PAGE_SIZE;
		u16 targetAddr = 0;
		u16 wAddr = 0;

		addr = REG_CMD_EER_XPROT;
		wdata = 0x0000;
		if (Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
			goto i2c_fail;

		addr = REG_CMD_EER_XEN;
		wdata = 0x0000;
		if (Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
			goto i2c_fail;

		addr = REG_CMD_EER_MODE;
		wdata = 0x0008;
		if (Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
			goto i2c_fail;

		addr = REG_CMD_EER_EXTEND;
		wdata = 0x0000;
		if (Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
			goto i2c_fail;

		for (pageIndex = 0; pageIndex < numOfPage; pageIndex++) {
			u16 wLen = (pageIndex == numOfPage - 1) ? \
				(fwSize - (numOfPage - 1) * TSC_EEPROM_PAGE_SIZE) : TSC_EEPROM_PAGE_SIZE;
			for (pageOffset = 0; pageOffset < wLen; pageOffset += 2) {
				targetAddr = pageIndex * TSC_EEPROM_PAGE_SIZE + pageOffset;
				wAddr = targetAddr;
				if (Semisense_I2C_Write((u8 *)&wAddr, (u8 *)&fwData[targetAddr],sizeof(wAddr), 2) < 0) {
					goto i2c_fail;
				}
			}

			addr = REG_CMD_EER_XEN;
			wdata = 0x0001;
			if(Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0) 
			goto i2c_fail;

			addr = (pageIndex * TSC_EEPROM_PAGE_SIZE);
			wdata = 0x0000; 
			if(Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0) 
			goto i2c_fail;

			addr = REG_CMD_EER_STATE;
			rdata = 0x0000;
			retry = 0;
			do {
				/* if it failed once then, waiting time will be increased */ 
				if (retry)
					mdelay(30);
#if 0
				if (Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata)) < 0)
					TOUCH_LOG("Busy check I2C read fail retry(%d)\n", retry);

				if (retry)
					TOUCH_LOG("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, addr);
#else
			/* May be log bad effect to firmware updatae. */
				Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata));
#endif
				retry++;
			} while(((rdata & 0x0004) == 0) && (retry < BUSY_CHECK_RETRY_COUNT)); /* check busy flag & retry count */

			if (retry >= BUSY_CHECK_RETRY_COUNT) {
				TOUCH_LOG("FW Upgrade Program Busy Check Fail.\n");
				goto fail;
			}

			addr = REG_CMD_EER_XEN;
			wdata = 0x0000;
			if (Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
				goto i2c_fail;
			if (pageIndex % 64 == 0 || pageIndex == numOfPage - 1) {
				TOUCH_LOG("FW Upgrade Write Page(%d) of Total(%d) OK!\n", pageIndex, numOfPage);
			}
		}

		addr = REG_CMD_EER_XPROT;
		wdata = 0x0001;
		if (Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
		    goto i2c_fail;

		addr = REG_CMD_EER_MODE;
		wdata = 0x0000;
		if (Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
		    goto i2c_fail;
	}

	if (FlashEndSequence(flashType) != TOUCH_SUCCESS) {
		TOUCH_LOG("START Flash sequence fail\n");
		goto fail;
	}
	/* Start of Program Operation */
	TOUCH_LOG("[PROGRAM] END.\n");
	return result;

i2c_fail:
	TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, wdata);
fail:
	result = TOUCH_FAIL;
	return result;
}

static int DoUpgrade(const struct firmware *fw_img, u8 flashType)
{
	int result = TOUCH_SUCCESS;
	u8 *fwData = NULL;
	u16 addr = 0;
	u16 flashAddr = 0;
	int fwSize = 0;
	u16 rdata = 0;
	u16 wdata = 0;
	fwData = (u8 *)(fw_img->data);
	fwSize = fw_img->size;

	if (SetISPMode() != TOUCH_SUCCESS) {
		TOUCH_ERR("ISP set fail.\n");
		goto fail;
	}

	if (FirmwareErase(flashType) != TOUCH_SUCCESS) {
		TOUCH_ERR("ERASE fail.\n");
		goto fail;
	}
	
	if (FirmwareProgram(fwData, fwSize, flashType) != TOUCH_SUCCESS) {
		TOUCH_ERR("Programming fail.\n");
		goto fail;
	}
	/* Start of Verify Operation */
	TOUCH_LOG("[VERIFY] Start.\n");  

	if (FlashStartSequnce(flashType, E_FLASH_OPMODE_READ) != TOUCH_SUCCESS) {
		TOUCH_LOG("Flash start sequence fail(operation mode = E_FLASH_OPMODE_READ)\n");
		goto fail;
	}

	for (flashAddr = 0; flashAddr < fwSize; flashAddr += 2) {
		u16 wAddr = flashAddr;
		u16 data = ((fwData[flashAddr]) | (fwData[flashAddr + 1] << 8));

		if(Semisense_I2C_Read((u8 *)&wAddr, (u8 *)&rdata, sizeof(wAddr), sizeof(rdata)) < 0)
			goto fail;
		if(data != rdata) {
		TOUCH_ERR("[VERIFY] Fw upgrade verify fail. data[0x%0x] at Addr [%d] \n", data, flashAddr);
		goto fail;    
		}
	}
	
	if (FlashEndSequence(flashType) != TOUCH_SUCCESS) {
		TOUCH_LOG("Flash end sequence fail(operation mode = E_FLASH_OPMODE_READ)\n");
		goto fail;
	}
	
	addr = REG_ISP_MODE;
	wdata = 0x0001;
	if (Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) <0) {
		TOUCH_ERR("Fw Upgrade Final  Step I2C Fail\n");
		goto fail;
	}
	TOUCH_LOG("[VERIFY] END.\n");  
	
	return result;
fail : 
	result = TOUCH_FAIL;
	//Add reset
	return result;
}

static int FirmwareUpgrade(const struct firmware *fw_img)
{
	int result = TOUCH_SUCCESS;
	u8 flashType = 0;

	if (FirmwareUpgradeCheck(&flashType) == TOUCH_SUCCESS) {
		TOUCH_LOG("==== [START] upgrading TSC F/W ... ====\n");
		if (DoUpgrade(fw_img, flashType) == TOUCH_SUCCESS) {
			TOUCH_LOG("==== [DONE] upgrading TSC F/W ... ====\n");
		} else {
		result = TOUCH_FAIL;
		}
	} else {
		result = TOUCH_FAIL;
	}

	return result;
}

int Sn280h_UpdateFirmware(char *pFilename )
{
	struct i2c_client *client = Touch_Get_I2C_Handle();
	const struct firmware *fw = NULL;
	char *pFwFilename = NULL;
	int ret = TOUCH_SUCCESS;
	u8 *pBin = NULL;

	TOUCH_FUNC();

	/* Select firmware */
	if (pFilename == NULL) {
		if (lge_get_board_revno() > HW_REV_A) {
			pFwFilename = (char *)defaultFirmwareRevB;
		} else {
			pFwFilename = (char *)defaultFirmware;
		}
		TOUCH_LOG("Revision : %d %s\n",
			lge_get_board_revno(), lge_get_board_rev());
	} else {
		pFwFilename = pFilename;
	}
	TOUCH_LOG("Firmware file name = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file*/
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if (ret < 0) {
		TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", ret);
		ret = TOUCH_FAIL;
		return ret;
	}
	pBin = (u8 *)(fw->data);

	/* Do firmware upgrade */
	ret = FirmwareUpgrade(fw);
	if(ret != TOUCH_SUCCESS) {
		TOUCH_ERR("Failed at FirmwareUpgrade() ( error = %d )\n", ret);
		ret = TOUCH_FAIL;
	}

	/* Free firmware image buffer */
	release_firmware(fw);
	return ret;
}

int Sn280h_SetLpwgMode(TouchState newState, LpwgSetting  *pLpwgSetting)
{
	struct i2c_client *client = Touch_Get_I2C_Handle();
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int ret = TOUCH_SUCCESS;
	unsigned short Addr = LPWG_STATUS_REG;
	unsigned short wData = 0;

	TOUCH_FUNC();
	gDeviceData.deviceState = newState;

	if (gpio_get_value(TOUCH_GPIO_RESET) == 0) {
		TOUCH_LOG("Reset pin is low %d", gpio_get_value(TOUCH_GPIO_RESET));
		Sn280h_Reset();
	}

	if (newState == STATE_NORMAL) {
		TOUCH_LOG("[NSM]SN280h was changed to NORMAL\n");

	} else if (newState == STATE_KNOCK_ON_ONLY) {
		knock_flag = 1;

		wData = 0x0021;
		ret = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData,
				sizeof(Addr), sizeof(wData));
		if (ret < 0)
			goto i2c_fail;

		TOUCH_LOG("SN280h was changed to STATE_KNOCK_ON_ONLY\n");
		TouchEnableIrq();

	} else if (newState == STATE_KNOCK_ON_CODE) {
		tap_count = pLpwgSetting->tapCount;
		knock_flag = 1;

		wData = (tap_count << 4) | 0x0003;
		ret = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData,
				sizeof(Addr), sizeof(wData));
		if (ret < 0)
			goto i2c_fail;

		TouchEnableIrq();
		TOUCH_LOG("SN280h was changed to STATE_KNOCK_ON_CODE tap_count : %x\n", wData);

	} else if (newState == STATE_NORMAL_HOVER) {
		//wData = 0x0404;
		//Semisense_I2C_Write(,(u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData));
		TOUCH_LOG("Set hover on mode.\n");
		/* To do List */

	/* NSM - Case is power key lcd off in calling*/
	} else if (newState == STATE_HOVER) {
		if (pDriverData->reportData.hover == 1) {
			if (pLpwgSetting->mode == 1) {
			}
			if (pLpwgSetting->mode == 2) {
			}
			if (pLpwgSetting->mode == 3) {
			}
		}

	} else if (newState == STATE_OFF) {
		TouchDisableIrq();
		TouchSetGpioReset(0);
		TOUCH_LOG("SN280h was changed to STATE_OFF\n");

	} else {
		TOUCH_ERR("Unknown State %d", newState);

	}

	return ret;
i2c_fail:
	TOUCH_LOG("I2C fail with addr(0x%x)\n", Addr);
	ret = TOUCH_FAIL;
	return ret;
}

void Sn280h_sd_write ( char *data , int time)
{
	int fd;
	char *fname = "/mnt/sdcard/touch_self_test.txt";
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND|O_SYNC, 0644);
	if (fd >= 0) {
		if (time > 0) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, 64, "\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
					my_date.tm_mon + 1,my_date.tm_mday, my_date.tm_hour,
					my_date.tm_min, my_date.tm_sec,
					(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}

	set_fs(old_fs);
}


int Sn280h_DoSelfDiagnosis(int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int offset = 0;

	/* setting initial value */
	*pRawStatus = TOUCH_SUCCESS;
	*pChannelStatus = TOUCH_SUCCESS;

	/* only write time */
	Sn280h_sd_write(pBuf, 1);
	msleep(30);

	offset += read_channel_data(pBuf, CAP_REFERENCE);
	Sn280h_sd_write(pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	/* Additional Test */
	WRITE_SYSBUF(pBuf, offset, "======ADDITIONAL======\n");
	Sn280h_sd_write(pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	*pDataLen = 0;
	return TOUCH_SUCCESS;
}

int Sn280h_DoSelfDiagnosis_Lpwg(int *lpwgStatus, char *pBuf, int bufSize, int *pDataLen)
{
	return TOUCH_SUCCESS;
}


static void Sn280h_NotifyHandler(TouchNotify notify, int data)
{
	switch (notify) {
		case NOTIFY_CALL:
			TOUCH_LOG("Sn280h NOTIFY_CALL Not Implemented !!\n");
			break;

		case NOTIFY_Q_COVER:
			TOUCH_LOG("Quick Cover was notified ( data = %d )\n", data);
			break;

		default:
			TOUCH_ERR("Invalid notification ( notify = %d )\n", notify);
			break;
	}

	return;
}
/* This code is TestCode */

int write_to_file(const char *path, int i)
{
	int fd;
	char buf[20];
	size_t count;

	TOUCH_LOG("%s : Called by Hall IC\n", __FUNCTION__);

	fd = sys_open(path, O_WRONLY, 0);

	if(fd == -1) {
		TOUCH_ERR("Write to file failed - %s\n", path);
		return -1;
	}

	sprintf(buf, "%d", i);

	count = sys_write(fd, buf, strlen(buf));

	sys_close(fd);

	return count;
}

static void Sn280h_PowerOn(int isOn)
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

TouchDeviceControlFunction Sn280h_Func = {
	.Power = Sn280h_PowerOn,
	.Initialize = Sn280h_Initialize,
	.Reset = Sn280h_Reset,
	.InitRegister = Sn280h_InitRegister,
	.ClearInterrupt = Sn280h_ClearInterrupt,
	.InterruptHandler = Sn280h_InterruptHandler,
	.ReadIcFirmwareInfo = Sn280h_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = Sn280h_GetBinFirmwareInfo,
	.UpdateFirmware = Sn280h_UpdateFirmware,
	.SetLpwgMode = Sn280h_SetLpwgMode,
	.DoSelfDiagnosis = Sn280h_DoSelfDiagnosis,
	.DoSelfDiagnosis_Lpwg = Sn280h_DoSelfDiagnosis_Lpwg,
	.device_attribute_list = sn280h_attribute_list,
	.NotifyHandler = Sn280h_NotifyHandler,
};


/* End Of File */
