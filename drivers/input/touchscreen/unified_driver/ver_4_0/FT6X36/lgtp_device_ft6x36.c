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
 *    File      : lgtp_device_dummy.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[FT6X36]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>

#include <linux/input/unified_driver_4/lgtp_common_driver.h>
#include <linux/input/unified_driver_4/lgtp_model_config_misc.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_i2c.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_misc.h>
#include <linux/input/unified_driver_4/lgtp_device_ft6x36.h>
#include "focaltech_ctl.h"
#include "ft6x06_ex_fun.h"

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
#if defined( TOUCH_MODEL_E1)
static const char defaultFirmware[] = "focaltech/e1/FT6x36_Ver0x0600_20151014_app.img";
#endif

static struct focaltech_ts_data *ts = NULL;

static int temp_hover_status = 0;
//static int knock_flag = 0;
static TouchDriverData *gTouchDriverData;
static int tap_count = 0;
int enable_iic_dev = 0;

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
static void Ft6x36_Reset(void);
static int Ft6x36_Set_Hover(int on);
static int Ft6x36_Set_KnockCode(u8 tap_count);


/****************************************************************************
* Local Functions
****************************************************************************/

static int FirmwareUpgrade(const struct firmware *fw_img)
{
    u8 * pbt_buf = NULL;

    int i_ret = 0;
    //int fw_len = fw_img->size;

    /*FW upgrade*/
    pbt_buf = (u8*)fw_img->data;

    /*call the upgrade function*/
    //i_ret =  fts_ctpm_fw_upgrade(ts->client, pbt_buf, fw_len);
    if (i_ret != 0)
    {
        TOUCH_ERR("Firwmare upgrade failed. err=%d.\n", i_ret);
    }
    else
    {
        #ifdef AUTO_CLB
        fts_ctpm_auto_clb(ts->client);  /*start auto CLB*/
        #endif
    }
    return i_ret;
}

static ssize_t show_use_iic_dev(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;

    ret = sprintf(buf, "%u\n", enable_iic_dev);

    return ret;
}

static ssize_t store_use_iic_dev(TouchDriverData *pDriverData, const char *buf, size_t count)
{

    int value = 0;

    sscanf(buf, "%d", &value);

    if (value < 0 || value > 1) {
        TOUCH_LOG("Invalid enable_rmi_dev value:%d\n", value);
        return count;
    }

    TOUCH_LOG("enable_iic_dev:%u value: %d \n", enable_iic_dev ,value);

    if (enable_iic_dev==0 && value==1) {

#ifdef SYSFS_DEBUG
        ft6x06_create_sysfs(ts->client);
#endif
#ifdef FTS_CTL_IIC
        if (ft_rw_iic_drv_init(ts->client) < 0)
            TOUCH_ERR("[FTS] create fts control iic driver failed\n");
#endif
#ifdef FTS_APK_DEBUG
        ft6x06_create_apk_debug_channel(ts->client);
#endif
        enable_iic_dev=value;

    }
    else if(enable_iic_dev==1 && value==0){
/*to do disable debug func, please reboot the device*/
#if 0
#ifdef SYSFS_DEBUG
        ft6x06_release_sysfs(client);
#endif
#ifdef FTS_CTL_IIC
        ft_rw_iic_drv_exit();
#endif
#ifdef FTS_APK_DEBUG
        ft6x06_release_apk_debug_channel();
#endif

        enable_iic_dev=value;
#endif
    }
    return count;
}
static LGE_TOUCH_ATTR(enable_iic_dev, S_IRUGO | S_IWUSR, show_use_iic_dev, store_use_iic_dev);


static ssize_t show_Model_Info(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;

    WRITE_SYSBUF(buf, ret, "======== Model info ========\n");
    #if 0
    if( TouchReadGpioMakerId() == 0 )
    {
        TOUCH_LOG("Touch IC : LeadingUI\n");
        WRITE_SYSBUF(buf, ret, "Maker ID PIN: 0\n");
        WRITE_SYSBUF(buf, ret, "Module Product : SUNTEL\n");
        WRITE_SYSBUF(buf, ret, "Touch IC : LeadingUI\n");
        //return TOUCH_SUCCESS;
    }
    else if(TouchReadGpioMakerId() == 1 )
    {
        WRITE_SYSBUF(buf, ret, "Maker ID PIN: 1\n");
        WRITE_SYSBUF(buf, ret, "Module Product : LGIT\n");
        WRITE_SYSBUF(buf, ret, "Touch IC : Focaltech\n");
    }
    #endif
    return ret;
}
static LGE_TOUCH_ATTR(Model_Info, S_IRUGO | S_IWUSR, show_Model_Info, NULL);

static ssize_t show_version_read(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	u8 addr = 0x00;
	u8 rdata[10] = {0,};
	u8 wdata[10] = {0,};

	TOUCH_LOG("DMA TEST \n");
	ret = Touch_I2C_Read(addr, rdata, 10);
	if (ret < 0) {
		TOUCH_LOG("I2C FAIL\n");
	}

	ret = Touch_I2C_Write(addr, wdata, 10);
	if(ret < 0) {
		TOUCH_LOG("I2C FAIL\n");
	}

	return ret;
}
static LGE_TOUCH_ATTR(version_read, S_IRUGO | S_IWUSR, show_version_read, NULL);

static ssize_t show_Pincheck(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int val_gpio = 0;
	val_gpio = gpio_get_value(TOUCH_GPIO_INTERRUPT);
	TOUCH_LOG("[TOUCH] get GPIO_TOUCH_INT value  = %d\n", val_gpio);

	val_gpio = gpio_get_value(TOUCH_GPIO_RESET);
	TOUCH_LOG("[TOUCH] get GPIO_TOUCH_RESET value  = %d\n", val_gpio);

	return ret;
}
static LGE_TOUCH_ATTR(Pincheck, S_IRUGO | S_IWUSR, show_Pincheck, NULL);

static ssize_t show_knockon(TouchDriverData *pDriverData, char *buf)
{
    int ret = TOUCH_SUCCESS;
    return ret;
}
static LGE_TOUCH_ATTR(knockon, S_IRUGO | S_IWUSR, show_knockon, NULL);

static ssize_t show_Touch_Reset(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;
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
    int cmd = 0;
    sscanf(buf, "%d", &cmd);
    if(cmd == 1)
    {
        Ft6x36_Set_Hover(1);
    }
    else if(cmd == 0)
    {
        Ft6x36_Set_Hover(0);
    }
    else
    {
        TOUCH_LOG("Invalid HOVER command\n");
    }

    return count;
}
static LGE_TOUCH_ATTR(Set_vp, S_IRUGO | S_IWUSR, show_Set_vp, store_Set_vp);

static ssize_t show_delta(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;
    return ret;
}
static LGE_TOUCH_ATTR(delta, S_IRUGO | S_IWUSR, show_delta, NULL);

static ssize_t show_raw_data(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;
    return ret;
}
static LGE_TOUCH_ATTR(raw_data, S_IRUGO, show_raw_data, NULL);



static struct attribute *Ft6x36_attribute_list[] = {
    &lge_touch_attr_Model_Info.attr,
    &lge_touch_attr_version_read.attr,
    &lge_touch_attr_Pincheck.attr,
    &lge_touch_attr_Touch_Reset.attr,
    &lge_touch_attr_Set_vp.attr,
    &lge_touch_attr_delta.attr,
    &lge_touch_attr_raw_data.attr,
    &lge_touch_attr_knockon.attr,
    &lge_touch_attr_enable_iic_dev.attr,
    NULL,
};

static int Ft6x36_Initialize(TouchDriverData *pDriverData)
{
    struct i2c_client *client = Touch_Get_I2C_Handle();
    TOUCH_FUNC();
    ts = devm_kzalloc(&client->dev, sizeof(struct focaltech_ts_data), GFP_KERNEL);
    if (ts == NULL)
    {
        TOUCH_ERR("failed to allocate memory for device driver data\n");
        return TOUCH_FAIL;
    }
    gTouchDriverData = pDriverData;
    ts->client = client;
    return TOUCH_SUCCESS;
}

static void Ft6x36_Reset(void)
{
    TOUCH_FUNC();
    TouchSetGpioReset(0);
    msleep(10);
    TouchSetGpioReset(1);
    msleep(200);
    temp_hover_status = 0;
    ts->currState = STATE_NORMAL;
    TOUCH_LOG("Device was reset\n");
}

static int Ft6x36_InitRegister(void)
{
    /* IMPLEMENT : Register initialization after reset */
    TOUCH_FUNC();
    return TOUCH_SUCCESS;
}

static int get_lpwg_data(TouchReadData *pData, u8 tap_count)
{
    u8 i = 0;
    u8 buffer[50] = {0,};

    if(Touch_I2C_Read(FT_KNOCK_READ_DATA_REG, buffer, tap_count*4 + 2) == 0)
    {
        pData->count = buffer[1];
        TOUCH_LOG("[NSM]TAP COUNT = %d\n", buffer[1]);
    }
    else
    {
        TOUCH_ERR("KNOCK TAP DATA Read Fail.\n");
        goto error;
    }

    if(!buffer[1])
    {
        TOUCH_LOG("TAP COUNT = %d\n", buffer[1]);
        goto error;
    }

    for(i = 0; i< buffer[1]; i++)
    {
        pData->knockData[i].x = GET_X_POSITION(buffer[4*i+2], buffer[4*i+3]);
        pData->knockData[i].y = GET_Y_POSITION(buffer[4*i+4], buffer[4*i+5]);
        TOUCH_LOG("LPWG data [%d, %d]\n", pData->knockData[i].x, pData->knockData[i].y);
    }

    return TOUCH_SUCCESS;
error:
    return TOUCH_FAIL;
}

static int Ft6x36_InterruptHandler(TouchReadData *pData)
{
    TouchFingerData *pFingerData = NULL;

    int i = 0;
    int ret = -1;
    u8 touch_event=0;
    u8 buf[POINT_READ_BUF] = { 0 };
    u8 pointid = FT_MAX_ID;
    u8 touch_count = 0;
    u8 event_type = 0;
    static u8 pressure_change = 0;

    pData->type = DATA_UNKNOWN;
    pData->count = 0;
    /* read Interrupt status */
    ret = Touch_I2C_Read(FT_INT_STATUS, &event_type, sizeof(u8));
    if(ret <0)
    {
        TOUCH_ERR("read int status failed.\n");
        return TOUCH_FAIL;
    }
    switch(event_type)
    {
        case EVENT_ABS :
            ret = Touch_I2C_Read(0x00, buf, POINT_READ_BUF);
            if (ret < 0)
            {
                TOUCH_ERR("read touchdata failed.\n");
                return TOUCH_FAIL;
            }
            pData->type = DATA_FINGER;
            touch_count = buf[2]&0xf;
            pressure_change ^= 1;

            for (i = 0; i < 2; i++)
            {
                pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
                if (pointid >= FT_MAX_ID)
                    break;
                pFingerData = &pData->fingerData[pData->count];
                touch_event = buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
                                //buf 3+6*i
                pFingerData->x = (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) << 8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
                pFingerData->y = (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) << 8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
                if(touch_event== 0 ||touch_event==2) {
                    pFingerData->id  = pointid;
                    pFingerData->width_major = 15;
                    pFingerData->width_minor = 10;
                    pFingerData->orientation = 1;
					pFingerData->status = FINGER_PRESSED;
                    /* to prevent Hovering detected in CFW.*/
                    pFingerData->pressure =100 + pressure_change;
                    pData->count++;
                } else if (touch_event == 1) {
					pFingerData->status = FINGER_RELEASED;
				}
            }
        break;
        case EVENT_KEY :
            break;

        case EVENT_KNOCK_ON :
            pData->type = DATA_KNOCK_ON;
            TOUCH_LOG("[KNOCK ON] Event Type = %d\n", event_type);
            //Ft6x36_Set_KnockOn(client);
            break;

        case EVENT_KNOCK_CODE :
            pData->type = DATA_KNOCK_CODE;
            get_lpwg_data(pData, tap_count);
            TOUCH_LOG("[KNOCK CODE] Event Type = %d\n", event_type);;
            Ft6x36_Set_KnockCode(tap_count);
            break;

        case EVENT_KNOCK_OVER :
            pData->type = DATA_KNOCK_CODE;
            pData->knockData[0].x = 1;
            pData->knockData[0].y = 1;
            pData->knockData[1].x = -1;
            pData->knockData[1].y = -1;
            TOUCH_LOG("[KNOCK CODE OVER] Event Type = %d\n", event_type);
            break;

        case EVENT_HOVERING_NEAR :
            /*
            pData->type = DATA_HOVER_NEAR; 
            pData->hoverState = 0;
            TOUCH_LOG("[HOVERING NEAR] Event Type = %d\n", event_type);
            */
            break;

        case EVENT_HOVERING_FAR :
            /*
            pData->type = DATA_HOVER_FAR;
            pData->hoverState = 1;
            TOUCH_LOG("[HOVERING FAR] Event Type = %d\n", event_type);
            */
            break;
        default:
            TOUCH_LOG("[Unknown] Event Type = %d\n",event_type);
            break;
    }
    return TOUCH_SUCCESS;
}

static int Ft6x36_ReadIcFirmwareInfo( TouchFirmwareInfo *pFwInfo)
{
    int result = TOUCH_SUCCESS;
    u8 readData = 0;

    TOUCH_FUNC();

    /* IMPLEMENT : read IC firmware information function */
    pFwInfo->moduleMakerID = 0;
    pFwInfo->moduleVersion = 0;
    pFwInfo->modelID = 0;
    pFwInfo->isOfficial = 0;
    pFwInfo->version = 0;

    if(Touch_I2C_Read(FT6x06_REG_FW_VER, &readData, 1) != 0)
    {
        TOUCH_ERR("Firmware version read fail (0xA6)\n");
        result = TOUCH_FAIL;
    }
    else
    {
        pFwInfo->version = readData;
        pFwInfo->isOfficial = 1;
        TOUCH_LOG("IC Firmware Official = %d\n", pFwInfo->isOfficial);
        TOUCH_LOG("IC Firmware Version = 0x%02X\n", readData);
    }

    if(Touch_I2C_Read(FT6x06_PANEL_ID, &readData, 1) != 0)
    {
        TOUCH_ERR("Touch Panel read fail (0xA8)\n");
        result = TOUCH_FAIL;
    }
    else
    {
        TOUCH_LOG("[NSM]PANEL ID = %x", readData);
    }

    if(Touch_I2C_Read(FT6x36_REG_FW_REL, &readData, 1) != 0)
    {
        TOUCH_ERR("Firmware release version read fail. (0xAF)\n");
        result = TOUCH_FAIL;
    }
    else
    {
        TOUCH_LOG("[NSM]IC Firmware Release ID = %d", readData);
    }

    return result;
}

static int Ft6x36_GetBinFirmwareInfo(char *pFilename, TouchFirmwareInfo *pFwInfo)
{
    const struct firmware *fw = NULL;
    int ret = TOUCH_SUCCESS;
    char *pFwFilename = NULL;
    u8 *pFw = NULL;

    TOUCH_FUNC();

    if( pFilename == NULL )
    {
        pFwFilename = (char *)defaultFirmware;
    }
    else
    {
        pFwFilename = pFilename;
    }

    TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

    /* Get firmware image buffer pointer from file */
    ret = request_firmware(&fw, pFwFilename, &(ts->client->dev)/*&client->dev*/);
    if(ret < 0)
    {
        TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", ret);
        ret = TOUCH_FAIL;
        goto earlyReturn;
    }
    pFw = (u8 *)(fw->data);
    pFwInfo->isOfficial = 1;
    pFwInfo->version = pFw[0x10a];

    TOUCH_LOG("BIN Firmware Official = %d\n", pFwInfo->isOfficial);
    TOUCH_LOG("BIN Firmware Version = 0x%04X\n", pFwInfo->version);

    /* Free firmware image buffer */
    release_firmware(fw);

earlyReturn:
    return ret;
}

static int Ft6x36_UpdateFirmware( char *pFilename)
{
    const struct firmware *fw = NULL;
    int result = TOUCH_SUCCESS;
    char *pFwFilename = NULL;
    u8 *pBin = NULL;
    TOUCH_FUNC();

    /* Select firmware */
    if( pFilename == NULL )
    {
        pFwFilename = (char *)defaultFirmware;
    }
    else
    {
        pFwFilename = pFilename;
    }

    TOUCH_LOG("Firmware file name = %s \n", pFwFilename);

    /* Get firmware image buffer pointer from file*/
    result = request_firmware(&fw, pFwFilename, &(ts->client->dev));
    if( result )
    {
        TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", result);
        result = TOUCH_FAIL;
        return result;
    }
    pBin = (u8 *)(fw->data);

    /* Do firmware upgrade */
    result = FirmwareUpgrade(fw);
    if(result != TOUCH_SUCCESS)
    {
        TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", result);
        result = TOUCH_FAIL;
    }

    /* Free firmware image buffer */
    release_firmware(fw);
    return result;
}

static int Ft6x36_Set_KnockOn(void)
{
    u8 buf = 0;
    buf = 0x01;
    if (Touch_I2C_Write(FT_LPWG_CONTROL_REG, &buf, 1) < 0)
    {
        TOUCH_ERR("KNOCK_ON Enable write fail\n");
        return TOUCH_FAIL;
    }
    buf = 0x00;
    /*Add interrupt delay time set 0*/
    if(Touch_I2C_Write(FT_LPWG_INT_DELAY, &buf, 1) < 0)
    {
        TOUCH_ERR("KNOCK ON INT DELAY TIME write fail\n");
        return TOUCH_FAIL;
    }
    TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_ONLY\n");
    return TOUCH_SUCCESS;
}

static int Ft6x36_Set_KnockCode(u8 tap_count)
{
    u8 buf = 0;
    buf = 0x3;
    if (Touch_I2C_Write(FT_LPWG_CONTROL_REG, &buf, 1) < 0)
    {
       TOUCH_ERR("KNOCK_CODE Enable write fail\n");
       return TOUCH_FAIL;
    }

    if(Touch_I2C_Write(FT_MULTITAP_COUNT_REG, &(tap_count), 1) <0)
    {
        TOUCH_ERR("KNOCK_CODE Tab count write fail\n");
    }
    TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_CODE\n");

    return TOUCH_SUCCESS;
}


static int Ft6x36_Set_Hover(int on)
{
    u8 buf = 0;
    buf = on ? 0x01 : 0x00;

    if(Touch_I2C_Write(0xB0, &buf, 1)<0)
    {
        TOUCH_LOG("Hover on setting fail.\n");
        return TOUCH_FAIL;
    }
    else
    {
        TOUCH_LOG("Mode Changed to HOVER %s.\n", on ? "On" : "Off");
    }
    return TOUCH_SUCCESS;
}

static int Ft6x36_Set_PDN(void)
{
    u8 buf = 0;
    buf = 0x00;
    if (Touch_I2C_Write(FT_LPWG_CONTROL_REG, &buf, 1) < 0)
    {
        TOUCH_ERR("KNOCK_ON Enable write fail\n");
        return TOUCH_FAIL;
    }
    return TOUCH_SUCCESS;
}

static int Ft6x36_SetLpwgMode( TouchState newState, LpwgSetting  *pLpwgSetting)
{
    int result = TOUCH_SUCCESS;

    TOUCH_FUNC();
    ts->currState = newState;
    TOUCH_LOG("ts->currState = %d \n",ts->currState);
    TouchDisableIrq();
    switch(ts->currState)
    {
       case STATE_NORMAL :
            Ft6x36_Reset();
            TOUCH_LOG("FT6x36 was changed to NORMAL\n");
        break;

        case STATE_KNOCK_ON_ONLY :
            result = Ft6x36_Set_KnockOn();
            if(result < 0)
                TOUCH_LOG("FT6x36 failed changed to STATE_KNOCK_ON_ONLY\n");
            TOUCH_LOG("FT6x36 was changed to STATE_KNOCK_ON_ONLY\n");
            break;

        case STATE_KNOCK_ON_CODE :
            tap_count = pLpwgSetting->tapCount;
            result = Ft6x36_Set_KnockCode(tap_count);
            if(result < 0)
                TOUCH_LOG("FT6x36 failed changed to STATE_KNOCK_ON_CODE\n");
            TOUCH_LOG("FT6x36 was changed to STATE_KNOCK_ON_CODE\n");
            break;

        case STATE_NORMAL_HOVER :
            result = Ft6x36_Set_Hover(1);
            break;

        case STATE_HOVER :
            if(gTouchDriverData->reportData.hover == 1)
            {
                result = Ft6x36_Set_Hover(0);
                if( pLpwgSetting->mode == 1 )
                {
                    result = Ft6x36_Set_KnockOn();
                }
                if( pLpwgSetting->mode == 2 )
                {
                    result = Ft6x36_Set_KnockCode(tap_count);
                }
                if( pLpwgSetting->mode == 3 )
                {
                }
            }
            break;
        case STATE_OFF :
            result = Ft6x36_Set_PDN();
            if(result < 0)
                TOUCH_LOG("FT6x36 failed changed to STATE_OFF\n");
            TOUCH_LOG("FT6x36 was changed to STATE_OFF\n");
            break;
        default :
            TOUCH_ERR("Unknown State %d", newState);
            break;
    }
    TouchEnableIrq();
    return result;
}

static void Ft6x36_sd_write(char *data)
{
    int fd;
    char *fname = "/mnt/sdcard/touch_self_test.txt";

    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

    fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND|O_SYNC, 0644);

    if (fd >= 0)
    {
        sys_write(fd, data, strlen(data));
        sys_close(fd);
    }

    set_fs(old_fs);
}
static int Ft6x36_ChCap_Test(char* pBuf ,int* pRawStatus, int* pChannelStatus, int* pDataLen)
{
    u8 addr = FT6x36_DEVICE_MODE;
    u8 command = 0x40;
    u8 reply_data = 0;
    u8 i = 0;
    u8 channel_num = 36;
    int dataLen = 0;
    u8 temp_data[72] = {0,};
    int channel_data[36] = {0,};
    int channel_data_avr = 0;

    *pRawStatus = 0;
    *pChannelStatus = 0;
    dataLen += sprintf(pBuf + dataLen, "=====Test Start=====\n");
    dataLen += sprintf(pBuf + dataLen, "    -RawCap Value-    \n");
    TOUCH_LOG("=====Test Start=====\n");
    TOUCH_LOG("    -RawCap Value-\n    ");

    /* enter factory mode */
    if(Touch_I2C_Write(addr, &command, 1) < 0)
    {
        TOUCH_LOG("Device mode enter error (0x00)\n");
        goto fail;
    }
    else
    {
        if(Touch_I2C_Read(FT6x36_DEVICE_MODE, &reply_data, 1) < 0)
        {
            TOUCH_LOG("Device mode reply data read error (0x00)\n");
            goto fail;
        }
        else
        {
            if(reply_data != 0x40)
            {
                TOUCH_LOG("Device mode reply data error(%x)", reply_data);
                goto fail;
            }
        }
    }
    msleep(300);

    /* Current factory test mode */
    /* 0x00 : F_NORMAL 0x01 : F_TESTMODE_1 0x02 : F_TESTMODE_2 */
    addr =  FT6x36_FACTORY_MODE;
    if(Touch_I2C_Read(addr, &reply_data, 1) < 0)
    {
        TOUCH_LOG("Current factory test mode read error (0xAE)\n");
        goto fail;
    }
    else
    {
        if(reply_data == 0x80)
        {
            TOUCH_LOG("Current factory test mode reply data ok(%x)", reply_data);
        }
        else
        {
            TOUCH_LOG("Current factory test mode reply data error(%x)", reply_data);
            goto fail;
        }
    }

    command = 0x02;
    if(Touch_I2C_Write(addr, &command, 1) < 0 )
    {
        TOUCH_LOG("Current factory test mode set error (0xAE)\n");
        goto fail;
    }

    addr = FT6x36_START_SCAN;
    if(Touch_I2C_Read(addr, &reply_data, 1) < 0)
    {
        TOUCH_LOG(" Start scanf reply data error (0x08)\n");
        goto fail;
    }
    else
    {
        if(reply_data == 0x00)
        {
            TOUCH_LOG("Start scanf reply data ok(%x)", reply_data);
        }
        else
        {
            TOUCH_LOG("Start scanf reply data error(%x)", reply_data);
            goto fail;
        }
    }

    /* Start scan command */
    command = 0x01;
    if(Touch_I2C_Write(addr, &command, 1) < 0 )
    {
        TOUCH_LOG("Start scanf set error (0x08)\n");
        goto fail;
    }
    do
    {
        msleep(15);
        if(Touch_I2C_Read(addr, &reply_data, 1) < 0)
        {
            TOUCH_LOG("Start scanf set reply data error (0x08)\n");
            goto fail;
        }
    }while(reply_data != 0x00);
    TOUCH_LOG("Data scan complete.\n");

    addr = 0x33;
    command = 0x00;
    if(Touch_I2C_Write(addr, &command, 1) < 0 )
    {
        TOUCH_LOG("Start scanf set error (0x08)\n");
        goto fail;
    }

    addr = 0x39;
    if(Touch_I2C_Read(addr, temp_data, channel_num * 2/*channel number(36)*2 = 72 */) < 0)
    {
        TOUCH_LOG("Channel data read fail.(0x39)\n");
        goto fail;
    }

    for(i=0; i < channel_num; i++)
    {
        channel_data[i] = (temp_data[i*2] << 8) | temp_data[i*2+1];
        channel_data_avr += channel_data[i];
        TOUCH_LOG("channel_data[%d] = %d\n", i, channel_data[i]);
        if( (channel_data[i] < 0) || (channel_data[i] > 900) )
        {
            dataLen += sprintf(pBuf + dataLen, "[ERROR]Raw cap value  [%d] = %d\n", i, channel_data[i]);
            *pRawStatus = 1;
            *pChannelStatus = 1;
        }
        else
        {
            dataLen += sprintf(pBuf + dataLen, "Raw cap value [%d] = %d\n", i, channel_data[i]);
        }
    }
    //channel_data_avr = channel_data[i]/channel_num;
    dataLen += sprintf(pBuf + dataLen, "Channel data average = %d\n", channel_data_avr);
    TOUCH_LOG("channel data average = %d\n", channel_data_avr);

    addr = FT6x36_FACTORY_MODE;
    command = 0x80;
    if(Touch_I2C_Write(addr, &command, 1) < 0 )
    {
        TOUCH_LOG("Current factory test mode set error (0xae)\n");
        goto fail;
    }
    *pDataLen += dataLen;

    return TOUCH_SUCCESS;
fail:
    *pRawStatus = 1;
    *pChannelStatus = 1;

    dataLen += sprintf(pBuf + dataLen, "Self D Error\n");
    *pDataLen += dataLen;
    return TOUCH_FAIL;
}
static int Ft6x36_DoSelfDiagnosis(int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int dataLen = 0;

	/* CAUTION : be careful not to exceed buffer size */

	/* do implementation for self-diagnosis */
	Ft6x36_ChCap_Test(pBuf, pRawStatus, pChannelStatus, &dataLen);
	dataLen += sprintf(pBuf+dataLen,  "======== RESULT File =======\n");
	dataLen += sprintf(pBuf+dataLen, "Channel Status : %s\n", (*pRawStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	dataLen += sprintf(pBuf+dataLen,  "Raw Data : %s\n", (*pChannelStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");

	TOUCH_LOG("======== RESULT File =======\n");
	TOUCH_LOG("Channel Status : %s\n", (*pRawStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	TOUCH_LOG("Raw Data : %s\n", (*pChannelStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	TOUCH_LOG("Channel Status : %d\n", *pRawStatus);
	TOUCH_LOG("Raw Data : %d\n", *pChannelStatus);
	Ft6x36_sd_write(pBuf);
	return TOUCH_SUCCESS;
}

static void Ft6x36_PowerOn(int isOn)
{
    TOUCH_FUNC();
	if(isOn == 1) {
		//hwPowerOn ( MT6328_POWER_LDO_VGP1, VOL_3000, "TP" );
		TouchSetGpioReset(0);
		msleep(10);
		TouchSetGpioReset(1);
		msleep(300);
		TOUCH_LOG ( "turned on the power ( VGP1 )\n" );
	}
	else {
		//hwPowerDown ( MT6328_POWER_LDO_VGP1, "TP" );
	}
}

static void Ft6x36_ClearInterrupt(void)
{

}

static void Ft6x36_NotifyHandler(TouchNotify notify, int data)
{

}

TouchDeviceControlFunction Ft6x36_Func = {
    .Power                  = Ft6x36_PowerOn,
    .Initialize             = Ft6x36_Initialize,
    .Reset                  = Ft6x36_Reset,
    .InitRegister           = Ft6x36_InitRegister,
    .ClearInterrupt         = Ft6x36_ClearInterrupt,
    .InterruptHandler       = Ft6x36_InterruptHandler,
    .ReadIcFirmwareInfo     = Ft6x36_ReadIcFirmwareInfo,
    .GetBinFirmwareInfo     = Ft6x36_GetBinFirmwareInfo,
    .UpdateFirmware         = Ft6x36_UpdateFirmware,
    .SetLpwgMode            = Ft6x36_SetLpwgMode,
    .DoSelfDiagnosis        = Ft6x36_DoSelfDiagnosis,
    .device_attribute_list  = Ft6x36_attribute_list,
    .NotifyHandler          = Ft6x36_NotifyHandler,
};


/* End Of File */


