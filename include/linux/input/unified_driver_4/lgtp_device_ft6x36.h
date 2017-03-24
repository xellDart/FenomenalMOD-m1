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
 *    File  	: lgtp_device_ft6x36.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_FT6X36_H_ )
#define _LGTP_DEVICE_FT6X36_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
//#include <TestLimits_sn280h.h>

/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS    2
#define MT_MAX_TOUCH_POINTS     9

#define PRESS_MAX               0xFF
#define FT_PRESS                0x7F

#define Proximity_Max           32

#define FT_FACE_DETECT_ON       0xc0
#define FT_FACE_DETECT_OFF      0xe0

#define FT_FACE_DETECT_ENABLE   1
#define FT_FACE_DETECT_DISABLE  0
#define FT_FACE_DETECT_REG      0xB0

#define FT6X06_NAME             "ft6x06_ts"

#define FT_MAX_ID               0x0F
#define FT_TOUCH_STEP           6
#define FT_FACE_DETECT_POS      1
#define FT_TOUCH_X_H_POS        3
#define FT_TOUCH_X_L_POS        4
#define FT_TOUCH_Y_H_POS        5
#define FT_TOUCH_Y_L_POS        6
#define FT_TOUCH_EVENT_POS      3
#define FT_TOUCH_ID_POS         5
#define FT_TOUCH_WEIGHT          7
#define POINT_READ_BUF  (3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

    /*register address*/
#define FT6x06_REG_FW_VER       0xA6
#define FT6x06_REG_POINT_RATE   0x88
#define FT6x06_REG_THGROUP      0x80
#define FT6x06_PANEL_ID         0xA8
#define FT6x36_REG_FW_REL       0xAF
#define FT6x36_DEVICE_MODE      0x00
#define FT6x36_FACTORY_MODE     0xAE
#define FT6x36_START_SCAN       0x08

/* focaltech vendor app want to use sysfs*/
#define TPD_I2C_ADDRESS         0x38
//#define SYSFS_DEBUG
//#define FTS_APK_DEBUG
//#define FTS_CTL_IIC

#define EVENT_NONE              0x00
#define EVENT_ABS               0x01
#define EVENT_KEY               0x02

#define EVENT_KNOCK_ON          0x04
#define EVENT_KNOCK_CODE        0x08
#define EVENT_KNOCK_OVER        0x16
#define EVENT_HOVERING_NEAR     0x32
#define EVENT_HOVERING_FAR      0x64
#define EVENT_GEST              0x04
#define EVENT_MOUSE             0x08


/*knock on &Touble Tap Cotrol Register*/
#define FT_INT_STATUS           0x9B
#define FT_LPWG_INT_DELAY       0xDF
#define FT_LPWG_CONTROL_REG     0xD0

#define FT_MULTITAP_COUNT_REG   0xE4 /* for Knock code */

#define FT_KNOCK_READ_DATA_REG 0xD3
#define KNOCK_TAP_COUNT         0xD4
#define FT_TOUCHKEY_STATUS_REG 0x9A

/*register address*/
#define FT_REG_DEV_MODE     0x00
#define FT_DEV_MODE_REG_CAL 0x02
#define FT_REG_ID       0xA3
#define FT_REG_PMODE        0xA5
#define FT_REG_FW_VER       0xA6
#define FT_REG_POINT_RATE   0x88
#define FT_REG_THGROUP      0x80
#define FT_REG_ECC      0xCC
#define FT_REG_RESET_FW     0x07
#define FT_REG_FW_MAJ_VER   0xB1
#define FT_REG_FW_MIN_VER   0xB2
#define FT_REG_FW_SUB_MIN_VER   0xB3

    /* power register bits*/
#define FT_PMODE_ACTIVE     0x00
#define FT_PMODE_MONITOR    0x01
#define FT_PMODE_STANDBY    0x02
#define FT_PMODE_HIBERNATE  0x03
#define FT_FACTORYMODE_VALUE    0x40
#define FT_WORKMODE_VALUE   0x00
#define FT_RST_CMD_REG1     0xFC
#define FT_RST_CMD_REG2     0xBC
#define FT_READ_ID_REG      0x90
#define FT_ERASE_APP_REG    0x61
#define FT_ERASE_PANEL_REG  0x63
#define FT_FW_START_REG     0xBF

#define FT_STATUS_NUM_TP_MASK   0x0F

#define FT_VTG_MIN_UV       2600000
#define FT_VTG_MAX_UV       3300000
#define FT_I2C_VTG_MIN_UV   1800000
#define FT_I2C_VTG_MAX_UV   1800000

#define FT_COORDS_ARR_SIZE  4
#define MAX_BUTTONS     4

#define FT_8BIT_SHIFT       8
#define FT_4BIT_SHIFT       4
#define FT_FW_NAME_MAX_LEN  50

#define FT5316_ID       0x0A
#define FT5306I_ID      0x55
#define FT6X06_ID       0x06

#define FT_UPGRADE_AA       0xAA
#define FT_UPGRADE_55       0x55

#define FT_FW_MIN_SIZE      8
#define FT_FW_MAX_SIZE      32768

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)   ((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)   0
#define FT_FW_FILE_SUB_MIN_VER(x) 0

#define FT_MAX_TRIES        5
#define FT_RETRY_DLY        20

#define FT_MAX_WR_BUF       10
#define FT_MAX_RD_BUF       2
#define FT_FW_PKT_LEN       128
#define FT_FW_PKT_META_LEN  6
#define FT_FW_PKT_DLY_MS    20
#define FT_FW_LAST_PKT      0x6ffa
#define FT_EARSE_DLY_MS     100
#define FT_55_AA_DLY_NS     5000

#define FT_UPGRADE_LOOP     30
#define FT_CAL_START        0x04
#define FT_CAL_FIN      0x00
#define FT_CAL_STORE        0x05
#define FT_CAL_RETRY        100
#define FT_REG_CAL      0x00
#define FT_CAL_MASK     0x70

#define FT_INFO_MAX_LEN     512

#define FT_BLOADER_SIZE_OFF 12
#define FT_BLOADER_NEW_SIZE 30
#define FT_DATA_LEN_OFF_OLD_FW  8
#define FT_DATA_LEN_OFF_NEW_FW  14
#define FT_FINISHING_PKT_LEN_OLD_FW 6
#define FT_FINISHING_PKT_LEN_NEW_FW 12
#define FT_MAGIC_BLOADER_Z7 0x7bfa
#define FT_MAGIC_BLOADER_LZ4    0x6ffa
#define FT_MAGIC_BLOADER_GZF_30 0x7ff4
#define FT_MAGIC_BLOADER_GZF    0x7bf4

/* position of button*/
#define X_POS_OF_BACKKEY    60
#define X_POS_OF_HOMEKEY    180
#define X_POS_OF_SIMSWITCHKEY   300
#define X_POS_OF_MENUKEY    420
#define Y_POS_OF_TOUCHKEY   900


/****************************************************************************
* enum value Definitions
****************************************************************************/
enum {
    FT_BLOADER_VERSION_LZ4 = 0,
    FT_BLOADER_VERSION_Z7 = 1,
    FT_BLOADER_VERSION_GZF = 2,
};

enum {
    FT_FT5336_FAMILY_ID_0x11 = 0x11,
    FT_FT5336_FAMILY_ID_0x12 = 0x12,
    FT_FT5336_FAMILY_ID_0x13 = 0x13,
    FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

/****************************************************************************
* Type Definitions
****************************************************************************/
struct focaltech_ts_data {
    struct i2c_client *client;
    TouchState currState;
    LpwgSetting lpwgSetting;
};

/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/
#define GET_X_POSITION(_msb_reg, _lsb_reg) \
        (((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_Y_POSITION(_msb_reg, _lsb_reg) \
        (((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_WIDTH_MAJOR(_width_x, _width_y) \
        ((_width_x - _width_y) > 0) ? _width_x : _width_y
#define GET_WIDTH_MINOR(_width_x, _width_y) \
        ((_width_x - _width_y) > 0) ? _width_y : _width_x

#define GET_ORIENTATION(_width_y, _width_x) \
        ((_width_y - _width_x) > 0) ? 0 : 1
#define GET_PRESSURE(_pressure) \
            _pressure





/****************************************************************************
* Global Function Prototypes
****************************************************************************/
void focaltech_knockbaseaddr_set(int sel);
int  focaltech_knockbaseaddr_get(void);

#endif /* _LGTP_DEVICE_FT6X36_H_ */

/* End Of File */

