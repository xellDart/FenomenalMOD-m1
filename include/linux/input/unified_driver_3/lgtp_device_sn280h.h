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
 *    File  	: lgtp_device_sn280h.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_SN280H_H_ )
#define _LGTP_DEVICE_SN280H_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/
//----------------------------------------------
// board-dependent define 
//----------------------------------------------
#define SEMISENS_TS_I2C_SLAVE_ADDR	(0x3c)
#define SEMISENS_TS_FINGER_MAX		5
#define SEMISENS_TS_KEY_MAX		3


//----------------------------------------------
// Common register address for firmware update
//----------------------------------------------
#define REG_ISP_MODE			0xF102	// ISP mode control
#define REG_ISP_MODE_BUS		0xF104	// ISP mode bus functions
#define REG_ISP_MODE_ENABLE		0xF108	// ISP mode enable
#define REG_ISP_MEM_TYPE		0xF100	// MEM TYPE: EEPROM/EFLASH

//----------------------------------------------
// EFLASH register address for firmware update
//----------------------------------------------
#define REG_CMD_FLASH_AUTH		0xF400	// 0x0100 : get eFlash approach authority
#define REG_CMD_FLASH_CON_EN		0xF402	// 0x0000 : enable eFlash controller
#define REG_CMD_FLASH_COMMAND		0xF404	// 0x0200 : erase eFlash, 0x0000 : write eFlash
#define REG_CMD_FLASH_BUSY		0xF408	// [15] bit is busy flag for eflash operation.

//----------------------------------------------
// EEPROM register address for firmware update
//----------------------------------------------
#define REG_CMD_EER_XPROT		0xF400	// 0x0000 : get EEPROM approach authority
#define REG_CMD_EER_PDOWN		0xF402	// 0x0000 : enable EEPROM controller
#define REG_CMD_EER_RESET		0xF404	// 0x0000 : set EEPROM state to ACTIVE
#define REG_CMD_EER_MODE		0xF406	// 0x0007 : Enable EEPROM erase function
						// 0x0008 : Enable EEPROM write function
#define REG_CMD_EER_XEN			0xF408	// 0x0001 : EEPROM Excution Enable
#define REG_CMD_EER_EXTEND		0xF40E	// 0x0001 : EEPROM chip select control
#define REG_CMD_EER_CSCON		0xF410	// 0x0001 : EEPROM chip select control
#define REG_CMD_EER_STATE		0xF412	// [2] bit is busy flag for EEPROM operation.
						// the value 0 of [2] bit means EEPROM busy

//----------------------------------------------
// Touch status & data register address
//----------------------------------------------
#define	REG_TS_STATUS			0xE000
#define REG_TS_GEST_STATUS		0xE040
#define	REG_TS_DATA_BASE		0xE002
#define	REG_TS_DATA(x)			(REG_TS_DATA_BASE+(6*x))
#define	REG_KNOCK_DATA_BASE		0x0640
#define	REG_KNOCK_DATA(x)		(((x * 4) >> 8) + REG_KNOCK_DATA_BASE)

#define REG_FIRMWARE_VERSION	(0x3EE0)
#define REG_IC_CODE		(0x3FC0)
#define TSC_FLASH_FW_VER_POS	(0x3FCA)
#define TSC_PANEL_TEST_VER	(0x3FCC)
#define REG_PROJECT_CODE	(0x3FCE)
#define REG_DEVIATION_CODE	(0x3FD0)
#define REG_MP_FIELD		(0x3FD2)

/* Temporary settings */
#define GESTURE_MODE_REG 0xE000

//--------------------------------------------
// Touch Event type define
//--------------------------------------------
#define	TS_EVENT_UNKNOWN		0x00
#define	TS_EVENT_PRESS			0x01
#define	TS_EVENT_MOVE			0x02
#define	TS_EVENT_RELEASE		0x03

//--------------------------------------------
// Touch Memory type define
//--------------------------------------------
#define REG_ISP_VAL_EEPROM		0xBABA	
#define REG_ISP_VAL_ERROR		0xDEAD	
enum {
	E_MEM_TYPE_EFLASH = 0,
	E_MEM_TYPE_EEPROM = 1,
	E_MEM_TYPE_MAX
};

//--------------------------------------------
// operation mode define for firmware update 
//--------------------------------------------
enum {
	E_FLASH_OPMODE_READ = 0,
	E_FLASH_OPMODE_WRITE = 1,
	E_FLASH_OPMODE_ERASE = 2,
	E_FLASH_OPMODE_MAX
};
//--------------------------------------------
// Button struct (1 = press, 0 = release)
//--------------------------------------------
typedef struct button__t {
	u8 bt0_press	:1; // lsb
	u8 bt1_press	:1;		
	u8 bt2_press	:1;
	u8 bt3_press	:1;
	u8 bt4_press	:1;
	u8 bt5_press	:1;
	u8 bt6_press	:1;
	u8 bt7_press	:1; // msb
} __attribute__ ((packed)) button_t;

typedef union button__u {
	u8			ubyte;
	button_t	bits;
} __attribute__ ((packed)) button_u;

typedef struct finger__t {
	u32 status;		// true : ts data updated, false : no update data
	u32 event;		// ts event type
	u32 id;			// ts received id
	u32 x;			// ts data x
	u32 y;			// ts data y
	u32 area;		// ts finger area
	u32 pressure;		// ts finger pressure
} __attribute__ ((packed)) finger_t;

typedef struct status_reg__t {
	u32 ts_cnt	:4;	// lsb
	u32 proximity	:4;
	u32 button	:5;
	u32 reserved2	:2;	
	u32 gesture	:1;	// msb
} __attribute__ ((packed)) status_reg_t;

typedef union status_reg__u {
	u16		uint;
	status_reg_t	bits;
}	__attribute__ ((packed)) status_reg_u;

typedef struct data_reg__t {
	u16 packet0;
	u16 packet1;
	u16 packet2;
} __attribute__ ((packed)) data_reg_t;

typedef union data_reg__u {
	u32			uint;
	data_reg_t	bits;
} __attribute__ ((packed)) data_reg_u;

/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/
#define TO_STR(X)		#X
#define SWAP_16BITS(p) \
	((((p) & 0xFF00) >> 8) | (((p) & 0x00FF) << 8))

/****************************************************************************
* Global Function Prototypes
****************************************************************************/



#endif /* _LGTP_DEVICE_SN280H_H_ */

/* End Of File */
