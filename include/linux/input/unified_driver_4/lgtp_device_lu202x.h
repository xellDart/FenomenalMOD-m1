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
 *    File  	: lgtp_device_lu202x.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description : 
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_LU202X_H_ )
#define _LGTP_DEVICE_LU202X_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/

/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/
enum TOUCH_DEBUG {
	_NONE                      = 0,
	BASE_INFO                 = (1U << 0),    /* 1 */
	TRACE                     = (1U << 1),    /* 2 */
	GET_DATA                  = (1U << 2),    /* 4 */
	ABS                       = (1U << 3),    /* 8 */
	BUTTON                    = (1U << 4),    /* 16*/
	FW_UPGRADE                = (1U << 5),    /* 32 */
	GHOST                     = (1U << 6),    /* 64 */
	IRQ_HANDLE                = (1U << 7),    /* 128 */
	POWER                     = (1U << 8),    /* 256 */
	JITTER                    = (1U << 9),    /* 512 */
	ACCURACY                  = (1U << 10),   /* 1024 */
	BOUNCING                  = (1U << 11),   /* 2048 */
	GRIP                      = (1U << 12),   /* 4096 */
	FILTER_RESULT             = (1U << 13),   /* 8192 */
	QUICKCOVER                = (1U << 12),   /* 4096 */
	LPWG                      = (1U << 14),   /* 16384 */
	NOISE                     = (1U << 15),   /* 32768 */
	LPWG_COORDINATES          = (1U << 16),   /* 65536 */
};

extern u32 touch_debug_mask;
#define LU202X_FUNC(condition)								\
	do {										\
		if (unlikely(touch_debug_mask & (condition)))				\
			printk(KERN_ERR LGTP_TAG"[F]"LGTP_MODULE" %s()\n", __FUNCTION__);\
	} while (0)

#define LU202X_LOG(condition, fmt, args...)						\
	do {										\
		if (unlikely(touch_debug_mask & (condition)))				\
			printk(KERN_ERR LGTP_TAG"[L]"LGTP_MODULE" " fmt, ##args);	\
	} while (0)

#define LU202X_ERR(fmt, args...) \
	printk(KERN_ERR LGTP_TAG"[E]"LGTP_MODULE" %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)
/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/



#endif /* _LGTP_DEVICE_LU202X_H_ */

/* End Of File */

