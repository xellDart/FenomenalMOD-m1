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
 *    File  	: semisense_misc_driver.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description : 
 *
 ***************************************************************************/
#define LGTP_MODULE "[COMMON]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>

#include <linux/input/unified_driver_3/lgtp_common_driver.h>
#include <linux/input/unified_driver_3/lgtp_platform_api.h>
#include <linux/input/unified_driver_3/lgtp_model_config.h>

#include <linux/miscdevice.h>
#include <linux/syscalls.h>
/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
//#define LGTP_MODULE "[SN_MISC]"


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/

/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static int semisense_misc_open(struct inode *inode, struct file *file);
static long semisense_misc_ioctl(struct file *file, u32 cmd, unsigned long arg);
static int semisense_misc_probe(void);


/****************************************************************************
* Variables
****************************************************************************/
static const struct file_operations semisense_misc_fops = 
{
    .owner = THIS_MODULE,
    .open = semisense_misc_open,
    .unlocked_ioctl = semisense_misc_ioctl,
};

static struct miscdevice semisense_miscdev = 
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sn310m_dist",
    .fops = &semisense_misc_fops,
};
typedef struct {
    u32 addr;
    u16 *buf;
    u32 size;
} packet_t;

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/

extern struct mutex *pMutexTouch;



/****************************************************************************
* Local Functions
****************************************************************************/
static int semisense_misc_open(struct inode *inode, struct file *file)
{
    TOUCH_LOG("[SN_MISC]Semisense misc open\n");
    return TOUCH_SUCCESS;
}
//TouchDriverData *pDriverData = container_of(self, TouchDriverData, fb_notif);
	
static long semisense_misc_ioctl(struct file *file, u32 cmd, unsigned long arg)
{
    packet_t* packet = (packet_t*)arg;
    TouchDriverData *pDriverData = container_of(pMutexTouch, TouchDriverData, thread_lock);
    int i = 0;
    u16 addr = 0;
    if(pDriverData == NULL)
    {
        TOUCH_LOG("[SN_MISC]container_of fail\n");
        return TOUCH_FAIL;
    }
    
    mutex_lock(pMutexTouch);

    switch(cmd)
    {
        case 0: /* Write Data */
            if(packet->size)
            {
                addr = (packet->addr >>8) | (packet->addr & 0x00ff) << 8; /*swap for endian*/
                Semisense_I2C_Write(pDriverData->client, (u8 * )&addr, (u8 *)&packet->buf, sizeof(addr), packet->size*2);
            }
            break;
        case 1: /* Read Data */
            if(packet->size)
            {
                u16 buffer[500] = {0,};
                addr = (packet->addr >>8) | (packet->addr & 0x00ff) << 8; /*swap for endian*/
                Semisense_I2C_Read(pDriverData->client, (u8 * )&addr, (u8 *)buffer, sizeof(addr), packet->size*2);
                for(i = 0; (i < packet->size) && (i < 500); i++) 
                {
            		packet->buf[i] = buffer[i];
                }
            }
            break;
        default :
            mutex_unlock(pMutexTouch);
            return -ENOIOCTLCMD;
            break;
    }
    mutex_unlock(pMutexTouch);
    return TOUCH_SUCCESS;
}

static int semisense_misc_probe(void)
{
    int result = 0;

    result = misc_register(&semisense_miscdev);
    if(result == 0)
    {
        TOUCH_LOG("[SN_MISC]Success to register semisense misc driver\n");
    }
    else
    {   
        result = TOUCH_FAIL;
        TOUCH_ERR("Fail to register semisense misc driver\n");
    }
    /* change authority */
	if(sys_chmod((const char __user *)"/dev/sn310m_dist", 0x0666) < 0) 
    {
		TOUCH_LOG("failed to change the \"/dev/sn310m_dist\" permission.\n");
	}
	else
    {
		TOUCH_LOG("succeeded to change the \"/dev/sn310m_dist\" permission.\n");
	}
    return TOUCH_SUCCESS;
}

static int __init Semisense_misc_init(void)
{
    TOUCH_LOG("[SN_MISC]Semisense misc Driver Init\n");
    if(semisense_misc_probe() <0)
    {
        TOUCH_LOG("[SN_MISC]Semisense misc probe fail\n");
        return -ENODEV;
    }
    return TOUCH_SUCCESS;
}

static void __exit Semisense_misc_exit(void)
{
    TOUCH_LOG("[SN_MISC]Semisense misc Driver exit\n");
}

module_init(Semisense_misc_init);
module_exit(Semisense_misc_exit);

MODULE_AUTHOR("D3 BSP Touch Team");
MODULE_DESCRIPTION("Semisense misc Driver");
MODULE_LICENSE("GPL");

/* End Of File */

