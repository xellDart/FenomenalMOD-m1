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
 *    File  	: lgtp_melfas_self_diagnostic.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[MIT_SELFD]"
 
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
#define MAX_ROW 32
#define MAX_COL 18

/*Interrupt Type*/
#define EVENT_INTERRUPT_TYPE_NONE 0x00
#define EVENT_INTERRUPT_TYPE 0x01

/*CS Test Mode & TOUCH SENSING MODE*/
#define MIP_MODE_TOUCH_SENSING 0x00
#define MIP_MODE_CS_TEST 0x02
#define MAX_ITERATOR 30000

/**/

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
* Type Definitions
****************************************************************************/
struct mit300_data_format{
	u8 row_num;
	u8 col_num;
	u8 buffer_col_num;
	u8 rotate;
	u8 key_num;
	u8 data_type;
	u8 data_type_size;
	u8 data_type_sign;
};
struct mit300_buf_addr{
	u8 buf_addr_h;
	u8 buf_addr_l;
};

/****************************************************************************
* Variables
****************************************************************************/
bool test_busy = false;

uint16_t *mit_data[MAX_ROW];
s16 *intensity_data[MAX_ROW];
int r_min;
int r_max;

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
int MIT300_GetReadyStatus(struct i2c_client *client)
{
	u8 wbuf[16];
	u8 rbuf[16];
	int ret = 0;

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_READY_STATUS;
	if(Mit300_I2C_Read(client, wbuf, 2, rbuf, 1) < 0) {
		printk(KERN_ERR "%s [ERROR] mip_i2c_read\n", __func__);
		goto ERROR;
	}
	ret = rbuf[0];

	//check status
	if((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG) || (ret == MIP_CTRL_STATUS_READY)){
		printk(KERN_INFO "%s - status [0x%02X]\n", __func__, ret);
	}
	else{
		printk(KERN_ERR "%s [ERROR] Unknown status [0x%02X]\n", __func__, ret);
		goto ERROR;
	}

	if(ret == MIP_CTRL_STATUS_LOG){
		//skip log event
		wbuf[0] = MIP_R0_LOG;
		wbuf[1] = MIP_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if(Mit300_I2C_Write(client, wbuf, 3)){
			printk(KERN_ERR "%s [ERROR] mip_i2c_write\n", __func__);
		}
	}

	printk(KERN_INFO "%s [DONE]\n", __func__);
	return ret;

ERROR:
	printk(KERN_ERR "%s [ERROR]\n", __func__);
	return -1;
}

static int MIT300_GetDataFormat(struct i2c_client *client, struct mit300_data_format *data_form)
{
	u8 reg_buf[2];
	u8 rbuf[10];

	reg_buf[0] = MIP_R0_IMAGE;
	reg_buf[1] = MIP_R1_IMAGE_DATA_FORMAT;
	
	if(Mit300_I2C_Read(client, reg_buf, 2, rbuf, 6)<0){
		printk(KERN_ERR "%s [ERROR] Read data format\n", __func__);
		return TOUCH_FAIL;
	}
	data_form->row_num = rbuf[0];
	data_form->col_num = rbuf[1];
	data_form->buffer_col_num = rbuf[2];
	data_form->rotate = rbuf[3];
	data_form->key_num = rbuf[4];
	data_form->data_type = rbuf[5];

	data_form->data_type_sign = (data_form->data_type & 0x80) >> 7;
	data_form->data_type_size = data_form->data_type & 0x7F;

	printk(KERN_INFO "%s - row_num[%d] col_num[%d] buffer_col_num[%d] rotate[%d] key_num[%d]\n",
					__func__, data_form->row_num, data_form->col_num, data_form->buffer_col_num, data_form->rotate, data_form->key_num);
	printk(KERN_INFO "%s - data_type[0x%02X] data_sign[%d] data_size[%d]\n",
					__func__, data_form->data_type, data_form->data_type_sign, data_form->data_type_size);
	return TOUCH_SUCCESS;
}

static int MIT300_GetBufAddr(struct i2c_client *client, struct mit300_buf_addr *buf_addr)
{
	u8 reg_buf[2];
	u8 rbuf[10];

	reg_buf[0] = MIP_R0_IMAGE;
	reg_buf[1] = MIP_R1_IMAGE_BUF_ADDR;
	if(Mit300_I2C_Read(client, reg_buf, 2,rbuf, 2)<0){
		printk(KERN_ERR "%s [ERROR] Read buf addr\n", __func__);
		return TOUCH_FAIL;
	}
	
	buf_addr->buf_addr_l = rbuf[0];
	buf_addr->buf_addr_h = rbuf[1];
	printk(KERN_INFO "%s - buf_addr[0x%02X 0x%02X]\n", __func__, buf_addr->buf_addr_h, buf_addr->buf_addr_l);

	return TOUCH_SUCCESS;
}
/*
static int MIT300_ReadRMIBuffer(struct i2c_client *client, u8 size, u8 data_type_size, 
										u8 data_type_sign, u8 buf_addr_h, u8 buf_addr_l, u8 row_num, 
										u8 col_num, u8 buf_col_num, u8 rotate, u8 key_num)
{
*/
static int MIT300_ReadRMIBuffer(struct i2c_client *client, struct mit300_data_format *data_form, struct mit300_buf_addr *buf_addr)
{

	char data[10];
	u8 size = 0;
	int i_col, i_row;
	int i_x, i_y;
	int lim_x, lim_y;
	int lim_col, lim_row;
	int max_x = 0;
	int max_y = 0;
	bool flip_x = false;
	int sValue = 0;
	unsigned int uValue = 0;
	int value = 0;
	u8 wbuf[8];
	u8 rbuf[512];
	unsigned int addr;
	int offset;
	int has_key = 0;

	memset(data, 0, 10);

	printk(KERN_INFO "%s [START]\n", __func__);

	//set axis
	if(data_form->rotate == 0){
		max_x = data_form->col_num;
		max_y = data_form->row_num;
		if(data_form->key_num > 0){
			max_y += 1;
			has_key = 1;
		}
		flip_x = false;
	}
	else if(data_form->rotate == 1){
		max_x = data_form->row_num;
		max_y = data_form->col_num;
		if(data_form->key_num > 0){
			max_y += 1;
			has_key = 1;
		}
		flip_x = true;
	}
	else{
		printk(KERN_ERR "%s [ERROR] rotate [%d]\n", __func__, data_form->rotate);
		goto ERROR;
	}

	//get table data
	lim_row = data_form->row_num + has_key;
	for(i_row = 0; i_row < lim_row; i_row++){
		//get line data
		offset = data_form->buffer_col_num * data_form->data_type_size;
		size = data_form->col_num * data_form->data_type_size;

		addr = (buf_addr->buf_addr_h << 8) | buf_addr->buf_addr_l | (offset * i_row);
		wbuf[0] = (addr >> 8) & 0xFF;
		wbuf[1] = addr & 0xFF;
		if(Mit300_I2C_Read(client, wbuf, 2, rbuf, size)){
			printk(KERN_ERR "%s [ERROR] Read data buffer\n", __func__);
			goto ERROR;
		}

		//save data
		if((data_form->key_num > 0) && (i_row == (lim_row - 1))){
			lim_col = data_form->key_num;
		}
		else{
			lim_col = data_form->col_num;
		}
		for(i_col = 0; i_col < lim_col; i_col++){
			if(data_form->data_type_size == 0){
				//unsigned
				if(data_form->data_type_size == 1){
					uValue = (u8)rbuf[i_col];
				}
				else if(data_form->data_type_size == 2){
					uValue = (u16)(rbuf[data_form->data_type_size * i_col] | (rbuf[data_form->data_type_size * i_col + 1] << 8));
				}
				else if(data_form->data_type_size == 4){
					uValue = (u32)(rbuf[data_form->data_type_size * i_col] | (rbuf[data_form->data_type_size * i_col + 1] << 8) | (rbuf[data_form->data_type_size * i_col + 2] << 16) | (rbuf[data_form->data_type_size * i_col + 3] << 24));
				}
				else{
					printk(KERN_ERR "%s [ERROR] data_size [%d]\n", __func__, data_form->data_type_size);
					goto ERROR;
				}
				value = (int)uValue;
			}
			else{
				//signed
				if(data_form->data_type_size == 1){
					sValue = (s8)rbuf[i_col];
				}
				else if(data_form->data_type_size == 2){
					sValue = (s16)(rbuf[data_form->data_type_size * i_col] | (rbuf[data_form->data_type_size * i_col + 1] << 8));
				}
				else if(data_form->data_type_size == 4){
					sValue = (s32)(rbuf[data_form->data_type_size * i_col] | (rbuf[data_form->data_type_size * i_col + 1] << 8) | (rbuf[data_form->data_type_size * i_col + 2] << 16) | (rbuf[data_form->data_type_size * i_col + 3] << 24));
				}
				else{
					printk(KERN_ERR "%s [ERROR] data_size [%d]\n", __func__, data_form->data_type_size);
					goto ERROR;
				}
				value = (int)sValue;
			}

			switch(data_form->rotate){
				case 0:
					mit_data[i_row][i_col] = value;
					intensity_data[i_row][i_col] = value;
					break;
				case 1:
					if((data_form->key_num > 0) && (i_row == (lim_row - 1))){	
						mit_data[i_row][i_col] = value;
						intensity_data[i_row][i_col] = value;
					}
					else{
						mit_data[i_col][i_row] = value;
						intensity_data[i_col][i_row] = value;
					}
					break;
				default:
					printk(KERN_ERR "%s [ERROR] rotate [%d]\n", __func__, data_form->rotate);
					goto ERROR;
					break;
			}
		}
	}

	//print table header
	printk("    ");
	sprintf(data, "    ");
	//strcat(print_buf, data);
	memset(data, 0, 10);

	switch(data_form->data_type_size){
		case 1:
			for(i_x = 0; i_x < max_x; i_x++){
				printk("[%2d]", i_x);
				sprintf(data, "[%2d]", i_x);
				memset(data, 0, 10);
			}
			break;
		case 2:
			for(i_x = 0; i_x < max_x; i_x++){
				printk("[%4d]", i_x);
				sprintf(data, "[%4d]", i_x);
				memset(data, 0, 10);
			}
			break;
		case 4:
			for(i_x = 0; i_x < max_x; i_x++){
				printk("[%5d]", i_x);
				sprintf(data, "[%5d]", i_x);
				memset(data, 0, 10);
			}
			break;
		default:
			printk(KERN_ERR "%s [ERROR] data_size [%d]\n", __func__, data_form->data_type_size);
			goto ERROR;
			break;
	}

	printk("\n");
	sprintf(data, "\n");
	memset(data, 0, 10);

	//print table
	lim_y = max_y;
	for(i_y = 0; i_y < lim_y; i_y++){
		//print line header
		if((data_form->key_num > 0) && (i_y == (lim_y -1))){
			printk("[TK]");
			sprintf(data, "[TK]");
		}
		else{
			printk("[%2d]", i_y);
			sprintf(data, "[%2d]", i_y);
		}
		memset(data, 0, 10);

		//print line
		if((data_form->key_num > 0) && (i_y == (lim_y - 1))){
			lim_x = data_form->key_num;
		}
		else{
			lim_x = max_x;
		}
		for(i_x = 0; i_x < lim_x; i_x++){
			switch(data_form->data_type_size){
				case 1:
				case 2:
				case 4:
					break;
				default:
					printk(KERN_ERR "%s [ERROR] data_size [%d]\n", __func__, data_form->data_type_size);
					goto ERROR;
					break;
			}

			memset(data, 0, 10);
		}

		printk("\n");
		sprintf(data, "\n");
		memset(data, 0, 10);
	}

	printk("\n");
	sprintf(data, "\n");
	memset(data, 0, 10);



	printk(KERN_INFO "%s [DONE]\n", __func__);
	return TOUCH_SUCCESS;

ERROR:

	printk(KERN_ERR "%s [ERROR]\n", __func__);
	return TOUCH_FAIL;
}

static int  MIT300_PrintRawData(struct i2c_client *client, char *buf,int type)
{
	int col = 0;
	int row = 0;
	int ret = 0;

	r_min = mit_data[0][0];
	r_max = mit_data[0][0];

	for (row = 0 ; row < MAX_ROW ; row++) {
		if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"[%2d]  ",row);
			printk("[Touch] [%2d]  ",row);
		}

		for (col = 0 ; col < MAX_COL ; col++) {

			ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
			printk("%5d ", mit_data[row][col]);
			if (type == RAW_DATA_STORE) {
				ret += sprintf(buf + ret,",");
			}

			r_min = (r_min > mit_data[row][col]) ? mit_data[row][col] : r_min;
			r_max = (r_max < mit_data[row][col]) ? mit_data[row][col] : r_max;

		}

		if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == RAW_DATA_SHOW) {
		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n\n",r_max , r_min, r_max - r_min);
		TOUCH_LOG("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n\n",r_max , r_min, r_max - r_min);
	}

	return ret;
}

static int  MIT300_PrintIntensity(struct i2c_client *client, char *buf) {
	int col = 0;
	int row = 0;
	int ret = 0;

	ret += sprintf(buf + ret, "Start-Intensity\n\n");

	for (row = 0 ; row < MAX_ROW ; row++) {
		printk("[Touch] [%2d]  ", row);
		ret += sprintf(buf + ret,"[%2d]  ", row);
		for (col = 0 ; col < MAX_COL ; col++) {
			ret += sprintf(buf + ret,"%4d ", intensity_data[row][col]);
			printk("%4d ", intensity_data[row][col]);
		}
		printk("\n");
		ret += sprintf(buf + ret,"\n");
	}

	return ret;
}

int MIT300_DoTest(struct i2c_client *client, u8 image_type){
  	int busy_cnt = 100;
	int wait_cnt = 50;
	u8 wbuf[8];
	struct mit300_data_format *data_form = NULL;
	struct mit300_buf_addr *buf_addr = NULL;

	printk(KERN_INFO "%s [START]\n", __func__);
	printk(KERN_INFO "%s - image_type[%d]\n", __func__, image_type);

	while(busy_cnt--){
		if(test_busy == false){
			break;
		}
		printk(KERN_INFO "%s - busy_cnt[%d]\n", __func__, busy_cnt);
		msleep(5);
	}

	test_busy = true;

	switch(image_type){
		case MIP_IMG_TYPE_INTENSITY:
			printk(KERN_INFO "=== Intensity Image ===\n");
			break;
		case MIP_IMG_TYPE_RAWDATA:
			printk(KERN_INFO "=== Rawdata Image ===\n");
			break;
		default:
			printk(KERN_ERR "%s [ERROR] Unknown image type\n", __func__);
			goto ERROR;
			break;
	}

	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_TYPE;
	wbuf[2] = image_type;

	if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
		printk(KERN_ERR "%s [ERROR] Write image type\n", __func__);
		goto ERROR;
	}

	printk(KERN_INFO "%s - set image type\n", __func__);

	while(wait_cnt--){
		if(MIT300_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
			break;
		}
		msleep(10);

		printk(KERN_INFO "%s - wait [%d]\n", __func__, wait_cnt);
	}

	if(wait_cnt <= 0){
		printk(KERN_ERR "%s [ERROR] Wait timeout\n", __func__);
		goto ERROR;
	}

	printk(KERN_INFO "%s - ready\n", __func__);

	if (MIT300_GetDataFormat(client, data_form)) {
		printk(KERN_ERR "%s [ERROR] MIT300_GetDataFormat\n", __func__);
		goto ERROR;
	}

	if (MIT300_GetBufAddr(client, buf_addr)) {
		printk(KERN_ERR "%s [ERROR] MIT300_GetBufAddr\n", __func__);
		goto ERROR;
	}

	if(MIT300_ReadRMIBuffer(client, data_form, buf_addr)) {
		printk(KERN_ERR "%s [ERROR] MIT300_ReadRMIBuffer\n", __func__);
		goto ERROR;
	}

	//clear image type
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_TYPE;
	wbuf[2] = MIP_IMG_TYPE_NONE;

	if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
		printk(KERN_ERR "%s [ERROR] Write image type\n", __func__);
		goto ERROR;
	}

	test_busy = false;

	printk(KERN_INFO "%s [DONE]\n", __func__);
	return TOUCH_SUCCESS;

ERROR:
	test_busy = false;

	printk(KERN_ERR "%s [ERROR]\n", __func__);
	return 1;
}

ssize_t MIT300_GetTestResult(struct i2c_client *client, char *buf, int *result, int type)
{

	char temp_buf[255] = {0,};
	int i = 0;
	int ret = 0;
	int fd = 0;
	char data_path[64] = {0,};
	char *read_buf = NULL;
	int retry_max = 3;
	int retry_count = 0;

	mm_segment_t old_fs = get_fs();

	for ( i = 0 ; i < MAX_ROW ; i++) {
		memset(mit_data[i], 0, sizeof(uint16_t) * MAX_COL);
	}

	read_buf = kzalloc(sizeof(u8) * 4096,GFP_KERNEL);
	if (read_buf == NULL) {
		printk(KERN_ERR "read_buf mem_error\n");
		goto mem_error;
	}


RETRY :
	if (retry_count > 0 && retry_count <= retry_max) {
		TOUCH_LOG("%s retry (%d/%d) \n", __func__, retry_count, retry_max);
		printk(KERN_ERR "%s retry (%d/%d) \n", __func__, retry_count, retry_max);
		TouchPower(0);
		TouchPower(1);
		mdelay(50);
	} else if (retry_count > retry_max) {
		TOUCH_LOG("%s all retry failed \n", __func__);
		printk(KERN_ERR "%s all retry failed \n", __func__);
		goto error;
	}

	retry_count++;



	if (type == RAW_DATA_SHOW || type == RAW_DATA_STORE) {
		if (MIT300_DoTest(client, MIP_IMG_TYPE_RAWDATA) == TOUCH_FAIL) {
			TOUCH_ERR("getting raw data failed\n");
			printk(KERN_ERR "getting raw data failed\n");
			goto RETRY;
		}
	} else if (type == DELTA_SHOW) {
		if (MIT300_DoTest(client, MIP_IMG_TYPE_INTENSITY) == TOUCH_FAIL) {
				TOUCH_ERR("getting raw data failed\n");
				printk(KERN_ERR "getting raw data failed\n");
		}
	}
/*
	else if (type == OPENSHORT_STORE || type == OPENSHORT) {
		if (get_openshort(ts) == TOUCH_FAIL) {
			TOUCH_ERR("getting open_short data failed\n");
			printk(KERN_ERR "getting open_short data failed\n");
			goto RETRY;
		}
	}
*/
	/*--------------------------------------*/
	/* To Do List*/
	/*1. compare to limitation value and test value.*/
	/*2. check the result value.*/
	/*--------------------------------------*/

	switch(type) {
		case RAW_DATA_SHOW:
			printk(KERN_ERR "%s RAW_DATA_SHOW\n", __func__);
			ret = MIT300_PrintRawData(client, buf, type);
			if (ret < 0) {
				TOUCH_ERR("fail to print raw data\n");
				goto error;
			}
			break;
		case RAW_DATA_STORE:
			snprintf(temp_buf, strlen(buf), "%s", buf);
			sprintf(data_path, "/sdcard/%s.csv", temp_buf);

			ret = MIT300_PrintRawData(client, read_buf, type);
			if (ret < 0) {
				TOUCH_ERR("fail to print raw data\n");
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_LOG("%s saved \n", data_path);
			} else {
				TOUCH_LOG("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
		case DELTA_SHOW:
			printk(KERN_ERR "%s DELTA_SHOW\n", __func__);
			ret = MIT300_PrintIntensity(client, buf);
			if (ret < 0) {
				TOUCH_ERR("fail to print intensity data\n");
				goto error;
			}
			break;
		default :
			TOUCH_LOG("type = default[%d]\n", type);
			break;
		}

	if (read_buf != NULL)
		kfree(read_buf);

	return ret; // ret is size of data buffer .

error :
	if (read_buf != NULL)
		kfree(read_buf);
	*result = 0;
	return TOUCH_FAIL;

mem_error :
	if (read_buf != NULL)
		kfree(read_buf);
	*result = 0;
	return TOUCH_FAIL;

}


