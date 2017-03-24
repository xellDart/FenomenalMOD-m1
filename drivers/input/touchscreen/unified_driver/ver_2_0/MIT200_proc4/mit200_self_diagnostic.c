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

#include <linux/input/unified_driver_2/lgtp_device_mit200_proc4.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
/*Use Test Register or Image Register*/
#define TESTREG 0
#define IMGREG 1

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
* Type Definitions
****************************************************************************/
struct mit200_data_format{
	u8 row_num;
	u8 col_num;
	u8 buffer_col_num;
	u8 rotate;
	u8 key_num;
	u8 data_type;
	u8 data_type_size;
	u8 data_type_sign;
};
struct mit200_buf_addr{
	u8 buf_addr_h;
	u8 buf_addr_l;
};

/****************************************************************************
* Variables
****************************************************************************/
bool test_busy = false;

uint16_t mit_data[MAX_ROW][MAX_COL];
s16 intensity_data[MAX_ROW][MAX_COL];

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
int MIT200_GetReadyStatus(struct i2c_client *client)
{
	u8 wbuf[16];
	u8 rbuf[16];
	int ret = 0;

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_READY_STATUS;
	if(Mit200_proc4_I2C_Read(client, wbuf, 2, rbuf, 1) < 0) {
		TOUCH_ERR("[ERROR] mip_i2c_read\n");
		goto ERROR;
	}
	ret = rbuf[0];

	//check status
	if((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG) || (ret == MIP_CTRL_STATUS_READY)){
		if (ret == 0xA0)
			TOUCH_LOG("status [0x%02X]\n", ret);
	}
	else{
		TOUCH_ERR("[ERROR] Unknown status [0x%02X]\n", ret);
		goto ERROR;
	}

	if(ret == MIP_CTRL_STATUS_LOG){
		//skip log event
		wbuf[0] = MIP_R0_LOG;
		wbuf[1] = MIP_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if(Mit200_proc4_I2C_Write(client, wbuf, 3)){
			TOUCH_ERR("[ERROR] mip_i2c_write\n");
		}
	}

	//TOUCH_LOG("%s [DONE]\n", __func__);
	return ret;

ERROR:
	TOUCH_ERR("%s [ERROR]\n", __func__);
	return -1;
}

static int MIT200_GetDataFormat(struct i2c_client *client, struct mit200_data_format *data, int reg_type)
{
	u8 wbuf[2];
	u8 rbuf[10] = {0,};

	if (reg_type == TESTREG){
		wbuf[0] = MIP_R0_TEST;
		wbuf[1] = MIP_R1_TEST_DATA_FORMAT;
		if(Mit200_proc4_I2C_Read(client, wbuf, 2, rbuf, 6)<0){
			TOUCH_ERR("%s [ERROR] Read data format\n", __func__);
			return TOUCH_FAIL;
		}
	} else if (reg_type == IMGREG) {
		wbuf[0] = MIP_R0_IMAGE;
		wbuf[1] = MIP_R1_IMAGE_DATA_FORMAT;
		if(Mit200_proc4_I2C_Read(client, wbuf, 2, rbuf, 6)<0){
			TOUCH_ERR("%s [ERROR] Read data format\n", __func__);
			return TOUCH_FAIL;
		}
	}

	data->row_num = rbuf[0];
	data->col_num = rbuf[1];
	data->buffer_col_num = rbuf[2];
	data->rotate = rbuf[3];
	data->key_num = rbuf[4];
	data->data_type = rbuf[5];

	data->data_type_sign = (data->data_type & 0x80) >> 7;
	data->data_type_size = data->data_type & 0x7F;

	TOUCH_LOG("row_num[%d] col_num[%d] buffer_col_num[%d] rotate[%d] key_num[%d]\n", data->row_num, data->col_num, data->buffer_col_num, data->rotate, data->key_num);
	TOUCH_LOG("data_type[0x%02X] data_sign[%d] data_size[%d]\n", data->data_type, data->data_type_sign, data->data_type_size);

	return TOUCH_SUCCESS;
}

static int MIT200_GetBufAddr(struct i2c_client *client, struct mit200_buf_addr *buf_addr, int reg_type)
{
	u8 wbuf[2];
	u8 rbuf[10];

	if (reg_type == TESTREG){
		wbuf[0] = MIP_R0_TEST;
		wbuf[1] = MIP_R1_TEST_BUF_ADDR;
		if(Mit200_proc4_I2C_Read(client, wbuf, 2,rbuf, 2)<0){
			TOUCH_ERR("%s [ERROR] Read buf addr\n", __func__);
			return TOUCH_FAIL;
		}
	} else if (reg_type == IMGREG) {
		wbuf[0] = MIP_R0_IMAGE;
		wbuf[1] = MIP_R1_IMAGE_BUF_ADDR;
		if(Mit200_proc4_I2C_Read(client, wbuf, 2,rbuf, 2)<0){
			TOUCH_ERR("%s [ERROR] Read buf addr\n", __func__);
			return TOUCH_FAIL;
		}
	}

	buf_addr->buf_addr_l = rbuf[0];
	buf_addr->buf_addr_h = rbuf[1];
	TOUCH_LOG("buf_addr[0x%02X 0x%02X]\n", buf_addr->buf_addr_l, buf_addr->buf_addr_h);

	return TOUCH_SUCCESS;
}


static int MIT200_ReadRMIBuffer(struct i2c_client *client, struct mit200_data_format *data, struct mit200_buf_addr *buf_addr)
{
	u8 size = 0;
	int i_col, i_row;
	int lim_col, lim_row;
	int max_x = 0;
	int max_y = 0;
	bool flip_x = false;
	int sValue = 0;
	unsigned int uValue = 0;
	int value = 0;
	u8 wbuf[8] = {0};
	u8 rbuf[512] = {0};
	unsigned int addr;
	int offset;
	int has_key = 0;

	//set axis
	if(data->rotate == 0){
		max_x = data->col_num;
		max_y = data->row_num;
		if(data->key_num > 0){
			max_y += 1;
			has_key = 1;
		}
		flip_x = false;
	}
	else if(data->rotate == 1){
		max_x = data->row_num;
		max_y = data->col_num;
		if(data->key_num > 0){
			max_y += 1;
			has_key = 1;
		}
		flip_x = true;
	}
	else{
		TOUCH_ERR("[ERROR] rotate [%d]\n", data->rotate);
		goto ERROR;
	}
	//get table data
	lim_row = data->row_num + has_key;
	for(i_row = 0; i_row < lim_row; i_row++){
		//get line data
		offset = data->buffer_col_num * data->data_type_size;
		size = data->col_num * data->data_type_size;

		addr = (buf_addr->buf_addr_h << 8) | buf_addr->buf_addr_l | (offset * i_row);
		wbuf[0] = (addr >> 8) & 0xFF;
		wbuf[1] = addr & 0xFF;
		if(Mit200_proc4_I2C_Read(client, wbuf, 2, rbuf, size)){
			TOUCH_ERR("[ERROR] Read data buffer\n");
			goto ERROR;
		}

		//save data
		if((data->key_num > 0) && (i_row == (lim_row - 1))){
			lim_col = data->key_num;
		}
		else{
			lim_col = data->col_num;
		}
		for(i_col = 0; i_col < lim_col; i_col++){
			if(data->data_type_sign == 0){
				//unsigned
				if(data->data_type_size == 1){
					uValue = (u8)rbuf[i_col];
				}
				else if(data->data_type_size == 2){
					uValue = (u16)(rbuf[data->data_type_size * i_col]
					| (rbuf[data->data_type_size * i_col + 1] << 8));
				}
				else if(data->data_type_size == 4){
					uValue = (u32)(rbuf[data->data_type_size * i_col]
					| (rbuf[data->data_type_size * i_col + 1] << 8) 
					| (rbuf[data->data_type_size * i_col + 2] << 16) 
					| (rbuf[data->data_type_size * i_col + 3] << 24));
				}
				else{
					TOUCH_ERR("[ERROR] data_size [%d]\n", data->data_type_size);
					goto ERROR;
				}
				value = (int)uValue;
			}
			else{
				//signed
				if(data->data_type_size == 1){
					sValue = (s8)rbuf[i_col];
				}
				else if(data->data_type_size == 2){
					sValue = (s16)(rbuf[data->data_type_size * i_col]
					| (rbuf[data->data_type_size * i_col + 1] << 8));
				}
				else if(data->data_type_size == 4){
					sValue = (s32)(rbuf[data->data_type_size * i_col]
					| (rbuf[data->data_type_size * i_col + 1] << 8)
					| (rbuf[data->data_type_size * i_col + 2] << 16)
					| (rbuf[data->data_type_size * i_col + 3] << 24));
				}
				else{
					TOUCH_ERR("[ERROR] data_size [%d]\n", data->data_type_size);
					goto ERROR;
				}
				value = (int)sValue;
			}

			switch(data->rotate){
				case 0:
					mit_data[i_row][i_col] = value;
					intensity_data[i_row][i_col] = value;
					break;
				case 1:
					if((data->key_num > 0) && (i_row == (lim_row - 1))){	
						mit_data[i_row][i_col] = value;
						intensity_data[i_row][i_col] = value;
					}
					else{
						mit_data[i_col][i_row] = value;
						intensity_data[i_col][i_row] = value;
					}
					break;
				default:
					TOUCH_ERR("[ERROR] rotate [%d]\n", data->rotate);
					goto ERROR;
					break;
			}
		}
	}

	TOUCH_LOG("%s [DONE]\n", __func__);
	return TOUCH_SUCCESS;

ERROR:

	TOUCH_ERR("%s [ERROR]\n", __func__);
	return TOUCH_FAIL;
}

static int  MIT200_PrintData(struct i2c_client *client, char *buf,int type, int* result)
{
	int col = 0;
	int row = 0;
	int ret = 0;
	int min_data = 0;
	int max_data = 0;
	int failcount = 0;
	int lcd_status = 0;
	
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	if(pDriverData->lpwgSetting.lcdState == 0){ lcd_status = 0;
	}else{ lcd_status = 1;}

	TOUCH_FUNC();

	switch (type) {
	case RAW_DATA_SHOW:
		TOUCH_LOG("[Rawdata Result]\n");
		ret += sprintf(buf + ret,"[Rawdata Result]\n");
		if(lcd_status == 0){
			min_data = mit_data[0][1];
			max_data = mit_data[0][1];
		}else{
			min_data = mit_data[0][0];
			max_data = mit_data[0][0];
		}
		TOUCH_LOG("mit_data = %d, mit_data = %d\n",mit_data[0][1],mit_data[0][1]);
		//min_data = mit_data[0][0];
		//max_data = mit_data[0][0];
		break;
	case OPENSHORT_SHOW:
		TOUCH_LOG("[Openshort Result]\n");
		ret += sprintf(buf + ret,"[Openshort Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
		break;
	case OPENSHORT2_SHOW:
		TOUCH_LOG("[Openshort2 Result]\n");
		ret += sprintf(buf + ret,"[Openshort2 Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
		break;
	case DELTA_SHOW:
		TOUCH_LOG("[Delta Result]\n");
		ret += sprintf(buf + ret,"[Delta Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
		break;
	case JITTER_SHOW:
		TOUCH_LOG("[Jitter Result]\n");
		ret += sprintf(buf + ret,"[Jitter Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
		break;
	case INTENSITY_SHOW:
		TOUCH_LOG("[Intensity Result]\n");
		ret += sprintf(buf + ret,"[Intensity Result]\n");
		break;
	case OPEN_SHOW:
		TOUCH_LOG("[OPEN Result]\n");
		TOUCH_LOG("=================================\n");
		ret += sprintf(buf + ret,"[OPEN Result]\n");
		break;
	case RAW_DATA_SHOW_IMG:
		TOUCH_LOG("[RAWDATA Result]\n");
		ret += sprintf(buf + ret,"[RAWDATA Result]\n");
		break;
	default:
		return TOUCH_FAIL;
	}

	for (row = 0 ; row < MAX_ROW ; row++) {
		//TOUCH_LOG("[%2d]  ", row);
		ret += sprintf(buf + ret,"[%2d]  ", row);
		
		for (col = 0 ; col < MAX_COL ; col++) {
			if (type == RAW_DATA_SHOW ||
				type == DELTA_SHOW) {
				//TOUCH_LOG("%5d ", mit_data[row][col]);
				if(lcd_status == 0){
					if(mit_data[row][col] > RAWDATA_LPWG_MAX_LIMIT || mit_data[row][col] < RAWDATA_LPWG_MIN_LIMIT){
						if(col != 0 && col != 15){
							ret += sprintf(buf + ret,"!%4d ", mit_data[row][col]);
							failcount++;
						}else{
							ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
						}
					} else {
						ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
					}
					if(col != 0 && col != 15){
						min_data = (min_data > mit_data[row][col]) ? mit_data[row][col] : min_data;
						max_data = (max_data < mit_data[row][col]) ? mit_data[row][col] : max_data;
					}

				}else{
					if(mit_data[row][col] > RAWDATA_MAX_LIMIT || mit_data[row][col] < RAWDATA_MIN_LIMIT){
						ret += sprintf(buf + ret,"!%4d ", mit_data[row][col]);
						failcount++;
					} else {
						ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
					}
					min_data = (min_data > mit_data[row][col]) ? mit_data[row][col] : min_data;
					max_data = (max_data < mit_data[row][col]) ? mit_data[row][col] : max_data;
				}
			} else if (type == OPENSHORT_SHOW || type == OPENSHORT2_SHOW || type == OPEN_SHOW ) {
				if((mit_data[row][col] != 0)){
					failcount++;
					ret += sprintf(buf + ret,"!%4d ", mit_data[row][col]);
				}else {
					ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
				}
				min_data = (min_data > mit_data[row][col]) ? mit_data[row][col] : min_data;
				max_data = (max_data < mit_data[row][col]) ? mit_data[row][col] : max_data;
			} else if (type == INTENSITY_SHOW || type == RAW_DATA_SHOW_IMG){
				//TOUCH_LOG("%4d ", intensity_data[row][col]);
				ret += sprintf(buf + ret,"%4d ", intensity_data[row][col]);
			} else if (type == JITTER_SHOW){
				ret += sprintf(buf + ret,"%4d ", intensity_data[row][col]);
			}
		}

		//TOUCH_LOG("\n");
		ret += sprintf(buf + ret,"\n");
	}

	if (type == RAW_DATA_SHOW ||
		type == OPENSHORT_SHOW ||
		type == OPENSHORT2_SHOW ||
		type == DELTA_SHOW ||
		type == JITTER_SHOW ||
		type == OPEN_SHOW) {
		TOUCH_LOG("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n\n",max_data , min_data, max_data - min_data);
		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n\n",max_data, min_data, max_data - min_data);
	}

	if (type == RAW_DATA_SHOW){
		if(failcount != 0){
			*result = TOUCH_FAIL;
		}else{
			*result = TOUCH_SUCCESS;
		}
	}else if (type == OPENSHORT_SHOW ||
			type == OPENSHORT2_SHOW ||
			type == OPEN_SHOW){
		if((max_data - min_data) != 0){
			*result = TOUCH_FAIL;
		}else{
			*result = TOUCH_SUCCESS;
		}
	}
	//TODO: Compare to limitation spec & determine result test

	/* sample code
	if (ts->module.otp == OTP_APPLIED) {
		limit_upper = ts->pdata->limit->raw_data_otp_max + ts->pdata->limit->raw_data_margin;
		limit_lower = ts->pdata->limit->raw_data_otp_min - ts->pdata->limit->raw_data_margin;
	} else {
		limit_upper = ts->pdata->limit->raw_data_max + ts->pdata->limit->raw_data_margin;
		limit_lower = ts->pdata->limit->raw_data_min - ts->pdata->limit->raw_data_margin;
	}

	for (row = 0; row < ts->dev.row_num; row++) {
		for (col = 0; col < ts->dev.col_num; col++) {
			if (ts->mit_data[row][col] < limit_lower || ts->mit_data[row][col] > limit_upper)
				ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 0; //result determine
		}
	}
	*/

	return ret;
}

int MIT200_DoTest(struct i2c_client *client, int type){
  	int busy_cnt = 50;
	int wait_cnt = 50;
	u8 wbuf[8];
	u8 test_type = 0;
	int reg_type = 0;
	struct mit200_data_format *data_form = NULL;
	struct mit200_buf_addr *buf_addr = NULL;
	
	/* Allocate memory */
    data_form = kzalloc(sizeof(struct mit200_data_format), GFP_KERNEL);
	if (data_form == NULL) {
		TOUCH_ERR("read_buf mem_error\n");
		goto ERROR;
	}  
	buf_addr = kzalloc(sizeof(struct mit200_buf_addr), GFP_KERNEL);
	if (buf_addr == NULL) {
		TOUCH_ERR("read_buf mem_error\n");
		goto ERROR;
	}

	/* Check test type */

	switch(type){
		case RAW_DATA_SHOW:
			TOUCH_LOG("=== Rawdata image ===\n");
			test_type = MIP_IMG_TYPE_RAWDATA;
			//reg_type = IMGREG;
			reg_type = TESTREG;
			break;
		case RAW_DATA_SHOW_IMG:
			TOUCH_LOG("=== Rawdata image(IMGREG) ===\n");
			test_type = MIP_IMG_TYPE_RAWDATA;
			reg_type = IMGREG;
			break;
		case INTENSITY_SHOW:
			TOUCH_LOG("=== Intensity image ===\n");
			test_type = MIP_IMG_TYPE_INTENSITY;
			reg_type = IMGREG;
			break;
		/*case ABS_SHOW:
			TOUCH_LOG("=== ABS test ===\n");
			test_type = MIP_TEST_TYPE_CM_ABS;
			reg_type = TESTREG;
			break;*/
		/*case DELTA_SHOW:
			TOUCH_LOG("=== Delta test ===\n");
			test_type = MIP_TEST_TYPE_CM_DELTA;
			reg_type = TESTREG;
			break;*/
		case JITTER_SHOW:
			TOUCH_LOG("=== Jitter test ===\n");
			test_type = MIP_TEST_TYPE_CM_JITTER;
			reg_type = TESTREG;
			break;
		case OPENSHORT_SHOW:
			TOUCH_LOG("=== Short1 test ===\n");
			test_type = MIP_TEST_TYPE_SHORT;
			reg_type = TESTREG;
			break;
		case OPENSHORT2_SHOW:
			TOUCH_LOG("=== Short2 test ===\n");
			test_type = MIP_TEST_TYPE_SHORT2;
			reg_type = TESTREG;
			break;
		case OPEN_SHOW : 
			TOUCH_LOG("== OPEN test ==\n");
			test_type = MIP_TEST_TYPE_OPEN ;
			reg_type = TESTREG;
			break;
		default:
			TOUCH_LOG("type select error, type : %d", type);
			goto ERROR;
	}

	TOUCH_LOG("reg_type = %s, test type = %d\n", (reg_type==TESTREG)?"Test Register":"Image Register", test_type);

	/* Check busy status */
	while(busy_cnt--){
		if(test_busy == false){
			break;
		}
		TOUCH_LOG("busy_cnt = %d\n", busy_cnt);
		msleep(10);
	}
	test_busy = true;

	/* Disable Interrupt */
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_EVENT_TRIGGER_TYPE;
	wbuf[2] = MIP_TRIGGER_TYPE_NONE;
	if (Mit200_proc4_I2C_Write(client, wbuf, 3) < 0) {
		TOUCH_ERR("[ERROR] Write - Disable interrupt\n");
		goto ERROR;
	}

	if (reg_type == TESTREG) { //Read Test Register (Change test mode)
		/* Set test mode */
		wbuf[0] = MIP_R0_CTRL;
		wbuf[1] = MIP_R1_CTRL_MODE;
		wbuf[2] = MIP_CTRL_MODE_TEST_CM;
		if (Mit200_proc4_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("[ERROR] Write - test mode\n");
			goto ERROR;
		}
		/* Wait ready status */
		wait_cnt = 50;
		while(wait_cnt--){
			if(MIT200_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
				break;
			}
			msleep(50);
			//TOUCH_LOG("wait_cnt : %d\n", wait_cnt);
		}
		if (wait_cnt <= 0){
			TOUCH_ERR("[ERROR] Wait timeout\n");
			goto ERROR;
		}

		/* Set test type */
		wbuf[0] = MIP_R0_TEST;
		wbuf[1] = MIP_R1_TEST_TYPE;
		wbuf[2] = test_type;
		if (Mit200_proc4_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("%s [ERROR] Write - test type\n", __func__);
			goto ERROR;
		}
		/* Wait ready status */
		wait_cnt = 100;
		while(wait_cnt--){
			if(MIT200_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
				TOUCH_LOG("wait_cnt : %d\n", wait_cnt);
				break;
			}
			msleep(50);
		}
		if (wait_cnt <= 0){
			TOUCH_ERR("[ERROR] Wait timeout\n");
			goto ERROR;
		}
	} else if (reg_type == IMGREG) { //Read Image Register
		/* Set image type */
		wbuf[0] = MIP_R0_IMAGE;
		wbuf[1] = MIP_R1_IMAGE_TYPE;
		wbuf[2] = test_type;
		if (Mit200_proc4_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("%s [ERROR] Write - image type\n", __func__);
			goto ERROR;
		}
	/* Wait ready status */
	wait_cnt = 50;
	while(wait_cnt--){
		if(MIT200_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
			break;
			}
			msleep(50);
		}
	}
	if (wait_cnt <= 0){
		TOUCH_ERR("[ERROR] Wait timeout\n");
		goto ERROR;
	}

	/* Read self-diagnosis information */
	//set test type
	if (MIT200_GetDataFormat(client, data_form, reg_type)) {
		TOUCH_ERR("[ERROR] MIT200_GetDataFormat\n");
		goto ERROR;
	}
	
	//get buf addr
	if (MIT200_GetBufAddr(client, buf_addr, reg_type)) {
		TOUCH_ERR("[ERROR] MIT200_GetBufAddr\n");
		goto ERROR;
	}
	
	//print data
	if(MIT200_ReadRMIBuffer(client, data_form, buf_addr)) {
		TOUCH_ERR("[ERROR] MIT200_ReadRMIBuffer\n");
		goto ERROR;
	}

	if (reg_type == TESTREG) {
		/* Set normal mode */
		wbuf[0] = MIP_R0_CTRL;
		wbuf[1] = MIP_R1_CTRL_MODE;
		wbuf[2] = MIP_CTRL_MODE_NORMAL;
		if (Mit200_proc4_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("[ERROR] Write image type\n");
			goto ERROR;
		}

		/* Wait ready status */
		wait_cnt = 50;
		while(wait_cnt--){
			if(MIT200_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
				break;
			}
			msleep(50);
			//TOUCH_LOG("wait_cnt : %d\n", wait_cnt);
		}
		if (wait_cnt <= 0){
			TOUCH_ERR("[ERROR] Wait timeout\n");
			goto ERROR;
		}
	} else if (reg_type == IMGREG) {
		/* Clear image type */
		wbuf[0] = MIP_R0_IMAGE;
		wbuf[1] = MIP_R1_IMAGE_TYPE;
		wbuf[2] = MIP_IMG_TYPE_NONE;
		if (Mit200_proc4_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("[ERROR] Write image type\n");
			goto ERROR;
		}
	}

	/* Enable Interrupt */
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_EVENT_TRIGGER_TYPE;
	wbuf[2] = MIP_TRIGGER_TYPE_INTR;
	if (Mit200_proc4_I2C_Write(client, wbuf, 3) < 0) {
		TOUCH_ERR("[ERROR] Write - Enable interrupt\n");
		goto ERROR;
	}

	test_busy = false;

	if (data_form != NULL) {
		kfree(data_form);
        }
	if (buf_addr != NULL) {
		kfree(buf_addr);
        }

	TOUCH_LOG("%s [DONE]\n", __func__);
	return TOUCH_SUCCESS;

ERROR:
	if (data_form != NULL) {
		kfree(data_form);
        }
	if (buf_addr != NULL) {
		kfree(buf_addr);
        }
	test_busy = false;
	TOUCH_ERR("%s [ERROR]\n", __func__);
	return TOUCH_FAIL;
}

ssize_t MIT200_GetTestResult(struct i2c_client *client, char *pBuf, int *result, int type)
{
	int i = 0;
	int ret = 0;
	int retry_max = 3;
	int retry_count = 0;
	
	TOUCH_FUNC();

	for ( i = 0 ; i < MAX_ROW ; i++) {
		memset(mit_data[i], 0, sizeof(uint16_t) * MAX_COL);
		memset(intensity_data[i], 0x00, sizeof(s16) * MAX_COL);
	}

	retry_count = 0;
	while (retry_count++ < retry_max) {
		if (MIT200_DoTest(client, type) == TOUCH_FAIL) {
			TOUCH_ERR("Getting data failed, retry (%d/%d)\n", retry_count, retry_max);
			TouchPower(0);
			TouchPower(1);
			mdelay(100);
		} else { break; }

		if (retry_count >= retry_max)
		{
			TOUCH_ERR("%s all retry failed \n", __func__);
			goto ERROR;
		}
	}

	/*--------------------------------------*/
	/* To Do List*/
	/*1. compare to limitation value and test value.*/
	/*2. check the result value.*/
	/*--------------------------------------*/

	ret += MIT200_PrintData(client, pBuf, type, result);
	if (ret < 0) {
		TOUCH_ERR("fail to print type data\n");
		goto ERROR;
	}

	//*result = TOUCH_SUCCESS;
	return ret; // ret is size of data buffer .

ERROR:
	*result = TOUCH_FAIL;
	return TOUCH_FAIL;
}


