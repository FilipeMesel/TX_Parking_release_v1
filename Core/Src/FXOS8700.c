/*
 * FXOS8700.c
 *
 *  Created on: Nov 3, 2023
 *      Author: JESUS TE AMA
 */

#include <math.h>
#include "FXOS8700.h"

SRAWDATA MAG;
SRAWDATA ACCEL;

uint8_t magnetometter_get_who_am_i(I2C_HandleTypeDef *I2Cx)
{
	/*uint8_t check; //Check if device is ok
	//20000
	HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_WHOAMI, 1, &check, 1, 20000);
	return check; //Check if device is ok*/

	uint8_t check; // Check if the device is ok
	uint8_t txData = FXOS8700CQ_WHOAMI; // Address of the WHO_AM_I register
	HAL_StatusTypeDef status;

	// Transmit the register address
	status = HAL_I2C_Master_Transmit(I2Cx, (uint16_t)(FXOS8700CQ_SLAVE_ADDR << 1), &txData, 1, 50);
	for(int i=0; i<20000; i++);
	if (status == HAL_OK) {
	    // Receive the data from the WHO_AM_I register
	    status = HAL_I2C_Master_Receive(I2Cx,  (uint16_t)(FXOS8700CQ_SLAVE_ADDR), &check, 1, 50);

	    if (status != HAL_OK) {
	        // Handle the receive error here, e.g., set an error flag or handle it as needed
	    }
	} else {
	    // Handle the transmit error here, e.g., set an error flag or handle it as needed
	}

	return check; // Check if the device is ok
}

uint8_t magnetometter_init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t ret = 1;
	uint8_t check; //Check if device is ok
	uint8_t Data; //Receive data
	uint8_t databyte; //Write data

	// read and check the FXOS8700CQ WHOAMI register
	HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_WHOAMI, 1, &check, 1, i2c_timeout);
	if(check == FXOS8700CQ_WHOAMI_VAL)
	{
		// write 0000 0000 = 0x00 to accelerometer control register 1 to place FXOS8700CQ into
		// standby
		// [7-1] = 0000 000
		// [0]: active=0
		databyte = 0x00;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &databyte, 1, i2c_timeout);

		// write 0001 1111 = 0x1F to magnetometer control register 1
		// [7]: m_acal=0: auto calibration disabled
		// [6]: m_rst=0: no one-shot magnetic reset
		// [5]: m_ost=0: no one-shot magnetic measurement
		// [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
		// [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
		databyte = 0x1F;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_CTRL_REG1, 1, &databyte, 1, i2c_timeout);

		// write 0010 0000 = 0x20 to magnetometer control register 2
		// [7]: reserved
		// [6]: reserved
		// [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the
		// accelerometer registers
		// [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
		// [3]: m_maxmin_dis_ths=0
		// [2]: m_maxmin_rst=0
		// [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
		databyte = 0x20;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_CTRL_REG2, 1, &databyte, 1, i2c_timeout);

		// write 0000 0001= 0x01 to XYZ_DATA_CFG register
		// [7]: reserved
		// [6]: reserved
		// [5]: reserved
		// [4]: hpf_out=0
		// [3]: reserved
		// [2]: reserved
		// [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB
		databyte = 0x01;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_XYZ_DATA_CFG, 1, &databyte, 1, i2c_timeout);

		// write 0000 1101 = 0x0D to accelerometer control register 1
		// [7-6]: aslp_rate=00
		// [5-3]: dr=001 for 200Hz data rate (when in hybrid mode)
		// [2]: lnoise=1 for low noise mode
		// [1]: f_read=0 for normal 16 bit reads
		// [0]: active=1 to take the part out of standby and enable sampling
		databyte =  0x0D;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &databyte, 1, i2c_timeout);
		ret = 0;
	}
	return ret;
}

// read status and the three channels of accelerometer and magnetometer data from
// FXOS8700CQ (13 bytes)
//void ReadAccelMagnData(SRAWDATA *pAccelData, SRAWDATA *pMagnData, I2C_HandleTypeDef *I2Cx)
void ReadAccelMagnData(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Buffer[FXOS8700CQ_READ_LEN]; // read buffer

	// read FXOS8700CQ_READ_LEN=13 bytes (status byte and the six channels of data)
	HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_STATUS, 1, &Buffer, FXOS8700CQ_READ_LEN, i2c_timeout);

	// copy the 14 bit accelerometer byte data into 16 bit words
	ACCEL.x = (int16_t)(((Buffer[1] << 8) | Buffer[2]))>> 2;
	ACCEL.y = (int16_t)(((Buffer[3] << 8) | Buffer[4]))>> 2;
	ACCEL.z = (int16_t)(((Buffer[5] << 8) | Buffer[6]))>> 2;

	// copy the magnetometer byte data into 16 bit words
	MAG.x = (Buffer[7] << 8) | Buffer[8];
	MAG.y = (Buffer[9] << 8) | Buffer[10];
	MAG.z = (Buffer[11] << 8) | Buffer[12];

}

/**
 * Get data from struct
 */
int16_t getData(uint8_t dado_a_receber)
{
	switch(dado_a_receber)
	{
	case GET_MAG_X:
	{

		return MAG.x;
	}
	break;
	case GET_MAG_Y:
	{

		return MAG.y;
	}
	break;
	case GET_MAG_Z:
	{

		return MAG.z;
	}
	break;
	case GET_ACCEL_X:
	{

		return ACCEL.x;
	}
	break;
	case GET_ACCEL_Y:
	{

		return ACCEL.y;
	}
	break;
	case GET_ACCEL_Z:
	{
		return ACCEL.z;

	}
	break;

	}
}
