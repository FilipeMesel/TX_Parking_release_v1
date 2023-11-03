/*
 * FXOS8700.h
 *
 *  Created on: Oct 31, 2023
 *      Author: Filipe Mesel
 */

/*
 * Criei essa lib com base em:
 * https://github.com/leech001/MPU6050 para ver o funcionamento do i2c em modo read e write
 *
 * E em:
 * https://www.nxp.com/docs/en/data-sheet/FXOS8700CQ.pdf
 *
 * Das páginas 25 a 28, onde há o exemplo de configuração e leitura dos dados do acelerômetro e magnômetro
 *
 * */

#ifndef INC_FXOS8700_H_
#define INC_FXOS8700_H_

#endif /* INC_FXOS8700_H_ */
#include "main.h"
#include <stdint.h>
#include <math.h>

// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS 						0x00 //Status register
#define FXOS8700CQ_WHOAMI 						0x0D //Who am i register
#define FXOS8700CQ_XYZ_DATA_CFG 				0x0E //XYZ Data register
#define FXOS8700CQ_CTRL_REG1 					0x2A //CTRL Register
#define FXOS8700CQ_M_CTRL_REG1 					0x5B //CTRL Magnetometter register
#define FXOS8700CQ_M_CTRL_REG2 					0x5C //CTRL 2 Magnetometter register
#define FXOS8700CQ_WHOAMI_VAL 					0xC7 //Who am i response
// number of bytes to be read from the FXOS8700CQ
#define FXOS8700CQ_READ_LEN 					13 // status plus 6 channels = 13 bytes

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)

/*
 * O endereço pode ser:
 * 0x1E quando SA0 = 0, SA1 = 0
 * 0x1D quando SA0 = 1, SA1 = 0
 * 0x1C quando SA0 = 0, SA1 = 1
 * 0x1F quando SA0 = 1, SA1 = 1
 * */
#define FXOS8700CQ_SLAVE_ADDR 					0x1E // with pins SA0=0, SA1=0

#define i2c_timeout 							500//20000

enum
{
	GET_MAG_X,
	GET_MAG_Y,
	GET_MAG_Z,
	GET_ACCEL_X,
	GET_ACCEL_Y,
	GET_ACCEL_Z
};

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;

	/*double value_x;
	double value_y;
	double value_z;*/

} SRAWDATA;

/*
 * Config function
 *
*/
uint8_t magnetometter_init(I2C_HandleTypeDef *I2Cx);

uint8_t i2cTransmit(I2C_HandleTypeDef *I2Cx);
/**
 * Function to test if sensor's ok
 */
uint8_t magnetometter_get_who_am_i(I2C_HandleTypeDef *I2Cx);

/*
 * Read data function
 *
*/
static void ReadAccelMagnData(I2C_HandleTypeDef *I2Cx);

/**
 * Get data from struct
 */
static int16_t getData(uint8_t dado_a_receber);
