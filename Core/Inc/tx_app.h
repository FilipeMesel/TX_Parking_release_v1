/*
 * tx_app.h
 *
 *  Created on: 19 de abr de 2021
 *      Author: Temistocles Chalaca
 */

#ifndef INC_TX_APP_H_
#define INC_TX_APP_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32_timer.h"
#include <stdint.h>
#include <math.h>

/* Exported types ----------------------------------------------------------------*/
typedef void (AppDioIrqHandler)(void);
typedef enum{
	LED_MODE_TX,
	LED_MODE_LORAWAN,
	LED_MODE_P2P,
	LED_MODE_OFF
}led_mode_t;


/* Exported variables ------------------------------------------------------------*/
extern UTIL_TIMER_Object_t LedTimer;

extern EXTI_HandleTypeDef hApp_DIO_exti[];
extern AppDioIrqHandler *AppDioIrq[];

/* Exported functions prototypes ---------------------------------------------*/
void TxAppInit(void);


int getInterruptFlag(void);
void tratarInterrupcao(void);
void resetInterruptFlag(void);
uint8_t magnetometerCheckCommunication(void);
void FXOS8700CQForceSleep(void);

/* Exported constants --------------------------------------------------------*/

/* LoraWAN application configuration (Mw is configured by lorawan_conf.h) */
#define ACTIVE_REGION                               LORAMAC_REGION_AU915

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
/*#define CAYENNE_LPP*/

/*!
 * Defines the application data transmission duty cycle. 10s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            30000

/*!
 * LoRaWAN User application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_USER_APP_PORT                       1

/*!
 * LoRaWAN Switch class application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_SWITCH_CLASS_PORT                   3

/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A

/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when LORAWAN_ADR_STATE is disabled
 */
#define LORAWAN_DEFAULT_DATA_RATE                   DR_0

/*!
 * LoRaWAN default activation type
 */
#define LORAWAN_DEFAULT_ACTIVATION_TYPE             ACTIVATION_TYPE_ABP

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * Default Unicast ping slots periodicity
 *
 * \remark periodicity is equal to 2^LORAWAN_DEFAULT_PING_SLOT_PERIODICITY seconds
 *         example: 2^3 = 8 seconds. The end-device will open an Rx slot every 8 seconds.
 */
#define LORAWAN_DEFAULT_PING_SLOT_PERIODICITY       4

#define FIRMWARE_VERSION				"V1_1_1"
#define MODEL_CODE						"PLCD.019-024"

#define APP_DOWNLINK_PORT				1

#define H_APP_EXTI_14					hApp_DIO_exti[0]
#define H_APP_EXTI_13					hApp_DIO_exti[1]
#define H_APP_EXTI_7					hApp_DIO_exti[2]
#define H_APP_EXTI_8					hApp_DIO_exti[3]
#define H_APP_EXTI_6					hApp_DIO_exti[4]

#define LED_TX_PORT						GPIOA
#define LED_TX_PIN						GPIO_PIN_8
#define EN_INPUT_PULSE_PORT				GPIOB
#define EN_INPUT_PULSE_PIN				GPIO_PIN_5

#define DEFAULT_TIME_TX					60

#define SAVE_CNT_OFF					0
#define SAVE_CNT_AT_PWR_OFF				1
#define SAVE_CNT_ON_PULSE				2
#define SAVE_CNT_MODE					SAVE_CNT_OFF
#define BLINK_PULSE_LED_ENABLE			0

#define CLICKS_TO_CHANGE_MODE			5//10
#define CLICKS_TO_TX					1
#define BTN_TIMEOUT						1000

#define DEFAULT_XTA_TRIM_VALUE			20

#define P2P_RF_FREQUENCY				915000000 /* Hz */

#define P2P_TX_OUTPUT_POWER				22        /* dBm */

#define P2P_BANDWIDTH                              0         /* [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] */
#define P2P_SPREADING_FACTOR                       12         /* [SF7..SF12] */
#define P2P_CODINGRATE                             1         /* [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
#define P2P_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define P2P_SYMBOL_TIMEOUT                         5         /* Symbols */
#define P2P_FIX_LENGTH_PAYLOAD_ON                  false
#define P2P_IQ_INVERSION_ON                        false
#define P2P_TX_TIMEOUT_VALUE						5000
#define LORA_BUFFER_SIZE                                 64  /* Define the payload size here */

#define LED_TX_BLINK_TIMES							5
#define LED_TX_LORAWAN_BLINK_TIME					100
#define LED_TX_LORAWAN_ON_TIME						6000
#define LED_TX_P2P_BLINK_TIME						500
#define LED_TX_P2P_ON_TIME							500
#define LED_LORAWAN_BLINK_TIMES						9
#define LED_LORAWAN_BLINK_TIME						100
#define LED_LORAWAN_ON_TIME							100
#define LED_P2P_BLINK_TIMES							0
#define LED_P2P_BLINK_TIME							1000
#define LED_P2P_ON_TIME								2000
#define INPUT_DEBOUNCE_TIME							10000//200//100

#define ADC_MAX_VALUE								4095
#define AN0_ADC_CHANNEL								ADC_CHANNEL_9
#define AN0_PIN										GPIO_PIN_1
#define AN0_PORT									GPIOB
#define ADC_DELAY_TIME								40
#define GPIO_AN0_ENABLE								GPIOA
#define PIN_AN0_ENABLE								GPIO_PIN_10

#define PWM_OUTPUT_GPIO								GPIOB
#define PWM_OUTPUT_PIN								GPIO_PIN_2
#define PWM_OUTPUT_MODE								GPIO_MODE_AF_PP
#define PWM_OUTPUT_PULL								GPIO_NOPULL
#define PWM_OUTPUT_SPEED							GPIO_SPEED_FAST
#define PWM_OUTPUT_AF								GPIO_AF2_LPTIM1

// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS 						0x00 //Status register
#define FXOS8700CQ_SYSMOD						0x0B //Sysmod register
#define FXOS8700CQ_INT_SOURCE					0x0C //Interrupt source register
#define FXOS8700CQ_WHOAMI 						0x0D //Who am i register
#define FXOS8700CQ_XYZ_DATA_CFG 				0x0E //XYZ Data register
#define FXOS8700CQ_CTRL_REG1 					0x2A //CTRL Register
#define FXOS8700CQ_M_CTRL_REG1 					0x5B //CTRL Magnetometter register
#define FXOS8700CQ_M_CTRL_REG2 					0x5C //CTRL 2 Magnetometter register
#define FXOS8700CQ_CTRL_REG2 					0x2B
#define FXOS8700CQ_M_CTRL_REG3 					0x5D //CTRL 3 Magnetometter register
#define FXOS8700CQ_CTRL_REG3 					0x2C
#define FXOS8700CQ_M_CTRL_REG4 					0x2D //CTRL 3 Magnetometter register
#define FXOS8700CQ_M_CTRL_REG5 					0x2E //CTRL 3 Magnetometter register
#define FXOS8700CQ_WHOAMI_VAL 					0xC7 //Who am i response

#define FXOS8700CQ_THRESHOLD_CFG_REG			0x52 // Magnetic threshold detection function configuration register

#define  FXOS8700CQ_M_THS_SRC_REG				0x53 //Indicador da polaridade do trigger observado.

#define FXOS8700CQ_INTERRUPT_X_MSB_REG 			0x54 //Registrador que configura o valor MSB para estourar a inerrupção no eixo X
#define FXOS8700CQ_INTERRUPT_X_LSB_REG 			0x55 //Registrador que configura o valor LSB para estourar a inerrupção no eixo X

#define FXOS8700CQ_INTERRUPT_Y_MSB_REG 			0x56 //Registrador que configura o valor MSB para estourar a inerrupção no eixo Y
#define FXOS8700CQ_INTERRUPT_Y_LSB_REG 			0x57 //Registrador que configura o valor LSB para estourar a inerrupção no eixo Y

#define FXOS8700CQ_INTERRUPT_Z_MSB_REG 			0x58 //Registrador que configura o valor MSB para estourar a inerrupção no eixo Z
#define FXOS8700CQ_INTERRUPT_Z_LSB_REG 			0x59 //Registrador que configura o valor LSB para estourar a inerrupção no eixo Z

#define FXOS8700CQ_THRESHOLD_DBC_COUNTER_REG	0x5A // Magnetic threshold debounce counter

#define FXOS8700CQ_M_INT_SRC_REG				0x5E // Magnetic threshold debounce counter

#define FXOS8700CQ_ASLP_COUNT_REG				0x29 // ASLP_COUNT Auto Sleep Inactivity Timer Register


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

#define i2c_timeout 							500//100//20000

enum
{
	GET_MAG_X,
	GET_MAG_Y,
	GET_MAG_Z,
	GET_ACCEL_X,
	GET_ACCEL_Y,
	GET_ACCEL_Z,
	INTERRUPT_MAIOR_QUE,
	INTERRUPT_MENOR_QUE,
	ESTACIONAMENTO_CARRO_SAIU,
	ESTACIONAMENTO_CARRO_CHEGOU
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

#endif
