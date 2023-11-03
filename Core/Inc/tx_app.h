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

/* Exported types ----------------------------------------------------------------*/
typedef void (AppDioIrqHandler)(void);
typedef enum{
	LED_MODE_TX,
	LED_MODE_LORAWAN,
	LED_MODE_P2P,
	LED_MODE_RESET_CNT,
	LED_MODE_OFF
}led_mode_t;


/* Exported variables ------------------------------------------------------------*/
extern UTIL_TIMER_Object_t LedTimer;

extern EXTI_HandleTypeDef hApp_DIO_exti[];
extern AppDioIrqHandler *AppDioIrq[];

/* Exported functions prototypes ---------------------------------------------*/
void TxAppInit(void);

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

#define FIRMWARE_VERSION				"V1_1_0"
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

#define CLICKS_TO_CHANGE_MODE			10
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
#define LED_CRC_ON_BLINK_TIMES						9
#define LED_CRC_ON_BLINK_TIME						100
#define LED_CRC_ON_ON_TIME							100
#define LED_CRC_OFF_BLINK_TIMES							0
#define LED_CRC_OFF_BLINK_TIME							1000
#define LED_CRC_OFF_ON_TIME								2000
#define LED_RESET_CNT_TIMES							3
#define LED_RESET_CNT_TIME							200
#define INPUT_DEBOUNCE_TIME							100

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

#endif
