/*
 * tx_app.c
 *
 *  Created on: 19 de abr de 2021
 *      Author: Temistocles Chalaca
 */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "tx_app.h"
#include "sys_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lorawan_conf.h"
#include "lora_info.h"
#include "radio.h"
#include "radio_board_if.h"
#include "LmHandler.h"
#include <stddef.h>
#include "usart_if.h"
#include "adc_if.h"
#include "stm32_lpm.h"
#include "stm32_adv_trace.h"
#include "sx126x.h"


static  I2C_HandleTypeDef hi2c2;

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint32_t line;
	uint32_t prio;
	uint32_t exti_port;
	GPIO_TypeDef *gpio_port;
	uint16_t pin;
	IRQn_Type irqn;
}app_exti_t;
typedef struct{
	GPIO_TypeDef *gpio_port;
	uint16_t pin;
}output_pin_t;

typedef enum{
	LORA_IDLE,
	LORA_BUSY
}lora_state_t;

typedef enum{
	LORAWAN_MODE,
	P2P_MODE
}tx_mode_t;

typedef enum{
	SEND_MODE_AUTO,
	SEND_MODE_MANUAL
}send_mode_t;

/* Private define ------------------------------------------------------------*/
#define SEND_TIMES			1

/* Private variables ---------------------------------------------------------*/
/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

UTIL_TIMER_Object_t inputTimer;
UTIL_TIMER_Object_t LedTimer;
UTIL_TIMER_Object_t ResetCntDelayTimer;

LPTIM_HandleTypeDef hlptim;

#if (BLINK_PULSE_LED_ENABLE == 1)
UTIL_TIMER_Object_t LedPulseTimer;
#endif
UTIL_TIMER_Object_t btnTimeOutTimer;
const app_exti_t app_exti_cnt[] = {
		{.line = EXTI_LINE_14, .prio = 0, .exti_port = EXTI_GPIOA, .gpio_port = GPIOA, .pin = GPIO_PIN_14, .irqn = EXTI4_15_IRQn},
		{.line = EXTI_LINE_13, .prio = 0, .exti_port = EXTI_GPIOA, .gpio_port = GPIOA, .pin = GPIO_PIN_13, .irqn = EXTI4_15_IRQn},
		{.line = EXTI_LINE_7, .prio = 0, .exti_port = EXTI_GPIOB, .gpio_port = GPIOB, .pin = GPIO_PIN_7, .irqn = EXTI4_15_IRQn},
		{.line = EXTI_LINE_8, .prio = 0, .exti_port = EXTI_GPIOB, .gpio_port = GPIOB, .pin = GPIO_PIN_8, .irqn = EXTI4_15_IRQn},
};
const app_exti_t app_exti_btn[] = {
		{.line = EXTI_LINE_6, .prio = 0, .exti_port = EXTI_GPIOB, .gpio_port = GPIOB, .pin = GPIO_PIN_6, .irqn = EXTI4_15_IRQn}
};
const output_pin_t output_pin[] = {
		{.gpio_port = LED_TX_PORT, .pin = LED_TX_PIN},
		{.gpio_port = EN_INPUT_PULSE_PORT, .pin = EN_INPUT_PULSE_PIN}
};
#define APP_EXTI_CNT_NUM	(sizeof(app_exti_cnt)/sizeof(app_exti_t))
#define APP_EXTI_BTN_NUM	(sizeof(app_exti_btn)/sizeof(app_exti_t))
#define APP_OUTPUT_PIN_NUM	(sizeof(output_pin)/sizeof(output_pin_t))
EXTI_HandleTypeDef hApp_DIO_exti[APP_EXTI_CNT_NUM+APP_EXTI_BTN_NUM];
uint16_t eeprom_wr_pos[APP_EXTI_CNT_NUM] = {0,0,0,0};

typedef union{
	struct cfga{
		//uint32_t pulse_count[APP_EXTI_CNT_NUM];
		uint32_t dev_addr;
		uint16_t tx_time;
		uint8_t xta_trim;
		tx_mode_t tx_mode;
	}cfg;
	uint32_t word[(sizeof(struct cfga)+sizeof(uint32_t)-1)/sizeof(uint32_t)];
}cfg_t;

cfg_t eeprom_cfg  __attribute__ ((section (".eeprom_rodata"))) = {
		.cfg = {
					//.pulse_count = { 0, 0, 0, 0},
					.dev_addr = 0,
					.tx_time = DEFAULT_TIME_TX,
					.xta_trim = DEFAULT_XTA_TRIM_VALUE,
					.tx_mode = LORAWAN_MODE
		}
};

char fver[] __attribute__ ((section (".eeprom_rodata"))) = FIRMWARE_VERSION;
char model[] __attribute__ ((section (".eeprom_rodata"))) = MODEL_CODE;

typedef struct{
	struct{
		//uint32_t pulse_count_tmp[APP_EXTI_CNT_NUM];
		uint16_t btnCnt;
		uint16_t tx_cnt;
		uint8_t cnt_per_hour[APP_EXTI_CNT_NUM][5];
		send_mode_t send_mode;
		led_mode_t led_mode;
		lora_state_t lora_state;
	}stt;
	cfg_t cfg;
}app_t;

app_t app = {
		.stt.btnCnt = 0,
		.stt.led_mode = LED_MODE_OFF,
		.stt.lora_state = LORA_IDLE
};
/* Radio events function pointer */
static RadioEvents_t RadioEvents;
uint16_t LoRaBufferSize = LORA_BUFFER_SIZE;
uint8_t LoRaBuffer[LORA_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
uint16_t fillBufferAutoTx(uint8_t *buffer);
uint16_t fillBufferManualTx(uint8_t *buffer);
void LED_TX(GPIO_PinState state);
void LED_Blink(led_mode_t mode);
static void SendTxData(void);
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);
static void OnTxData(LmHandlerTxParams_t *params);
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);
static void OnMacProcessNotify(void);

static void OnTimer(void *contextid);
static void OnTxTimerLedEvent(void *context);
void OnDelayResetCnt(void *context);

#if (BLINK_PULSE_LED_ENABLE == 1)
static void OnLedPulseTimer(void *contextid);
#endif
static void OnBtnTimeoutTimer(void *contextid);
static void ReadInput0(void);
static void ReadInput1(void);
static void ReadInput2(void);
static void ReadInput3(void);
static void ReadInput4(void);
void TxAppInit(void);
void EXTI_Init(AppDioIrqHandler **irqHandlers);
void EEPROM_Write(uint32_t d, uint32_t *addr);
uint32_t *EEPROM_NextAddr(uint16_t offset);
void LoRaWAN_Mode_Init(void);
void P2P_Radio_Init(void);
void rx_bytes(uint8_t *, uint16_t, uint8_t);
static void OnP2PTxDone(void);
static void OnP2PRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
static void OnP2PTxTimeout(void);
static void OnP2PRxTimeout(void);
static void OnP2PRxError(void);



static void MX_I2C1_Init(void);
static int16_t flux;
static int16_t savedValue;
static uint8_t vagaStatus = VAGA_VAZIA;
//static int16_t lastFlux;

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
	  hi2c2.Instance = I2C2;
	  hi2c2.Init.Timing = 0x10909CEC;//0x00303D5B; // Timing configurado para operação a 400 kHz em Fast Mode
	  hi2c2.Init.OwnAddress1 = 0;
	  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c2.Init.OwnAddress2 = 0;
	  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	  {
	    Error_Handler(); // Tratamento de erro
	  }
	  /** Configure Digital filter
	  */
	  /*if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }*/
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

static SRAWDATA MAG;

/* Private variables ---------------------------------------------------------*/
/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks = {
  .GetBatteryLevel =           GetBatteryLevel,
  .GetTemperature =            GetTemperatureLevel,
  .OnMacProcess =              OnMacProcessNotify,
  .OnJoinRequest =             OnJoinRequest,
  .OnTxData =                  OnTxData,
  .OnRxData =                  OnRxData
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams = {
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingPeriodicity =          LORAWAN_DEFAULT_PING_SLOT_PERIODICITY
};

AppDioIrqHandler *AppDioIrq[APP_EXTI_CNT_NUM+APP_EXTI_BTN_NUM] = {ReadInput0, ReadInput1, ReadInput2, ReadInput3, ReadInput4};


/*
 * Inicia o FXOS8700CQ sem o modo sleep. Habilita o Magnetômetro e o Acelerômetro
 * */
static uint8_t FXOS8700CQInit(I2C_HandleTypeDef *I2Cx)
{
	uint8_t databyte; //Write data

	// read and check the FXOS8700CQ WHOAMI register
	HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_WHOAMI, 1, &databyte, 1, i2c_timeout);
	if(databyte == FXOS8700CQ_WHOAMI_VAL)
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
		databyte = 0x05;//Original 0x1F;
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
		databyte =  0xE9;//Original 0x0D;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &databyte, 1, i2c_timeout);
		return 0;

	}
	return 1;
}

int16_t squareRoot(int32_t number) {
    int32_t x = number;
    int32_t y = 1;
    int32_t error = number;

    while (x - y > 1) {
        x = (x + y) / 2;
        y = number / x;
        if (x - y < 0) {
            error = y - x;
        }
        else {
            error = x - y;
        }
    }

    return (int16_t)x;
}

/* Private user code ---------------------------------------------------------*/

void TxAppInit(void){
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
	for(uint8_t i = 0; i < (sizeof(app.cfg)/sizeof(uint32_t)); i++){
		((uint32_t *)(&app.cfg))[i] = ((uint32_t *)(&eeprom_cfg))[i];
	}
	SetDevAddr(app.cfg.cfg.dev_addr);
	/*app.cfg.cfg.pulse_count[0] = 0;
	app.cfg.cfg.pulse_count[1] = 0;
	app.cfg.cfg.pulse_count[2] = 0;
	app.cfg.cfg.pulse_count[3] = 0;*/
	app.cfg.cfg.tx_time = DEFAULT_TIME_TX;
	app.cfg.cfg.xta_trim = DEFAULT_XTA_TRIM_VALUE;
	app.cfg.cfg.tx_mode = LORAWAN_MODE;
	/*Atualiza capacitores do cristal do sx*/
	SX126xFreqCorrection(app.cfg.cfg.xta_trim);
	/*Inicia recebimento por serial*/
	UTIL_ADV_TRACE_StartRxProcess(rx_bytes);
	/*Sempre inicia modo LoRaWAN*/
	LoRaWAN_Mode_Init();
#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	uint32_t tmp1, tmp2;
	uint16_t pos;
#endif
	FLASH_OBProgramInitTypeDef pOBInit = {
			.BORLevel = OB_BOR_OFF,
			.OptionType = OPTIONBYTE_BOR
	};
	HAL_FLASHEx_OBProgram(&pOBInit);
	EXTI_Init(AppDioIrq);
	GPIO_InitTypeDef gpio_cfg = {
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_LOW
	};
	for(uint8_t i = 0; i < APP_EXTI_CNT_NUM; i++){
		gpio_cfg.Pin = app_exti_cnt[i].pin;
		HAL_GPIO_Init(app_exti_cnt[i].gpio_port, &gpio_cfg);
		//app.stt.pulse_count_tmp[i] = 0;
#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
		/*escaneia a eeprom em busca do maior valor salvo para cada contador*/
		for(pos = i;pos < EEPROM_WORDS; pos+=4){
			tmp1 = eeprom_cfg.word[pos];
			if(tmp1 == 0){
				break;
			}
			if(pos < 4){/**/
				tmp2 = eeprom_cfg.word[pos+4];
				if(tmp1 > tmp2){
					tmp2 = eeprom_cfg.word[pos+(EEPROM_WORDS-4)];
					if(tmp1 > tmp2){
						break;
					}
				}
			}else if(pos >= (EEPROM_WORDS-4)){
				tmp2 = eeprom_cfg.word[pos-(EEPROM_WORDS-4)];
				if(tmp1 > tmp2){
					tmp2 = eeprom_cfg.word[pos-4];
					if(tmp1 > tmp2){
						break;
					}
				}
			}else{
				tmp2 = eeprom_cfg.word[pos+4];
				if(tmp1 > tmp2){
					tmp2 = eeprom_cfg.word[pos-4];
					if(tmp1 > tmp2){
						break;
					}
				}
			}
		}
		//app.cfg.pulse_count[i] = tmp1;
		eeprom_wr_pos[i] = pos+4;
#endif
	}
	if(app.cfg.cfg.tx_mode == P2P_MODE){
		P2P_Radio_Init();
	}
	gpio_cfg.Pull = GPIO_PULLUP;
	gpio_cfg.Speed = GPIO_SPEED_FAST;
	for(uint8_t i = 0; i < APP_EXTI_BTN_NUM; i++){
		gpio_cfg.Pin = app_exti_btn[i].pin;
		HAL_GPIO_Init(app_exti_btn[i].gpio_port, &gpio_cfg);
	}
	/*Configura pinos de saida*/
	gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_cfg.Pull = GPIO_NOPULL;
	gpio_cfg.Speed = GPIO_SPEED_LOW;
	for(uint8_t i = 0; i < APP_OUTPUT_PIN_NUM; i++){
		gpio_cfg.Pin = output_pin[i].pin;
		HAL_GPIO_Init(output_pin[i].gpio_port, &gpio_cfg);
	}
	LED_TX(false);
	HAL_GPIO_WritePin(EN_INPUT_PULSE_PORT, EN_INPUT_PULSE_PIN, GPIO_PIN_SET);
	UTIL_TIMER_Create(&inputTimer, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, OnTimer, NULL);
	UTIL_TIMER_SetPeriod(&inputTimer, app.cfg.cfg.tx_time*20);//60000
	UTIL_TIMER_Start(&inputTimer);
#if (BLINK_PULSE_LED_ENABLE == 1)
	UTIL_TIMER_Create(&LedPulseTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnLedPulseTimer, NULL);
	UTIL_TIMER_SetPeriod(&LedPulseTimer, 20);
#endif
	UTIL_TIMER_Create(&btnTimeOutTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnBtnTimeoutTimer, NULL);
	UTIL_TIMER_SetPeriod(&btnTimeOutTimer, BTN_TIMEOUT);
	UTIL_TIMER_Create(&LedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
	UTIL_TIMER_Create(&ResetCntDelayTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnDelayResetCnt, NULL);
	UTIL_TIMER_SetPeriod(&ResetCntDelayTimer, 3000);
#if	(SAVE_CNT_MODE == SAVE_CNT_AT_PWR_OFF)
	/*configura interrupcap por brown out*/
	PWR_PVDTypeDef sConfigPVD = {
			.Mode = PWR_PVD_MODE_IT_RISING,
			.PVDLevel = PWR_PVDLEVEL_0
	};
	HAL_PWR_ConfigPVD(&sConfigPVD);
	HAL_PWR_EnablePVD();
	EXTI_HandleTypeDef hexti;
	EXTI_ConfigTypeDef pExtiConfig = {
			.Line = EXTI_LINE_16,
			.Mode = EXTI_MODE_INTERRUPT,
			.Trigger = EXTI_TRIGGER_RISING
	};
	HAL_EXTI_SetConfigLine(&hexti, &pExtiConfig);
	HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(PVD_IRQn);
#endif
	/*Gera clock no pino de saida*//*
	hlptim.Instance = LPTIM1;
	hlptim.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	hlptim.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
	hlptim.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
	hlptim.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_LOW;
	hlptim.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
	HAL_LPTIM_Init(&hlptim);
	HAL_LPTIM_PWM_Start(&hlptim, 100, 50);
	*/
	/*Imprime Dev EUI e Versao de firmware ao iniciar*/
	uint8_t eui[8];
	LmHandlerGetDevEUI(&eui[0]);
	APP_LOG(TS_OFF, VLEVEL_L, "SN:%02X%02X%02X%02X%02X%02X%02X%02X\r\n", eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7]);
	APP_LOG(TS_OFF, VLEVEL_L, "%s\r\n", fver);
	APP_LOG(TS_OFF, VLEVEL_L, "%s\r\n", model);

	MX_I2C1_Init();



	uint8_t wiam = FXOS8700CQInit(&hi2c2);//FXOS8700CQInitFlux(&hi2c2);//FXOS8700CQInitSleep(&hi2c2, FXOS8700CQCalibrate(INTERRUPT_MAIOR_QUE), INTERRUPT_MAIOR_QUE);//magnetometter_init(&hi2c2);//magnetometter_init_sleep(&hi2c2, 65097);
	//APP_LOG(TS_OFF, VLEVEL_L, "WHO I AM %02X\r\n", wiam);


	uint8_t databyteHI=0, databyteLO = 0;
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x33, 1, &databyteHI, 1, i2c_timeout);
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x34, 1, &databyteLO, 1, i2c_timeout);
	MAG.x = (databyteHI << 8) | databyteLO;
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x35, 1, &databyteHI, 1, i2c_timeout);
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x36, 1, &databyteLO, 1, i2c_timeout);
	MAG.y = (databyteHI << 8) | databyteLO;
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x37, 1, &databyteHI, 1, i2c_timeout);
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x38, 1, &databyteLO, 1, i2c_timeout);
	MAG.z = (databyteHI << 8) | databyteLO;

	int32_t magX = MAG.x * MAG.x;
	int32_t magY = MAG.y * MAG.y;
	int32_t magZ = MAG.z * MAG.z;

	//savedValue = (squareRoot(magZ + magY + magX)) + 50;
	//APP_LOG(TS_OFF, VLEVEL_L, "flux: %d\r\n", flux);

	/*int32_t z =MAG.x *MAG.x;
	int32_t y = MAG.y * MAG.y;
	int32_t x = MAG.z * MAG.z;
	lastFlux = squareRoot(z + y + x);*/
	//APP_LOG(TS_OFF, VLEVEL_L, "lastFlux %d\r\n", lastFlux);
	if(wiam != 0)
	{
		//Reseta se wiam for diferente de 0
		HAL_Delay(10000);
	}
}
/*
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim){
	GPIO_InitTypeDef GPIO_Init;
	__HAL_RCC_LPTIM1_CONFIG(RCC_LPTIM1CLKSOURCE_LSE);
	//while (__HAL_RCC_GET_LPTIM1_SOURCE() != RCC_LPTIM1CLKSOURCE_LSE);
	__HAL_RCC_LPTIM1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_Init.Pin = PWM_OUTPUT_PIN;
	GPIO_Init.Alternate = PWM_OUTPUT_AF;
	GPIO_Init.Mode = PWM_OUTPUT_MODE;
	GPIO_Init.Pull = PWM_OUTPUT_PULL;
	GPIO_Init.Speed = PWM_OUTPUT_SPEED;
	HAL_GPIO_Init(PWM_OUTPUT_GPIO, &GPIO_Init);
}*/

/*
 * Interrupcao de brown out para salvar valores de contadores na eeprom
 */
#if	(SAVE_CNT_MODE == SAVE_CNT_AT_PWR_OFF)
void HAL_PWR_PVDCallback(void){
	for(uint8_t i = 0; i < APP_EXTI_CNT_NUM; i++){
		EEPROM_Write(((uint32_t *)(&app.cfg))[i], &eeprom_cfg.word[i]);
	}
}
#endif

/*
 * Salva uma Word (32 bits) na eeprom
 */
void EEPROM_Write(uint32_t d, uint32_t *addr){
	//unlock eeprom
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0){
		FLASH->PEKEYR = FLASH_PEKEY1;
		FLASH->PEKEYR = FLASH_PEKEY2;
	}
	*addr = d;
	//lock eeprom
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	FLASH->PECR |= FLASH_PECR_PELOCK;
}

#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
uint32_t *EEPROM_NextAddr(uint16_t offset){
	uint32_t *p;
	p = &eeprom_cfg.word[eeprom_wr_pos[offset]];
	eeprom_wr_pos[offset] += 4;
	if(eeprom_wr_pos[offset] >= EEPROM_WORDS){
		eeprom_wr_pos[offset] %= 4;
	}
	return p;
}
#endif

/*
 * Controle de picadas do LED
 */
void LED_Blink(led_mode_t mode){
	if(mode >= LED_MODE_OFF || app.stt.led_mode != LED_MODE_OFF){
		return;
	}
	LED_TX(true);
	app.stt.led_mode = mode;
	UTIL_TIMER_Stop(&LedTimer);
	switch(mode){
	case LED_MODE_TX:
		if(app.cfg.cfg.tx_mode == P2P_MODE){
			UTIL_TIMER_StartWithPeriod(&LedTimer, LED_TX_P2P_ON_TIME);
		}else{
			UTIL_TIMER_StartWithPeriod(&LedTimer, LED_TX_LORAWAN_ON_TIME);
		}
		break;
	case LED_MODE_LORAWAN:
		UTIL_TIMER_StartWithPeriod(&LedTimer, LED_CRC_ON_ON_TIME);
		break;
	case LED_MODE_P2P:
		UTIL_TIMER_StartWithPeriod(&LedTimer, LED_CRC_OFF_ON_TIME);
		break;
	case LED_MODE_RESET_CNT:
		UTIL_TIMER_StartWithPeriod(&LedTimer, LED_RESET_CNT_TIME);
		break;
	default:
		LED_TX(false);
		break;
	}
}

/*
 * Transmiste dados por LoRa P2P/LoRaWAN
 */
void App_Send(send_mode_t mode){
	uint8_t eui[10];
	if(app.stt.lora_state == LORA_IDLE){
		app.stt.lora_state = LORA_BUSY;
		LmHandlerGetDevEUI(&eui[0]);
		APP_LOG(TS_OFF, VLEVEL_L, "SN:%02X%02X%02X%02X%02X%02X%02X%02X\r\n", eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7]);
		/*salva valores dos contadores antes de cada transmissao*//*
		for(uint8_t i = 0; i < APP_EXTI_CNT_NUM; i++){
			EEPROM_Write(((uint32_t *)(&app.cfg))[i], &eeprom_cfg.word[i]);
		}*/
		LED_Blink(LED_MODE_TX);
		if(app.cfg.cfg.tx_mode == LORAWAN_MODE){
			if(mode == SEND_MODE_AUTO){
				AppData.Port = LORAWAN_USER_APP_PORT;
				AppData.BufferSize = fillBufferAutoTx(AppData.Buffer);
			}else{
				AppData.Port = LORAWAN_USER_APP_PORT;
				AppData.BufferSize = fillBufferManualTx(AppData.Buffer);
			}
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
		}else if(app.cfg.cfg.tx_mode == P2P_MODE){
			Radio.SetChannel(P2P_RF_FREQUENCY);
			LmHandlerGetDevEUI(&LoRaBuffer[0]);
			LoRaBufferSize = 8;
			if(mode == SEND_MODE_AUTO){
				LoRaBufferSize += fillBufferAutoTx((uint8_t *)&LoRaBuffer[LoRaBufferSize]);
			}else{
				LoRaBufferSize += fillBufferManualTx((uint8_t *)&LoRaBuffer[LoRaBufferSize]);
			}
			app.stt.send_mode = mode;
			Radio.Send(&LoRaBuffer[2], LoRaBufferSize-2);
			app.stt.tx_cnt = 0;
		}
	}
}

/*
 * Evento que ocorrera a cada uma hora para salvar as diferencas de contagem por hora e envia todas a cada 6 horas
 */
static void OnTimer(void *contextid){

	static uint8_t debounce;
	uint8_t databyteHI=0, databyteLO = 0, currentVagaStatus = vagaStatus;
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x33, 1, &databyteHI, 1, i2c_timeout);
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x34, 1, &databyteLO, 1, i2c_timeout);
	MAG.x = (databyteHI << 8) | databyteLO;
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x35, 1, &databyteHI, 1, i2c_timeout);
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x36, 1, &databyteLO, 1, i2c_timeout);
	MAG.y = (databyteHI << 8) | databyteLO;
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x37, 1, &databyteHI, 1, i2c_timeout);
	HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 0x38, 1, &databyteLO, 1, i2c_timeout);
	MAG.z = (databyteHI << 8) | databyteLO;

	int32_t magX = MAG.x * MAG.x;
	int32_t magY = MAG.y * MAG.y;
	int32_t magZ = MAG.z * MAG.z;

	flux = squareRoot(magZ + magY + magX);

	if(flux > savedValue)
	{
		debounce++;
		if(debounce > 3)
		{
			currentVagaStatus = VAGA_OCUPADA;
			APP_LOG(TS_OFF, VLEVEL_L, "Vaga ocupada\r\n");
		}

	}else {
		currentVagaStatus = VAGA_VAZIA;
		debounce = 0;
		//APP_LOG(TS_OFF, VLEVEL_L, "Vaga vazia\r\n");
	}

	if(currentVagaStatus != vagaStatus)
	{
		vagaStatus = currentVagaStatus;

		App_Send(SEND_MODE_MANUAL);
		HAL_Delay(100);

	}
}

/*
 * Ocorre em cada transmissao e repete conforme o padrao de piscada do led tx
 */
static void OnTxTimerLedEvent(void *context){
	static uint8_t blinkCntTX = (LED_TX_BLINK_TIMES*2);
	static uint8_t blinkCntLoRa = (LED_CRC_ON_BLINK_TIMES*2);
	static uint8_t blinkCntP2P = (LED_CRC_OFF_BLINK_TIMES*2);
	static uint8_t blinkCntRst = (LED_RESET_CNT_TIMES*2);
	switch(app.stt.led_mode){
	case LED_MODE_TX:
		if(blinkCntTX & 0x01){
			LED_TX(true);
		}else{
			LED_TX(false);
		}
		if(blinkCntTX){
			blinkCntTX--;
			if(app.cfg.cfg.tx_mode == P2P_MODE){
				UTIL_TIMER_StartWithPeriod(&LedTimer, LED_TX_P2P_BLINK_TIME);
			}else{
				UTIL_TIMER_StartWithPeriod(&LedTimer, LED_TX_LORAWAN_BLINK_TIME);
			}
		}else{
			blinkCntTX = (LED_TX_BLINK_TIMES*2);
			app.stt.led_mode = LED_MODE_OFF;
		}
		break;
	case LED_MODE_LORAWAN:
		if(blinkCntLoRa & 0x01){
			LED_TX(true);
		}else{
			LED_TX(false);
		}
		if(blinkCntLoRa){
			blinkCntLoRa--;
			UTIL_TIMER_StartWithPeriod(&LedTimer, LED_CRC_ON_BLINK_TIME);
		}else{
			blinkCntLoRa = (LED_CRC_ON_BLINK_TIMES*2);
			app.stt.led_mode = LED_MODE_OFF;
		}
		break;
	case LED_MODE_P2P:
		if(blinkCntP2P & 0x01){
			LED_TX(true);
		}else{
			LED_TX(false);
		}
		if(blinkCntP2P){
			blinkCntP2P--;
			UTIL_TIMER_StartWithPeriod(&LedTimer, LED_CRC_OFF_BLINK_TIME);
		}else{
			blinkCntP2P = (LED_CRC_OFF_BLINK_TIMES*2);
			app.stt.led_mode = LED_MODE_OFF;
		}
		break;
	case LED_MODE_RESET_CNT:
		if(blinkCntRst & 0x01){
			LED_TX(true);
		}else{
			LED_TX(false);
		}
		if(blinkCntRst){
			/*padrao de piscadas com intervalos crescentes*/
			uint32_t t = LED_RESET_CNT_TIME * (1 << (((LED_RESET_CNT_TIMES*2) - blinkCntRst) >> 1));
			blinkCntRst--;
			UTIL_TIMER_StartWithPeriod(&LedTimer, t);
		}else{
			blinkCntRst = (LED_RESET_CNT_TIMES*2);
			app.stt.led_mode = LED_MODE_OFF;
		}
		break;
	default:
		blinkCntTX = (LED_TX_BLINK_TIMES*2);
		blinkCntLoRa = (LED_CRC_ON_BLINK_TIMES*2);
		blinkCntP2P = (LED_CRC_OFF_BLINK_TIMES*2);
		blinkCntRst = (LED_RESET_CNT_TIMES*2);
		LED_TX(false);
		app.stt.led_mode = LED_MODE_OFF;
		break;
	}
}

/*
 * Ocorre ao finalizar o tempo de acendimento do led para indicacao de recebimento de pulso
 */
#if (BLINK_PULSE_LED_ENABLE == 1)
static void OnLedPulseTimer(void *contextid){
	LED_TX(false);
}
#endif

/*
 * Ocorre apos temporizacao da ultima subida da entrada do botao
 */
static void OnBtnTimeoutTimer(void *contextid){
	if(app.stt.btnCnt >= CLICKS_TO_TX && app.stt.btnCnt < (CLICKS_TO_CHANGE_MODE/2)){
		vagaStatus = VAGA_VAZIA;
		savedValue = flux + 100;
		//APP_LOG(TS_OFF, VLEVEL_L, "flux calibrado: %d\r\n", savedValue);
		App_Send(SEND_MODE_MANUAL);
	}else if(app.stt.btnCnt > (CLICKS_TO_CHANGE_MODE/2)){
		if(app.cfg.cfg.tx_mode != P2P_MODE){
			app.cfg.cfg.tx_mode = P2P_MODE;
			LED_Blink(LED_MODE_P2P);
			/*Salva o novo modo na eeprom independentemente da posicao que estiver na estrutura*/
			//EEPROM_Write(((uint32_t *)(&app.cfg))[offsetof(cfg_t, cfg.tx_mode)/sizeof(uint32_t)], &eeprom_cfg.word[offsetof(cfg_t, cfg.tx_mode)/sizeof(uint32_t)]);
			P2P_Radio_Init();
			Radio.Sleep();
		}else if(app.cfg.cfg.tx_mode != LORAWAN_MODE){
			app.cfg.cfg.tx_mode = LORAWAN_MODE;
			LED_Blink(LED_MODE_LORAWAN);
			/*Salva o novo modo na eeprom independentemente da posicao que estiver na estrutura*/
			//EEPROM_Write(((uint32_t *)(&app.cfg))[offsetof(cfg_t, cfg.tx_mode)/sizeof(uint32_t)], &eeprom_cfg.word[offsetof(cfg_t, cfg.tx_mode)/sizeof(uint32_t)]);
			LoRaWAN_Mode_Init();
		}
	}
	app.stt.btnCnt = 0;
}

/*
 * Inicializacao das configuracoes do modo LoRa P2P
 */
void P2P_Radio_Init(void){
	if(app.stt.lora_state == LORA_IDLE){
		app.stt.lora_state = LORA_BUSY;
		/* Radio initialization */
		RadioEvents.TxDone = OnP2PTxDone;
		RadioEvents.RxDone = OnP2PRxDone;
		RadioEvents.TxTimeout = OnP2PTxTimeout;
		RadioEvents.RxTimeout = OnP2PRxTimeout;
		RadioEvents.RxError = OnP2PRxError;

		Radio.Init(&RadioEvents);

		Radio.SetTxConfig(MODEM_LORA, P2P_TX_OUTPUT_POWER, 0, P2P_BANDWIDTH,
						  P2P_SPREADING_FACTOR, P2P_CODINGRATE,
						  P2P_PREAMBLE_LENGTH, P2P_FIX_LENGTH_PAYLOAD_ON,
						1, 0, 0, P2P_IQ_INVERSION_ON, P2P_TX_TIMEOUT_VALUE);

		Radio.SetMaxPayloadLength(MODEM_LORA, LORA_BUFFER_SIZE);
		APP_LOG(TS_ON, VLEVEL_L,  "Modo P2P\n\r");
		app.stt.lora_state = LORA_IDLE;
	}
}

/*
 * Inicializacao das configuracoes do modo LoRaWAN
 */
void LoRaWAN_Mode_Init(void){
	if(app.stt.lora_state == LORA_IDLE){
		app.stt.lora_state = LORA_BUSY;
		/* Init Info table used by LmHandler*/
		LoraInfo_Init();
		/* Init the Lora Stack*/
		LmHandlerInit(&LmHandlerCallbacks);
		LmHandlerConfigure(&LmHandlerParams);

		LmHandlerJoin(LORAWAN_DEFAULT_ACTIVATION_TYPE);
		APP_LOG(TS_ON, VLEVEL_L,  "Modo LORAWAN\n\r");
		app.stt.lora_state = LORA_IDLE;
	}
}

/*
 * Preenche dados para envio Automatico (periodico)
 */
uint16_t fillBufferAutoTx(uint8_t *buffer){
	uint16_t i = 0;
	//uint32_t tmp;
	buffer[i++] = 5;
	/*leitura da tensao de bateria com arredondamento*/
	buffer[i++] = (uint8_t)((SYS_GetBatteryLevel() + 50)/100);

	buffer[i++] = (uint8_t)((flux >> 8) & 0xff);
	buffer[i++] = (uint8_t)(flux & 0xff);
	/*int16_t tmp = getData(GET_MAG_X);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = getData(GET_MAG_Y);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = getData(GET_MAG_Z);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);*/

	//int tmp = 255;
	buffer[i++] = vagaStatus;
	return i;
}

/*
 * Preenche dados para envio Manual (entrada digital)
 */
uint16_t fillBufferManualTx(uint8_t *buffer){
	uint16_t i = 0;
	//uint32_t tmp;
	buffer[i++] = 0;
	/*leitura da tensao de bateria com arredondamento*/
	buffer[i++] = (uint8_t)((SYS_GetBatteryLevel() + 50)/100);

	buffer[i++] = (uint8_t)((flux >> 8) & 0xff);
	buffer[i++] = (uint8_t)(flux & 0xff);

	/*int16_t tmp = getData(GET_MAG_X);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = getData(GET_MAG_Y);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = getData(GET_MAG_Z);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);*/

	//int tmp = 255;
	buffer[i++] = vagaStatus;
	return i;
}

/*
 * Controle do Led indicador de transmissao
 */
void LED_TX(GPIO_PinState state){
	GPIO_InitTypeDef gpio_cfg;
	gpio_cfg.Pin = LED_TX_PIN;
	gpio_cfg.Speed = GPIO_SPEED_LOW;
	if(state == false){
		HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, state);
		gpio_cfg.Mode = GPIO_MODE_INPUT;
		gpio_cfg.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(LED_TX_PORT, &gpio_cfg);
	}else{
		gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_cfg.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LED_TX_PORT, &gpio_cfg);
		HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, state);
	}
}

/*
 * Interrupcao externa para a entrada de pulso 1
 */
void ReadInput0(void){
	/*static uint32_t last_tick = 0;
	uint32_t tick;
#if (BLINK_PULSE_LED_ENABLE == 1)
	LED_TX(true);
	UTIL_TIMER_Start(&LedPulseTimer);
#endif
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		//app.cfg.cfg.pulse_count[0]++;
		last_tick = tick;
	}*/
	//APP_LOG(TS_OFF, VLEVEL_M, "PL01 = %d\r\n", app.cfg.cfg.pulse_count[0]);
/*#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	EEPROM_Write(app.cfg.pulse_count[0], EEPROM_NextAddr(0));
#endif*/
	HAL_EXTI_ClearPending(&hApp_DIO_exti[0], EXTI_TRIGGER_RISING_FALLING);
}

/*
 * Interrupcao externa para a entrada de pulso 2
 */
void ReadInput1(void){
	/*static uint32_t last_tick = 0;
	uint32_t tick;
#if (BLINK_PULSE_LED_ENABLE == 1)
	LED_TX(true);
	UTIL_TIMER_Start(&LedPulseTimer);
#endif
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		//app.cfg.cfg.pulse_count[1]++;
		last_tick = tick;
	}*/
	//APP_LOG(TS_OFF, VLEVEL_M, "PL02 = %d\r\n", app.cfg.cfg.pulse_count[1]);
/*#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	EEPROM_Write(app.cfg.pulse_count[1], EEPROM_NextAddr(1));
#endif*/
	HAL_EXTI_ClearPending(&hApp_DIO_exti[1], EXTI_TRIGGER_RISING_FALLING);
}

/*
 * Interrupcao externa para a entrada de pulso 3
 */
void ReadInput2(void){
	/*static uint32_t last_tick = 0;
	uint32_t tick;
#if (BLINK_PULSE_LED_ENABLE == 1)
	LED_TX(true);
	UTIL_TIMER_Start(&LedPulseTimer);
#endif
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		//app.cfg.cfg.pulse_count[2]++;
		last_tick = tick;
	}*/
	//APP_LOG(TS_OFF, VLEVEL_M, "PL03 = %d\r\n", app.cfg.cfg.pulse_count[2]);
/*#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	EEPROM_Write(app.cfg.pulse_count[2], EEPROM_NextAddr(2));
#endif*/
	HAL_EXTI_ClearPending(&hApp_DIO_exti[2], EXTI_TRIGGER_RISING_FALLING);
}

/*
 * Interrupcao externa para a entrada de pulso 4
 */
void ReadInput3(void){
	/*static uint32_t last_tick = 0;
	uint32_t tick;
#if (BLINK_PULSE_LED_ENABLE == 1)
	LED_TX(true);
	UTIL_TIMER_Start(&LedPulseTimer);
#endif
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		//app.cfg.cfg.pulse_count[3]++;
		last_tick = tick;
	}*/
	//APP_LOG(TS_OFF, VLEVEL_M, "PL04 = %d\r\n", app.cfg.cfg.pulse_count[3]);
/*#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	EEPROM_Write(app.cfg.pulse_count[3], EEPROM_NextAddr(3));
#endif*/
	HAL_EXTI_ClearPending(&hApp_DIO_exti[3], EXTI_TRIGGER_RISING_FALLING);
}

/*
 * Interrupcao externa para a entrada digital do usuario
 */
void ReadInput4(void){
	static uint32_t last_tick = 0;
	uint32_t tick;
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		last_tick = tick;
		app.stt.btnCnt++;
		UTIL_TIMER_Stop(&btnTimeOutTimer);
		UTIL_TIMER_Start(&btnTimeOutTimer);
	}
	HAL_EXTI_ClearPending(&hApp_DIO_exti[4], EXTI_TRIGGER_RISING_FALLING);
}

/*
 * Inicializa as entradas de interrupcões externas
 */
void EXTI_Init(AppDioIrqHandler **irqHandlers){
	EXTI_ConfigTypeDef pExtiConfig;
	pExtiConfig.Mode = EXTI_MODE_INTERRUPT;
	pExtiConfig.Trigger = EXTI_TRIGGER_FALLING;
	CRITICAL_SECTION_BEGIN();
	for (uint32_t i = 0; i < APP_EXTI_CNT_NUM ; i++){
		pExtiConfig.Line = app_exti_cnt[i].line;
		pExtiConfig.GPIOSel = app_exti_cnt[i].exti_port;
		HAL_EXTI_SetConfigLine(&hApp_DIO_exti[i], &pExtiConfig);
		HAL_EXTI_RegisterCallback(&hApp_DIO_exti[i], HAL_EXTI_COMMON_CB_ID, irqHandlers[i]);
		HAL_NVIC_SetPriority(app_exti_cnt[i].irqn, app_exti_cnt[i].prio, 0x00);
		HAL_NVIC_EnableIRQ(app_exti_cnt[i].irqn);
	}
	for (uint32_t i = 0; i < APP_EXTI_BTN_NUM ; i++){
		pExtiConfig.Line = app_exti_btn[i].line;
		pExtiConfig.GPIOSel = app_exti_btn[i].exti_port;
		HAL_EXTI_SetConfigLine(&hApp_DIO_exti[i+APP_EXTI_CNT_NUM], &pExtiConfig);
		HAL_EXTI_RegisterCallback(&hApp_DIO_exti[i+APP_EXTI_CNT_NUM], HAL_EXTI_COMMON_CB_ID, irqHandlers[i+APP_EXTI_CNT_NUM]);
		HAL_NVIC_SetPriority(app_exti_btn[i].irqn, app_exti_btn[i].prio, 0x00);
		HAL_NVIC_EnableIRQ(app_exti_btn[i].irqn);
	}
	CRITICAL_SECTION_END();
}

/*
 * Callback para transmissao P2P finalizada
 */
static void OnP2PTxDone(void){
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
  app.stt.tx_cnt++;
  if(app.stt.tx_cnt >= SEND_TIMES){
	Radio.Sleep();
	app.stt.lora_state = LORA_IDLE;
  }else{
	Radio.SetChannel(P2P_RF_FREQUENCY);
	Radio.Send(&LoRaBuffer[2], LoRaBufferSize-2);
  }
}

/*
 * Callback de Recepcao P2P *nao usado
 */
static void OnP2PRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
  //APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
  //APP_LOG(TS_ON, VLEVEL_L,  "RssiValue=%d dBm, SnrValue=%d\n\r", rssi, snr);
  Radio.Sleep();
}

/*
 * Callback de timeout de Transmissao
 */
static void OnP2PTxTimeout(void){
	APP_LOG(TS_ON, VLEVEL_L,  "OnTxTimeout\n\r");
	Radio.Sleep();
	app.stt.lora_state = LORA_IDLE;
}

/*
 * Callback para timeout de recepcao P2P *nao usado
 */
static void OnP2PRxTimeout(void){
  //APP_LOG(TS_ON, VLEVEL_L,  "OnRxTimeout\n\r");
  Radio.Sleep();
}

/*
 * Callback de erro na recepcao P2p *nao usado
 */
static void OnP2PRxError(void){
  //APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");
  Radio.Sleep();
}

/*
 * Callback de Recepcao LoRaWAN usado para receber downlink tempo de registro
 */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params){
	APP_LOG(TS_OFF, VLEVEL_L, "Downlink recebido. Rssi: %d\r\n", params->Rssi);
	if ((appData != NULL) && (params != NULL)){
		if(appData->Port == APP_DOWNLINK_PORT){
			if(appData->BufferSize == 4){
				uint16_t tmp = ((uint16_t)appData->Buffer[1] << 8) | ((uint16_t)appData->Buffer[2] & 0x00FF);
				if(tmp > 0){
					app.cfg.cfg.tx_time = tmp;
					UTIL_TIMER_Stop(&inputTimer);
					UTIL_TIMER_SetPeriod(&inputTimer, app.cfg.cfg.tx_time*60000);
					UTIL_TIMER_Start(&inputTimer);
					APP_LOG(TS_OFF, VLEVEL_L, "Tempo de registro: %d\r\n", app.cfg.cfg.tx_time);
				}else{
					APP_LOG(TS_OFF, VLEVEL_M, "Valor de registro não pode ser nulo!\r\n");
				}
				uint8_t clr_cnt = appData->Buffer[3];
				uint8_t blink = 0;
				if((clr_cnt & 0xf0) == 0xa0){
					for(int i = 0; i < APP_EXTI_CNT_NUM && i < 4; i++){
						if(clr_cnt & (0x01 << i)){
							//app.cfg.cfg.pulse_count[i] = 0;
							blink = 1;
						}
					}
				}
				if(blink){
					/*agenda piscada de indicacao de reset de contadores*/
					UTIL_TIMER_Start(&ResetCntDelayTimer);
				}
			}else{
				APP_LOG(TS_OFF, VLEVEL_M, "Tamanho de Downlink invalido: %d\r\n", appData->BufferSize);
			}
		}else{
			APP_LOG(TS_OFF, VLEVEL_M, "Porta de Downlink invalida: %d\r\n", appData->Port);
		}
		return;
	}
	APP_LOG(TS_OFF, VLEVEL_M, "Downlink invalido!\r\n");
}

/*
 * Atraso para afastar a piscada do reset de contadores da piscada de transmissao
 */
void OnDelayResetCnt(void *context){
	LED_Blink(LED_MODE_RESET_CNT);
}

/*
 * Task de trasnsmissao LoRaWAN
 */
static void SendTxData(void){
	UTIL_TIMER_Time_t nextTxIn = 0;
	if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false)){
		APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
	}
}

/*
 * Callback para transmissao LoRaWAN finalizada
 */
static void OnTxData(LmHandlerTxParams_t *params){
	app.stt.lora_state = LORA_IDLE;
}

/*
 * Callback para Join LoRaWAN *nao usado
 */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams){

}

/*
 * Callback para interrupcoes LoRaWAN
 */
static void OnMacProcessNotify(void){
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);
}

/*
 * Callback para recepcao na serial em modo timeout (timeout apos ultimo byte recebido)
 *//*
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		if(huart->ErrorCode & HAL_UART_ERROR_RTO){
			rx_bytes(rxBuffer, huart->RxXferSize - huart->RxXferCount, 0);
		    HAL_UART_Receive_IT(huart, rxBuffer, 16);
		}
	}
}*/

/*
 * Processa comandos recebidos pela serial
 */
void rx_bytes(uint8_t *bytes, uint16_t sz, uint8_t error){
	if(sz >= 4){
		if(bytes[0] == '$' && bytes[1] == 'W' && bytes[3] == '#' && bytes[sz-1]){
			if(bytes[2] == 'A'){
				for(int i = 0; i < 4; i++){
					if(bytes[4+i*2] >= 'a' && bytes[4+i*2] <= 'f'){
						((uint8_t *)&app.cfg.cfg.dev_addr)[3-i] = (bytes[4+i*2] - 'a' + 10) * 16;
					}else if(bytes[4+i*2] >= '0' && bytes[4+i*2] <= '9'){
						((uint8_t *)&app.cfg.cfg.dev_addr)[3-i] = (bytes[4+i*2] - '0') * 16;
					}
					if(bytes[5+i*2] >= 'a' && bytes[5+i*2] <= 'f'){
						((uint8_t *)&app.cfg.cfg.dev_addr)[3-i] += bytes[5+i*2] - 'a' + 10;
					}else if(bytes[5+i*2] >= '0' && bytes[5+i*2] <= '9'){
						((uint8_t *)&app.cfg.cfg.dev_addr)[3-i] += bytes[5+i*2] - '0';
					}
				}
				EEPROM_Write(((uint32_t *)(&app.cfg))[offsetof(cfg_t, cfg.dev_addr)/sizeof(uint32_t)], &eeprom_cfg.word[offsetof(cfg_t, cfg.dev_addr)/sizeof(uint32_t)]);
				SetDevAddr(app.cfg.cfg.dev_addr);
				LoRaWAN_Mode_Init();
				uint8_t eui[8];
				LmHandlerGetDevEUI(&eui[0]);
				APP_LOG(TS_OFF, VLEVEL_L, "DEV_EUI:%02X%02X%02X%02X%02X%02X%02X%02X\r\n", eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7]);
			}else if(bytes[2] == 'R'){
				//SX126xFreqCorrection(app.cfg.cfg.xta_trim);
				SX126xSetStandby( STDBY_XOSC );
				SX126xWriteRegister(REG_XTA_TRIM, app.cfg.cfg.xta_trim);
				SX126xWriteRegister(REG_XTB_TRIM, app.cfg.cfg.xta_trim);
				Radio.SetTxContinuousWave( 915800000, P2P_TX_OUTPUT_POWER, 60 );
				SX126xSetStandby( STDBY_XOSC );
				SX126xWriteRegister(REG_XTA_TRIM, app.cfg.cfg.xta_trim);
				SX126xWriteRegister(REG_XTB_TRIM, app.cfg.cfg.xta_trim);
				Radio.SetTxContinuousWave( 915800000, P2P_TX_OUTPUT_POWER, 60 );
				APP_LOG(TS_OFF, VLEVEL_L, "XTA_TRIM=%d\n\r", app.cfg.cfg.xta_trim);
			}else if(bytes[2] == 'S'){
				//EEPROM_Write(((uint32_t *)(&app.cfg))[offsetof(cfg_t, cfg.xta_trim)/sizeof(uint32_t)], &eeprom_cfg.word[offsetof(cfg_t, cfg.xta_trim)/sizeof(uint32_t)]);
				/*Reinicia para garantir reconfiguracão segura*/
				while(1);
			}else if(bytes[2] == '+'){
				if(app.cfg.cfg.xta_trim < 0x2F){
					app.cfg.cfg.xta_trim++;
					SX126xFreqCorrection(app.cfg.cfg.xta_trim);
					SX126xSetStandby( STDBY_XOSC );
					SX126xWriteRegister(REG_XTA_TRIM, app.cfg.cfg.xta_trim);
					SX126xWriteRegister(REG_XTB_TRIM, app.cfg.cfg.xta_trim);
					Radio.SetTxContinuousWave( 915800000, P2P_TX_OUTPUT_POWER, 60 );
				}
				APP_LOG(TS_OFF, VLEVEL_L, "XTA_TRIM=%02d\r\n", app.cfg.cfg.xta_trim);
			}else if(bytes[2] == '-'){
				if(app.cfg.cfg.xta_trim > 0){
					app.cfg.cfg.xta_trim--;
					SX126xFreqCorrection(app.cfg.cfg.xta_trim);
					SX126xSetStandby( STDBY_XOSC );
					SX126xWriteRegister(REG_XTA_TRIM, app.cfg.cfg.xta_trim);
					SX126xWriteRegister(REG_XTB_TRIM, app.cfg.cfg.xta_trim);
					Radio.SetTxContinuousWave( 915800000, P2P_TX_OUTPUT_POWER, 60 );
				}
				APP_LOG(TS_OFF, VLEVEL_L, "XTA_TRIM=%02d\r\n", app.cfg.cfg.xta_trim);
			}else if(bytes[2] == 'E' ){
				uint8_t val = 0;
				if(bytes[3] >= '0' && bytes[3] <= '9' && bytes[4] >= '0' && bytes[4] <= '9'){
					val = ((bytes[3] - '0') * 10) + (bytes[4] - '0');
					if(val <= 0x2f){
						app.cfg.cfg.xta_trim = val;
					}
					SX126xFreqCorrection(app.cfg.cfg.xta_trim);
					SX126xSetStandby( STDBY_XOSC );
					SX126xWriteRegister(REG_XTA_TRIM, app.cfg.cfg.xta_trim);
					SX126xWriteRegister(REG_XTB_TRIM, app.cfg.cfg.xta_trim);
					Radio.SetTxContinuousWave( 915800000, P2P_TX_OUTPUT_POWER, 60 );
					APP_LOG(TS_OFF, VLEVEL_L, "XTA_TRIM=%d\n\r", app.cfg.cfg.xta_trim);
				}
			}else if(bytes[2] == '>'){
				if(app.cfg.cfg.xta_trim < 47){
					if(app.cfg.cfg.xta_trim <= 42){
						app.cfg.cfg.xta_trim += 5;
					}else{
						app.cfg.cfg.xta_trim = 47;
					}
					SX126xFreqCorrection(app.cfg.cfg.xta_trim);
					SX126xSetStandby( STDBY_XOSC );
					SX126xWriteRegister(REG_XTA_TRIM, app.cfg.cfg.xta_trim);
					SX126xWriteRegister(REG_XTB_TRIM, app.cfg.cfg.xta_trim);
					Radio.SetTxContinuousWave( 915800000, P2P_TX_OUTPUT_POWER, 60 );
				}
				APP_LOG(TS_OFF, VLEVEL_L, "XTA_TRIM=%02d\r\n", app.cfg.cfg.xta_trim);
			}else if(bytes[2] == '<'){
				if(app.cfg.cfg.xta_trim > 0){
					if(app.cfg.cfg.xta_trim >= 5){
						app.cfg.cfg.xta_trim -= 5;
					}else{
						app.cfg.cfg.xta_trim = 0;
					}
					SX126xFreqCorrection(app.cfg.cfg.xta_trim);
					SX126xSetStandby( STDBY_XOSC );
					SX126xWriteRegister(REG_XTA_TRIM, app.cfg.cfg.xta_trim);
					SX126xWriteRegister(REG_XTB_TRIM, app.cfg.cfg.xta_trim);
					Radio.SetTxContinuousWave( 915800000, P2P_TX_OUTPUT_POWER, 60 );
				}
				APP_LOG(TS_OFF, VLEVEL_L, "XTA_TRIM=%02d\r\n", app.cfg.cfg.xta_trim);
			}else if(bytes[2] == 'N'){
				uint8_t eui[8];
				LmHandlerGetDevEUI(&eui[0]);
				APP_LOG(TS_OFF, VLEVEL_L, "%02X%02X%02X%02X%02X%02X%02X%02X\r\n", eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7]);
			}
		}
	}
}
