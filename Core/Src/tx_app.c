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


static  I2C_HandleTypeDef hi2c1;
static int flagInterrupt;

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

LPTIM_HandleTypeDef hlptim;

#if (BLINK_PULSE_LED_ENABLE == 1)
UTIL_TIMER_Object_t LedPulseTimer;
#endif
UTIL_TIMER_Object_t btnTimeOutTimer;
const app_exti_t app_exti_cnt[] = {
		{.line = EXTI_LINE_13, .prio = 0, .exti_port = EXTI_GPIOA, .gpio_port = GPIOA, .pin = GPIO_PIN_13, .irqn = EXTI4_15_IRQn},
		/*{.line = EXTI_LINE_14, .prio = 0, .exti_port = EXTI_GPIOA, .gpio_port = GPIOA, .pin = GPIO_PIN_14, .irqn = EXTI4_15_IRQn},
		{.line = EXTI_LINE_13, .prio = 0, .exti_port = EXTI_GPIOA, .gpio_port = GPIOA, .pin = GPIO_PIN_13, .irqn = EXTI4_15_IRQn},
		{.line = EXTI_LINE_7, .prio = 0, .exti_port = EXTI_GPIOB, .gpio_port = GPIOB, .pin = GPIO_PIN_7, .irqn = EXTI4_15_IRQn},
		{.line = EXTI_LINE_8, .prio = 0, .exti_port = EXTI_GPIOB, .gpio_port = GPIOB, .pin = GPIO_PIN_8, .irqn = EXTI4_15_IRQn},*/
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
uint16_t eeprom_wr_pos[APP_EXTI_CNT_NUM] = {0};//{0,0,0,0};

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
		//uint8_t cnt_per_hour[APP_EXTI_CNT_NUM][5];
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

#if (BLINK_PULSE_LED_ENABLE == 1)
static void OnLedPulseTimer(void *contextid);
#endif
static void OnBtnTimeoutTimer(void *contextid);
//static void ReadInput0(void);
static void ReadInput1(void);
/*static void ReadInput2(void);
static void ReadInput3(void);*/
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


static int estacionamento_interrupt_tongle = 0; //Controla a chegada e saida de carros na vaga
static int sendEstacionamentoTongle = 0; //Controla a chegada e saida de carros na vaga

static void MX_I2C1_Init(void);


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
	  hi2c1.Instance = I2C1;
	  hi2c1.Init.Timing = 0x10909CEC;//0x00303D5B; // Timing configurado para operação a 400 kHz em Fast Mode
	  hi2c1.Init.OwnAddress1 = 0;
	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.OwnAddress2 = 0;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
	    Error_Handler(); // Tratamento de erro
	  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

static SRAWDATA MAG;
//static SRAWDATA ACCEL;


// Sets the FXOS8700CQ to standby mode.
// It must be in standby to change most register settings
void FXOS8700CQStandby(I2C_HandleTypeDef *I2Cx)
{
	uint8_t CTRL_REG1_Data;
	HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &CTRL_REG1_Data, 1, i2c_timeout);
	CTRL_REG1_Data = CTRL_REG1_Data & ~(0x01); // Limpar o bit Active 0x01
	HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &CTRL_REG1_Data, 1, i2c_timeout);
}

// Sets the FXOS8700CQ to active mode.
// Needs to be in this mode to output data
void FXOS8700CQActive(I2C_HandleTypeDef *I2Cx)
{

  uint8_t CTRL_REG1_Data;
  HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &CTRL_REG1_Data, 1, i2c_timeout);
  CTRL_REG1_Data = CTRL_REG1_Data | 0x01; // Limpar o bit Active
  HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &CTRL_REG1_Data, 1, i2c_timeout);

}

/*
 * Função para resetar os registradores do FXOS8700CQ
 * */
void FXOS8700CQReset(I2C_HandleTypeDef *I2Cx)
{
	uint8_t value = 0x40;
	HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG2, 1, &value, 1, i2c_timeout);
	HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_CTRL_REG2, 1, &value, 1, i2c_timeout);
}


/*
 * Verifica status de funcionamento do barramento I2C
 * */
static uint8_t FXOS8700CQGetWhoAmI(I2C_HandleTypeDef *I2Cx)
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

uint8_t FXOS8700CQCheckCommunication(void)
{

	uint8_t check; //Check if device is ok
	uint8_t Data; //Receive data
	uint8_t databyte; //Write data

	// read and check the FXOS8700CQ WHOAMI register
	HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_WHOAMI, 1, &check, 1, i2c_timeout);
	if(check == FXOS8700CQ_WHOAMI_VAL)
	{
		return HAL_OK;
	}
	return 1;
}

/*
 * Inicia o FXOS8700CQ sem o modo sleep. Habilita o Magnetômetro e o Acelerômetro
 * */
static uint8_t FXOS8700CQInit(I2C_HandleTypeDef *I2Cx)
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

//Versão de configuração que habilita o Mag em modo sleep acionando INT1 quando valor de
//X sobe para mais de 1000 unidades de medida.
//Não usa o modo sleep do FXOS8700CQ
//type (INTERRUPT_MAIOR_QUE INTERRUPT_MENOR_QUE)
static uint8_t FXOS8700CQInitSleep(I2C_HandleTypeDef *I2Cx, int threshold, uint8_t type)
{
	uint8_t ret = 1;
	uint8_t check; //Check if device is ok
	uint8_t databyte; //Write data
	uint8_t HighByteThreshold = (uint8_t)((threshold >> 8) & 0xFF);
	uint8_t LowByteThreshold = (uint8_t)((threshold >> 8) & 0xFF);

	//Reset FXOS8700CQ before configure
	FXOS8700CQReset(I2Cx);

	// read and check the FXOS8700CQ WHOAMI register
	HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_WHOAMI, 1, &check, 1, i2c_timeout);
	if(check == FXOS8700CQ_WHOAMI_VAL)
	{
		// Put it in standby mode every time before reconfigure anything
		/*uint8_t CTRL_REG1_Data;
		HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &CTRL_REG1_Data, 1, i2c_timeout);
		CTRL_REG1_Data &= 0xFE; // Limpar o bit Active
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &CTRL_REG1_Data, 1, i2c_timeout);*/

		FXOS8700CQStandby(I2Cx);
		HAL_Delay(1);

		//=========================INICIA CONFIGURAÇÃO LOW POWER ABAIXO!===================================================

		//Configurando o valor 1000 para a interrupção em X
		databyte = HighByteThreshold; //MSB de 1000
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_INTERRUPT_Z_MSB_REG, 1, &databyte, 1, i2c_timeout);
		databyte = LowByteThreshold; //LSB de 1000
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_INTERRUPT_Z_LSB_REG, 1, &databyte, 1, i2c_timeout);

		// Event flag latch enabled, logic OR of enabled axes, X and Y-axis enabled, wake on magnetic threshold event, threshold interrupt enabled and routed to INT1
		if(type == INTERRUPT_MAIOR_QUE)
			databyte = 0xE7;//0xCF; //Valor maior que -> 0xCF. Valor menor que 0x8F
		else if(type == INTERRUPT_MENOR_QUE)
			databyte = 0xA7;//0x8F; //Valor maior que -> 0xCF. Valor menor que 0x8F
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_THRESHOLD_CFG_REG, 1, &databyte, 1, i2c_timeout);

		// Esse registrador é um multiplicador do tempo presente na tabela 196 do datasheet.
		// Como estamos trabalhando a 12,5Hz; o multiplicador é de 80 em 80 ms.
		databyte = 0x01;//Original 0x01; //0x0D (1040ms)//0x01; 80ms//Original 0x0A (800 ms)
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_THRESHOLD_DBC_COUNTER_REG, 1, &databyte, 1, i2c_timeout);

		// Max OSR, only magnetometer is active
		databyte = 0x01;//Original -> 0x1D
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_CTRL_REG1, 1, &databyte, 1, i2c_timeout);

		// Push-pull, active low interrupt
		databyte = 0x00;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG3, 1, &databyte, 1, i2c_timeout);

		// Auto-sleep mode enabled
		databyte = 0x04;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG2, 1, &databyte, 1, i2c_timeout);

		// Auto-sleep counter set to 1.92s (6 x 0.32s = 1.92s)
		databyte = 0x03;//0x03; (1s)//0x5E; (20s)//Original 0x06; ->Faz dormir depois de y ssegundos calculados conforme: x = y * 0.32s
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_ASLP_COUNT_REG, 1, &databyte, 1, i2c_timeout);

		// Auto-sleep/wake interrupt enabled
		databyte = 0x80;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1,FXOS8700CQ_M_CTRL_REG4, 1, &databyte, 1, i2c_timeout);

		// Route auto-sleep/wake interrupt to INT1
		databyte = 0x80;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_CTRL_REG5, 1, &databyte, 1, i2c_timeout);

		// Finally activate device with ASLP ODR = 6.25Hz, Normal ODR = 100Hz
		databyte = 0xA9;//0xF9 (1,5Hz normal e 1,5Hz sleep)//0xA9; (12,5Hz normal e 6,25 Hz sleep)//Original 0x99; (6,25Hz) //Original 0x19; (50Hz)
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_CTRL_REG1, 1, &databyte, 1, i2c_timeout);

		FXOS8700CQActive(I2Cx);

		ret = 0;
	}
	return ret;
}

/*
//Versão OK
//Versão de configuração que habilita o Mag em modo sleep acionando INT1 quando valor de
//X sobe para mais de 1000 unidades de medida.
//Não usa o modo sleep do FXOS8700CQ
static uint8_t magnetometter_init(I2C_HandleTypeDef *I2Cx, int threshold)
{
	uint8_t ret = 1;
	uint8_t check; //Check if device is ok
	uint8_t Data; //Receive data
	uint8_t databyte; //Write data
	uint8_t HighByteThreshold = (uint8_t)((threshold >> 8) & 0xFF);
	uint8_t LowByteThreshold = (uint8_t)((threshold >> 8) & 0xFF);

	// read and check the FXOS8700CQ WHOAMI register
	HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_WHOAMI, 1, &check, 1, i2c_timeout);
	if(check == FXOS8700CQ_WHOAMI_VAL)
	{
		// write 0100 0000 = 0x40 to accelerometer control register 2 to place FXOS8700CQ into
		// POR
		// [7-1] = 0000 000
		// [0]: active=0
		//databyte = 0x40;
		//HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x2B, 1, &databyte, 1, i2c_timeout);
		//HAL_Delay(1); // Gere um atraso de 1 ms

		//Configurando o valor 1000 para a interrupção em X
		databyte = 0x05; //MSB de 1000
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x54, 1, &databyte, 1, i2c_timeout);
		databyte = 0xDC; //LSB de 1000
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x55, 1, &databyte, 1, i2c_timeout);

		// Event flag latch enabled, logic OR of enabled axes, X and Y-axis enabled, wake on magnetic threshold event, threshold interrupt enabled and routed to INT1
		databyte = 0xCF;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x52, 1, &databyte, 1, i2c_timeout);

		// Debounce timer period is 320ms (sleep mode) / 20ms (normal mode)
		databyte = 0x0A;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x5A, 1, &databyte, 1, i2c_timeout);

		// Max OSR, only magnetometer is active
		databyte = 0x1D;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x5B, 1, &databyte, 1, i2c_timeout);

		// Push-pull, active low interrupt
		databyte = 0x00;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x2C, 1, &databyte, 1, i2c_timeout);

		// Auto-sleep mode enabled
		databyte = 0x04;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x2B, 1, &databyte, 1, i2c_timeout);

		// Auto-sleep counter set to 1.92s (6 x 0.32s = 1.92s)
		databyte = 0x06;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x29, 1, &databyte, 1, i2c_timeout);

		// Auto-sleep/wake interrupt enabled
		databyte = 0x80;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x2D, 1, &databyte, 1, i2c_timeout);

		// Route auto-sleep/wake interrupt to INT1
		databyte = 0x80;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x2E, 1, &databyte, 1, i2c_timeout);

		// Finally activate device with ASLP ODR = 6.25Hz, Normal ODR = 100Hz
		databyte = 0x19;
		HAL_I2C_Mem_Write(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, 0x2A, 1, &databyte, 1, i2c_timeout);
		ret = 0;
	}
	return ret;
}*/

// read status and the three channels of accelerometer and magnetometer data from
// FXOS8700CQ (13 bytes)
//void ReadAccelMagnData(SRAWDATA *pAccelData, SRAWDATA *pMagnData, I2C_HandleTypeDef *I2Cx)
static void FXOS8700CQReadAccelMagnData(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Buffer[FXOS8700CQ_READ_LEN]; // read buffer

	// read FXOS8700CQ_READ_LEN=13 bytes (status byte and the six channels of data)
	HAL_I2C_Mem_Read(I2Cx, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_STATUS, 1, &Buffer, FXOS8700CQ_READ_LEN, i2c_timeout);

	// copy the 14 bit accelerometer byte data into 16 bit words
	/*ACCEL.x = (int16_t)(((Buffer[1] << 8) | Buffer[2]))>> 2;
	ACCEL.y = (int16_t)(((Buffer[3] << 8) | Buffer[4]))>> 2;
	ACCEL.z = (int16_t)(((Buffer[5] << 8) | Buffer[6]))>> 2;*/

	// copy the magnetometer byte data into 16 bit words
	MAG.x = (Buffer[7] << 8) | Buffer[8];
	MAG.y = (Buffer[9] << 8) | Buffer[10];
	MAG.z = (Buffer[11] << 8) | Buffer[12];

}

/**
 * Get data from struct
 */
static int16_t FXOS8700CQGetData(uint8_t dado_a_receber)
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
	/*case GET_ACCEL_X:
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
	break;*/

	}
}


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

AppDioIrqHandler *AppDioIrq[APP_EXTI_CNT_NUM+APP_EXTI_BTN_NUM] = {ReadInput1, ReadInput4};//{ReadInput0, ReadInput1, ReadInput2, ReadInput3, ReadInput4};

/* Private user code ---------------------------------------------------------*/

/*int FXOS8700CQCalibrate(uint8_t type)
{
	FXOS8700CQReset(&hi2c1);
	FXOS8700CQStandby(&hi2c1);
	if(FXOS8700CQInit(&hi2c1) != HAL_OK){
		FXOS8700CQReset(&hi2c1);
		FXOS8700CQStandby(&hi2c1);
	}
	FXOS8700CQActive(&hi2c1);

	int calibrate=0;

	for(int i =0; i < 10; i++)
	{
		HAL_Delay(1);
		FXOS8700CQReadAccelMagnData(&hi2c1);
		if((int)(FXOS8700CQGetData(GET_MAG_Z)) > calibrate)
		{
			int auxiliar = (int)(FXOS8700CQGetData(GET_MAG_Z));
			if(auxiliar == 0)
			{
				i--;
				if(i < 0)
				{
					i = 0;
				}
			}else {
				calibrate += auxiliar;
			}
		}

	}
	calibrate = (int)(FXOS8700CQGetData(GET_MAG_Z));//calibrate/10;
	//APP_LOG(TS_OFF, VLEVEL_L, "Calibrate antes da redução =  %d\r\n", calibrate);
	if(type == INTERRUPT_MAIOR_QUE)
		calibrate += 100;//500;//1000;
	else
		calibrate -= 100;//500;//1000;
	APP_LOG(TS_OFF, VLEVEL_L, "Calibrate =  %d| %d\r\n", calibrate, type);

	return (int)calibrate;
}*/

int FXOS8700CQCalibrate(uint8_t type)
{
	FXOS8700CQReset(&hi2c1);
	FXOS8700CQStandby(&hi2c1);
	if(FXOS8700CQInit(&hi2c1) != HAL_OK){
		FXOS8700CQReset(&hi2c1);
		FXOS8700CQStandby(&hi2c1);
	}
	FXOS8700CQActive(&hi2c1);

	int calibrate=0;
	int valores[10] = {0};

	for(int i =0; i < 10; i++)
	{
		HAL_Delay(1);
		FXOS8700CQReadAccelMagnData(&hi2c1);
		valores[i] = (int)(FXOS8700CQGetData(GET_MAG_Z));
		/*if((int)(FXOS8700CQGetData(GET_MAG_Z)) > calibrate)
		{
			int auxiliar = (int)(FXOS8700CQGetData(GET_MAG_Z));
			if(auxiliar == 0)
			{
				i--;
				if(i < 0)
				{
					i = 0;
				}
			}else {
				calibrate += auxiliar;
			}
		}*/

	}
	int maior = 0;
	for(int i =0; i < 10; i++)
	{
		if(valores[i] > maior)
		{
			maior = valores[i];
		}
	}
	calibrate = maior;//calibrate/10;
	//APP_LOG(TS_OFF, VLEVEL_L, "Calibrate antes da redução =  %d\r\n", calibrate);
	if(type == INTERRUPT_MAIOR_QUE)
		calibrate += 200;//500;//1000;
	else
		calibrate -= 200;//500;//1000;
	APP_LOG(TS_OFF, VLEVEL_L, "Calibrate =  %d| %d\r\n", calibrate, type);

	return (int)calibrate;
}

void TxAppInit(void){

	/*MX_I2C1_Init();
	uint8_t wiam = magnetometter_init(&hi2c1);
	ReadAccelMagnData(&hi2c1);*/
	flagInterrupt = 0;

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
	UTIL_TIMER_SetPeriod(&inputTimer, app.cfg.cfg.tx_time*1000);//60000
	UTIL_TIMER_Start(&inputTimer);
#if (BLINK_PULSE_LED_ENABLE == 1)
	UTIL_TIMER_Create(&LedPulseTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnLedPulseTimer, NULL);
	UTIL_TIMER_SetPeriod(&LedPulseTimer, 20);
#endif
	UTIL_TIMER_Create(&btnTimeOutTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnBtnTimeoutTimer, NULL);
	UTIL_TIMER_SetPeriod(&btnTimeOutTimer, BTN_TIMEOUT);
	UTIL_TIMER_Create(&LedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
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



	uint8_t wiam = FXOS8700CQInitSleep(&hi2c1, FXOS8700CQCalibrate(INTERRUPT_MAIOR_QUE), INTERRUPT_MAIOR_QUE);//magnetometter_init(&hi2c1);//magnetometter_init_sleep(&hi2c1, 65097);
	APP_LOG(TS_OFF, VLEVEL_L, "WHO I AM %02X\r\n", wiam);
	if(wiam != 0)
	{
		//Reseta se wiam for diferente de 0
		HAL_Delay(10000);
	}

	/*FXOS8700CQStandby(&hi2c1);
	FXOS8700CQActive(&hi2c1);*/

	/*ReadAccelMagnData(&hi2c1);
	uint16_t magX = (uint16_t)(getData(GET_MAG_X));
	uint16_t magY = (uint16_t)(getData(GET_MAG_Y));
	uint16_t magZ = (uint16_t)(getData(GET_MAG_Z));
	APP_LOG(TS_OFF, VLEVEL_L, "nMG - X: %d Y: %d Z: %d\r\n",  magX, magY, magZ);*/
	//App_Send(SEND_MODE_MANUAL);

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
	if(mode > LED_MODE_P2P || app.stt.led_mode != LED_MODE_OFF){
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
		UTIL_TIMER_StartWithPeriod(&LedTimer, LED_LORAWAN_ON_TIME);
		break;
	case LED_MODE_P2P:
		UTIL_TIMER_StartWithPeriod(&LedTimer, LED_P2P_ON_TIME);
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
	//APP_LOG(TS_OFF, VLEVEL_L, "OnTimer\r\n");
	static uint8_t ind = 5; /*marca indice de horas passadas*/
	if(ind == 0){
		ind = 5;

		//App_Send(SEND_MODE_AUTO);
	}else{
		ind--;
		/*for(uint8_t cnt = 0; cnt < APP_EXTI_CNT_NUM; cnt++){
			app.stt.cnt_per_hour[cnt][ind] = app.cfg.cfg.pulse_count[cnt] - app.stt.pulse_count_tmp[cnt];
			app.stt.pulse_count_tmp[cnt] = app.cfg.cfg.pulse_count[cnt];
		}*/
		/*ReadAccelMagnData(&hi2c1);
		int16_t magX = getData(GET_MAG_X);
		int16_t magY = getData(GET_MAG_Y);
		int16_t magZ = getData(GET_MAG_Z);
		APP_LOG(TS_OFF, VLEVEL_L, "nTX_Mg - X: %d Y: %d Z: %d| %d\r\n",  magX, magY, magZ, ind);*/
		FXOS8700CQForceSleep();
	}
}

/*
 * Ocorre em cada transmissao e repete conforme o padrao de piscada do led tx
 */
static void OnTxTimerLedEvent(void *context){
	static uint8_t blinkCntTX = (LED_TX_BLINK_TIMES*2);
	static uint8_t blinkCntLoRa = (LED_LORAWAN_BLINK_TIMES*2);
	static uint8_t blinkCntP2P = (LED_P2P_BLINK_TIMES*2);
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
			UTIL_TIMER_StartWithPeriod(&LedTimer, LED_LORAWAN_BLINK_TIME);
		}else{
			blinkCntLoRa = (LED_LORAWAN_BLINK_TIMES*2);
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
			UTIL_TIMER_StartWithPeriod(&LedTimer, LED_P2P_BLINK_TIME);
		}else{
			blinkCntP2P = (LED_P2P_BLINK_TIMES*2);
			app.stt.led_mode = LED_MODE_OFF;
		}
		break;
	default:
		blinkCntTX = (LED_TX_BLINK_TIMES*2);
		blinkCntLoRa = (LED_LORAWAN_BLINK_TIMES*2);
		blinkCntP2P = (LED_P2P_BLINK_TIMES*2);
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
	if(app.stt.btnCnt >= CLICKS_TO_TX && app.stt.btnCnt < (CLICKS_TO_CHANGE_MODE)){
		if(sendEstacionamentoTongle == 1)
		{
			sendEstacionamentoTongle = 0;

		}else {

			sendEstacionamentoTongle = 1;

		}
		App_Send(SEND_MODE_MANUAL);
	}else if(app.stt.btnCnt >= (CLICKS_TO_CHANGE_MODE)){
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

	/*int16_t tmp = getData(GET_MAG_X);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = getData(GET_MAG_Y);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = getData(GET_MAG_Z);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);*/


	int tmp = 255;
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = FXOS8700CQGetData(GET_MAG_Y);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = FXOS8700CQGetData(GET_MAG_Z);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);

	buffer[i++] = sendEstacionamentoTongle;//estacionamento_interrupt_tongle; //Controla a chegada e saida de carros na vaga


	/*for(uint8_t cnt = 0; cnt < APP_EXTI_CNT_NUM;cnt+=2){
		//mover a contagem inteira de uma só vez evita erros em caso de interrupcao no meio da tranferencia de bytes
		tmp = app.cfg.cfg.pulse_count[cnt];
		//salva exatamente o valor que foi enviado evitando erros em caso de interrupcao entre duas leituras da mesma variavel
		app.stt.pulse_count_tmp[cnt] = tmp;
		buffer[i++] = (tmp >> 24) & 0xff;
		buffer[i++] = (tmp >> 16) & 0xff;
		buffer[i++] = (tmp >> 8) & 0xff;
		buffer[i++] = tmp & 0xff;
		tmp = app.cfg.cfg.pulse_count[cnt+1];
		app.stt.pulse_count_tmp[cnt+1] = tmp;
		buffer[i++] = (tmp >> 24) & 0xff;
		buffer[i++] = (tmp >> 16) & 0xff;
		buffer[i++] = (tmp >> 8) & 0xff;
		buffer[i++] = tmp & 0xff;
		for(uint8_t j = 0; j < 5; j++){
			buffer[i++] = app.stt.cnt_per_hour[cnt][j];
			buffer[i++] = app.stt.cnt_per_hour[cnt+1][j];
		}
	}*/
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

	/*int16_t tmp = getData(GET_MAG_X);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = getData(GET_MAG_Y);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = getData(GET_MAG_Z);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);*/

	int tmp = 255;
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = FXOS8700CQGetData(GET_MAG_Y);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	tmp = FXOS8700CQGetData(GET_MAG_Z);
	buffer[i++] = (uint8_t)((tmp >> 8) & 0xff);
	buffer[i++] = (uint8_t)(tmp & 0xff);
	buffer[i++] = sendEstacionamentoTongle;//estacionamento_interrupt_tongle; //Controla a chegada e saida de carros na vaga


	/*for(uint8_t cnt = 0; cnt < APP_EXTI_CNT_NUM;cnt++){
		//mover a contagem inteira de uma só vez evita erros em caso de interrupcao no meio da tranferencia de bytes
		tmp = app.cfg.cfg.pulse_count[cnt];
		buffer[i++] = (tmp >> 24) & 0xff;
		buffer[i++] = (tmp >> 16) & 0xff;
		buffer[i++] = (tmp >> 8) & 0xff;
		buffer[i++] = tmp & 0xff;
	}*/
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
/*void ReadInput0(void){


	static uint32_t last_tick = 0;
	uint32_t tick;
#if (BLINK_PULSE_LED_ENABLE == 1)
	LED_TX(true);
	UTIL_TIMER_Start(&LedPulseTimer);
#endif
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		app.cfg.cfg.pulse_count[0]++;
		last_tick = tick;
	}
	//APP_LOG(TS_OFF, VLEVEL_M, "PL01 = %d\r\n", app.cfg.cfg.pulse_count[0]);
#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	EEPROM_Write(app.cfg.pulse_count[0], EEPROM_NextAddr(0));
#endif
	HAL_EXTI_ClearPending(&hApp_DIO_exti[0], EXTI_TRIGGER_RISING_FALLING);
}*/
int getInterruptFlag(void)
{
	return flagInterrupt;
}
void resetInterruptFlag(void)
{
	flagInterrupt = 0;
}

void FXOS8700CQForceSleep(void)
{
	uint8_t Sysmod;
	HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_SYSMOD, 1, &Sysmod, 1, i2c_timeout);
	//APP_LOG(TS_OFF, VLEVEL_L, "magnetometerForceSleep = %d\n\r", Sysmod);
	if(Sysmod != 0x02)
	{
		FXOS8700CQReset(&hi2c1);
		HAL_Delay(10000);
	}
}

/*void tratarInterrupcao(void)
{
	uint8_t Int_Source;
	HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_INT_SRC_REG, 1, &Int_Source, 1, i2c_timeout);
	if (Int_Source & 0x04)  // Magnetic threshold event detected?
	{

		APP_LOG(TS_OFF, VLEVEL_L, "nMG - BP1 -> INTERRUPT THRESHOLD!\r\n");
		uint8_t M_Ths_Src; //Caputra a polaridade do trigger (Positiva ou Negativa) para os eixos configurados
		HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_THS_SRC_REG, 1, &M_Ths_Src, 1, i2c_timeout);
		if(estacionamento_interrupt_tongle == 0)
		{

			APP_LOG(TS_OFF, VLEVEL_L, "nMG - BP1 -> Carro Estacionou\r\n");
			estacionamento_interrupt_tongle = 1;
			int value = magnetometter_calibrate(INTERRUPT_MENOR_QUE);
			magnetometter_init_sleep(&hi2c1, value, INTERRUPT_MENOR_QUE);

		}else {

			APP_LOG(TS_OFF, VLEVEL_L, "nMG - BP1 -> Carro Saiu\r\n");
			estacionamento_interrupt_tongle = 0;
			int value = magnetometter_calibrate(INTERRUPT_MAIOR_QUE);
			magnetometter_init_sleep(&hi2c1, value, INTERRUPT_MAIOR_QUE);
		}
	}

	//Indica se ocorreu interrupção ou não.
	HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_INT_SOURCE, 1, &Int_Source, 1, i2c_timeout);
	if (Int_Source & 0x80)  // Auto-sleep/"ake interrupt?
	{
		//Sysmod -> Indica o status do dispositivo: 0b01 (Ativo) 0b10 (Dormindo)
		uint8_t Sysmod;
		HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_SYSMOD, 1, &Sysmod, 1, i2c_timeout);
		APP_LOG(TS_OFF, VLEVEL_L, "nMG - BP2 -> Return to sleep mode Int_Source %02X, Sysmod %02X\r\n", Int_Source, Sysmod);
		if(Sysmod != 0x02)
		{
			while(1);
		}
	}
	flagInterrupt = 0;
	App_Send(SEND_MODE_AUTO);
}*/

void tratarInterrupcao(void)
{
	//static uint32_t last_tick = 0;

	uint8_t Int_Source;
	HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_INT_SRC_REG, 1, &Int_Source, 1, i2c_timeout);
	if (Int_Source & 0x04)  // Magnetic threshold event detected?
	{

		APP_LOG(TS_OFF, VLEVEL_L, "INTERRUPT THRESHOLD\r\n");
		uint8_t M_Ths_Src; //Caputra a polaridade do trigger (Positiva ou Negativa) para os eixos configurados
		HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_THS_SRC_REG, 1, &M_Ths_Src, 1, i2c_timeout);

		/*uint32_t tick;
		tick = UTIL_TIMER_GetCurrentTime();
		if(tick - last_tick >= INPUT_DEBOUNCE_TIME)
		{
			last_tick = tick;
			app.stt.btnCnt++;
			UTIL_TIMER_Stop(&btnTimeOutTimer);
			UTIL_TIMER_Start(&btnTimeOutTimer);
		}*/
		app.stt.btnCnt++;
		if(estacionamento_interrupt_tongle == 0)
		{

			estacionamento_interrupt_tongle = 1;
			int value = FXOS8700CQCalibrate(INTERRUPT_MENOR_QUE);
			FXOS8700CQInitSleep(&hi2c1, value, INTERRUPT_MENOR_QUE);

		}else {

			estacionamento_interrupt_tongle = 0;
			int value = FXOS8700CQCalibrate(INTERRUPT_MAIOR_QUE);
			FXOS8700CQInitSleep(&hi2c1, value, INTERRUPT_MAIOR_QUE);
		}

	}

	//Indica se ocorreu interrupção ou não de sleep mode.
	HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_INT_SOURCE, 1, &Int_Source, 1, i2c_timeout);
	if (Int_Source & 0x80)  // Auto-sleep/"ake interrupt?
	{
		//Sysmod -> Indica o status do dispositivo: 0b01 (Ativo) 0b10 (Dormindo)
		uint8_t Sysmod;
		HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_SYSMOD, 1, &Sysmod, 1, i2c_timeout);
		APP_LOG(TS_OFF, VLEVEL_L, "MAGNETOMETER: GO TO SLEEP| Int_Source %02X, Sysmod %02X\r\n", Int_Source, Sysmod);
		if(Sysmod != 0x02)
		{
			HAL_Delay(10000);
		}
		UTIL_TIMER_Stop(&btnTimeOutTimer);
		UTIL_TIMER_Start(&btnTimeOutTimer);

	}
	APP_LOG(TS_OFF, VLEVEL_L, "app.stt.btnCnt = %d\n\r", app.stt.btnCnt);
}


/*
 * Interrupcao externa para a entrada do magnetômetro
 */
void ReadInput1(void){

	flagInterrupt = 1;
	HAL_EXTI_ClearPending(&hApp_DIO_exti[1], EXTI_TRIGGER_RISING_FALLING);
	/*uint8_t Int_Source;
	HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_INT_SRC_REG, 1, &Int_Source, 1, i2c_timeout);
	if (Int_Source & 0x04)  // Magnetic threshold event detected?
	{

		APP_LOG(TS_OFF, VLEVEL_L, "nMG - BP1 -> INTERRUPT THRESHOLD!\r\n");
		uint8_t M_Ths_Src; //Caputra a polaridade do trigger (Positiva ou Negativa) para os eixos configurados
		HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_M_THS_SRC_REG, 1, &M_Ths_Src, 1, i2c_timeout);
		if(estacionamento_interrupt_tongle == 0)
		{

			APP_LOG(TS_OFF, VLEVEL_L, "nMG - BP1 -> Carro Estacionou\r\n");
			estacionamento_interrupt_tongle = 1;
			int value = magnetometter_calibrate(INTERRUPT_MENOR_QUE);
			magnetometter_init_sleep(&hi2c1, value, INTERRUPT_MENOR_QUE);

		}else {

			APP_LOG(TS_OFF, VLEVEL_L, "nMG - BP1 -> Carro Saiu\r\n");
			estacionamento_interrupt_tongle = 0;
			int value = magnetometter_calibrate(INTERRUPT_MAIOR_QUE);
			magnetometter_init_sleep(&hi2c1, value, INTERRUPT_MAIOR_QUE);
		}
	}

	//Indica se ocorreu interrupção ou não.
	HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_INT_SOURCE, 1, &Int_Source, 1, i2c_timeout);
	if (Int_Source & 0x80)  // Auto-sleep/"ake interrupt?
	{
		//Sysmod -> Indica o status do dispositivo: 0b01 (Ativo) 0b10 (Dormindo)
		uint8_t Sysmod;
		HAL_I2C_Mem_Read(&hi2c1, FXOS8700CQ_SLAVE_ADDR << 1, FXOS8700CQ_SYSMOD, 1, &Sysmod, 1, i2c_timeout);
		APP_LOG(TS_OFF, VLEVEL_L, "nMG - BP2 -> Return to sleep mode Int_Source %02X, Sysmod %02X\r\n", Int_Source, Sysmod);
		if(Sysmod != 0x02)
		{
			while(1);
		}
		App_Send(SEND_MODE_MANUAL);

	}*/

/*
	static uint32_t last_tick = 0;
	uint32_t tick;
#if (BLINK_PULSE_LED_ENABLE == 1)
	LED_TX(true);
	UTIL_TIMER_Start(&LedPulseTimer);
#endif
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		app.cfg.cfg.pulse_count[1]++;
		last_tick = tick;
	}
	//APP_LOG(TS_OFF, VLEVEL_M, "PL02 = %d\r\n", app.cfg.cfg.pulse_count[1]);
#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	EEPROM_Write(app.cfg.pulse_count[1], EEPROM_NextAddr(1));
#endif
	HAL_EXTI_ClearPending(&hApp_DIO_exti[1], EXTI_TRIGGER_RISING_FALLING);*/
}

/*
 * Interrupcao externa para a entrada de pulso 3
 */
/*void ReadInput2(void){
	static uint32_t last_tick = 0;
	uint32_t tick;
#if (BLINK_PULSE_LED_ENABLE == 1)
	LED_TX(true);
	UTIL_TIMER_Start(&LedPulseTimer);
#endif
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		//app.cfg.cfg.pulse_count[2]++;
		last_tick = tick;
	}
	//APP_LOG(TS_OFF, VLEVEL_M, "PL03 = %d\r\n", app.cfg.cfg.pulse_count[2]);
#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	//EEPROM_Write(app.cfg.pulse_count[2], EEPROM_NextAddr(2));
#endif
	HAL_EXTI_ClearPending(&hApp_DIO_exti[2], EXTI_TRIGGER_RISING_FALLING);
}*/

/*
 * Interrupcao externa para a entrada de pulso 4
 */
/*void ReadInput3(void){
	static uint32_t last_tick = 0;
	uint32_t tick;
#if (BLINK_PULSE_LED_ENABLE == 1)
	LED_TX(true);
	UTIL_TIMER_Start(&LedPulseTimer);
#endif
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		//app.cfg.cfg.pulse_count[3]++;
		last_tick = tick;
	}
	//APP_LOG(TS_OFF, VLEVEL_M, "PL04 = %d\r\n", app.cfg.cfg.pulse_count[3]);
#if	(SAVE_CNT_MODE == SAVE_CNT_ON_PULSE)
	//EEPROM_Write(app.cfg.pulse_count[3], EEPROM_NextAddr(3));
#endif
	HAL_EXTI_ClearPending(&hApp_DIO_exti[3], EXTI_TRIGGER_RISING_FALLING);
}*/

/*
 * Interrupcao externa para a entrada digital do usuario
 */
void ReadInput4(void){
	/*static uint32_t last_tick = 0;
	uint32_t tick;
	tick = UTIL_TIMER_GetCurrentTime();
	if(tick - last_tick >= INPUT_DEBOUNCE_TIME){
		last_tick = tick;
		estacionamento_interrupt_tongle = 0;
		uint8_t wiam = FXOS8700CQInitSleep(&hi2c1, FXOS8700CQCalibrate(INTERRUPT_MAIOR_QUE), INTERRUPT_MAIOR_QUE);//magnetometter_init(&hi2c1);//magnetometter_init_sleep(&hi2c1, 65097);
		//app.stt.btnCnt++;
		//UTIL_TIMER_Stop(&btnTimeOutTimer);
		//UTIL_TIMER_Start(&btnTimeOutTimer);
	}*/

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
	/*if ((appData != NULL) && (params != NULL)){
		if(appData->Port == APP_DOWNLINK_PORT){
			if(appData->BufferSize == 3){
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
			}else{
				APP_LOG(TS_OFF, VLEVEL_M, "Tamanho de Downlink invalido: %d\r\n", appData->BufferSize);
			}
		}else{
			APP_LOG(TS_OFF, VLEVEL_M, "Porta de Downlink invalida: %d\r\n", appData->Port);
		}
		return;
	}
	APP_LOG(TS_OFF, VLEVEL_M, "Downlink invalido!\r\n");*/
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
