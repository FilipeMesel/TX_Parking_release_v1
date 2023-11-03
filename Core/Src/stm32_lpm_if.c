/**
  ******************************************************************************
  * @file    stm32_lpm_if.c
  * @author  MCD Application Team
  * @brief   Low layer function to enter/exit low power modes (stop, sleep)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sys_debug.h"
#include "stm32_lpm_if.h"
#include "usart_if.h"
#include "radio_board_if.h"
#include "adc.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "sys_app.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief Power driver callbacks handler
  */
const struct UTIL_LPM_Driver_s UTIL_PowerDriver =
{
  PWR_EnterSleepMode,
  PWR_ExitSleepMode,

  PWR_EnterStopMode,
  PWR_ExitStopMode,

  PWR_EnterOffMode,
  PWR_ExitOffMode,
};

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

void PWR_EnterOffMode(void)
{
  /* USER CODE BEGIN EnterOffMode_1 */

  /* USER CODE END EnterOffMode_1 */
}

void PWR_ExitOffMode(void)
{
  /* USER CODE BEGIN ExitOffMode_1 */

  /* USER CODE END ExitOffMode_1 */
}

void PWR_EnterStopMode(void)
{
  UTILS_ENTER_CRITICAL_SECTION();

  Sx_Board_IoDeInit();
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//  HAL_ADC_MspDeInit(&hadc1);
  //APP_LOG(TS_ON, VLEVEL_M, "\r\nAntes de SystemLowClock_Config\r\n");
  //SystemClock_Config();
  //APP_LOG(TS_ON, VLEVEL_M, "\r\nDepois de SystemLowClock_Config\r\n");
  /*clear wake up flag*/
  SET_BIT(PWR->CR, PWR_CR_CWUF);

  UTILS_EXIT_CRITICAL_SECTION();

  /* Enter Stop Mode */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

void PWR_ExitStopMode(void)
{
  /* Disable IRQ while the MCU is not running on HSI */
	//APP_LOG(TS_OFF, VLEVEL_M, "\nA\n");
  UTILS_ENTER_CRITICAL_SECTION();
  //APP_LOG(TS_OFF, VLEVEL_M, "\r\nExitStopMode 1\r\n");
  /* After wake-up from STOP reconfigure the system clock */
  /* Enable HSI */
  __HAL_RCC_HSI_ENABLE();//__HAL_RCC_MSI_ENABLE();//__HAL_RCC_HSI_ENABLE();
  /* Wait till HSI is ready */
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET) {}//while (__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) == RESET) {}//while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET) {}

  //APP_LOG(TS_OFF, VLEVEL_M, "\r\nExitStopMode 2\r\n");
  //__HAL_RCC_MSI_ENABLE();
  //while (__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) == RESET) {}
  //APP_LOG(TS_OFF, VLEVEL_M, "\r\nExitStopMode 3\r\n");
  /* Enable PLL */
  __HAL_RCC_PLL_ENABLE();
  /* Wait till PLL is ready */
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET) {}

  /* Select PLL as system clock source */
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);//__HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);//__HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_MSI);//__HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);

  /* Wait till PLL is used as system clock source */
  while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK) {};//while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI) {}//while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_MSI) {}//while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK) {}

  //SystemClock_Config();
 // APP_LOG(TS_OFF, VLEVEL_M, "\r\nExitStopMode 4\r\n");
  /* initializes the peripherals */
  Sx_Board_IoInit();

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  UTILS_EXIT_CRITICAL_SECTION();
  //APP_LOG(TS_OFF, VLEVEL_M, "\nB\n");
}

void PWR_EnterSleepMode(void)
{
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void PWR_ExitSleepMode(void)
{
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

