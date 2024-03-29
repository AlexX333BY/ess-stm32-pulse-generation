/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define FRACTIONAL_FREQ_DISPLAY_LENGTH 4

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
uint16_t signalPeriod;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
uint16_t Max(const uint16_t, const uint16_t);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
void InitDisplay(void);
void UpdateDisplay(void);
void ResetDisplay(void);
bool PrintNextNumber(const uint8_t nextNumber);
void PrintNextSymbol(const char symbol);
void SendToDisplay(const bool isSymbol, const uint8_t data, const uint32_t cmdDelay);
uint8_t GetDigitsCount(uint16_t number);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);
  InitDisplay();
  HAL_ADC_Start_IT(&hadc1);

  /* Infinite loop */
  while (true)
  {
    HAL_GPIO_TogglePin(SIG_GPIO_Port, SIG_Pin);
    HAL_Delay(Max(signalPeriod / 2, 1));
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1)
  {
    const uint16_t oldPeriod = signalPeriod;
    signalPeriod = Max((uint16_t)HAL_ADC_GetValue(&hadc1), 1);
    if (oldPeriod != signalPeriod)
    {
      UpdateDisplay();
    }
    HAL_ADC_Start_IT(&hadc1);
  }
}

uint16_t Max(const uint16_t a, const uint16_t b)
{
  return a > b ? a : b;
}

void InitDisplay(void)
{
  HAL_Delay(16);
  const uint8_t setBusTo8Bit = 0x34,
    shiftOnWrite = 0x6,
    enableDisplay = 0xC;
  const uint8_t busCmdDelay = 1,
    shiftCmdDelay = 1,
    enableCmdDelay = 1;
  SendToDisplay(false, setBusTo8Bit, busCmdDelay);
  SendToDisplay(false, shiftOnWrite, shiftCmdDelay);
  SendToDisplay(false, enableDisplay, enableCmdDelay);
  ResetDisplay();
}

void ResetDisplay(void)
{
  const uint8_t resetDataCmd = 0x1,
    resetDataDelay = 2;
  SendToDisplay(false, resetDataCmd, resetDataDelay);
}

void SendToDisplay(const bool isSymbol, const uint8_t data, const uint32_t cmdDelay)
{
  HAL_GPIO_WritePin(DISPLAY_CMD_GPIO_Port, DISPLAY_CMD_Pin, isSymbol ? GPIO_PIN_SET : GPIO_PIN_RESET);
  
  const uint8_t dataStartPin = 8;
  HAL_GPIO_WritePin(GPIOA, data << dataStartPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, (~data) << dataStartPin, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(DISPLAY_SYNC_GPIO_Port, DISPLAY_SYNC_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(DISPLAY_SYNC_GPIO_Port, DISPLAY_SYNC_Pin, GPIO_PIN_RESET);
  HAL_Delay(cmdDelay);
  HAL_GPIO_WritePin(GPIOA, data << dataStartPin, GPIO_PIN_RESET);
}

void PrintNextSymbol(const char symbol)
{
  SendToDisplay(true, symbol, 1);
}

bool PrintNextNumber(const uint8_t nextNumber)
{
  const bool isNumberValid = nextNumber < 10;
  if (isNumberValid)
  {
    const uint8_t asciiNumberOffset = 0x30;
    PrintNextSymbol(nextNumber + asciiNumberOffset);
  }
  return isNumberValid;
}

void UpdateDisplay(void)
{
  ResetDisplay();
  const double frequency = (double)1000 / signalPeriod;
  const uint16_t integralFrequency = (uint16_t)frequency;
  
  const uint8_t integralFreqDigitsCount = GetDigitsCount(integralFrequency);
  for (uint8_t curDigit = integralFreqDigitsCount; curDigit > 0; --curDigit)
  {
    PrintNextNumber(integralFrequency % (uint16_t)pow(10, curDigit) / pow(10, (curDigit - 1)));
  }
  
  if (FRACTIONAL_FREQ_DISPLAY_LENGTH > 0)
  {
    PrintNextSymbol('.');
    const double fractionalFrequency = frequency - integralFrequency;
    for (uint8_t curDigit = 1; curDigit <= FRACTIONAL_FREQ_DISPLAY_LENGTH; ++curDigit)
    {
      PrintNextNumber((uint8_t)(fractionalFrequency * pow(10, curDigit)) % 10);
    }
  }
  
  const char postfix[] = " Hz";
  for (uint8_t symbol = 0; symbol < sizeof(postfix) / sizeof(postfix[0]); ++symbol)
  {
    PrintNextSymbol(postfix[symbol]);
  }
}

uint8_t GetDigitsCount(uint16_t number)
{
  uint8_t digitsCount = 0;
  do
  {
    number /= 10;
    ++digitsCount;
  } while (number != 0);
  
  return digitsCount;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIG_Pin|DISPLAY_CMD_Pin|DISPLAY_SYNC_Pin|DISPLAY_DATA_0_Pin 
                          |DISPLAY_DATA_1_Pin|DISPLAY_DATA_2_Pin|DISPLAY_DATA_3_Pin|DISPLAY_DATA_4_Pin 
                          |DISPLAY_DATA_5_Pin|DISPLAY_DATA_6_Pin|DISPLAY_DATA_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SIG_Pin DISPLAY_CMD_Pin DISPLAY_SYNC_Pin DISPLAY_DATA_0_Pin 
                           DISPLAY_DATA_1_Pin DISPLAY_DATA_2_Pin DISPLAY_DATA_3_Pin DISPLAY_DATA_4_Pin 
                           DISPLAY_DATA_5_Pin DISPLAY_DATA_6_Pin DISPLAY_DATA_7_Pin */
  GPIO_InitStruct.Pin = SIG_Pin|DISPLAY_CMD_Pin|DISPLAY_SYNC_Pin|DISPLAY_DATA_0_Pin 
                          |DISPLAY_DATA_1_Pin|DISPLAY_DATA_2_Pin|DISPLAY_DATA_3_Pin|DISPLAY_DATA_4_Pin 
                          |DISPLAY_DATA_5_Pin|DISPLAY_DATA_6_Pin|DISPLAY_DATA_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
