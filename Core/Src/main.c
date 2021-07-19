/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f407xx.h"
#include "arm_math.h"
#include "stm32f4_discovery_accelerometer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IWDG_BIT_rtcTask		(1 << 0)			/*RTC flag			(0000 0001)*/
#define IWDG_BIT_acceleroTask           (1 << 1)			/*Accelero flag 		(0000 0010)*/
#define IWDG_BIT_transmitTask		(1 << 2)			/*Transmit flag			(0000 0100)*/
#define IWDG_BIT_payloadTask		(1 << 3)			/*payload flag			(0000 1000)*/
/* Group bit define */
#define	IWDG_BIT_Beacon	 (IWDG_BIT_rtcTask | IWDG_BIT_acceleroTask | IWDG_BIT_transmitTask)	/* Beacon  Group (0000 0111) */
#define	IWDG_BIT_Payload (IWDG_BIT_rtcTask | IWDG_BIT_payloadTask | IWDG_BIT_transmitTask)	/* payload Group (0000 1011) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t RX1_Char = 0x00;/* MODE Change variable Global variable*/
data_store acceler_store;/* store Accelero */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
TaskHandle_t LaunchTest = NULL;
TaskHandle_t Referance = NULL;
TaskHandle_t Iwdg = NULL;
TaskHandle_t RealTimeClock = NULL;
TaskHandle_t Accelero = NULL;
TaskHandle_t Payload = NULL;
TaskHandle_t Transmit = NULL;

xQueueHandle rtcQueue, acceleroQueue;
EventGroupHandle_t iwdgEventGroup;
EventBits_t uxBits;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* User function call */
uint8_t Mode_take(void);
void UARTRx_IT(void);
void LoRa_Set(void);
void Uplink_Ground(void);
void Accelero_funct(void);
void Accelero_RMS_q(void);
void beacon_data_f(void);
void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data);//Ex : Flash_Write(0x080E0000,Data);
uint32_t Flash_Read(uint32_t Flash_Address);		      //Ex : data =Flash_Read(0x080E0000);
void StartCreateTask(void);
void IWDGReset_Init(void);
void LaunchTestTask(void *argument);
void ReferanceTask(void *argument);
void IwdgTask(void *argument);
void RealTimeClockTask(void *argument);
void AcceleroTask(void *argument);
void PayloadTask(void *argument);
void TransmitTask(void *argument);

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
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  //MX_IWDG_Init();//program enable when debug 2 sec need to reset
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  BSP_ACCELERO_Init();
  UARTRx_IT();
  LoRa_Set();

  /* USER CODE BEGIN RTOS_QUEUES */
  rtcQueue = xQueueCreate(1,sizeof(RTC_TimeTypeDef));
  acceleroQueue = xQueueCreate(1,sizeof(accelero_datastruct));
	
  /* USER CODE END RTOS_QUEUES */
	
  RX1_Char = Mode_take();/* Retake Mode if reseet from watchdag */
  IWDGReset_Init();//PA power on
  	
  /* USER CODE BEGIN RTOS_THREADS */	
  StartCreateTask();
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  iwdgEventGroup = xEventGroupCreate();
	
  /* USER CODE END RTOS_EVENTS */
	
  
	
	
  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD6_Pin|Payload_EPS_DOUT_Pin|PWR_SW_DOUT_Pin|PA_SW_DOUT_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FC_LO_DIN_Pin FC_ABT_DIN_Pin */
  GPIO_InitStruct.Pin = FC_LO_DIN_Pin|FC_ABT_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD6_Pin Payload_EPS_DOUT_Pin PWR_SW_DOUT_Pin PA_SW_DOUT_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD6_Pin|Payload_EPS_DOUT_Pin|PWR_SW_DOUT_Pin|PA_SW_DOUT_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
void StartCreateTask(void)// Stack size need to test
{
  if(RX1_Char == 0x00) xTaskCreate( LaunchTestTask, "launctest", 256, NULL, tskIDLE_PRIORITY + 6, &LaunchTest);
  xTaskCreate( ReferanceTask,  "referance", 512, NULL, tskIDLE_PRIORITY + 5, &Referance);
  xTaskCreate( IwdgTask, "iwdg", 256, NULL, tskIDLE_PRIORITY + 4, &Iwdg);
  xTaskCreate( RealTimeClockTask, "rtc", 256, NULL, tskIDLE_PRIORITY + 3, &RealTimeClock);
  if(RX1_Char != '8') xTaskCreate( AcceleroTask,  "accelero", 512, NULL, tskIDLE_PRIORITY + 2, &Accelero);
  xTaskCreate( PayloadTask,  "payload", 256, NULL, tskIDLE_PRIORITY + 2, &Payload);
  xTaskCreate( TransmitTask,  "transmit", 512, NULL, tskIDLE_PRIORITY + 1, &Transmit);
}
/* USER CODE END Header_StartDefaultTask */

/* Interrupt Handle */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t default_t[] = "\r\nError input!\r\n";
  uint8_t uplink[] = "\r\nGround station uplink succeed\r\n";
  HAL_UART_Receive_IT(&huart3, &RX1_Char, 1);
  switch(RX1_Char)
  {
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case 'A':
      case 'B':
      case 'C':
        HAL_UART_Transmit( &huart3, uplink, sizeof(uplink)-1,50);
       break;
      case '6':
        xTaskResumeFromISR(LaunchTest);
        break;
      /*case '7':
        xTaskResumeFromISR(Referance);
        break;//Debug test use*/
      default:
        HAL_UART_Transmit( &huart3, default_t, sizeof(default_t)-1,100);//Error output
        RX1_Char = 0x00; 
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//Need Hardware check
{
  if(GPIO_Pin == GPIO_PIN_9){
      RX1_Char = '7';
      xTaskResumeFromISR(Referance);
  }
  if(GPIO_Pin == GPIO_PIN_11){
      RX1_Char = '8';
      xTaskResumeFromISR(Referance);
  }
}

/* User CODE BEGIN My function */
void UARTRx_IT(void)
{
  HAL_UART_Receive_IT(&huart3, &RX1_Char, 1);
}

void LoRa_Set(void)
{
  /* LoRa Parameter Setting*/
  uint8_t PARAMETER[] = "AT+PARAMETER=10,7,1,7\r\n";//LoRa parameter setting
  uint8_t BAND[] = "AT+BAND=434000000\r\n";//434 MHz
  uint8_t CRFOP[] = "AT+CRFOP=00\r\n";// Power output  0 dbm		
  /* HAL function to set LoRa */
  HAL_UART_Transmit( &huart2, PARAMETER, sizeof(PARAMETER)-1, 10000);
  HAL_Delay(500);
  HAL_UART_Transmit( &huart2, BAND, sizeof(BAND)-1, 10000);
  HAL_Delay(500);
  HAL_UART_Transmit( &huart2, CRFOP, sizeof(CRFOP)-1, 10000);
  HAL_Delay(500);
  /* LoRa Setting END */
}
void Uplink_Ground(void)
{
  uint8_t emergency[] ="Emergency return!\r\n";
  uint8_t response[] = "Response OK!\r\n";
  uint8_t mode1_1[] = "AT+SEND=3,24,LoRa communication Link!\r\n";
  switch(RX1_Char)
  {
    case '1'://LoRa Link check
      HAL_UART_Transmit( &huart3, &RX1_Char, 1, 100);
      HAL_UART_Transmit( &huart2, mode1_1, sizeof(mode1_1)-1,100);
      RX1_Char = 0x00;//Mode Reset
      break;
    case '2':
      HAL_UART_Transmit( &huart3, &RX1_Char, 1, 100);//RTC Init response check
      if (HAL_RTC_Init(&hrtc) == HAL_OK) HAL_UART_Transmit( &huart3, response, sizeof(response)-1,100);
      RX1_Char =0x00;
      break;
    case '3'://Accelero response check
      HAL_UART_Transmit( &huart3, &RX1_Char, 1, 100);
      if(BSP_ACCELERO_Init() != ACCELERO_OK) HAL_UART_Transmit( &huart3, response, sizeof(response)-1,100);
      RX1_Char = 0x00;
      break;
    case '4'://Payload test
      HAL_UART_Transmit( &huart3, &RX1_Char, 1, 100);
      RX1_Char = 0x00;
      break;
    case '5'://Enter Beacon Mode
      HAL_UART_Transmit( &huart3, &RX1_Char, 1, 100);
      vTaskSuspend(NULL);
      break;
    case '6'://Emergency Mode 
      HAL_UART_Transmit( &huart3, emergency, sizeof(emergency)-1,100);
      RX1_Char = 0x00;
      break;
    case 'A'://Power Ampifilier on
      HAL_UART_Transmit( &huart3, &RX1_Char, 1, 100);
      HAL_GPIO_WritePin(GPIOD,PA_SW_DOUT_Pin,GPIO_PIN_SET);
      RX1_Char = 0x00;
      break;
    case 'B'://Power Ampifilier off 
      HAL_UART_Transmit( &huart3, &RX1_Char, 1, 100);
      HAL_GPIO_WritePin(GPIOD,PA_SW_DOUT_Pin,GPIO_PIN_RESET);
      RX1_Char = 0x00;
      break;
    case 'C'://Battery voltage ADC 
      HAL_UART_Transmit( &huart3, &RX1_Char, 1, 100);
      /* Add convert */
      RX1_Char = 0x00;
      break;
  }
}

void Accelero_funct(void)
{
  /*USER CODE BEGIN Accelero Init */
  int16_t xyz[3]={0};
  TickType_t xLastWakeTime;
  const portTickType xFrequency = 40;//Sampling rate 40Hz
  xLastWakeTime = xTaskGetTickCount();
  /*USER CODE END Accelero Init */
  for(int i = 0; i < 25; i++)
  {
    BSP_ACCELERO_GetXYZ(xyz);
    acceler_store.data_x[i]=xyz[0];
    acceler_store.data_y[i]=xyz[1];
    acceler_store.data_z[i]=xyz[2];
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void Accelero_RMS_q(void)
{
  /*USER CODE BEGIN Accelero RMS Init */
  accelero_datastruct accelero_data = {.RMS_accelero_x = 0,
                                       .RMS_accelero_y = 0,
                                       .RMS_accelero_z = 0};
  /*USER CODE END Accelero RMS Init */
  arm_rms_q15(acceler_store.data_x,25, &accelero_data.RMS_accelero_x);
  arm_rms_q15(acceler_store.data_y,25, &accelero_data.RMS_accelero_y);
  arm_rms_q15(acceler_store.data_z,25, &accelero_data.RMS_accelero_z);
  
  xQueueSend( acceleroQueue, &accelero_data,1000);
  /*USER CODE END Accelero RMS */
}

void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data)
{
  HAL_FLASH_Unlock();

  FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_3);
  for(int i = 0; i < 3; i++)//need test 
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Flash_Address, Flash_Data);
     
  HAL_FLASH_Lock();
}

uint32_t Flash_Read(uint32_t Flash_Address)
{
  uint32_t Flash_Data;

  Flash_Data = *(uint32_t*) Flash_Address;
  return Flash_Data;
}

uint8_t Mode_take(void)
{
  uint32_t data;
  data = Flash_Read(0x080E0000);
  if(data == 7)
      return '7';
  if(data == 8)
      return '8';
  return 0x00;
}

void IWDGReset_Init(void)
{
  if(RX1_Char == '7' || RX1_Char == '8'){
    HAL_GPIO_WritePin(GPIOD,PA_SW_DOUT_Pin,GPIO_PIN_SET);
  }
}

void beacon_data_f(void)
{
  /* USER CODE BEGIN Beacon_Init */
  RTC_TimeTypeDef sTime={0};
  accelero_datastruct accelero_data;
  char *str;
  /* USER CODE END Beacon_Init  */
  xQueueReceive( rtcQueue, &sTime, portMAX_DELAY);
  xQueueReceive( acceleroQueue, &accelero_data, portMAX_DELAY);		
  //taskENTER_CRITICAL();	
  str = pvPortMalloc(20 * sizeof(char));
  sprintf(str,"AT+SEND=3,47,%c,%02d%02d%04d%04d%04d\r\n",RX1_Char,sTime.Minutes,sTime.Seconds,accelero_data.RMS_accelero_x,accelero_data.RMS_accelero_y,accelero_data.RMS_accelero_z);
  HAL_UART_Transmit( &huart2, (uint8_t*)str, strlen(str), 1000);			
  memset(&accelero_data, 0, sizeof(accelero_datastruct));
  vPortFree(str);
  //taskEXIT_CRITICAL();		
}

/* User CODE END My function */
void LaunchTestTask(void *argument)
{
  /* USER CODE BEGIN  */
  uint8_t response[] = "Launch Test Task\r\n";
  HAL_UART_Transmit( &huart3, response, sizeof(response)-1, 100);   
  /* Infinite loop */
  for(;;)
  {
    if(RX1_Char != 0x00){
      Uplink_Ground();
    }
  }
  /* USER CODE END  */
}
void ReferanceTask(void *argument)
{
  /* USER CODE BEGIN  */
  uint8_t response[] = "Enter Referance Task\r\n";
  uint8_t mode5[] = "Ready to Launch\r\n";
  HAL_UART_Transmit( &huart2, response, sizeof(response)-1,100);
  if(RX1_Char != 8) vTaskSuspend(Payload);
  /* Infinite loop */
  for(;;)
  {
    switch(RX1_Char)
    {
      case '5':
        HAL_UART_Transmit( &huart2, mode5, sizeof(mode5)-1,100); /* Mode 5 response */
        vTaskSuspend(NULL);
        break;
      case '7':/* Launch IWDG Init */
        Flash_Write(0x080E0000,0x07);
        vTaskDelete(LaunchTest);
        vTaskSuspend(NULL);
        break;
      case '8':/* Seperate Task ready */
        /*Write flash store 8 */
        vTaskResume(Payload);
        vTaskDelete(Accelero);
        /*Queue Event Reset */
        vQueueDelete(acceleroQueue);// untest
        /* Payload EPS ON */
        vTaskDelete(NULL);
        break;
    }
  }
  /* USER CODE END  */
}
void IwdgTask(void *argument)
{
  /* USER CODE BEGIN  */
  const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS*10;	/* MAX 1 sec */
  /* Infinite loop */
  for(;;)
  {
    switch(RX1_Char)
    {
      case '5':
        uxBits = xEventGroupWaitBits(iwdgEventGroup,IWDG_BIT_Beacon,pdTRUE,pdTRUE,xTicksToWait);
      case '7':
        uxBits = xEventGroupWaitBits(iwdgEventGroup,IWDG_BIT_Beacon,pdTRUE,pdTRUE,xTicksToWait);
        if((uxBits & IWDG_BIT_Beacon) == IWDG_BIT_Beacon) HAL_IWDG_Refresh(&hiwdg);
        break;
      case '8':
        uxBits = xEventGroupWaitBits(iwdgEventGroup,IWDG_BIT_Payload,pdTRUE,pdTRUE,xTicksToWait);
        if((uxBits & IWDG_BIT_Payload) == IWDG_BIT_Payload) HAL_IWDG_Refresh(&hiwdg);
        break;
    }		
  }
  /* USER CODE END  */
}

void RealTimeClockTask(void *argument)
{
  /* USER CODE BEGIN  */
  RTC_TimeTypeDef sTime={0};
  RTC_DateTypeDef sDate={0};
  /* Infinite loop */
  for(;;)
  {
    HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN);
    xQueueSend( rtcQueue, &sTime, 1000);
    uxBits = xEventGroupSetBits(iwdgEventGroup, IWDG_BIT_rtcTask);
  }
  /* USER CODE END  */
}

void AcceleroTask(void *argument)
{
  /* USER CODE BEGIN  */
	
  /* Infinite loop */
  for(;;)
  {
    Accelero_funct();
    Accelero_RMS_q();
    memset(&acceler_store, 0, sizeof(data_store));
    uxBits = xEventGroupSetBits(iwdgEventGroup, IWDG_BIT_acceleroTask);
  }
  /* USER CODE END  */
}

void PayloadTask(void *argument)//By Geroge
{
  /* USER CODE BEGIN  */
	
  /* Infinite loop */
  for(;;)
  {
   /* Payload Data receive */ 
  }
  /* USER CODE END  */
}
void TransmitTask(void *argument)
{
  /* USER CODE BEGIN  */
	
  /* Infinite loop */
  for(;;)
  {
    switch(RX1_Char)
    {
      case '5'://unLuanch Mode 
      case '7'://Launch Mode  
        beacon_data_f();
        uxBits = xEventGroupSetBits(iwdgEventGroup,IWDG_BIT_transmitTask);
        break;
      case '8'://payload Mode
        /* Payload  function*/
        break;
    }
  }
  /* USER CODE END  */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
