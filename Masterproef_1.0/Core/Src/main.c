/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADF7242.h"
#include "OLED.h"
#include "CBUF.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s5;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S5_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);

/* Basic functions */
void Potmeter_Init(uint8_t);
void Setup(void);
void Play_Audio(void);
void Transmit(void);
void Receive(void);
void ResponseToTXBuffer(void);
void ReadPacket(void);

/* Encryption functions */
void WriteKeyPacket(void);
void ReadKeyPacket(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Settings */
char settingsMode;
uint8_t settingsDownsampling;
uint16_t settingsSampleRate;
uint8_t settingsVolume;
uint8_t settingsPacketLength;
uint8_t settingsResolution;					// 8 or 12
uint8_t settingsEncryption;					// 0 = Encryption off, 1 = Encryption on
uint32_t settingsFrequency;					// Frequency in kHz

/* General variables */
uint8_t returnValue;
uint16_t adcVal;
uint16_t adcValDownSampled;
uint32_t counter = 0;

// Tx variables
cbuf_handle_t Tx_buffer_handle_t;
uint32_t Tx_teller = 0;
uint8_t TX_BUFFER_BASE;
uint32_t Tx_Pkt_counter = 0;
uint8_t Tx_byte1;
uint8_t Tx_byte2;
uint8_t Tx_byte3;
uint8_t Tx_byteCounter;
uint16_t Tx_sample1;
uint16_t Tx_sample2;
uint32_t Tx_test_teller;

// Rx variables
cbuf_handle_t Rx_buffer_handle_t;
uint32_t DAC_teller = 0;
uint32_t Rx_teller = 0;
uint8_t RX_BUFFER_BASE;
uint8_t Rx_resolution;
uint32_t Rx_Pkt_counter = 0;
uint8_t Rx_Pkt_length;
uint8_t Rx_byte1;
uint8_t Rx_byte2;
uint8_t Rx_byte3;
uint8_t Rx_byteCounter;
uint16_t Rx_sample1;
uint16_t Rx_sample2;
uint8_t Rx_RSSI;
uint8_t Rx_SQI;
uint8_t Rx_play;
uint32_t test;
uint8_t Rx_Pkt_type;

uint8_t Response_Pkt_length;
uint8_t Response_Packet_Type;
uint8_t Response_RSSI;
uint8_t Response_SQI;

uint8_t RX_PACKET_RECEIVED = 0;
uint8_t TX_PACKET_SEND = 0;

uint8_t Dummy;

uint8_t status;

uint32_t KeyPacketCounter;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		if (settingsMode == 'T')
		{
			Transmit();
		}
		else if (settingsMode == 'R')
		{
			test++;
			Play_Audio();
		}
	}

	if (htim->Instance == TIM9)
	{
		KeyPacketCounter++;

		WriteKeyPacket();
		ADF_set_Tx_mode();
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2S5_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  /* Settings */
  settingsMode = 'T';
  settingsDownsampling = 1;
  settingsSampleRate = 16000;
  settingsVolume = 24;
  settingsPacketLength = 30;
  settingsResolution = 8;
  settingsEncryption = 0;
  settingsFrequency = 247000;

  ADF_Init(settingsFrequency, settingsMode);
  Setup();

  if (settingsMode == 'T')
  {
//	  HAL_TIM_Base_Start_IT(&htim9);
	  WriteKeyPacket();
	  ADF_set_Tx_mode();
  }
  else if (settingsMode == 'R')
  {
	  WriteKeyPacket();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// Packet Received Interrupt
	if (RX_PACKET_RECEIVED)
	{
		RX_PACKET_RECEIVED = 0;

		if (settingsMode == 'T')
		{
			KeyPacketCounter++;

			ReadKeyPacket();

			HAL_Delay(10);

			WriteKeyPacket();
			ADF_set_Tx_mode();
		}
	}

	// Packet Send Interrupt
	if (TX_PACKET_SEND)
	{
		TX_PACKET_SEND = 0;

		if (settingsMode == 'R')
		{
			ReadPacket();
			WriteKeyPacket();

			ADF_set_Rx_mode();
		}
	}


//	if (settingsMode == 'R')
//	{
//		if (RX_PACKET_RECEIVED)
//		{
//			RX_PACKET_RECEIVED = 0;
//			Rx_Pkt_counter++;
//			Receive();
//		}
//	}
//	else if (settingsMode == 'T')
//	{
//		if (RX_PACKET_RECEIVED)
//		{
//			RX_PACKET_RECEIVED = 0;
//			ADF_SPI_RD_Response();
//		}
//	}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.I2SClockSelection = RCC_I2SAPBCLKSOURCE_PLLR;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T5_CC1;
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
  sConfig.Channel = ADC_CHANNEL_2;
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  * @brief I2S5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S5_Init(void)
{

  /* USER CODE BEGIN I2S5_Init 0 */

  /* USER CODE END I2S5_Init 0 */

  /* USER CODE BEGIN I2S5_Init 1 */

  /* USER CODE END I2S5_Init 1 */
  hi2s5.Instance = SPI5;
  hi2s5.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s5.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s5.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s5.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s5.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s5.Init.CPOL = I2S_CPOL_LOW;
  hi2s5.Init.ClockSource = I2S_CLOCK_PLLR;
  hi2s5.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S5_Init 2 */

  /* USER CODE END I2S5_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Period = 12000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 3000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 3000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 96;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 10000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin|POT_CS_Pin|A_MIC_POWER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BUTTON4_Pin BUTTON3_Pin BUTTON2_Pin BUTTON_1_Pin */
  GPIO_InitStruct.Pin = BUTTON4_Pin|BUTTON3_Pin|BUTTON2_Pin|BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON5_Pin */
  GPIO_InitStruct.Pin = BUTTON5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF7242_CS_Pin SWITCH_E_Pin POT_CS_Pin A_MIC_POWER_Pin */
  GPIO_InitStruct.Pin = ADF7242_CS_Pin|SWITCH_E_Pin|POT_CS_Pin|A_MIC_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF7242_IRQ1_Pin ADF7242_IRQ2_Pin */
  GPIO_InitStruct.Pin = ADF7242_IRQ1_Pin|ADF7242_IRQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void Setup(void)
{
	OLED_init();
	OLED_print_text("Jelle's Walkie", 10, 30);
	OLED_update();

	HAL_Delay(2000);

	OLED_clear_screen();

	switch(settingsMode)
	{
		case 'I':
			OLED_print_text("Walkie in Idle mode", 0, 0);

			break;

		case 'T':
			OLED_print_text("Walkie in Tx mode", 0, 0);

			/* Power savings */
			HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin, GPIO_PIN_RESET);								// Shutdown LM386 to prevent power consumption

			/* Buffer Settings for Tx mode */
			uint16_t Tx_buffer_size = 512;
			uint16_t *Tx_buffer = malloc(Tx_buffer_size * sizeof(uint16_t));					// Tx_buffer with size of 512 bytes (32 ms)
			Tx_buffer_handle_t = circular_buf_init(Tx_buffer, Tx_buffer_size);					// Tx buffer handle type

			/* OLED debug */
			OLED_print_variable("Sample rate:", settingsSampleRate, 0, 10);
			OLED_print_variable("Cbuf size:", Tx_buffer_size, 0, 20);
			OLED_print_variable("Tx buf base:", TX_BUFFER_BASE, 0, 30);
			uint32_t frequency = ADF_RD_Frequency_MHz();
			OLED_print_variable("Freq (MHz):", frequency, 0, 40);
			OLED_print_variable("Downsampling:", settingsDownsampling, 0, 50);
			OLED_update();

//			/* HAL audio settings for Tx mode */
//			HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_1);											// Start timer 5 (frequency = 16 kHz)
//			HAL_ADC_Start_IT(&hadc1);															// Start ADC interrupt triggered by timer 5
//			HAL_TIM_Base_Start_IT(&htim1);														// Start timer 1 (frequency = 8 kHz)

			break;

		case 'R':
			OLED_print_text("Walkie in Rx mode", 0, 0);

			/* Power savings */
			HAL_GPIO_WritePin(GPIOB, A_MIC_POWER_Pin, GPIO_PIN_RESET);							// Shutdown the analog microphone to prevent power consumption

			/* Buffer Settings for Rx mode */
			uint16_t Rx_buffer_size = 512;
			uint16_t *Rx_buffer = malloc(Rx_buffer_size * sizeof(uint16_t));					// Rx_buffer with size of 512 bytes (32 ms)
			Rx_buffer_handle_t = circular_buf_init(Rx_buffer, Rx_buffer_size);					// Rx buffer handle type

			/* OLED debug */
			OLED_print_variable("POT volume:", settingsVolume, 0, 10);

//			/* HAL audio settings for Rx mode */
//			Potmeter_Init(settingsVolume);														// Setting the volume with the potentiometer
//			HAL_DAC_Start(&hdac, DAC_CHANNEL_1);												// Start the DAC interface
//			HAL_TIM_Base_Start_IT(&htim1);														// Start timer 1 (frequency = 8 kHz)

			/* Rx mode */
			ADF_set_Rx_mode();																	// Rx mode

			/* OLED debug */
			uint8_t status;
			status = ADF_SPI_SEND_BYTE(0xff);
			OLED_print_variable("Status:", status, 0, 20);
			OLED_update();

			break;
	}
}

void Potmeter_Init(uint8_t volume)
{
	uint8_t value[1];
	value[0]= volume;
	HAL_GPIO_WritePin(GPIOB, POT_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, value, 1, 50);
	HAL_GPIO_WritePin(GPIOB, POT_CS_Pin, GPIO_PIN_SET);
}

void Play_Audio(void)
{
	uint16_t sample;

	uint16_t size = circular_buf_size(Rx_buffer_handle_t);
	if (size != 0)
	{
		if (Rx_resolution == 12)
		{
			returnValue = circular_buf_get(Rx_buffer_handle_t, &sample);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sample);
		}
		else if (Rx_resolution == 8)
		{
			returnValue = circular_buf_get(Rx_buffer_handle_t, &sample);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sample);
		}
		DAC_teller++;
	}
}

void Transmit(void)
{
	Tx_test_teller++;
	if (Tx_teller == settingsPacketLength + 2)
	{
		ADF_set_Tx_mode();
		Tx_Pkt_counter++;
		Tx_teller = 0;
	}

	if (Tx_teller == 0)
	{
		//Write Header byte with the length of the packet (1 + 1 + lengte + 2)
		uint8_t packet_length = settingsPacketLength + 4;
		ADF_SPI_MEM_WR(TX_BUFFER_BASE, packet_length);
		Tx_teller++;
		ADF_SPI_MEM_WR(TX_BUFFER_BASE + Tx_teller, settingsResolution);
		Tx_teller++;

		Tx_byteCounter = 0;
	}

	uint16_t size = circular_buf_size(Tx_buffer_handle_t);
	if (size != 0)
	{
		if (settingsResolution == 8)
		{
			returnValue = circular_buf_get(Tx_buffer_handle_t, &Tx_sample1);
			ADF_SPI_MEM_WR(TX_BUFFER_BASE + Tx_teller, Tx_sample1);
			Tx_teller++;
		}
		else if (settingsResolution == 12)
		{
			if (Tx_byteCounter == 0)
			{
				returnValue = circular_buf_get(Tx_buffer_handle_t, &Tx_sample1);
				Tx_byte1 = Tx_sample1&0x0ff;
				ADF_SPI_MEM_WR(TX_BUFFER_BASE + Tx_teller, Tx_byte1);
				Tx_teller++;

				returnValue = circular_buf_get(Tx_buffer_handle_t, &Tx_sample2);
				Tx_byte2 = (Tx_sample1>>4)&0x0f0;
				Tx_byte2 = Tx_byte2|(Tx_sample2&0x00f);
				ADF_SPI_MEM_WR(TX_BUFFER_BASE + Tx_teller, Tx_byte2);
				Tx_teller++;

				Tx_byteCounter++;
			}
			else if (Tx_byteCounter == 2)
			{
				Tx_byte3 = (Tx_sample2>>4)&0xff;
				ADF_SPI_MEM_WR(TX_BUFFER_BASE + Tx_teller, Tx_byte3);
				Tx_teller++;
				Tx_byteCounter = -1;
			}
			Tx_byteCounter++;
		}
	}
}

void Receive(void)
{
	ADF_SPI_RD_Rx_Buffer();
	ADF_clear_Rx_flag();
	ADF_set_Rx_mode();
}

void ReadPacket(void)
{
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	uint8_t bytes[2];
	bytes[0] = 0x30;
	bytes[1] = 0xff;

	HAL_SPI_Transmit(&hspi2, bytes, 2, 50);

	HAL_SPI_Receive(&hspi2, &Rx_Pkt_length, 1, 50);
	HAL_SPI_Receive(&hspi2, &Rx_Pkt_type, 1, 50);

	switch(Rx_Pkt_type)
	{
		// Key packet
		case 1:
			HAL_SPI_Receive(&hspi2, &Dummy, 1, 50);
			HAL_SPI_Receive(&hspi2, &Rx_RSSI, 1, 50);
			HAL_SPI_Receive(&hspi2, &Rx_SQI, 1, 50);
	}

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (ADF_SPI_READY() == 0);
}

void ReadKeyPacket(void)
{
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	uint8_t bytes[2];
	bytes[0] = 0x30;
	bytes[1] = 0xff;

	HAL_SPI_Transmit(&hspi2, bytes, 2, 50);

	HAL_SPI_Receive(&hspi2, &Rx_Pkt_length, 1, 50);
	HAL_SPI_Receive(&hspi2, &Rx_Pkt_type, 1, 50);
	HAL_SPI_Receive(&hspi2, &Dummy, 1, 50);
	uint8_t extra;
	HAL_SPI_Receive(&hspi2, &extra, 1, 50);
	HAL_SPI_Receive(&hspi2, &Rx_RSSI, 1, 50);
	HAL_SPI_Receive(&hspi2, &Rx_SQI, 1, 50);
	HAL_SPI_Receive(&hspi2, &status, 1, 50);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (ADF_SPI_READY() == 0);
}

void WriteKeyPacket(void)
{
	ADF_SPI_MEM_WR(TX_BUFFER_BASE, 5);					// Packet length
	ADF_SPI_MEM_WR(TX_BUFFER_BASE + 1, 1);				// Packet type; 1=key packet
	ADF_SPI_MEM_WR(TX_BUFFER_BASE + 2, 15);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (settingsDownsampling)
	{
		if (counter%2)
		{
			adcVal = HAL_ADC_GetValue(&hadc1);
			counter++;
		}
		else
		{
			adcValDownSampled = HAL_ADC_GetValue(&hadc1);
			adcValDownSampled += adcVal;
			adcValDownSampled /= 2;
			circular_buf_put_overwrite(Tx_buffer_handle_t, adcValDownSampled);
			counter++;
		}
	}
	else
	{
		adcVal = HAL_ADC_GetValue(&hadc1);
		circular_buf_put_overwrite(Tx_buffer_handle_t, adcVal);
		counter++;
	}
}

/* Callback external interrupts */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ADF7242_IRQ1_Pin)
	{
		TX_PACKET_SEND = 1;
		ADF_clear_Tx_flag();
	}
	else if (GPIO_Pin == ADF7242_IRQ2_Pin)
	{
		RX_PACKET_RECEIVED = 1;
		ADF_clear_Rx_flag();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
