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

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s5;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

/* Default Settings */
char settingsMode = 'R';
uint8_t settingsVolume = 70;					// Potentiometer for volume
uint8_t settingsDataLength = 20;				// Databytes in audio packet
uint8_t settingsResolution = 8;					// 8 or 12 (ADC resolution)
uint8_t settingsEncryption = 1;					// 0 = off, 1 = on
uint32_t settingsFrequency = 245000;			// Frequency in kHz

/* General variables */
uint16_t adcVal;
uint16_t adcValDownSampled;

/* Debug variables */
uint8_t returnValue;
uint32_t counter;
uint32_t Packets_Received;
uint32_t Packets_Send;

//debug
uint8_t KeyCounter;
uint8_t KeyCounterLength = 5;
uint8_t TxKeys[5];
uint8_t RxKeys[5];
uint8_t RxKeysAfterHamming[5];
uint8_t TxRSSIValues[8];
uint8_t TxRSSIMeanValues[8];
uint8_t RxRSSIValues[8];
uint8_t RxRSSIMeanValues[8];
uint32_t TxCounterTable[8];
uint32_t RxCounterTable[8];

/* External interrupts */
uint8_t INT_PACKET_RECEIVED = 0;
uint8_t INT_PACKET_SEND = 0;

/* Tx variables */
cbuf_handle_t Tx_buffer_handle_t;
uint8_t TX_BUFFER_BASE;

/* Rx variables */
cbuf_handle_t Rx_buffer_handle_t;
uint8_t RX_BUFFER_BASE;
uint8_t Rx_Pkt_length;
uint8_t Rx_Pkt_type;
uint8_t Rx_Data_length;
uint8_t Rx_Encryption_byte;
uint8_t Rx_Dummy_byte;
uint8_t Rx_CRC_response;

/* Encryption variables */
uint32_t Tx_RSSI_counter;
uint32_t RSSI_counter;
uint16_t Key_RSSI_Mean;
double Key_RSSI_Mean_Double;
uint32_t Key_32bit_Array[4] = {0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA};
uint32_t Key_32bit = 0xAAAAAAAA;
uint8_t Key_Start = 0xAA;
uint8_t Key_Current;
uint8_t Key_New;
uint8_t Key_bits;
uint8_t Key_bytes = 0;
uint8_t Key_RSSI_Threshold = 3;
uint8_t Encryption_byte;
uint8_t Key_chosen_wait_timer;
double ALPHA = 0.1;
uint32_t KeyCounter32bit;
uint8_t Key_32bit_Array_Index;

uint32_t Old_Key_32bit;
uint8_t Old_Key_32bit_index;

uint32_t CRC_Tx_value;
uint8_t CRC_Tx_bytes[4];

uint32_t CRC_Rx_value;
uint8_t CRC_Rx_bytes[4];

/* Button states */
static volatile POWER_state = 1;
static volatile TALK_state = 1;
static volatile UP_state = 1;
static volatile DOWN_state = 1;

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
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *);

/* Basic functions */
void Startup(void);
void Setup();
void Potmeter_Init(uint8_t);

/* Audio functions */
void Play_Audio(void);
void SendPacket8bit(void);
void ReadPacket(void);

/* Encryption functions */
uint8_t ReadKeyPacket(void);
void WriteKeyPacket(void);
uint8_t ReadRSSI(void);
void Hamming_send(uint8_t);
void Hamming_check(uint8_t);
void CRC_send(void);
void CRC_check(void);

/* EEPROM functions */
void Clear_Flash_Sector4(void);
void Write_Flash(uint32_t, uint32_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Callback external interrupts */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case ADF7242_IRQ1_Pin:
			INT_PACKET_SEND = 1;
			ADF_clear_Tx_flag();
			Packets_Send++;

			break;

		case ADF7242_IRQ2_Pin:
			INT_PACKET_RECEIVED = 1;
			ADF_clear_Rx_flag();
			Packets_Received++;

			break;

		case BUTTON_TALK_Pin:
			if (TALK_state)
			{
				TALK_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}

			break;

		case BUTTON_UP_Pin:
			if (UP_state)
			{
				UP_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}

			break;

		case BUTTON_DOWN_Pin:
			if (DOWN_state)
			{
				DOWN_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}

			break;

		case BUTTON_PWR_Pin:
			if (POWER_state)
			{
				POWER_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}

			break;
	}
}

/* Callback timers */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Play audio samples on DAC
	if (htim->Instance == TIM1)
	{
		Play_Audio();
	}

	// Timer debouncing for external interrupts
	if (htim->Instance == TIM11)
	{
		if (HAL_GPIO_ReadPin(BUTTON_PWR_GPIO_Port, BUTTON_PWR_Pin))
		{
			POWER_state = 1;

			ADF_sleep();

			OLED_shutdown();												// Shutdown OLED display
			HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin, GPIO_PIN_RESET);			// Shutdown LM386
			HAL_GPIO_WritePin(GPIOB, A_MIC_POWER_Pin, GPIO_PIN_RESET);		// Shutdown the analog microphone

			HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);								// Stop the DAC interface
			HAL_TIM_Base_Stop_IT(&htim1);									// Stop timer 1 (frequency = 8 kHz)

			HAL_TIM_OC_Stop(&htim5, TIM_CHANNEL_1);							// Stop timer 5 (frequency = 16 kHz)
			HAL_ADC_Stop_IT(&hadc1);										// Stop ADC interrupt triggered by timer 5

			HAL_Delay(250);

			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
			HAL_PWR_EnterSTANDBYMode();

			HAL_TIM_Base_Stop_IT(&htim11);
		}

		if (HAL_GPIO_ReadPin(BUTTON_TALK_GPIO_Port, BUTTON_TALK_Pin))
		{
			TALK_state = 1;

			if (settingsMode == 'R')
			{
				settingsMode = 'T';
			}
			else if (settingsMode == 'T')
			{
				settingsMode = 'R';
			}
			HAL_TIM_Base_Stop_IT(&htim11);

			Setup();
		}

		if (HAL_GPIO_ReadPin(BUTTON_UP_GPIO_Port, BUTTON_UP_Pin))
		{
			UP_state = 1;

			if (settingsMode == 'R')
			{
				Key_RSSI_Threshold = Key_RSSI_Threshold + 1;
				OLED_print_volume(Key_RSSI_Threshold);
				OLED_update();

//				if (settingsVolume < 70)
//				{
//					settingsVolume = settingsVolume + 10;
//					Potmeter_Init(settingsVolume);
//					OLED_print_volume(settingsVolume);
//					OLED_update();
//				}
//				else
//				{
//					OLED_print_volume(settingsVolume);
//					OLED_update();
//				}
			}
			HAL_TIM_Base_Stop_IT(&htim11);
		}

		if (HAL_GPIO_ReadPin(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin))
		{
			DOWN_state = 1;

			if (settingsMode == 'R')
			{
				Key_RSSI_Threshold = Key_RSSI_Threshold - 1;
				OLED_print_volume(Key_RSSI_Threshold);
				OLED_update();

//				if (settingsVolume > 0)
//				{
//					settingsVolume = settingsVolume - 10;
//					Potmeter_Init(settingsVolume);
//					OLED_print_volume(settingsVolume);
//					OLED_update();
//				}
//				else
//				{
//					OLED_print_volume(settingsVolume);
//					OLED_update();
//				}
			}
			HAL_TIM_Base_Stop_IT(&htim11);
		}
	}

	// Update OLED display
	if (htim->Instance == TIM9)
	{
		/* Update every 3 seconds */
		OLED_print_variable("RSSI mean: ", Key_RSSI_Mean, 0, 26);
		OLED_print_variable("Key bits:", Key_bits, 0, 36);
		OLED_print_hexadecimal("Key:", Key_New, 80, 36);

		OLED_print_date_and_time();

		if (settingsMode == 'T')
		{
			OLED_print_variable("Encryption: ", settingsEncryption, 0, 16);
			OLED_print_stoptalk();
		}
		else if (settingsMode == 'R')
		{
			OLED_print_variable("Encrypted? ", (Rx_Pkt_type>>4 & 0x01), 0, 16);
			OLED_print_talk();
		}

		OLED_update();
	}
}

/* Callback ADC */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
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
  MX_RTC_Init();
  MX_CRC_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  Startup();

  Clear_Flash_Sector4();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (settingsMode == 'T')
	  {
		  // Send audio packet
		  if (circular_buf_size(Tx_buffer_handle_t) > settingsDataLength)
		  {
			  if (settingsResolution == 8)
			  {
				  SendPacket8bit();
			  }
		  }
	  }

	  if (INT_PACKET_RECEIVED){
		  INT_PACKET_RECEIVED = 0;

		  if (settingsMode == 'T')
		  {
			  // Update mean RSSI
			  uint8_t RSSI = ReadKeyPacket();

//			  // Debug
//			  uint32_t address0 = 0x08012000 + (16*(Tx_RSSI_counter-1));
//			  Write_Flash(address0, (uint32_t) RSSI);


			  if (Rx_CRC_response == 0x01)			// CRC correct
			  {
				  // Debug
				  uint32_t address1 = 0x08011000 + (4*(KeyCounter32bit-1));
				  Write_Flash(address1, Key_32bit);
			  }
			  else if (Rx_CRC_response == 0x10)	// CRC not correct
			  {
				  // Update key
				  Key_32bit_Array[Old_Key_32bit_index] = Old_Key_32bit;

				  // Try again
				  KeyCounter32bit--;
			  }

			  if (!Tx_RSSI_counter)
			  {
				  Key_RSSI_Mean_Double = RSSI;
				  Key_RSSI_Mean = round(Key_RSSI_Mean_Double);
			  }
			  else
			  {
				  Key_RSSI_Mean_Double = (ALPHA*RSSI) + ((1-ALPHA)*Key_RSSI_Mean_Double);
			      Key_RSSI_Mean = round(Key_RSSI_Mean_Double);
			  }

//			  // Debug
//			  uint32_t address2 = 0x08016000 + (16*(Tx_RSSI_counter-1));
//			  Write_Flash(address2, (uint32_t) Key_RSSI_Mean);

			  Tx_RSSI_counter++;

			  if (Key_bytes == 4)
			  {
				  // Index
				  Key_32bit_Array_Index = KeyCounter32bit % 4;

				  // Save old 32bit key
				  Old_Key_32bit_index = Key_32bit_Array_Index;
				  Old_Key_32bit = Key_32bit_Array[Old_Key_32bit_index];

				  // Update key
				  Key_32bit_Array[Key_32bit_Array_Index] = Key_32bit;

				  CRC_send();

				  KeyCounter32bit++;
				  Key_bytes = 0;
			  }
			  else if (Key_bits == 8)	// Hamming check every 8 bits
			  {
				  Hamming_send(Key_New);
				  Key_Current = Key_New;

				  Key_32bit = (Key_32bit<<8) | Key_New;

				  Key_New = 0;
				  Key_bits = 0;
				  Key_bytes++;
			  }
			  else	// Choose key bit
			  {
				  if (Tx_RSSI_counter > 100)						// Wait for 100 RSSI values
				  {
					  if (Key_chosen_wait_timer == 0)
					  {
						  if (RSSI < (Key_RSSI_Mean - Key_RSSI_Threshold))
						  {
							  if (Key_bits<8)
							  {
								  Key_New = (Key_New<<1) | 0;
								  Key_bits++;
								  Encryption_byte = 1;

								  Key_chosen_wait_timer = 8;		// Wait to choose new key bit

							  }
						  }
						  else if (RSSI > (Key_RSSI_Mean + Key_RSSI_Threshold))
						  {
							  if (Key_bits<8)
							  {
								  Key_New = (Key_New<<1) | 1;
								  Key_bits++;
								  Encryption_byte = 1;

								  Key_chosen_wait_timer = 8;		// Wait to choose new key bit
							  }
						  }
					  }
					  else
					  {
						  Key_chosen_wait_timer--;
					  }
				  }
			  }
		  }
	  }

	  if (INT_PACKET_SEND){
		  INT_PACKET_SEND = 0;

		  if (settingsMode == 'R')
		  {
			  WriteKeyPacket();
			  ADF_set_Rx_mode();

			  ReadPacket();
		  }
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  sTime.Hours = 12;
  sTime.Minutes = 59;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 24;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
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
  htim9.Init.Prescaler = 48000;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2000;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 48000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 500;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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

  /*Configure GPIO pin : BUTTON_TALK_Pin */
  GPIO_InitStruct.Pin = BUTTON_TALK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_TALK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_UP_Pin BUTTON_OK_Pin BUTTON_DOWN_Pin ADF7242_IRQ1_Pin 
                           ADF7242_IRQ2_Pin */
  GPIO_InitStruct.Pin = BUTTON_UP_Pin|BUTTON_OK_Pin|BUTTON_DOWN_Pin|ADF7242_IRQ1_Pin 
                          |ADF7242_IRQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_PWR_Pin */
  GPIO_InitStruct.Pin = BUTTON_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF7242_CS_Pin SWITCH_E_Pin POT_CS_Pin A_MIC_POWER_Pin */
  GPIO_InitStruct.Pin = ADF7242_CS_Pin|SWITCH_E_Pin|POT_CS_Pin|A_MIC_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void Startup(void)
{
	/* Clear PWR wake up Flag */
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

	/* Welcome screen */
	OLED_init();
	OLED_print_text("Jelle's", 39, 16);
	OLED_print_text("Walkie Talkie", 18, 26);
	OLED_update();
	HAL_Delay(1000);

	/* OLED settings */
	OLED_clear_screen();

	/* RTC time */
	OLED_print_date_and_time();

	/* potentiometer settings */
	Potmeter_Init(settingsVolume);
	OLED_print_volume(Key_RSSI_Threshold);
	OLED_update();

	ADF_Init(settingsFrequency);

	/* Setup */
	Setup();
}

void Setup()
{
	if (settingsEncryption)
	{
		Key_Current = Key_Start;

		if (settingsMode == 'R')
		{
			WriteKeyPacket();
		}
	}

	switch(settingsMode)
	{
		case 'T':
			OLED_print_text("Tx", 0, 0);
			OLED_print_stoptalk();
			OLED_update();

			/* Reverse HAL audio settings for Rx mode */
			HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);										// Stop the DAC interface
			HAL_TIM_Base_Stop_IT(&htim1);											// Stop timer 1 (frequency = 8 kHz)

			/* ADF settings */
			ADF_set_turnaround_Tx_Rx();

			/* Power settings */
			HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin, GPIO_PIN_RESET);					// Shutdown LM386 to prevent power consumption
			HAL_GPIO_WritePin(GPIOB, A_MIC_POWER_Pin, GPIO_PIN_SET);				// Enable the analog microphone

			/* Buffer Settings for Tx mode */
			uint16_t Tx_buffer_size = 400;
			uint16_t *Tx_buffer = malloc(Tx_buffer_size * sizeof(uint16_t));		// Tx_buffer with size of 400 bytes
			Tx_buffer_handle_t = circular_buf_init(Tx_buffer, Tx_buffer_size);		// Tx buffer handle type

			/* HAL audio settings for Tx mode */
			HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_1);								// Start timer 5 (frequency = 16 kHz)
			HAL_ADC_Start_IT(&hadc1);												// Start ADC interrupt triggered by timer 5

			break;

		case 'R':
			OLED_print_text("Rx", 0, 0);
			OLED_print_talk();
			OLED_update();

			/* Reverse HAL audio settings for Tx mode */
			HAL_TIM_OC_Stop(&htim5, TIM_CHANNEL_1);									// Stop timer 5 (frequency = 16 kHz)
			HAL_ADC_Stop_IT(&hadc1);												// Stop ADC interrupt triggered by timer 5

			/* ADF settings */
			ADF_set_turnaround_Rx_Tx();

			/* Power settings */
			HAL_GPIO_WritePin(GPIOB, A_MIC_POWER_Pin, GPIO_PIN_RESET);				// Shutdown the analog microphone to prevent power consumption
			HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin, GPIO_PIN_SET);					// Enable the LM386

			/* Buffer Settings for Rx mode */
			uint16_t Rx_buffer_size = 400;
			uint16_t *Rx_buffer = malloc(Rx_buffer_size * sizeof(uint16_t));		// Rx_buffer with size of 400 bytes = 5 packets
			Rx_buffer_handle_t = circular_buf_init(Rx_buffer, Rx_buffer_size);		// Rx buffer handle type

			/* HAL audio settings for Rx mode */
			HAL_DAC_Start(&hdac, DAC_CHANNEL_1);									// Start the DAC interface
			HAL_TIM_Base_Start_IT(&htim1);											// Start timer 1 (frequency = 8 kHz)

			/* Rx mode */
			ADF_set_Rx_mode();														// Rx mode

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
		if (settingsResolution == 8)
		{
			returnValue = circular_buf_get(Rx_buffer_handle_t, &sample);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sample);
		}
		else if (settingsResolution == 12)
		{
			returnValue = circular_buf_get(Rx_buffer_handle_t, &sample);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sample);
		}
	}
}

void SendPacket8bit(void)
{
	uint8_t PacketTotalLength = settingsDataLength + 5;

	if (Encryption_byte == 0x10)
	{
		PacketTotalLength = PacketTotalLength + 4;		// 4 extra bytes with CRC check
	}

	uint8_t header[] = {0x10, PacketTotalLength, 0x18, settingsDataLength};

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, header, 4);

	if (settingsEncryption)
	{
		// Write 40 audio samples to transceiver
		for (int i=0; i<settingsDataLength; i++)
		{
			uint8_t encryptedSample[1];
			returnValue = circular_buf_get(Tx_buffer_handle_t, &encryptedSample);
			encryptedSample[0] = encryptedSample[0] ^ Key_Current;

			HAL_SPI_Transmit_IT(&hspi2, encryptedSample, 1);
		}
	}
	else
	{
		// Write 40 audio samples to transceiver
		for (int i=0; i<settingsDataLength; i++)
		{
			uint8_t sample[1];
			returnValue = circular_buf_get(Tx_buffer_handle_t, &sample);

			HAL_SPI_Transmit_IT(&hspi2, sample, 1);
		}
	}

	HAL_SPI_Transmit_IT(&hspi2, &Encryption_byte, 1);

	if (Encryption_byte == 0x10)
	{
		HAL_SPI_Transmit_IT(&hspi2, CRC_Tx_bytes, 4);
	}

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	ADF_set_Tx_mode();

//	if (Encryption_byte >= 240)
//	{
//		TxKeys[KeyCounter] = Key_Current;
//		KeyCounter++;
//
//		if (KeyCounter == KeyCounterLength)
//		{
//			for (int i=0; i<KeyCounterLength; i++)
//			{
//				OLED_print_hexadecimal("", TxKeys[i], 0 + (20*i), 20);
//
////				for (int i=0; i<8; i++)
////				{
////					OLED_print_hexadecimal("", TxRSSIValues[i], 0 + (16*i), 30);
////					OLED_print_hexadecimal("", TxRSSIMeanValues[i], 0 + (16*i), 40);
////				}
//			}
//			OLED_update();
//		}
//
////		OLED_print_variable("RSSI mean: ", Key_RSSI_Mean, 0, 26);
////		OLED_print_hexadecimal("Key:", Key_Current, 0, 36);
////		OLED_update();
//	}

	Encryption_byte = 0;
}

void ReadPacket(void)
{
	Rx_CRC_response = 0x00;

	uint8_t bytes[] = {0x30, 0xff};
	uint8_t Rx_RSSI;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit_IT(&hspi2, bytes, 2);

	HAL_SPI_Receive_IT(&hspi2, &Rx_Pkt_length, 1);
	HAL_SPI_Receive_IT(&hspi2, &Rx_Pkt_type, 1);
	HAL_SPI_Receive_IT(&hspi2, &Rx_Data_length, 1);

	uint8_t Rx_data[Rx_Data_length];
	HAL_SPI_Receive_IT(&hspi2, &Rx_data, Rx_Data_length);

	HAL_SPI_Receive_IT(&hspi2, &Rx_Encryption_byte, 1);

	if (Rx_Encryption_byte == 0x10)
	{
		HAL_SPI_Receive_IT(&hspi2, &CRC_Rx_bytes, 4);
	}

	HAL_SPI_Receive_IT(&hspi2, &Rx_RSSI, 1);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (ADF_SPI_READY() == 0);

//	// Debug
//	uint32_t address0 = 0x08012000 + (4*(RSSI_counter-1));
//	Write_Flash(address0, (uint32_t) Rx_RSSI);

	// Update RSSI mean
	if (!RSSI_counter)
	{
		Key_RSSI_Mean_Double = Rx_RSSI;
		Key_RSSI_Mean = round(Key_RSSI_Mean_Double);
	}
	else
	{
		Key_RSSI_Mean_Double = (ALPHA*Rx_RSSI) + ((1-ALPHA)*Key_RSSI_Mean_Double);
		Key_RSSI_Mean = round(Key_RSSI_Mean_Double);
	}

//	// Debug
//	uint32_t address1 = 0x08016000 + (4*(RSSI_counter-1));
//	Write_Flash(address1, (uint32_t) Key_RSSI_Mean);

	RSSI_counter++;

	if ((Rx_Pkt_type == 0x18) || (Rx_Pkt_type == 0x1c))			// Audio packet
	{
		uint8_t decryptedSample;

		// Decryption + Write packet to buffer
		for (int i=0; i<Rx_Data_length; i++)
		{
			decryptedSample = Rx_data[i] ^ Key_Current;
			circular_buf_put_overwrite(Rx_buffer_handle_t, decryptedSample);
		}
	}
	else
	{
		// Write packet to buffer
		for (int i=0; i<Rx_Data_length; i++)
		{
			circular_buf_put_overwrite(Rx_buffer_handle_t, Rx_data[i]);
		}
	}

	// Key bit chosen by transmitter
	if (Rx_Encryption_byte == 0x01)
	{
		if (Rx_RSSI <= Key_RSSI_Mean)
		{
			if (Key_bits < 8)
			{
				//debug
				RxRSSIValues[Key_bits] = Rx_RSSI;
				RxRSSIMeanValues[Key_bits] = Key_RSSI_Mean;
				RxCounterTable[Key_bits] = RSSI_counter;

				Key_New = (Key_New<<1) | 0;
				Key_bits++;
			}
		}
		else if (Rx_RSSI > Key_RSSI_Mean)
		{
			if (Key_bits < 8)
			{
				//debug
				RxRSSIValues[Key_bits] = Rx_RSSI;
				RxRSSIMeanValues[Key_bits] = Key_RSSI_Mean;
				RxCounterTable[Key_bits] = RSSI_counter;

				Key_New = (Key_New<<1) | 1;
				Key_bits++;
			}
		}
	}
	// CRC
	else if (Rx_Encryption_byte == 0x10)
	{
		for (int i=0; i < 4; i++)
		{
			CRC_Rx_value = (CRC_Rx_value << 8) | CRC_Rx_bytes[i];
		}

		CRC_check();
	}
	// Hamming packet send by transmitter
	else if (Rx_Encryption_byte >= 240)
	{
		RxKeys[KeyCounter] = Key_New;

		Hamming_check(Rx_Encryption_byte & 0x0F);
		Rx_Encryption_byte = 0;

		Key_Current = Key_New;

		Key_32bit = (Key_32bit<<8) | Key_New;
		Key_bytes++;


		RxKeysAfterHamming[KeyCounter] = Key_Current;
		KeyCounter++;

		if (Key_bytes == 4)
		{
			// Index
			Key_32bit_Array_Index = KeyCounter32bit % 4;

			// Save old 32bit key
			Old_Key_32bit_index = Key_32bit_Array_Index;
			Old_Key_32bit = Key_32bit_Array[Old_Key_32bit_index];

			// Update key
			Key_32bit_Array[Key_32bit_Array_Index] = Key_32bit;

			KeyCounter32bit++;
			Key_bytes = 0;
		}

//		if (KeyCounter == KeyCounterLength)
//		{
//			for (int i=0; i<KeyCounterLength; i++)
//			{
//				OLED_print_hexadecimal("", RxKeys[i], 0 + (20*i), 20);
//				OLED_print_hexadecimal("", RxKeysAfterHamming[i], 0 + (20*i), 30);
//
////				for (int i=0; i<8; i++)
////				{
////					OLED_print_hexadecimal("", RxRSSIValues[i], 0 + (16*i), 30);
////					OLED_print_hexadecimal("", RxRSSIMeanValues[i], 0 + (16*i), 40);
////				}
//			}
//			OLED_update();
//		}

//		OLED_print_variable("RSSI mean: ", Key_RSSI_Mean, 0, 26);
//		OLED_print_hexadecimal("Key:", Key_Current, 0, 36);
//		OLED_update();

		Key_New = 0;
		Key_bits = 0;
	}
}

uint8_t ReadKeyPacket(void)
{
	uint8_t bytes[] = {0x30, 0xff};

	uint8_t Rx_RSSI;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit_IT(&hspi2, bytes, 2);

	HAL_SPI_Receive_IT(&hspi2, &Rx_Pkt_length, 1);
	HAL_SPI_Receive_IT(&hspi2, &Rx_Pkt_type, 1);
	HAL_SPI_Receive_IT(&hspi2, &Rx_CRC_response, 1);
	HAL_SPI_Receive_IT(&hspi2, &Rx_Dummy_byte, 1);

	HAL_SPI_Receive_IT(&hspi2, &Rx_RSSI, 1);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (ADF_SPI_READY() == 0);

	return Rx_RSSI;
}

void WriteKeyPacket(void)
{
	while (ADF_SPI_READY() == 0);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	uint8_t bytes[] = {0x10, 0x05, 0x10, Rx_CRC_response};								// TYPE = 0x10 => Key packet
	HAL_SPI_Transmit_IT(&hspi2, bytes, 4);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (ADF_SPI_READY() == 0);
}

uint8_t ReadRSSI(void)
{
	uint8_t RSSI = ADF_SPI_MEM_RD(0x30C);
	return RSSI;
}

void Hamming_send(uint8_t key)
{
	uint8_t bit_0 = key & 0x01;
	uint8_t bit_1 = (key>>1) & 0x01;
	uint8_t bit_2 = (key>>2) & 0x01;
	uint8_t bit_3 = (key>>3) & 0x01;
	uint8_t bit_4 = (key>>4) & 0x01;
	uint8_t bit_5 = (key>>5) & 0x01;
	uint8_t bit_6 = (key>>6) & 0x01;
	uint8_t bit_7 = (key>>7) & 0x01;

	uint8_t parity_0 = bit_0 ^ bit_1 ^ bit_3 ^ bit_4 ^ bit_6;
	uint8_t parity_1 = bit_0 ^ bit_2 ^ bit_3 ^ bit_5 ^ bit_6;
	uint8_t parity_2 = bit_1 ^ bit_2 ^ bit_3 ^ bit_7;
	uint8_t parity_3 = bit_4 ^ bit_5 ^ bit_6 ^ bit_7;

	uint8_t code = (parity_3 * 8) + (parity_2 * 4) + (parity_1 * 2) + parity_0;

	Encryption_byte = 0xF0 | code;
}

void Hamming_check(uint8_t Tx_code)
{
	uint8_t bit_0 = Key_New & 0x01;
	uint8_t bit_1 = (Key_New>>1) & 0x01;
	uint8_t bit_2 = (Key_New>>2) & 0x01;
	uint8_t bit_3 = (Key_New>>3) & 0x01;
	uint8_t bit_4 = (Key_New>>4) & 0x01;
	uint8_t bit_5 = (Key_New>>5) & 0x01;
	uint8_t bit_6 = (Key_New>>6) & 0x01;
	uint8_t bit_7 = (Key_New>>7) & 0x01;

	uint8_t parity_0 = bit_0 ^ bit_1 ^ bit_3 ^ bit_4 ^ bit_6;
	uint8_t parity_1 = bit_0 ^ bit_2 ^ bit_3 ^ bit_5 ^ bit_6;
	uint8_t parity_2 = bit_1 ^ bit_2 ^ bit_3 ^ bit_7;
	uint8_t parity_3 = bit_4 ^ bit_5 ^ bit_6 ^ bit_7;

	uint8_t Rx_code = (parity_3 * 8) + (parity_2 * 4) + (parity_1 * 2) + parity_0;

	if (Rx_code != Tx_code)
	{
		uint8_t control_0 = 0;
		uint8_t control_1 = 0;
		uint8_t control_2 = 0;
		uint8_t control_3 = 0;

		if (parity_0 != (Tx_code & 0x01))
			control_0 = 1;
		if (parity_1 != ((Tx_code>>1) & 0x01))
			control_1 = 1;
		if (parity_2 != ((Tx_code>>2) & 0x01))
			control_2 = 1;
		if (parity_3 != ((Tx_code>>3) & 0x01))
			control_3 = 1;

		uint8_t control = (control_3 * 8) + (control_2 * 4) + (control_1 * 2) + control_0;

		// Key correction
		switch(control)
		{
			case 3:
				Key_New ^= 1UL << 0;
				break;
			case 5:
				Key_New ^= 1UL << 1;
				break;
			case 6:
				Key_New ^= 1UL << 2;
				break;
			case 7:
				Key_New ^= 1UL << 3;
				break;
			case 9:
				Key_New ^= 1UL << 4;
				break;
			case 10:
				Key_New ^= 1UL << 5;
				break;
			case 11:
				Key_New ^= 1UL << 6;
				break;
			case 12:
				Key_New ^= 1UL << 7;
				break;
		}
	}
}

void CRC_send(void)
{
	CRC_Tx_value = HAL_CRC_Calculate(&hcrc, Key_32bit_Array, 4);

	for (int i=0; i<4; i++)
	{
		CRC_Tx_bytes[3-i] = (CRC_Tx_value >> (i*8));		// LSB to MSB (0 to 3)
	}

	Encryption_byte = 0x10;
}
void CRC_check(void)
{
	uint32_t CRC_check = HAL_CRC_Calculate(&hcrc, Key_32bit_Array, 4);
	if (CRC_Rx_value == CRC_check)
	{
		Rx_CRC_response = 0x01;			// CRC correct

		// Debug
		uint32_t address = 0x08011000 + (4*(KeyCounter32bit-1));
		Write_Flash(address, Key_32bit);
	}
	else
	{
		Rx_CRC_response = 0x10;			// CRC not correct

		// Update key
		Key_32bit_Array[Old_Key_32bit_index] = Old_Key_32bit;

		// Try again
		KeyCounter32bit--;
	}
}

/********** EEPROM functions **********/
void Clear_Flash_Sector4(void)
{
     HAL_FLASH_Unlock();
     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
     FLASH_Erase_Sector(FLASH_SECTOR_4, VOLTAGE_RANGE_3);
     HAL_FLASH_Lock();
}

void Write_Flash(uint32_t address, uint32_t data)
{
     HAL_FLASH_Unlock();
     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
//     FLASH_Erase_Sector(FLASH_SECTOR_4, VOLTAGE_RANGE_3);
     HAL_FLASH_Program(TYPEPROGRAM_WORD, address, data);
     HAL_FLASH_Lock();
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
