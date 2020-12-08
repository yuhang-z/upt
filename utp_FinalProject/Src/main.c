/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stdio.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_tsensor.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "stm32l475e_iot01_qspi.h"
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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c1;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t acceleroReading[3] = {0,0,0};
int16_t Sample_X;
int16_t Sample_Y;
int16_t Sample_Z;

uint8_t soundBuffer[22050];
uint8_t soundBufferDac[44100];
uint8_t this_note = 0;
uint8_t beepTwice = 0;

int8_t calibrated = 0;
int16_t sstatex;
int16_t sstatey;
int16_t sstatez;

int16_t accelerationx[2] = {0,0};
int16_t accelerationy[2] = {0,0};
int16_t accelerationz[2] = {0,0};

int8_t is_checking = 0;

int16_t velocityx[2] = {0,0};
int pushupAim = 20; //set the aim for pushups
int pushupCur = 0;  //keep track of the pushup done already

float humidityReading = 0;
float temperatureReading = 0;
float gyroscopeReading[3] = {0,0,0};

char humidityStr[20];
char temperatureStr[20];
char accelerometerStr[30];
char beepStr[30];
char gyroscopeStr[30];
char buffer[100] = {0};
int acceleration[3][100] = {0};
char AxBuff[60];
char AyBuff[20];
char AzBuff[20];
//int counter = 0;

//define constant variables:
uint8_t acc_y_ref = 10;
uint8_t num_pushups = 0;
uint8_t integral_y = 0;

//for calories calculation
double weight = 50;
double armLength = 0.25;


//for DAC & flash
#define pi 3.14159265
uint8_t play[22050] = {0};
uint8_t numToBeep = 0;

//for push-up counter
int integral_velo = 0;
int arr[100] = { };
int calibratedz = 0;
int16_t counter = 0;
int16_t target = 6;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_QUADSPI_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	//I2C Sensor Initialization
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_GYRO_Init();
	BSP_HSENSOR_Init();

	// Start the timer

	BSP_QSPI_Init();

	//generate wave
	uint32_t note_c6[42];
	uint32_t note_e6[33];
	uint32_t note_g6[28];
	uint32_t note_c7[21];
	uint32_t note_e7[17];
	uint32_t note_g7[14];

	// Tone C6
	for (uint8_t i=0; i<42;i++) {
		note_c6[i] = ((arm_sin_f32(i*2*PI/42) + 1)*((0xFF + 1)/2)) / 2;
	}

	// Tone E6
	for (uint8_t i=0; i<33;i++) {
		note_e6[i] = ((arm_sin_f32(i*2*PI/33) + 1)*((0xFF + 1)/2)) / 2;
	}

	// Tone G6
	for (uint8_t i=0; i<28;i++) {
		note_g6[i] = ((arm_sin_f32(i*2*PI/28) + 1)*((0xFF + 1)/2)) / 2;
	}

	// Tone C7
	for (uint8_t i=0; i<21;i++) {
		note_c7[i] = ((arm_sin_f32(i*2*PI/21) + 1)*((0xFF + 1)/2)) / 2;
	}

	// Tone E7
	for (uint8_t i=0; i<17;i++) {
		note_e7[i] = ((arm_sin_f32(i*2*PI/17) + 1)*((0xFF + 1)/2)) / 2;
	}

	// Tone G7
	for (uint8_t i=0; i<14;i++) {
		note_g7[i] = ((arm_sin_f32(i*2*PI/14) + 1)*((0xFF + 1)/2)) / 2;
	}


	/*
	 * Prepare the Flash - note: probably need quite a few blocks
	 */
	BSP_QSPI_Erase_Block(0);
	BSP_QSPI_Erase_Block(64000);
	BSP_QSPI_Erase_Block(128000);



	/*
	 * Write the notes into flash
	 */

	// C6
	for(int i=0; i<526; i++){
		for(int j=0; j<42; j++){
			soundBuffer[42*i+j] = note_c6[j] % 256;
		}
	}
	BSP_QSPI_Write((uint8_t*)&soundBuffer[0], 11025*0, 22050);


	// E6
	for(int i=0; i<668; i++){
		for(int j=0; j<33; j++){
			soundBuffer[i*33+j] = note_e6[j] % 256;
		}
	}
	BSP_QSPI_Write((uint8_t*)&soundBuffer[0], 22050*1, 22050);

	// G6
	for(int i=0; i<786; i++){
		for(int j=0; j<28; j++){
			soundBuffer[i*28+j] = note_g6[j] % 256;
		}
	}
	BSP_QSPI_Write((uint8_t*)&soundBuffer[0], 22050*2, 22050);



	// C7
	for(int i=0; i<1050; i++){
		for(int j=0; j<21; j++){
			soundBuffer[i*21+j] = note_c7[j] % 256;
		}
	}
	BSP_QSPI_Write((uint8_t*)&soundBuffer[0], 22050*3, 22050);


	// E7
	for(int i=0; i<1296; i++){
		for(int j=0; j<17; j++){
			soundBuffer[i*17+j] = note_e7[j] % 256;
		}
	}
	BSP_QSPI_Write((uint8_t*)&soundBuffer[0], 22050*4, 22050);


	// G7
	for(int i=0; i<1574; i++){
		for(int j=0; j<14; j++){
			soundBuffer[i*14+j] = note_g7[j] % 256;
		}
	}
	BSP_QSPI_Write((uint8_t*)&soundBuffer[0], 22050*5, 22050);



	// Prepare to play the first two notes
	BSP_QSPI_Read((uint8_t *)&soundBufferDac[0], 0, 22050);
	BSP_QSPI_Read((uint8_t *)&soundBufferDac[22050], 22050, 22050);

	sprintf(accelerometerStr, "Ready! \n");
	HAL_UART_Transmit(&huart1, (uint8_t*)accelerometerStr, sizeof(accelerometerStr), 100);

	// Beep twice to indicate it's ready
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)soundBufferDac, 11025, DAC_ALIGN_8B_R);

	// Start the measurement
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7256;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2) {
	readAccelerometer();
}

void beepOnce(){
	BSP_QSPI_Read((uint8_t *)&soundBufferDac[0], 0, 22050);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)soundBufferDac, 5512, DAC_ALIGN_8B_R);
}

void beepMany() {
	numToBeep = counter / target;
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)soundBufferDac, 11025, DAC_ALIGN_8B_R);
	sprintf(beepStr, "Congratulations!\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)beepStr, sizeof(beepStr), 100);
}

void getXYZ() {
	int16_t count2 = 0;
	Sample_X = 0;
	do{
		BSP_ACCELERO_AccGetXYZ(acceleroReading);
		Sample_X = Sample_X + (int)acceleroReading[0];
		count2++; // average represents the acceleration of
		// an instant.
	} while (count2!=0x40); // 64 sums of the acceleration sample

	Sample_X = Sample_X >> 6; // division by 64
	Sample_Y = Sample_X >> 6;

	//push up counter
	Sample_Z = (int)acceleroReading[2];
	if(abs(Sample_Z-1032)>100)/*otherwise in noise range*/{
		calibratedz = Sample_Z-1032;
		integral_velo += calibratedz/100;
		if(abs(integral_velo/100 - arr[counter])> 4){
			counter++;
			sprintf(accelerometerStr, "%d th Pushup detected.\n", counter);
			HAL_UART_Transmit(&huart1, (uint8_t*)accelerometerStr, sizeof(accelerometerStr), 100);
			arr[counter] = integral_velo/100;
			if (counter == target) {
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);	// toggle LED
			}
			if((counter != 0) && (counter%target) == 0){
				beepMany();
				int8_t times = counter / target;
				sprintf(accelerometerStr, "Achieved %d x the GOAL.\n", times);
				HAL_UART_Transmit(&huart1, (uint8_t*)accelerometerStr, sizeof(accelerometerStr), 100);
				sprintf(accelerometerStr, "Burned %d calories.\n", (int)(weight*armLength*10*counter / 4.184));
				HAL_UART_Transmit(&huart1, (uint8_t*)accelerometerStr, sizeof(accelerometerStr), 100);
			}else{
				beepOnce();
				if ((target-counter) > 0) {
					sprintf(beepStr, "%d more until your goal.\n",(int)(target-counter));
					HAL_UART_Transmit(&huart1, (uint8_t*)beepStr, sizeof(beepStr), 100);
				}
			}

		}
	}
}

void calibrate() {
	int16_t count1 = 0;

	HAL_Delay(3000);

	do {
		getXYZ();
		sstatex = sstatex + Sample_X; // Accumulate Samples
		sstatey = sstatey + Sample_Y;
		sstatez = sstatez + Sample_Z;
		count1++;
	} while (count1 != 0x0400); // 1024 times

	sstatex = sstatex>>10; // division between 1024
	sstatey = sstatey>>10;
	sstatez = sstatez>>10;

	calibrated = 1;

	sprintf(accelerometerStr, "Calibration completed. \n");
	HAL_UART_Transmit(&huart1, (uint8_t*)accelerometerStr, sizeof(accelerometerStr), 100);
}

void readAccelerometer() {

	// get one
	getXYZ();
	accelerationx[0] = Sample_X - sstatex;
	//velocityx[0] = velocityx[1] + accelerationx[1] + ((accelerationx[0] - accelerationx[1])>>1) ;

	// get one
	getXYZ();
	accelerationx[1] = Sample_X - sstatex;
	//velocityx[1] = velocityx[0] + accelerationx[0] + ((accelerationx[1] - accelerationx[0])>>1) ;

	//	if (!is_checking && isMovementDetected()) {
	//		is_checking = 1;
	//		num_pushups++;
	//		sprintf(accelerometerStr, "Pushup detected. %d\n", num_pushups);
	//		HAL_UART_Transmit(&huart1, (uint8_t*)accelerometerStr, sizeof(accelerometerStr), 100);
	//		pushupCur++;
	//		if(pushupCur%pushupAim !=0 ){
	//			beepOnce();
	//		}else{
	//			beepMany();
	//		}
	//	}
	//	is_checking = 0;
}

int isMovementDetected() {
	int16_t accelerationx_f = (accelerationx[0] + accelerationx[1]) / 2;
	if (abs(accelerationx_f) > 100) {
		sprintf(accelerometerStr, "Acceleration %d\n", accelerationx_f);
		HAL_UART_Transmit(&huart1, (uint8_t*)accelerometerStr, sizeof(accelerometerStr), 100);
		return 1;
	}

	return 0;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef * hdac){
	if (numToBeep >= 1) {
		if(this_note == 0) {
			BSP_QSPI_Read((uint8_t *)&soundBufferDac[0], 22050*2, 22050);
			BSP_QSPI_Read((uint8_t *)&soundBufferDac[22050], 22050*3, 22050);
		} else if (this_note == 1) {
			BSP_QSPI_Read((uint8_t *)&soundBufferDac[0], 22050*4, 22050);
			BSP_QSPI_Read((uint8_t *)&soundBufferDac[22050], 22050*5, 22050);
		} else if (this_note == 2) {
			BSP_QSPI_Read((uint8_t *)&soundBufferDac[0], 22050*0, 22050);
			BSP_QSPI_Read((uint8_t *)&soundBufferDac[22050], 22050*1, 22050);
		}

		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)soundBufferDac, 11025, DAC_ALIGN_8B_R);

		if(this_note < 2){
			this_note++;
		} else {
			this_note = 0;
		}

		numToBeep--;
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
