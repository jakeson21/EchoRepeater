/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

// Flash uses SPI3 - PC10=SCK, PC11=MISO, PC12=MOSI, PD2=CS
// ADC uses PA0 with DMA, toggles PB0 Green LED
// DAC uses PA4 with DMA, toggles PB7 Blue LED
// Sample Timer triggers PD12
// Process loop toggles RED LED on PB14
// EXT_SRC_Pin uses PC0



/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "string.h"
#include "stdbool.h"
//#include "IIR_LowPass.h"
//#include "IIR_HighPass.h"
//#include "IIR_Peak.h"
#include "Delay.h"
//#include "TappedDelay.h"
#include "FbComb.h"
//#include "AllpassPhaser.h"
#include "arm_IIR_AllpassPhaser.h"
#include "lp1_coeffs.h"
#include "sst26_flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 64
#define BLOCK_SIZE ADC_BUF_LEN/2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

//uint16_t dmaBuffer[ADC_BUF_LEN];
//uint16_t dacBuffer[ADC_BUF_LEN];
q15_t dmaBuffer[ADC_BUF_LEN];
q15_t dacBuffer[ADC_BUF_LEN];
float32_t dsp_buffer[ADC_BUF_LEN];

q15_t tmpdsp_buffer[BLOCK_SIZE];
q15_t tmpdsp_buffer2[BLOCK_SIZE];
float32_t ftmpdsp_buffer[BLOCK_SIZE];
float32_t out_buffer[BLOCK_SIZE];


float32_t fdata_log;
q15_t q15data_log;
//float delay_buffer[ADC_BUF_LEN];

uint32_t count = 0;

uint16_t ready_1 = false;
uint16_t ready_2 = false;

//IIR_LPF lowpass_filt1;
//IIR_LPF lowpass_filt2;
//IIR_HPF highpass_filt1;
//IIR_HPF highpass_filt2;
//IIR_Peak peak_filt1;
//IIR_Peak peak_filt2;
fDelay_TypeDef delay1;
//fTappedDelay_TypeDef tapped_delay1;
//fFbComb_TypeDef fb_comb1;
//AllpassPhaser_t phaser;

ArmAllpassPhaser_t arm_phaser;
float32_t dsp_buffer_dry[BLOCK_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t SPI_xfer(uint32_t ui32Base, uint8_t* inTxData, uint8_t* inRxData, uint32_t inLength, int8_t inFssHold)
{
    // Assert EN low
    if ((SPI_HOLD_ACTIVE == inFssHold) || (SPI_HOLD_CLR == inFssHold))
    {
    	HAL_GPIO_WritePin(MY_SPI3_CS_GPIO_Port, MY_SPI3_CS_Pin, GPIO_PIN_RESET);
    }

    if (inTxData && !inRxData) // only TX
    {
    	HAL_SPI_Transmit(&hspi3, inTxData, inLength, HAL_MAX_DELAY);
    }
    else if (!inTxData && inRxData) // only RX
    {
    	HAL_SPI_Receive(&hspi3, inRxData, inLength, HAL_MAX_DELAY);
    }
    else // Both Tx and Rx
    {
		HAL_SPI_TransmitReceive(&hspi3, inTxData, inRxData, inLength, HAL_MAX_DELAY);
    }

    // De-assert EN on last byte
    if ((inFssHold == SPI_HOLD_CLR))
    {
        HAL_GPIO_WritePin(MY_SPI3_CS_GPIO_Port, MY_SPI3_CS_Pin, GPIO_PIN_SET);
    }

    return 0;
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

  // NOTE: Ensure MX_DMA_Init() is called after MX_GPIO_Init()

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

/***
 * Make sure MX_DMA_Init(); is 2nd in the list above
 *
 */

    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXT_SRC_GPIO_Port, EXT_SRC_Pin, GPIO_PIN_RESET);

    // start base timer, start compare function
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1); // optional - debug line on timer output compare pin
    //  HAL_TIM_Base_Start(&htim6);

    // initiate ADC transfer to DMA, clocked by compare register
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) dmaBuffer, ADC_BUF_LEN);
    // configure 12-bit right-align mode for DAC
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) dacBuffer, ADC_BUF_LEN, DAC_ALIGN_12B_R);


//    IIR_LPF_Init(&lowpass_filt1, 0.2);
//    IIR_LPF_Init(&lowpass_filt2, 0.2);
//
//    IIR_HPF_Init(&highpass_filt1, 0.5);
//    IIR_HPF_Init(&highpass_filt2, 0.5);
//
//    IIR_Peak_Init(&peak_filt1, 0.125, 0.05);
//    IIR_Peak_Init(&peak_filt2, 0.125, 0.05);
//
//    FbComb_Init(&fb_comb1, (uint32_t)(50000*0.025), 0.45, 0.75);

//    uint32_t delay_len_adj = 0;
//    int32_t delay_adj_val = 2;

//    AllpassPhaser_Init(&phaser, 2, 0.99);
//    phaser.N = 25000/8;

//    float a2d_d2a_shift[BLOCK_SIZE];
//    arm_fill_f32(2048.0 / 32768.0, a2d_d2a_shift, BLOCK_SIZE);

    Delay_Init(&delay1, (uint32_t)(25000));


    float f_depth = 0.90; // range from 0.25 to 1.0
    float f_period_s = 0.125; // range from 0.1 to T1
    float f_width_Hz = 20000 / 5;
    float f_sample_rate_Hz = 50000;
    ArmAllpassPhaser_Init(&arm_phaser,
						f_depth,
						f_period_s,
						f_width_Hz,
						f_sample_rate_Hz,
						BLOCK_SIZE);

	int buf_offset = 0;

	// Threshold detection and Echo
	float32_t max_val = 0;
	uint32_t max_index = 0;
	float32_t max_threshold = 0.1;
	uint8_t th_reached = false;
	bool echo_stored = false;
	bool echo_complete = true;
	bool flush_out = false;

	/* Call FIR init function to initialize the instance structure.
	 * https://arm-software.github.io/CMSIS_5/DSP/html/arm_fir_example_f32_8c-example.html
	 * */
	arm_fir_instance_f32 Flt1;
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
	static float32_t firStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
	static float32_t firStateF32[BLOCK_SIZE + FLT1_NUM_TAPS - 1];
#endif
	arm_fir_init_f32(&Flt1, FLT1_NUM_TAPS, (float32_t *)&flt1_coeffs[0], &firStateF32[0], BLOCK_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	typedef enum STATE_E {
		STAGE_1 = 1,
		STAGE_2 = 2,
		STAGE_3 = 3,
		STAGE_4 = 4
	} STATE_E;

	STATE_E state_e = STAGE_1;

    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    	if (ready_1 || ready_2)
    	{
			if (ready_1)
			{
				buf_offset = 0;
			}
			else if (ready_2)
			{
				buf_offset = BLOCK_SIZE;
			}

			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);

			arm_shift_q15(dmaBuffer+buf_offset, 3, tmpdsp_buffer, BLOCK_SIZE);
			arm_q15_to_float(tmpdsp_buffer, dsp_buffer+buf_offset, BLOCK_SIZE);
			arm_offset_f32(dsp_buffer+buf_offset, -0.5, ftmpdsp_buffer, BLOCK_SIZE);
			arm_negate_f32(ftmpdsp_buffer, dsp_buffer+buf_offset, BLOCK_SIZE);

			// Power threshold check
			if (state_e == STAGE_1)
			{
				arm_abs_f32(dsp_buffer+buf_offset, ftmpdsp_buffer, BLOCK_SIZE);
				arm_max_f32(ftmpdsp_buffer, BLOCK_SIZE, &max_val, &max_index);
				if (max_val >= max_threshold)
				{
					HAL_GPIO_WritePin(EXT_SRC_GPIO_Port, EXT_SRC_Pin, GPIO_PIN_SET);
					state_e = STAGE_2;
				}
				max_val = 0;
			}
			if (state_e == STAGE_2)
			{
				for (size_t n = buf_offset+max_index; n < buf_offset+BLOCK_SIZE; n++)
				{
					Delay_Step(&delay1, dsp_buffer[n]);
					// Fill the buffer
					if (delay1.position == 0)
					{
						HAL_GPIO_WritePin(EXT_SRC_GPIO_Port, EXT_SRC_Pin, 0);
						state_e = STAGE_3;
						break;
					}
				}
				max_index = 0;
			}


			// BEGIN Signal Processing
//			if (false)
//			{
//				ArmAllpassPhaser_StepBlock(&arm_phaser, dsp_buffer+buf_offset, dsp_buffer_dry, dsp_buffer+buf_offset, BLOCK_SIZE);
//
//				for (size_t n = buf_offset; n < buf_offset+BLOCK_SIZE; n++)
//				{
//					dsp_buffer[n] = FbComb_Step(&fb_comb1, dsp_buffer[n]);
//		//            	dsp_buffer[n] = Delay_Step(&delay1, dsp_buffer[n]);
//		//            	data_log = dsp_buffer[n];
//				}
//			}

			// END Signal Processing
			if (state_e == STAGE_3)
			{
				size_t k=0;
				for (size_t n = buf_offset; n < buf_offset+BLOCK_SIZE; n++)
				{
					out_buffer[k++] = Delay_Step(&delay1, 0.0);
					// Fill the buffer
					if (delay1.position == 0)
					{
						state_e = STAGE_4;
						break;
					}
				}
			}

			arm_fir_f32(&Flt1, out_buffer, ftmpdsp_buffer, BLOCK_SIZE);

			arm_scale_f32(ftmpdsp_buffer, 2.0, dsp_buffer+buf_offset, BLOCK_SIZE);		// range is now from [-1.0, 1.0]
			//arm_scale_f32(dsp_buffer+buf_offset, 2.0, dsp_buffer+buf_offset, BLOCK_SIZE);		// range is now from [-1.0, 1.0]
			arm_float_to_q15(dsp_buffer+buf_offset, tmpdsp_buffer, BLOCK_SIZE);	// Write as right-aligned, range is now from [-32768.0, 32768.0]
			arm_scale_q15(tmpdsp_buffer, (q15_t)1, 11, tmpdsp_buffer2, BLOCK_SIZE);		// range is now from [-4096.0, 4096.0]/2
			arm_offset_q15(tmpdsp_buffer2, 2048, dacBuffer+buf_offset, BLOCK_SIZE); // range is now 12-bits, from [0.0, 4096.0]

			if (state_e == STAGE_4)
			{
				arm_fill_f32(0.0, out_buffer, BLOCK_SIZE);
				state_e = STAGE_1;
			}

			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);

			ready_1 ^= ready_1;
			ready_2 ^= ready_2;
    	}

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
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
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
    // Configure GPIOC pins : MISO, MOSI, CLK
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1080*2-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EXT_SRC_GPIO_Port, EXT_SRC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MY_SPI3_CS_GPIO_Port, MY_SPI3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_SRC_Pin */
  GPIO_InitStruct.Pin = EXT_SRC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(EXT_SRC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MY_SPI3_CS_Pin */
  GPIO_InitStruct.Pin = MY_SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MY_SPI3_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim4)
    {
//        HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
        // kick off another one
        //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dmaBuffer, ADC_BUF_LEN);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    ready_1 = 1;
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    ready_2 = 1;
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *_hdac)
{
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *_hdac)
{
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
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
