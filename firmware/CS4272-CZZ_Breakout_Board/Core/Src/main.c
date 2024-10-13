/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "CS4272.h"
#include "EMAFilter.h"
#include "PeakingFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_DATA_BUFFER_SIZE  128
#define AUDIO_SAMPLE_RATE_HZ 	48000.0f

#define NUM_CTRL_KNOBS 		 8 // 7 Frequency Bands, 1 Volume Level
#define NUM_FREQ_BANDS 		 7
#define CTRL_KNOBS_DEADBAND  0.05f
#define CTRL_KNOBS_EMA_ALPHA 0.85f // Alpha factor for EMA Low Pass Filter (Lower alpha -> Lower cutoff frequency)

#define MAX_OUTPUT_VOLUME_SCALE 4.0f
#define MAX_OUTPUT_GAIN_SCALE 5.0f
#define MAX_OUTPUT_BANDWIDTH_SCALE 1000.0f

#define ADC_RAW_NORMALIZATION_FACTOR 65535 // 2^16 - 1
#define INT32_TO_FLOAT 				 4.6566128752e-10f // 1 / (2^(32-1) - 1)
#define FLOAT_TO_INT32 				 2.147483647e9f // 2^(32-1) - 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// CS4272 Codec
CS4272 codec;

// CMSIS IIR Filters for Stereo Channel
IIR_Direct_Form_1 left_iir_filter;
IIR_Direct_Form_1 right_iir_filter;

// Struct to hold our filter parameters that can be changed by the control knobs
PeakingFilterParameters peaking_filters_params[NUM_FREQ_BANDS];

// EMA Low Pass Filters for the Control Knobs
EMAFilter ema_filters[NUM_CTRL_KNOBS];

// Pre-defined fixed center frequencies for our peaking filters
static const float peaking_filter_center_freqs[NUM_FREQ_BANDS] = {
		100.0f,
		200.0f,
		400.0f,
		800.0f,
		1600.0f,
		3200.0f,
		6400.0f
};

// Ping Pong buffers for our codec's ADC and DAC
volatile int32_t audio_adc_buff[AUDIO_DATA_BUFFER_SIZE] = {0};
volatile int32_t audio_dac_buff[AUDIO_DATA_BUFFER_SIZE] = {0};
// Access pointers to our audio data buffers
static volatile int32_t *audio_in_buff_ptr = &audio_adc_buff[0];
static volatile int32_t *audio_out_buff_ptr = &audio_dac_buff[0];
// Stereo buffers for IIR filters
static float left_input_buffer[AUDIO_DATA_BUFFER_SIZE/4] = {0};
static float left_output_buffer[AUDIO_DATA_BUFFER_SIZE/4] = {0};
static float right_input_buffer[AUDIO_DATA_BUFFER_SIZE/4] = {0};
static float right_output_buffer[AUDIO_DATA_BUFFER_SIZE/4] = {0};
// Flag to indicate that our current in buffer has filled up and is ready to be switched with the out buffer
volatile uint8_t audio_buff_samples_rdy = 0;

// Buffer to hold our raw sampled control knob signals
volatile uint16_t ctrl_knobs_settings_raw[NUM_CTRL_KNOBS] = {0};
// Buffer to hold our control knob signals normalized to [0.0, 1.0] (First 7 are frequency bands, Last is Volume Level)
volatile float 	  ctrl_knobs_settings[NUM_CTRL_KNOBS] = {0};
// Buffer to hold memory of the control knob samples for low pass filtering purposes/deadband calculations
volatile float    prev_ctrl_knobs_settings[NUM_CTRL_KNOBS] = {0};

volatile uint8_t  ctrl_knobs_adc_samples_rdy = 0;
// Flag to indicate control knobs are being changed
volatile uint8_t  ctrl_knobs_adc_lock = 0;
// Lock to make sure we don't change ctrl_knobs_settings in the middle of processing
volatile uint8_t  ctrl_knobs_audio_proc_lock = 0;
// Toggle to modify either the filter's gain (mode_select=0) or the bandwidth (mode_select=1)
volatile uint8_t mode_select = 0;

// Keeps track of the audio output level (Left and Right channels are equally leveled)
volatile float output_volume_level = 1.0f;

// Buffer used for UART transmission
char              uart_buff[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Callback for our control knobs
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (!ctrl_knobs_adc_lock)
	{
		for (size_t i=0; i<NUM_CTRL_KNOBS; ++i)
		{
			// Convert raw ADC samples from the control knobs to floats and normalize to [0.0, 1.0]
			ctrl_knobs_settings[i] = (float) (ctrl_knobs_settings_raw[i]) / ADC_RAW_NORMALIZATION_FACTOR;
		}
		ctrl_knobs_adc_samples_rdy = 1;
	}
}

// Converts processed control knob samples into filter parameters
void processControlKnobs()
{
	// Set our lock to prevent the ADC from updating the ctrl_knobs_settings during this function call
	ctrl_knobs_adc_lock = 1;

	if (!ctrl_knobs_audio_proc_lock)
	{
		for (size_t i=0; i<NUM_CTRL_KNOBS; ++i)
		{
			// Low Pass Filter the control knob signals
			ctrl_knobs_settings[i] = EMAFilter_Update(&ema_filters[i], ctrl_knobs_settings[i]);

			if (i != NUM_CTRL_KNOBS-1)
			{
				if (fabs(ctrl_knobs_settings[i]-prev_ctrl_knobs_settings[i]) > CTRL_KNOBS_DEADBAND || 1)
				{
					// If we are past the deadband then update our filter_params

					// Center Frequencies are fixed and go from [100, 6400] Hz
					if (peaking_filters_params[i].center_freq_hz != peaking_filter_center_freqs[i])
					{
						peaking_filters_params[i].center_freq_hz = peaking_filter_center_freqs[i];
					}

					// Update the filter gain
//					peaking_filters_params[i].gain_linear = MAX_OUTPUT_GAIN_SCALE * ctrl_knobs_settings[i];
					if (i == 2)
					{
						peaking_filters_params[i].gain_linear = MAX_OUTPUT_GAIN_SCALE;
					}
					else
					{
						peaking_filters_params[i].gain_linear = 1.0f;
					}

					// Update the filter bandwidth
//					peaking_filters_params[i].bandwidth_hz = (MAX_OUTPUT_BANDWIDTH_SCALE * ctrl_knobs_settings[i]) + 1.0f;
//					if (peaking_filters_params[i].bandwidth_hz > peaking_filters_params[i].center_freq_hz)
//					{
//						// Since the center frequencies are not evenly spaced out we also need to make the bandwidth skewed
//						peaking_filters_params[i].bandwidth_hz = peaking_filters_params[i].center_freq_hz;
//					}
					peaking_filters_params[i].bandwidth_hz = 50.0f;
				}
			}
			else
			{
				// Set output_level
//				output_volume_level = MAX_OUTPUT_VOLUME_SCALE * ctrl_knobs_settings[i];
				output_volume_level = 1.0f;
			}

			// Update the prev_ctrl_knobs_settings
			prev_ctrl_knobs_settings[i] = ctrl_knobs_settings[i];
		}
		// Update our IIR filters with our new parameters
		PeakingFilter_Set_Coefficients_CMSIS(&left_iir_filter, peaking_filters_params, NUM_FREQ_BANDS);
		PeakingFilter_Set_Coefficients_CMSIS(&right_iir_filter, peaking_filters_params, NUM_FREQ_BANDS);
	}

	// Reset ctrl_knobs_adc_lock
	ctrl_knobs_adc_lock = 0;
	// Reset ctrl_knobs_adc_samples_rdy to indicate we need a new batch of ctrl_knobs_settings samples from the ADC
	ctrl_knobs_adc_samples_rdy = 0;
}



// Callbacks for ping ponging I2S audio TX and RX buffers
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	audio_in_buff_ptr = &audio_adc_buff[0];
	audio_out_buff_ptr = &audio_dac_buff[0];

	audio_buff_samples_rdy = 1;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	audio_in_buff_ptr = &audio_adc_buff[AUDIO_DATA_BUFFER_SIZE/2];
	audio_out_buff_ptr = &audio_dac_buff[AUDIO_DATA_BUFFER_SIZE/2];

	audio_buff_samples_rdy = 1;
}

// Function that processes the audio data held in the buffers
void processAudioData()
{
	// Set our control knob lock to prevent any changes to the control settings during this function call
	ctrl_knobs_audio_proc_lock = 1;


	for (size_t i=0; i<(AUDIO_DATA_BUFFER_SIZE/4); ++i)
	{
		// Fill in our input buffers and convert int32_t to floats for processing
		left_input_buffer[i] = INT32_TO_FLOAT * ((float) audio_in_buff_ptr[2*i]);
		right_input_buffer[i] = INT32_TO_FLOAT * ((float) audio_in_buff_ptr[(2*i)+1]);
	}

	PeakingFilter_Update_CMSIS(&left_iir_filter, left_input_buffer, left_output_buffer, AUDIO_DATA_BUFFER_SIZE/4);
	PeakingFilter_Update_CMSIS(&right_iir_filter, right_input_buffer, right_output_buffer, AUDIO_DATA_BUFFER_SIZE/4);

	const float scaling_factor = FLOAT_TO_INT32 * output_volume_level;
	for (size_t i=0; i<(AUDIO_DATA_BUFFER_SIZE/4); ++i)
	{
		// Fill in our output buffers
		audio_out_buff_ptr[2*i] = scaling_factor * left_output_buffer[i];
		audio_out_buff_ptr[(2*i)+1] = scaling_factor * right_output_buffer[i];
	}

	// Reset our control knob lock since we completed processing
	ctrl_knobs_audio_proc_lock = 0;
	// Reset audio_buff_samples_rdy to indicate we need a new batch of audio samples from I2S
	audio_buff_samples_rdy = 0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2S1_Init();
  /* USER CODE BEGIN 2 */
  // Calibrate ADC 1
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  // Start ADC 1 with DMA to read in our control knobs
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ctrl_knobs_settings_raw, NUM_CTRL_KNOBS);
  // Start timer 2 for clocking ADC 1
  HAL_TIM_Base_Start(&htim2);


  // Initialize our codec
  CS4272_Init(&codec, &hi2c1);
  // Initialize our IIR Peaking Filters
  PeakingFilter_Init_CMSIS(&left_iir_filter, AUDIO_SAMPLE_RATE_HZ);
  PeakingFilter_Init_CMSIS(&right_iir_filter, AUDIO_SAMPLE_RATE_HZ);
  // Initialize our EMA Low Pass Filters
  EMAFilters_Init(ema_filters, (size_t) NUM_CTRL_KNOBS, CTRL_KNOBS_EMA_ALPHA);


  // Initialize our I2S data stream
  HAL_I2SEx_TransmitReceive_DMA(&hi2s1, (uint16_t *) audio_dac_buff, (uint16_t *) audio_adc_buff, AUDIO_DATA_BUFFER_SIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (audio_buff_samples_rdy)
	  {
		  // Buffers are initialized with audio data, we can process the data now
		  HAL_GPIO_WritePin(GPIO_STATUS_GPIO_Port, GPIO_STATUS_Pin, GPIO_PIN_SET);
		  processAudioData();
		  HAL_GPIO_WritePin(GPIO_STATUS_GPIO_Port, GPIO_STATUS_Pin, GPIO_PIN_RESET);
	  }

	  if (ctrl_knobs_adc_samples_rdy)
	  {
		  // Buffers are initialized with control knob data, we can process the data now
		  processControlKnobs();
	  }


//	  for (size_t i=0; i<NUM_CTRL_KNOBS; ++i)
//	  {
//		  if (i == 2)
//		  {
//			  sprintf(uart_buff, "Control Knob %i Normalized Value: %lf \r\n", i+1, ctrl_knobs_settings[i]);
//			  HAL_UART_Transmit(&huart3, (uint8_t *) uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
//		  }
//	  }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x307075B1;
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
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_SLAVE_FULLDUPLEX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s1.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s1.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_LEFT;
  hi2s1.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CODEC_NRST_GPIO_Port, CODEC_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_STATUS_GPIO_Port, GPIO_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CODEC_NRST_Pin */
  GPIO_InitStruct.Pin = CODEC_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CODEC_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_STATUS_Pin */
  GPIO_InitStruct.Pin = GPIO_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_STATUS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
