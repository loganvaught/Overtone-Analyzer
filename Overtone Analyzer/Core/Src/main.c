/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 4096
#define SAMPLE_RATE 48031 // Calculated from TIM 3 frequency
#define FREQUENCIES_TO_GATHER 100 // The number of frequencies (peaks) to measure as candidates for fundamental frequencies and harmonics. Priority is chosen by FFT magnitude
#define MINIMUM_FUNDAMENTAL_FREQUENCY 40 // The lowest frequency in HZ to consider as a fundamental frequency
#define MAXIMUM_FUNDAMENTAL_FREQUENCY 5000 // The highest frequency in HZ to consider as a fundamental frequency
#define MINIMUM_HARMONICS_FUNDAMENTAL 0 // The lowest number of harmonics to have detected to be considered a fundamental frequency
#define HARMONIC_FACTOR_INTEGER_ERROR 0.02f // The max amount of error for a factor to have off from an integer for a frequency to be considered a harmonic of a fundamental
#define MAX_HARMONIC_GROUP_FACTOR_INTEGER 15 // The maximum factor for a harmonic to be considered a part of a fundamental. Similar to maximum distance
#define FUNDAMENTAL_VOLUME_THRESHOLD_RATIO 0.22f // To be considered a fundamental frequency, it must be at least X * loudest magnitude
#define HARMONIC_VOLUME_THRESHOLD_RATIO 0.03f // To be considered a harmonic, it must be at least X * fundamental magnitude
#define EXPERIMENTAL_FLAG_CHECK_FOR_QUIET_FUNDAMENTAL 1 // If set, harmonic check will look for frequencies below the fundamental candidate to identify a different, quiet fundamental that was missed. Useful for instruments with brighter profiles where fundamentals are registered incorrectly

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for processADC */
osThreadId_t processADCHandle;
const osThreadAttr_t processADC_attributes = {
  .name = "processADC",
  .stack_size = 2500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for do_fft */
osSemaphoreId_t do_fftHandle;
const osSemaphoreAttr_t do_fft_attributes = {
  .name = "do_fft"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
void StartProcessADC(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirect printf through UART
int _write(int fd, char* ptr, int len) {
	HAL_StatusTypeDef hstatus;

	if (fd == 1 || fd == 2) {
	  hstatus = HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
	  if (hstatus == HAL_OK)
		return len;
	  else
		return -1;
	}
	return -1;
}

// ADC with DMA. Giving semaphore for FFT processing
uint16_t fft_in_index = 0;
uint16_t adc_buffer[BUFFER_SIZE] = {};

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		// DMA writing to second half, process the first half.
		fft_in_index = 0;
		osSemaphoreRelease(do_fftHandle);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		// DMA writing to first half, process the second half.
		fft_in_index = BUFFER_SIZE / 2;
		osSemaphoreRelease(do_fftHandle);
	}
}

float fft_in[BUFFER_SIZE];
float fft_out[BUFFER_SIZE];
float fft_mag[BUFFER_SIZE / 2];
float hann_window[BUFFER_SIZE];
arm_rfft_fast_instance_f32 fft_Handle;

void Setup_Hann_Window(float *window, int size){
	for (int i = 0; i < size; i ++){
		window[i] = 0.5f * (1.0f - arm_cos_f32((2.0f * PI * i) / (size - 1)));
	}
}

float get_frequency(uint32_t this_bin) {
	// Use parabolic interpolation with natural log to estimate actual frequency across nearest bins
	// Check to make sure there is actually initialized data
	float frequency_hz = this_bin * SAMPLE_RATE / (BUFFER_SIZE);
	if (this_bin > 0 && this_bin < BUFFER_SIZE / 2 - 1) {
		// Avoid log 0 errors
		if (fft_mag[this_bin-1] > 0 && fft_mag[this_bin] > 0 && fft_mag[this_bin+1] > 0) {
			float alpha = logf(fft_mag[this_bin - 1]);
			float beta = logf(fft_mag[this_bin]);
			float gamma = logf(fft_mag[this_bin + 1]);
			float actual_bin = this_bin;

			// Formula for two immediate neighbors
			float denominator = alpha - 2.0f * beta + gamma;
			if (denominator != 0) { // XV COME BACK: Floats dont have to be 0 to cause problems
				actual_bin = this_bin + (0.5f * ((alpha - gamma) / denominator));
			}

			frequency_hz = actual_bin * SAMPLE_RATE / (BUFFER_SIZE);
		}
	}
	return frequency_hz;
}

float get_amplitude(uint32_t this_bin) {
	// Use parabolic interpolation with natural log to estimate actual magnitude across nearest bins
	// Check to make sure there is actually initialized data
	float amplitude = fft_mag[this_bin];
	if (this_bin > 0 && this_bin < BUFFER_SIZE / 2 - 1) {
		// Avoid log 0 errors
		if (fft_mag[this_bin - 1] > 0 && fft_mag[this_bin] > 0 && fft_mag[this_bin + 1] > 0) {
			float alpha = logf(fft_mag[this_bin - 1]);
			float beta = logf(fft_mag[this_bin]);
			float gamma = logf(fft_mag[this_bin + 1]);

			// Formula for two immediate neighbors
			float denominator = alpha - 2.0f * beta + gamma;
			if (denominator != 0) { // XV COME BACK: Floats dont have to be 0 to cause problems
				// Calculate vertex height
				float peak_log = beta - 0.125f * ((alpha - gamma) * (alpha - gamma) / denominator);
				// Reverse the logarithm
				amplitude = expf(peak_log);
			}
		}
	}
	return amplitude;
}

float get_relative_volume_db(float fundamental_amplitude, float reference_amplitude) {
	float ratio = fundamental_amplitude / reference_amplitude;
	return 20.0f * log10f(ratio);
}

void Process_Half_Buffer() {
	// Convert the ADC samples to float
	for (int i = 0; i < BUFFER_SIZE / 2; i ++) {
		uint16_t actual_index = fft_in_index + i;
		fft_in[actual_index] = ((float) adc_buffer[actual_index]);
	}

	if (fft_in_index == BUFFER_SIZE / 2) {
		// DC bias correction
		float mean = 0.0f;
		arm_mean_f32(fft_in, BUFFER_SIZE, &mean);

		for (int i = 0; i < BUFFER_SIZE; i ++) {
			fft_in[i] -= mean;

			// Use Hann window to fade edges of fft_in to zero, avoiding spectral leakage
			fft_in[i] *= hann_window[i];
		}

		// Just filled fft_in
		// Run the FFT on the fft_in buffer, generate output in fft_out buffer
		arm_rfft_fast_f32(&fft_Handle, fft_in, fft_out, 0);

		// Compute magnitudes of real and complex parts for each bucket
		arm_cmplx_mag_f32(fft_out, fft_mag, BUFFER_SIZE / 2);

		// Gather top frequencies
		float max_vals[FREQUENCIES_TO_GATHER] = {0};
		uint32_t fund_bin[FREQUENCIES_TO_GATHER] = {0};

		// Explicitly ignore the edge-most bins. This helps with maxima finding, and future parabolic interpolation
		for (int i = 1; i < BUFFER_SIZE / 2 - 1; i ++) {
			float this_val = fft_mag[i];
			// Check if it belongs in the max vals list

			// Check if it is a local maxima
			if (this_val > fft_mag[i - 1] && this_val > fft_mag[i + 1]) {
				// Find a place, shift right the smaller values.
				// Left most index is highest peak
				for (int j = 0; j < FREQUENCIES_TO_GATHER; j ++) {
					if (this_val > max_vals[j]) {
						// Move everything right one
						for (int k = FREQUENCIES_TO_GATHER - 2; k >= j; k --) {
							max_vals[k + 1] = max_vals[k];
							fund_bin[k + 1] = fund_bin[k];
						}
						// Put at this index
						max_vals[j] = this_val;
						fund_bin[j] = i;
						break;
					}
				}
			}
		}

		float gathered_frequencies[FREQUENCIES_TO_GATHER] = {0}; // Calculate frequencies
		for (int i = 0; i < FREQUENCIES_TO_GATHER; i++) {
			uint32_t this_bin = fund_bin[i];

			gathered_frequencies[i] = get_frequency(this_bin);
		}

		// Store most prominent peak for determining loudest magnitude before sorted by frequency
		// XV: What happens when fund_bin is 0 (there were no peaks found?)
		float max_magnitude_found = get_amplitude(fund_bin[0]);

		// Sort the gathered frequencies from low to high. Insertion sort
		for (int i = 1; i < FREQUENCIES_TO_GATHER; i ++) {
			int j = i;
			float this_frequency = gathered_frequencies[j];
			while (j > 0 && this_frequency < gathered_frequencies[j - 1]) {
				// Swap frequencies in j and j - 1
				gathered_frequencies[j] = gathered_frequencies[j - 1];
				gathered_frequencies[j - 1] = this_frequency;

				// Swap magnitudes in j and j - 1
				float temp_magnitude = max_vals[j];
				max_vals[j] = max_vals[j - 1];
				max_vals[j - 1] = temp_magnitude;

				// Swap bins of j and j - 1
				uint32_t temp_bin = fund_bin[j];
				fund_bin[j] = fund_bin[j-1];
				fund_bin[j-1] = temp_bin;

				// Decrement j to go down list finding proper spot for this_frequency
				j -= 1;
			}
			// j is the new index this should be at
		}

		printf("----------------------------------------------------\r\n");

		// Go through each frequency, group by fundamentals and harmonics and noise
		uint8_t harmonics[FREQUENCIES_TO_GATHER] = {0}; // Grouped by fundamental frequency. 1 = family of frequency 1, 2 = family of frequency 2
		uint8_t harmonic_group_on = 1;
		uint8_t max_harmonic_group = 0;
		for (int i = 0; i < FREQUENCIES_TO_GATHER; i ++) {
			if (harmonics[i] != 0) continue; // Already part of another group
			float this_frequency = gathered_frequencies[i];
			float this_frequency_amplitude = get_amplitude(fund_bin[i]);

			if (this_frequency < MINIMUM_FUNDAMENTAL_FREQUENCY || this_frequency > MAXIMUM_FUNDAMENTAL_FREQUENCY) continue; // Noise (too low HZ)
			if (this_frequency_amplitude < max_magnitude_found * FUNDAMENTAL_VOLUME_THRESHOLD_RATIO) continue; // Noise (too quiet for fundamental)

			harmonics[i] = harmonic_group_on;

			// Go through all remaining frequencies, detect harmonics, count them
			int harmonic_amount = 0;
			int j = i + 1;
			uint8_t new_fundamental_found = 0;

			if (EXPERIMENTAL_FLAG_CHECK_FOR_QUIET_FUNDAMENTAL) j = 0;

			for (; j < FREQUENCIES_TO_GATHER; j ++) {
				if (harmonics[j] != 0) continue; // Frequency already belonged to a harmonic group
				if (get_amplitude(fund_bin[j]) < this_frequency_amplitude * HARMONIC_VOLUME_THRESHOLD_RATIO) continue; // too quiet to be a harmonic for this fundamental

				float checking_frequency = gathered_frequencies[j];
				if (EXPERIMENTAL_FLAG_CHECK_FOR_QUIET_FUNDAMENTAL && checking_frequency < MINIMUM_FUNDAMENTAL_FREQUENCY) continue; // Too low for reconsideration as a fundamental frequency

				float factor = checking_frequency / this_frequency; // Factor for harmonics above current fundamental
				if (EXPERIMENTAL_FLAG_CHECK_FOR_QUIET_FUNDAMENTAL && j < i && new_fundamental_found == 0) factor = this_frequency / checking_frequency;


				// CHECK IF FACTOR IS INTEGER
				float nearest = roundf(factor);
				if (fabsf(nearest - factor) <= HARMONIC_FACTOR_INTEGER_ERROR && nearest >= 2 && nearest <= MAX_HARMONIC_GROUP_FACTOR_INTEGER) { // Consider checking volume for "likely a harmonic, not noise
					// Part of the harmonic group
					harmonic_amount += 1;
					harmonics[j] = harmonic_group_on;
					// Check if this is a new fundamental (if experiemntal quiet fundamental is set). Then update the fundamental frequency we are checking harmonics with
					if (EXPERIMENTAL_FLAG_CHECK_FOR_QUIET_FUNDAMENTAL && j < i && new_fundamental_found == 0) {
						// A new, lower, quiet fundamental has been found. Use this for the rest of the harmonic calculations
						this_frequency = checking_frequency;
						this_frequency_amplitude = get_amplitude(fund_bin[j]);
						new_fundamental_found = 1;
					}
				}
			}

			if (harmonic_amount < MINIMUM_HARMONICS_FUNDAMENTAL) {
				// Minimum harmonic requirement not met, this frequency is likely noise. write a 0 in calculated harmonics
				for (int k = 0; k < FREQUENCIES_TO_GATHER; k++) {
					if (harmonics[k] == harmonic_group_on) {
						harmonics[k] = 0;
					}
				}
			}
			else {
				max_harmonic_group = harmonic_group_on;
				harmonic_group_on += 1;
			}
		}

		// XV CHECK LATER: Maybe do something different when EXPERIMENTAL_FLAG_CHECK_FOR_QUIET_FUNDAMENTAL is set. Or else super quiet fundamentals will be all the way at bottom, even with a very powerful harmonic
		// Sort the groups by volume (loudest fundamentals in front)
		uint8_t sorted_groups[FREQUENCIES_TO_GATHER] = {0};
		float group_volumes[FREQUENCIES_TO_GATHER] = {0};
		// Initialize arrays to sort by volume
		for (int i = 0; i < max_harmonic_group; i ++) {
			sorted_groups[i] = i + 1;
			// get group volume
			for (int j = 0; j < FREQUENCIES_TO_GATHER; j ++) {
				if (harmonics[j] == i + 1) {
					group_volumes[i] = get_amplitude(fund_bin[j]);
					break;
				}
			}
		}

		// Begin sorting, insertion sort
		for (int i = 1; i < max_harmonic_group; i ++) {
			int j = i;
			float this_volume = group_volumes[j];
			while (j > 0 && this_volume > group_volumes[j - 1]) {
				// Swap volumes in j and j - 1
				group_volumes[j] = group_volumes[j - 1];
				group_volumes[j - 1] = this_volume;

				// Swap groups in j and j - 1
				uint8_t this_group = sorted_groups[j];
				sorted_groups[j] = sorted_groups[j - 1];
				sorted_groups[j - 1] = this_group;

				// Decrement j to go down list finding proper spot for this group
				j -= 1;
			}
		}

		// Print out data / compute relative DB for fundamentals compared to loudest fundamental, and harmonics relative to their fundamental

		float loudest_fundamental_volume = 0;
		uint8_t loudest_fundamental_found = 0;
		for (int i = 0; i < max_harmonic_group; i ++) {
			uint8_t group = sorted_groups[i];
			uint8_t fundamental_found = 0;
			float fundamental_volume = group_volumes[i];
			for (int j = 0; j < FREQUENCIES_TO_GATHER; j ++) {
				if (harmonics[j] == group) {
					// PART OF HARMONIC GROUP
					if (fundamental_found == 0) {
						if (loudest_fundamental_found == 0) {
							loudest_fundamental_found = 1;
							loudest_fundamental_volume = fundamental_volume;
						}
						fundamental_found = 1;
						printf("%dhz (%ddb): ", (int) gathered_frequencies[j], (int) get_relative_volume_db(fundamental_volume,loudest_fundamental_volume));
					}
					else {
						printf("%dhz (%ddb), ", (int) gathered_frequencies[j], (int) get_relative_volume_db(get_amplitude(fund_bin[j]), fundamental_volume));
					}
				}
			}
			printf("\r\n");
		}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\nWorking!\r\n");

  // Setup the Hann window to be used with FFT processing
  Setup_Hann_Window(&hann_window[0], BUFFER_SIZE);

  // Setup FFT handle
  arm_rfft_fast_init_f32(&fft_Handle, BUFFER_SIZE);

  // Start timer 3 to trigger ADC conversion
  HAL_TIM_Base_Start(&htim3);

  // Start DMA transfer with ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);

  printf("All setup.\r\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of do_fft */
  do_fftHandle = osSemaphoreNew(1, 0, &do_fft_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of processADC */
  processADCHandle = osThreadNew(StartProcessADC, NULL, &processADC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim3.Init.Period = 2083 - 1;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartProcessADC */
/**
* @brief Function implementing the processADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessADC */
void StartProcessADC(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    // Acquire semaphore and process correct half of circular ADC buffer
	osSemaphoreAcquire(do_fftHandle, osWaitForever);
	// Perform FFT on this half

	Process_Half_Buffer();

  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
#ifdef USE_FULL_ASSERT
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
