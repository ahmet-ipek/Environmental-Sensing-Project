/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include <math.h>
#include <string.h>
#include "sensors.h"
#include "circular_buffer.h"
#include "moving_median_filter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Sampling Configuration
circular_buffer_t temp_buf, humid_buf, press_buf;
median_filter_t temp_filter = {.window_size = 5};
median_filter_t humid_filter = {.window_size = 5};
median_filter_t press_filter = {.window_size = 5};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t sample_ready = 0;
volatile uint8_t tx_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Global Sensor Buffers
circular_buffer_t temperature_buffer;
circular_buffer_t humidity_buffer;
circular_buffer_t pressure_buffer;

void send_sensor_stats() {
    float data[MAX_BUFFER_SIZE];
    uint8_t count;
    sensor_stats_t stats;

    // Temperature statistics
    if (buffer_get_all_values(&temp_buf, data, &count) == 0) {
        calculate_sensor_stats(data, count, &stats);
        char temp_msg[100];
        snprintf(temp_msg, sizeof(temp_msg),
                "[TEMP] StdDev:%.2f | Max:%.2f | Min:%.2f | Median:%.2f\r\n",
                stats.standard_deviation, stats.max_value,
                stats.min_value, stats.median_value);
        HAL_UART_Transmit(&huart2, (uint8_t*)temp_msg, strlen(temp_msg), 100);
    }

    // Humidity statistics
    if (buffer_get_all_values(&humid_buf, data, &count) == 0) {
        calculate_sensor_stats(data, count, &stats);
        char humid_msg[100];
        snprintf(humid_msg, sizeof(humid_msg),
                "[HUMID] StdDev:%.2f | Max:%.2f | Min:%.2f | Median:%.2f\r\n",
                stats.standard_deviation, stats.max_value,
                stats.min_value, stats.median_value);
        HAL_UART_Transmit(&huart2, (uint8_t*)humid_msg, strlen(humid_msg), 100);
    }

    // Pressure statistics
    if (buffer_get_all_values(&press_buf, data, &count) == 0) {
        calculate_sensor_stats(data, count, &stats);
        char press_msg[100];
        snprintf(press_msg, sizeof(press_msg),
                "[PRESS] StdDev:%.2f | Max:%.2f | Min:%.2f | Median:%.2f\r\n",
                stats.standard_deviation, stats.max_value,
                stats.min_value, stats.median_value);
        HAL_UART_Transmit(&huart2, (uint8_t*)press_msg, strlen(press_msg), 100);
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim2) {

	  // 30-second transmission flag
	  tx_ready = 1;

  }

  if (htim == &htim3) {

	  // 1Hz sampling flag
	  sample_ready = 1;
  }
}


//////////////////////////////////////////////////////

// Simulation state to create more realistic data
typedef struct {
    float last_temperature;
    float last_humidity;
    float last_pressure;
    uint32_t seed;
} SimulationState;

SimulationState sim_state = {0};

// Improved random number generation with better distribution
float custom_rand_float() {
    sim_state.seed = sim_state.seed * 1664525 + 1013904223;
    return (float)sim_state.seed / UINT32_MAX;
}

// Gaussian (normal) distribution random number generation
float gaussian_rand() {
    float u1 = custom_rand_float();
    float u2 = custom_rand_float();

    // Box-Muller transform
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159f * u2);
    return z0;
}

float generate_realistic_temperature() {
    // Simulate diurnal temperature variation with added noise
    static float base_temp = 20.0f;  // Average daily temperature
    static uint32_t time_counter = 0;

    // Simulate day/night cycle (roughly 24-hour period)
    float time_of_day = sinf(2 * 3.14159f * time_counter / (24 * 3600));

    // Temperature variation
    float temp_variation = 10.0f * time_of_day;  // +/- 10 degrees

    // Add some randomness
    float noise = gaussian_rand() * 1.5f;

    // Update for next iteration
    time_counter++;

    return base_temp + temp_variation + noise;
}

float generate_realistic_humidity() {
    // Simulate humidity with daily and random variations
    static float base_humidity = 50.0f;  // Average humidity

    // Slight daily variation
    float daily_variation = 10.0f * sinf(2 * 3.14159f * HAL_GetTick() / (24 * 3600));

    // Random walk with mean reversion
    float drift = (base_humidity - sim_state.last_humidity) * 0.1f;

    // Add gaussian noise
    float noise = gaussian_rand() * 2.0f;

    float new_humidity = sim_state.last_humidity + drift + noise + daily_variation;

    // Clamp between 30% and 90%
    new_humidity = fmaxf(30.0f, fminf(90.0f, new_humidity));

    sim_state.last_humidity = new_humidity;
    return new_humidity;
}

float generate_realistic_pressure() {
    // Simulate atmospheric pressure with weather-like variations
    static float base_pressure = 1013.25f;  // Standard atmospheric pressure

    // Simulate weather changes
    float weather_variation = gaussian_rand() * 5.0f;

    // Slight trending
    float drift = (base_pressure - sim_state.last_pressure) * 0.05f;

    float new_pressure = sim_state.last_pressure + drift + weather_variation;

    // Constrain to realistic range
    new_pressure = fmaxf(980.0f, fminf(1050.0f, new_pressure));

    sim_state.last_pressure = new_pressure;
    return new_pressure;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

//  // Initialize sensors
//  LM75A_Init(&hi2c1);
//  Si7021_Init(&hi2c1);
//  LPS25HB_Init(&hi2c1);

  // Initialize simulation state
  sim_state.seed = HAL_GetTick();
  sim_state.last_temperature = 20.0f;
  sim_state.last_humidity = 50.0f;
  sim_state.last_pressure = 1013.25f;

  buffer_init(&temp_buf);
  buffer_init(&humid_buf);
  buffer_init(&press_buf);

    temp_filter.window_size = 5;
    humid_filter.window_size = 5;
    press_filter.window_size = 5;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Real Data
//	  if (sample_ready) {
//		  sample_ready = 0;
//      // Sample sensors at 1 Hz
//      float temp_raw = LM75A_ReadTemperature(&hi2c1);
//      float humid_raw = Si7021_ReadHumidity(&hi2c1);
//      float press_raw = LPS25HB_ReadPressure(&hi2c1);
//
//      // Apply median filter
//      float temp_filt = filter_sensor_value(&temp_filter, temp_raw);
//      float humid_filt = filter_sensor_value(&humid_filter, humid_raw);
//      float press_filt = filter_sensor_value(&press_filter, press_raw);
//
//      // Store filtered values
//      buffer_add_value(&temp_buf, temp_filt);
//      buffer_add_value(&humid_buf, humid_filt);
//      buffer_add_value(&press_buf, press_filt);
//	  }
//
//	  if (tx_ready) {
//		  tx_ready = 0;
//          send_sensor_stats();
//
//          // Clear buffers after transmission
//          buffer_clear(&temp_buf);
//          buffer_clear(&humid_buf);
//          buffer_clear(&press_buf);
//      }

	  // simulated data
	     if (sample_ready) {
	          sample_ready = 0;

	          // Generate realistic mock sensor data
	          float temp_raw = generate_realistic_temperature();
	          float humid_raw = generate_realistic_humidity();
	          float press_raw = generate_realistic_pressure();


	          // Apply median filter
	          float temp_filt = filter_sensor_value(&temp_filter, temp_raw);
	          float humid_filt = filter_sensor_value(&humid_filter, humid_raw);
	          float press_filt = filter_sensor_value(&press_filter, press_raw);

	          // Store filtered values
	          buffer_add_value(&temp_buf, temp_filt);
	          buffer_add_value(&humid_buf, humid_filt);
	          buffer_add_value(&press_buf, press_filt);

	          // Send debug info about raw and filtered values
	          char debug_msg[150];
	          snprintf(debug_msg, sizeof(debug_msg),
	              "Raw: T=%.2f°C H=%.2f%% P=%.2fhPa | Filtered: T=%.2f°C H=%.2f%% P=%.2fhPa\r\n",
	              temp_raw, humid_raw, press_raw,
	              temp_filt, humid_filt, press_filt);
	          HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);
	      }

	      if (tx_ready) {
	          tx_ready = 0;
	          send_sensor_stats();

	          // Clear buffers after transmission
	          buffer_clear(&temp_buf);
	          buffer_clear(&humid_buf);
	          buffer_clear(&press_buf);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 300000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  htim3.Init.Prescaler = 8400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
