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
#include <math.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ITER 100
#define NUM_SIZES 5
#define LED_DELAY 2000 // 2 seconds delay
#define MAX_SIZE 256

// Image dimensions from Practical 1B
const uint32_t height[NUM_SIZES] = {128, 160, 192, 224, 256};
const uint32_t width[NUM_SIZES] = {128, 160, 192, 224, 256};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Performance timing variables
uint32_t start_time, end_time;
uint32_t execution_time_float_fpu, execution_time_float_no_fpu;
uint32_t execution_time_double_fpu, execution_time_double_no_fpu;
uint32_t pixels_processed;

// Results storage (single row at a time to save memory)
uint8_t mandelbrot_row_float[MAX_SIZE];
uint8_t mandelbrot_row_double[MAX_SIZE];

// For accuracy comparison - we'll store a sample of points instead of the whole image
#define SAMPLE_POINTS 100
typedef struct {
    uint32_t x;
    uint32_t y;
    uint8_t float_val;
    uint8_t double_val;
} sample_point_t;

sample_point_t sample_points[SAMPLE_POINTS];

// Test control
uint8_t test_completed = 0;
uint8_t current_size_index = 0;
uint8_t current_test_phase = 0; // 0: float, 1: double
uint8_t fpu_enabled = 0;

// ADD THESE NEW GLOBAL VARIABLES FOR RESULTS
float accuracy_difference_percentage = 0.0f;
float float_speedup_factor = 0.0f;
float double_speedup_factor = 0.0f;
uint32_t current_size = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
// Mandelbrot calculation functions
void calculate_mandelbrot_float(uint32_t w, uint32_t h);
void calculate_mandelbrot_double(uint32_t w, uint32_t h);

// Utility functions
void start_timer(void);
uint32_t stop_timer(void);
void delay_ms(uint32_t ms);
void run_test_series(void);
void log_results(void);
void collect_samples(uint32_t w, uint32_t h, uint32_t row, uint8_t is_float);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
  // Initialize performance counters
  execution_time_float_fpu = 0;
  execution_time_float_no_fpu = 0;
  execution_time_double_fpu = 0;
  execution_time_double_no_fpu = 0;

  // Initialize sample points with random coordinates
  for (int i = 0; i < SAMPLE_POINTS; i++) {
      sample_points[i].x = i * 7 % MAX_SIZE;  // Pseudo-random distribution
      sample_points[i].y = i * 11 % MAX_SIZE; // Pseudo-random distribution
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Visual indicator: Turn on LED0 to signal processing start
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // Run the test series
    run_test_series();

    // Visual indicator: Turn on LED1 to signal processing complete
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    // Keep the LEDs ON for 2s
    delay_ms(LED_DELAY);

    // Turn OFF LEDs
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    // Wait before potentially running tests again
    delay_ms(LED_DELAY);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Mandelbrot calculation with float precision
void calculate_mandelbrot_float(uint32_t w, uint32_t h) {
    float x0, y0, x, y, xtemp;
    uint32_t i, j, iter;

    for (i = 0; i < h; i++) {
        for (j = 0; j < w; j++) {
            x0 = (j * 3.5f / w) - 2.5f;
            y0 = (i * 2.0f / h) - 1.0f;

            x = 0.0f;
            y = 0.0f;
            iter = 0;

            while (x*x + y*y <= 4.0f && iter < MAX_ITER) {
                xtemp = x*x - y*y + x0;
                y = 2.0f*x*y + y0;
                x = xtemp;
                iter++;
            }

            mandelbrot_row_float[j] = (uint8_t)iter;
        }

        // Collect samples from this row if needed
        for (int k = 0; k < SAMPLE_POINTS; k++) {
            if (sample_points[k].y == i) {
                sample_points[k].float_val = mandelbrot_row_float[sample_points[k].x];
            }
        }
    }
}

// Mandelbrot calculation with double precision
void calculate_mandelbrot_double(uint32_t w, uint32_t h) {
    double x0, y0, x, y, xtemp;
    uint32_t i, j, iter;

    for (i = 0; i < h; i++) {
        for (j = 0; j < w; j++) {
            x0 = (j * 3.5 / w) - 2.5;
            y0 = (i * 2.0 / h) - 1.0;

            x = 0.0;
            y = 0.0;
            iter = 0;

            while (x*x + y*y <= 4.0 && iter < MAX_ITER) {
                xtemp = x*x - y*y + x0;
                y = 2.0*x*y + y0;
                x = xtemp;
                iter++;
            }

            mandelbrot_row_double[j] = (uint8_t)iter;
        }

        // Collect samples from this row if needed
        for (int k = 0; k < SAMPLE_POINTS; k++) {
            if (sample_points[k].y == i) {
                sample_points[k].double_val = mandelbrot_row_double[sample_points[k].x];
            }
        }
    }
}

// Timer functions
void start_timer(void) {
    start_time = HAL_GetTick();
}

uint32_t stop_timer(void) {
    end_time = HAL_GetTick();
    return end_time - start_time;
}

// Simple delay function
void delay_ms(uint32_t ms) {
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < ms) {
        // Wait
    }
}

// Run test series for all image sizes
void run_test_series(void) {
    uint32_t w, h;

    for (current_size_index = 0; current_size_index < NUM_SIZES; current_size_index++) {
        w = width[current_size_index];
        h = height[current_size_index];
        pixels_processed = w * h;

        // Reset sample values
        for (int i = 0; i < SAMPLE_POINTS; i++) {
            sample_points[i].float_val = 0;
            sample_points[i].double_val = 0;
        }

        // Test with float precision
        start_timer();
        calculate_mandelbrot_float(w, h);
        if (fpu_enabled) {
            execution_time_float_fpu = stop_timer();
        } else {
            execution_time_float_no_fpu = stop_timer();
        }

        // Test with double precision
        start_timer();
        calculate_mandelbrot_double(w, h);
        if (fpu_enabled) {
            execution_time_double_fpu = stop_timer();
        } else {
            execution_time_double_no_fpu = stop_timer();
        }

        // Log results for this size
        log_results();

        // Small delay between sizes
        delay_ms(100);
    }

    test_completed = 1;
}

// Log results (in a real implementation, this would output to UART or storage)
void log_results(void) {
    uint32_t w = width[current_size_index];
    uint32_t h = height[current_size_index];

    // Calculate speed-up factors
    float float_speedup = 0;
    float double_speedup = 0;

    if (fpu_enabled && execution_time_float_no_fpu > 0) {
        float_speedup = (float)execution_time_float_no_fpu / execution_time_float_fpu;
    }

    if (fpu_enabled && execution_time_double_no_fpu > 0) {
        double_speedup = (float)execution_time_double_no_fpu / execution_time_double_fpu;
    }

    // Compare accuracy by counting differing sample points
    uint32_t diff_count = 0;
    for (int i = 0; i < SAMPLE_POINTS; i++) {
        if (sample_points[i].x < w && sample_points[i].y < h) {
            if (sample_points[i].float_val != sample_points[i].double_val) {
                diff_count++;
            }
        }
    }

    float accuracy_diff = 0;
    if (SAMPLE_POINTS > 0) {
        accuracy_diff = (float)diff_count / SAMPLE_POINTS * 100.0f;
    }

    // STORE THE RESULTS IN THE GLOBAL VARIABLES FOR DEBUGGING
    if (fpu_enabled) {
        // These are already stored in the global timing variables
    } else {
        float_speedup_factor = float_speedup;
        double_speedup_factor = double_speedup;
        accuracy_difference_percentage = accuracy_diff;
        current_size = w; // Store which size this test was for
    }

    // In a real implementation, you would send these results via UART
    // For now, we'll use the LEDs to indicate status
    if (fpu_enabled) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // LED2 for FPU enabled
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // LED3 for FPU disabled
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
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
