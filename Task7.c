* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Task 7 - Fixed Point Arithmetic Scaling Factor Testing (STM32F4)
  *
  * Tests different scaling factors (10^3, 10^4, 10^6) to analyze:
  * - Effect on precision
  * - Risk of overflow
  * - Execution speed
  * STM32F4 specific considerations:
  * - FPU available but disabled for fixed-point comparison
  * - More memory available for larger scaling factors
  * - Higher clock speed enables more precise timing
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ITER 100

// Image dimensions from Practical 1B
uint32_t height[5] = {128, 160, 192, 224, 256};
uint32_t width[5] = {128, 160, 192, 224, 256};

// Fixed-point scaling factors to test
typedef struct {
    int64_t scale_factor;
    const char* description;
    uint8_t precision_bits;
    uint8_t overflow_risk;  // 0=low, 1=medium, 2=high
} ScalingTest_t;

ScalingTest_t scaling_tests[] = {
    {1000,      "10^3 - Low precision, safe",     10, 0},
    {10000,     "10^4 - Medium precision",        13, 0},
    {100000,    "10^5 - Good precision",          17, 0},
    {1000000,   "10^6 - High precision",          20, 1},
    {10000000,  "10^7 - Very high precision",     23, 2},
    {100000000, "10^8 - Extreme precision",       27, 2}
};

#define NUM_SCALING_TESTS (sizeof(scaling_tests)/sizeof(scaling_tests[0]))

// Performance measurement variables
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time_ms[30];    // 6 scaling factors × 5 image sizes
volatile uint64_t checksum_results[30];
volatile uint32_t overflow_detected[30];
volatile uint32_t precision_score[30];

// Reference results (double precision for comparison)
volatile uint64_t reference_checksum[5];
volatile uint32_t reference_time[5];

// DWT timing for F4 (more precise)
volatile uint32_t dwt_start = 0;
volatile uint32_t dwt_end = 0;
volatile uint32_t dwt_cycles[30];

// Current test tracking
volatile uint32_t current_scale_test = 0;
volatile uint32_t current_image = 0;
volatile uint32_t test_index = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
// Function prototypes
void run_reference_tests(void);
void run_scaling_factor_tests(void);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_fixed_point_scaled(int width, int height, int max_iterations, int64_t scale_factor);
uint32_t check_overflow_risk_f4(int64_t scale_factor, int width, int height);
uint32_t calculate_precision_score(uint64_t fixed_result, uint64_t double_result);
void enable_dwt(void);
uint32_t get_dwt_cycles(void);
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
  // Enable DWT for cycle-accurate timing
  enable_dwt();

  // Initialize result arrays
  for (int i = 0; i < 30; i++) {
      execution_time_ms[i] = 0;
      checksum_results[i] = 0;
      dwt_cycles[i] = 0;
      overflow_detected[i] = 0;
      precision_score[i] = 0;
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  // Flash startup pattern
  for (int i = 0; i < 3; i++) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(200);
  }

  // Step 1: Run reference tests (double precision - uses FPU on F4)
  run_reference_tests();

  // Step 2: Run all fixed-point scaling factor tests
  run_scaling_factor_tests();

  // Flash completion pattern
  for (int i = 0; i < 6; i++) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(100);
  }

  // Set breakpoint here to examine all results
  volatile int debug_complete = 1; // Breakpoint marker
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // STM32F4 Fixed-point scaling factor test results:
    // execution_time_ms[0-29]   - Execution times for all tests
    // checksum_results[0-29]    - Verification checksums
    // dwt_cycles[0-29]          - DWT cycles for precise timing
    // overflow_detected[0-29]   - Overflow detection flags
    // precision_score[0-29]     - Accuracy compared to double precision
    //
    // Data layout (test_index = scale_test * 5 + image_size):
    // Indices 0-4:   Scale factor 10^3, image sizes 128x128 to 256x256
    // Indices 5-9:   Scale factor 10^4, image sizes 128x128 to 256x256
    // Indices 10-14: Scale factor 10^5, image sizes 128x128 to 256x256
    // Indices 15-19: Scale factor 10^6, image sizes 128x128 to 256x256
    // Indices 20-24: Scale factor 10^7, image sizes 128x128 to 256x256
    // Indices 25-29: Scale factor 10^8, image sizes 128x128 to 256x256
    //
    // STM32F4 specific observations:
    // - FPU makes double-precision reference much faster than on F0
    // - Fixed-point still faster than double for many cases
    // - Can handle larger scaling factors due to more memory
    // - Compare with F0 results to see platform differences
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/* Run reference tests for precision comparison (uses FPU on F4) */
void run_reference_tests(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    for (int i = 0; i < 5; i++) {
        start_time = HAL_GetTick();
        dwt_start = get_dwt_cycles();

        reference_checksum[i] = calculate_mandelbrot_double(width[i], height[i], MAX_ITER);

        dwt_end = get_dwt_cycles();
        end_time = HAL_GetTick();

        reference_time[i] = end_time - start_time;

        // LED indication for progress
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        HAL_Delay(100);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(500);
}

/* Run all scaling factor tests */
void run_scaling_factor_tests(void) {
    test_index = 0;

    for (int scale_idx = 0; scale_idx < NUM_SCALING_TESTS; scale_idx++) {
        current_scale_test = scale_idx;


        // LED pattern to indicate current scaling test
        for (int blink = 0; blink < scale_idx + 1; blink++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_Delay(100);
        }
        HAL_Delay(300);

        for (int img = 0; img < 5; img++) {
            current_image = img;
            test_index = scale_idx * 5 + img;


            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

            // Check for potential overflow before running test
            overflow_detected[test_index] = check_overflow_risk_f4(
                scaling_tests[scale_idx].scale_factor, width[img], height[img]);

            // Start timing
            start_time = HAL_GetTick();
            dwt_start = get_dwt_cycles();

            // Run fixed-point Mandelbrot with current scaling factor
            checksum_results[test_index] = calculate_mandelbrot_fixed_point_scaled(
                width[img], height[img], MAX_ITER, scaling_tests[scale_idx].scale_factor);

            // End timing
            dwt_end = get_dwt_cycles();
            end_time = HAL_GetTick();

            execution_time_ms[test_index] = end_time - start_time;

            // Calculate DWT cycles
            if (dwt_end >= dwt_start) {
                dwt_cycles[test_index] = dwt_end - dwt_start;
            } else {
                // Handle DWT counter overflow
                dwt_cycles[test_index] = (0xFFFFFFFF - dwt_start) + dwt_end;
            }

            // Calculate precision score compared to double precision reference
            precision_score[test_index] = calculate_precision_score(
                checksum_results[test_index], reference_checksum[img]);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_Delay(50);
        }

        // Pause between scaling factor groups
        HAL_Delay(500);
    }
}

/* Check potential overflow risk for F4 (more aggressive) */
uint32_t check_overflow_risk_f4(int64_t scale_factor, int width, int height) {
    // F4 can handle larger values, so less conservative
    int64_t max_coord = (scale_factor * 35 / 10);
    int64_t max_square = max_coord * max_coord;

    if (max_square > (INT64_MAX / 4)) {
        return 2;  // High overflow risk
    } else if (max_square > (INT64_MAX / 16)) {
        return 1;  // Medium overflow risk
    } else {
        return 0;  // Low overflow risk
    }
}

/* Calculate precision score (lower = more accurate) */
uint32_t calculate_precision_score(uint64_t fixed_result, uint64_t double_result) {
    if (double_result == 0) return 0;  // Avoid division by zero

    if (double_result > fixed_result) {
        return (uint32_t)((double_result - fixed_result) * 1000 / double_result);
    } else {
        return (uint32_t)((fixed_result - double_result) * 1000 / double_result);
    }
}

/* Enable DWT for cycle-accurate timing on F4 */
void enable_dwt(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Get current DWT cycle count */
uint32_t get_dwt_cycles(void) {
    return DWT->CYCCNT;
}

// Mandelbrot using configurable fixed-point scaling factor
uint64_t calculate_mandelbrot_fixed_point_scaled(int width, int height, int max_iterations, int64_t scale_factor) {
    uint64_t mandelbrot_checksum = 0;

    for (int y = 0; y <= height - 1; y++) {
        for (int x = 0; x <= width - 1; x++) {
            int64_t x0 = (x * scale_factor / width) * 35 / 10 - 25 * scale_factor / 10;
            int64_t y0 = (y * scale_factor / height) * 2 - scale_factor;
            int64_t xi = 0;
            int64_t yi = 0;
            int64_t iteration = 0;

            while ((iteration < max_iterations) && (xi * xi + yi * yi <= 4 * scale_factor * scale_factor)) {
                int64_t xi_temp = (xi * xi - yi * yi) / scale_factor;
                yi = (2 * xi * yi) / scale_factor + y0;
                xi = xi_temp + x0;
                ++iteration;
            }
            mandelbrot_checksum += iteration;
        }
    }
    return mandelbrot_checksum;
}

// Reference double precision implementation (uses FPU on F4)
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {
    uint64_t mandelbrot_checksum = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double x0 = ((double)x / (double)width) * 3.5 - 2.5;
            double y0 = ((double)y / (double)height) * 2.0 - 1.0;
            double xi = 0.0;
            double yi = 0.0;
            int iteration = 0;

            while (iteration < max_iterations && (xi * xi + yi * yi <= 4.0)) {
                double xi_temp = xi * xi - yi * yi + x0;
                yi = 2.0 * xi * yi + y0;
                xi = xi_temp;
                iteration++;
            }
            mandelbrot_checksum += iteration;
        }
    }
    return mandelbrot_checksum;
}
/* USER CODE END 4 */

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
