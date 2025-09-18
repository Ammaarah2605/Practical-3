/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Task 7 - Fixed Point Arithmetic Scaling Factor Testing (STM32F0)
* 
* Tests different scaling factors (10^3, 10^4, 10^6) to analyze:
* - Effect on precision
* - Risk of overflow 
* - Execution speed
* STM32F0 specific considerations:
* - Limited RAM for larger scaling factors
* - No FPU, so fixed-point is especially important
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_rcc.h"

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#define MAX_ITER 100

// Image dimensions from Practical 1B
uint32_t height[5] = {128, 160, 192, 224, 256};
uint32_t width[5] = {128, 160, 192, 224, 256};

// Fixed-point scaling factors to test (conservative for F0)
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
    {10000000,  "10^7 - Very high precision",     23, 2}
    // Note: 10^8 removed for F0 due to higher overflow risk
};

#define NUM_SCALING_TESTS (sizeof(scaling_tests)/sizeof(scaling_tests[0]))

// Reduced array sizes for F0's limited RAM
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time_ms[25];    // 5 scaling factors × 5 image sizes
volatile uint64_t checksum_results[25];
volatile uint32_t overflow_detected[25];
volatile uint32_t precision_score[25];

// Reference results (double precision for comparison)
volatile uint64_t reference_checksum[5];
volatile uint32_t reference_time[5];

// SysTick timing for F0
volatile uint32_t systick_start = 0;
volatile uint32_t systick_end = 0;
volatile uint32_t systick_cycles[25];

// Current test tracking
volatile uint32_t current_scale_test = 0;
volatile uint32_t current_image = 0;
volatile uint32_t test_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
uint32_t Get_SysTick_Cycles(void);
void run_reference_tests(void);
void run_scaling_factor_tests(void);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_fixed_point_scaled(int width, int height, int max_iterations, int64_t scale_factor);
uint32_t check_overflow_risk_f0(int64_t scale_factor, int width, int height);
uint32_t calculate_precision_score(uint64_t fixed_result, uint64_t double_result);

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* USER CODE BEGIN 2 */
    
    // Initialize result arrays
    for (int i = 0; i < 25; i++) {
        execution_time_ms[i] = 0;
        checksum_results[i] = 0;
        systick_cycles[i] = 0;
        overflow_detected[i] = 0;
        precision_score[i] = 0;
    }
    
    // Flash startup pattern
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
    
    // Step 1: Run reference tests (double precision - slow on F0!)
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
    while (1)
    {
        // STM32F0 Fixed-point scaling factor test results:
        // execution_time_ms[0-24]   - Execution times for all tests
        // checksum_results[0-24]    - Verification checksums
        // systick_cycles[0-24]      - SysTick cycles for timing
        // overflow_detected[0-24]   - Overflow detection flags
        // precision_score[0-24]     - Accuracy compared to double precision
        //
        // Data layout (test_index = scale_test * 5 + image_size):
        // Indices 0-4:   Scale factor 10^3, image sizes 128x128 to 256x256
        // Indices 5-9:   Scale factor 10^4, image sizes 128x128 to 256x256
        // Indices 10-14: Scale factor 10^5, image sizes 128x128 to 256x256
        // Indices 15-19: Scale factor 10^6, image sizes 128x128 to 256x256
        // Indices 20-24: Scale factor 10^7, image sizes 128x128 to 256x256
        //
        // STM32F0 specific observations:
        // - Fixed-point arithmetic is much faster than software floating point
        // - Higher scaling factors show diminishing returns due to overhead
        // - Memory constraints may limit very high precision
        // - Optimal scaling factor balances precision vs integer overflow risk
        // - Compare with F4 results to see FPU vs fixed-point tradeoffs
    }
}

/* Run reference tests for precision comparison (slow on F0) */
void run_reference_tests(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    
    for (int i = 0; i < 5; i++) {
        start_time = HAL_GetTick();
        systick_start = Get_SysTick_Cycles();
        
        // Note: This will be VERY slow on F0 due to software floating point
        reference_checksum[i] = calculate_mandelbrot_double(width[i], height[i], MAX_ITER);
        
        systick_end = Get_SysTick_Cycles();
        end_time = HAL_GetTick();
        
        reference_time[i] = end_time - start_time;
        
        // LED indication for progress (double precision is slow on F0)
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        HAL_Delay(200);
    }
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(1000);
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
        HAL_Delay(500);
        
        for (int img = 0; img < 5; img++) {
            current_image = img;
            test_index = scale_idx * 5 + img;
            
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            
            // Check for potential overflow before running test
            overflow_detected[test_index] = check_overflow_risk_f0(
                scaling_tests[scale_idx].scale_factor, width[img], height[img]);
            
            // Start timing
            start_time = HAL_GetTick();
            systick_start = Get_SysTick_Cycles();
            
            // Run fixed-point Mandelbrot with current scaling factor
            checksum_results[test_index] = calculate_mandelbrot_fixed_point_scaled(
                width[img], height[img], MAX_ITER, scaling_tests[scale_idx].scale_factor);
            
            // End timing
            systick_end = Get_SysTick_Cycles();
            end_time = HAL_GetTick();
            
            execution_time_ms[test_index] = end_time - start_time;
            
            // Calculate SysTick cycles
            if (systick_end >= systick_start) {
                systick_cycles[test_index] = systick_end - systick_start;
            } else {
                // Handle SysTick overflow
                systick_cycles[test_index] = (0xFFFFFF - systick_start) + systick_end;
            }
            
            // Calculate precision score compared to double precision reference
            precision_score[test_index] = calculate_precision_score(
                checksum_results[test_index], reference_checksum[img]);
            
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_Delay(200);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_Delay(100);
        }
        
        // Pause between scaling factor groups
        HAL_Delay(1000);
    }
}

/* Check potential overflow risk for F0 (more conservative) */
uint32_t check_overflow_risk_f0(int64_t scale_factor, int width, int height) {
    // More conservative overflow detection for F0
    int64_t max_coord = (scale_factor * 35 / 10);
    int64_t max_square = max_coord * max_coord;
    
    // F0 has less computational headroom, so be more conservative
    if (max_square > (INT64_MAX / 16)) {
        return 2;  // High overflow risk
    } else if (max_square > (INT64_MAX / 128)) {
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

/* SysTick Functions for F0 timing */
uint32_t Get_SysTick_Cycles(void) {
    return (0xFFFFFF - SysTick->VAL);
}

/* System Clock Configuration for STM32F0 - 48MHz */
void SystemClock_Config(void)
{
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
Error_Handler();
}
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
Error_Handler();
}
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
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

// Reference double precision implementation (VERY slow on F0!)
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

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */