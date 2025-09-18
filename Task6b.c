/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Task 6 - Compiler Optimizations Testing (STM32F0)
*
* Instructions for use:
* 1. Compile with different optimization levels by changing Makefile OPT = line:
*    - OPT = -O0  (No optimization)
*    - OPT = -O1  (Basic optimization)
*    - OPT = -O2  (Standard optimization)
*    - OPT = -O3  (Aggressive optimization)
*    - OPT = -Os  (Size optimization)
* 2. Or use IDE: Project → Properties → C/C++ Build → Settings → MCU Settings
* 3. Record binary size and runtime for each optimization level
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

// Results storage for compiler optimization analysis
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time_ms[5];
volatile uint64_t checksum_results[5];

// Optimization level tracking (set manually based on build configuration)
volatile uint32_t optimization_level = 0; // 0=O0, 1=O1, 2=O2, 3=O3, 4=Os
volatile uint32_t binary_size_estimate = 0; // Set this manually after checking build output

// Test configuration
volatile int test_mode = 0; // Use fixed-point for F0 (no FPU)
volatile int test_algorithm = 0; // 0=fixed-point, 1=double

// SysTick timing for F0
volatile uint32_t systick_start = 0;
volatile uint32_t systick_end = 0;
volatile uint32_t systick_cycles[5];

// Compiler optimization test info
const char* optimization_names[] = {
    "O0 - No optimization",
    "O1 - Basic optimization",
    "O2 - Standard optimization",
    "O3 - Aggressive optimization",
    "Os - Size optimization"
};

// Test results for optimization-sensitive functions
volatile uint32_t optimization_test_result = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
uint32_t Get_SysTick_Cycles(void);
void run_optimization_test(void);
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

// Test functions to measure optimization impact on F0
void optimization_test_function_1(void);
void optimization_test_function_2(void);

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* USER CODE BEGIN 2 */

    // Flash startup pattern to indicate test start
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(300);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(300);
    }

    // Run the main optimization test
    run_optimization_test();

    // Run additional optimization-sensitive functions
    optimization_test_function_1();
    optimization_test_function_2();

    // Flash completion pattern
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(150);
    }

    // Set breakpoint here to examine results
    volatile int debug_complete = 1; // Breakpoint marker
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // STM32F0 Compiler optimization test results:
        // execution_time_ms[0-4]   - Runtime for each image size
        // checksum_results[0-4]    - Verification checksums
        // systick_cycles[0-4]      - SysTick cycles for timing
        // optimization_test_result - Additional test function result
        //
        // IMPORTANT: To complete this task:
        // 1. Build this code with different -O flags (-O0, -O1, -O2, -O3, -Os)
        // 2. Record binary size from build output for each optimization level
        // 3. Record execution times for each optimization level
        // 4. Compare with STM32F4 results to analyze optimization effectiveness
        //
        // Expected F0-specific behaviors:
        // - O0: Very slow due to no optimization + software floating point
        // - O1: Significant improvement for integer operations
        // - O2: Good performance for fixed-point arithmetic
        // - O3: May provide best performance for computation-heavy tasks
        // - Os: Smallest binary, important for F0's limited flash memory
        //
        // F0 vs F4 comparison points:
        // - F0 benefits more from optimization due to software floating point
        // - Size optimization (-Os) more critical on F0 due to memory constraints
        // - Fixed-point vs floating-point optimization differences
    }
}

void run_optimization_test(void) {
    // Test all image sizes with current optimization level
    for (int i = 0; i < 5; i++) {
        // LED indication
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

        // Start timing
        systick_start = Get_SysTick_Cycles();
        start_time = HAL_GetTick();

        // Run Mandelbrot calculation
        if (test_algorithm == 0) {
            checksum_results[i] = calculate_mandelbrot_fixed_point_arithmetic(
                width[i], height[i], MAX_ITER);
        } else {
            // Note: Double precision is very slow on F0
            checksum_results[i] = calculate_mandelbrot_double(
                width[i], height[i], MAX_ITER);
        }

        // End timing
        end_time = HAL_GetTick();
        systick_end = Get_SysTick_Cycles();

        execution_time_ms[i] = end_time - start_time;

        // Calculate SysTick cycles
        if (systick_end >= systick_start) {
            systick_cycles[i] = systick_end - systick_start;
        } else {
            // Handle SysTick overflow
            systick_cycles[i] = (0xFFFFFF - systick_start) + systick_end;
        }

        // LED indication
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(300);
    }
}

// Test function 1: Integer arithmetic optimization test
void optimization_test_function_1(void) {
    volatile uint32_t result = 0;
    systick_start = Get_SysTick_Cycles();

    // Loop-heavy integer computation (should benefit significantly from optimization)
    for (int i = 0; i < 5000; i++) {
        for (int j = 0; j < 50; j++) {
            result += i * j;
            result = result ^ (result >> 1);
            result = (result << 2) + (result >> 3);
        }
    }

    systick_end = Get_SysTick_Cycles();

    if (systick_end >= systick_start) {
        optimization_test_result = systick_end - systick_start;
    } else {
        optimization_test_result = (0xFFFFFF - systick_start) + systick_end;
    }
}

// Test function 2: Software floating-point test (very optimization sensitive on F0)
void optimization_test_function_2(void) {
    volatile float result = 1.0f;
    uint32_t cycles_start = Get_SysTick_Cycles();

    // Floating-point operations (software implementation on F0)
    for (int i = 1; i < 500; i++) {
        result = result * 1.001f;
        result = result + (float)i / 1000.0f;
        if (result > 100.0f) result = result / 2.0f;
    }

    uint32_t cycles_end = Get_SysTick_Cycles();

    // Add to optimization test result
    if (cycles_end >= cycles_start) {
        optimization_test_result += cycles_end - cycles_start;
    } else {
        optimization_test_result += (0xFFFFFF - cycles_start) + cycles_end;
    }

    // Store result to prevent optimization elimination
    optimization_test_result += (uint32_t)(result * 100);
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
// Mandelbrot using fixed point arithmetic (preferred for F0)
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations) {
    uint64_t mandelbrot_checksum = 0;
    int64_t scale_factor = 1000000;

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

// Mandelbrot using double precision (very slow on F0 - software floating point)
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
