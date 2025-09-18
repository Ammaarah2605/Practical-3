/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Task 5 - FPU Impact Testing (Float vs Double)
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#define MAX_ITER 100

// Image dimensions from Practical 1B
uint32_t height[5] = {128, 160, 192, 224, 256};
uint32_t width[5] = {128, 160, 192, 224, 256};

// Test configuration
typedef struct {
    uint8_t precision_type;  // 0=float, 1=double
    uint8_t fpu_enabled;     // 0=disabled, 1=enabled (simulated in software)
    const char* description;
} FPUTest_t;

// Test scenarios
FPUTest_t test_scenarios[] = {
    {1, 1, "Double precision with FPU"},
    {0, 1, "Float precision with FPU"},
    {1, 0, "Double precision without FPU"},  // Simulated
    {0, 0, "Float precision without FPU"}    // Simulated
};

#define NUM_TEST_SCENARIOS (sizeof(test_scenarios)/sizeof(test_scenarios[0]))
#define TOTAL_TESTS (NUM_TEST_SCENARIOS * 5)  // 4 scenarios × 5 image sizes

// Global variables for FPU test results
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time_ms[TOTAL_TESTS];
volatile uint64_t checksum_results[TOTAL_TESTS];
volatile uint32_t accuracy_differences[TOTAL_TESTS];  // Difference from reference
volatile float speedup_factors[TOTAL_TESTS];          // Speedup compared to baseline

// Reference results (double precision with FPU)
volatile uint64_t reference_checksum[5];
volatile uint32_t reference_time[5];

// Current test tracking
volatile uint32_t current_scenario = 0;
volatile uint32_t current_image = 0;
volatile uint32_t test_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
uint64_t calculate_mandelbrot_double_fpu(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_float_fpu(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double_no_fpu(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_float_no_fpu(int width, int height, int max_iterations);
void run_reference_tests(void);
void run_fpu_comparison_tests(void);

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* USER CODE BEGIN 2 */
    
    // Initialize result arrays
    for (int i = 0; i < TOTAL_TESTS; i++) {
        execution_time_ms[i] = 0;
        checksum_results[i] = 0;
        accuracy_differences[i] = 0;
        speedup_factors[i] = 0.0f;
    }
    
    // Flash startup pattern
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(300);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(300);
    }
    
    // Step 1: Run reference tests (double precision with FPU)
    run_reference_tests();
    
    // Step 2: Run all FPU comparison tests
    run_fpu_comparison_tests();
    
    // Flash completion pattern (5 quick flashes)
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(150);
    }

    // Set breakpoint here to examine all results
    volatile int debug_complete = 1; // Breakpoint marker
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // FPU Impact test results are stored in:
        // execution_time_ms[0-19]   - Execution times for all tests
        // checksum_results[0-19]    - Verification checksums
        // accuracy_differences[0-19]- Accuracy differences from reference
        // speedup_factors[0-19]     - Speedup factors
        //
        // Data layout (test_index = scenario * 5 + image_size):
        // Indices 0-4:   Double precision with FPU (reference)
        // Indices 5-9:   Float precision with FPU
        // Indices 10-14: Double precision without FPU
        // Indices 15-19: Float precision without FPU
        //
        // Analysis suggestions:
        // - Compare float vs double precision accuracy
        // - Measure FPU performance impact
        // - Calculate speedup factors
        // - Identify precision vs performance tradeoffs
    }
}

/* Run reference tests to establish baseline performance */
void run_reference_tests(void) {
    for (int i = 0; i < 5; i++) {
        current_scenario = 0;  // Reference scenario
        current_image = i;
        
        // Turn on LED 0 for reference tests
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        
        // Record start time
        start_time = HAL_GetTick();
        
        // Run reference test (double precision with FPU)
        reference_checksum[i] = calculate_mandelbrot_double_fpu(width[i], height[i], MAX_ITER);
        
        // Record end time
        end_time = HAL_GetTick();
        reference_time[i] = end_time - start_time;
        
        // Store in results array
        checksum_results[i] = reference_checksum[i];
        execution_time_ms[i] = reference_time[i];
        accuracy_differences[i] = 0;  // Reference has no difference
        speedup_factors[i] = 1.0f;    // Reference has 1x speedup
        
        // LED indication
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(300);
    }
}

/* Run all FPU comparison tests */
void run_fpu_comparison_tests(void) {
    // Start from scenario 1 (skip reference scenario 0)
    for (int scenario = 1; scenario < NUM_TEST_SCENARIOS; scenario++) {
        for (int img = 0; img < 5; img++) {
            current_scenario = scenario;
            current_image = img;
            test_index = scenario * 5 + img;
            
            // Indicate test type with LED pattern
            if (test_scenarios[scenario].precision_type == 0) {
                // Float tests - single flash
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
                HAL_Delay(100);
            } else {
                // Double tests - double flash
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
                HAL_Delay(50);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
                HAL_Delay(100);
            }
            
            // Record start time
            start_time = HAL_GetTick();
            
            // Run appropriate test
            uint64_t result = 0;
            if (test_scenarios[scenario].precision_type == 0) {
                // Float precision
                if (test_scenarios[scenario].fpu_enabled) {
                    result = calculate_mandelbrot_float_fpu(width[img], height[img], MAX_ITER);
                } else {
                    result = calculate_mandelbrot_float_no_fpu(width[img], height[img], MAX_ITER);
                }
            } else {
                // Double precision
                if (test_scenarios[scenario].fpu_enabled) {
                    result = calculate_mandelbrot_double_fpu(width[img], height[img], MAX_ITER);
                } else {
                    result = calculate_mandelbrot_double_no_fpu(width[img], height[img], MAX_ITER);
                }
            }
            
            // Record end time
            end_time = HAL_GetTick();
            execution_time_ms[test_index] = end_time - start_time;
            checksum_results[test_index] = result;
            
            // Calculate accuracy difference from reference
            if (reference_checksum[img] > result) {
                accuracy_differences[test_index] = (uint32_t)(reference_checksum[img] - result);
            } else {
                accuracy_differences[test_index] = (uint32_t)(result - reference_checksum[img]);
            }
            
            // Calculate speedup factor
            if (execution_time_ms[test_index] > 0) {
                speedup_factors[test_index] = (float)reference_time[img] / (float)execution_time_ms[test_index];
            } else {
                speedup_factors[test_index] = 100.0f;  // Very fast
            }
            
            // Success indication
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_Delay(200);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_Delay(200);
        }
        
        // Pause between scenarios
        HAL_Delay(1000);
    }
}

/* System Clock Configuration for STM32F446RC - 120MHz */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;  
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;             // <- 4MHz * 12 = 48MHz
  
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
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
// Mandelbrot using double precision with FPU
uint64_t calculate_mandelbrot_double_fpu(int width, int height, int max_iterations) {
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

// Mandelbrot using float precision with FPU
uint64_t calculate_mandelbrot_float_fpu(int width, int height, int max_iterations) {
    uint64_t mandelbrot_checksum = 0;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float x0 = ((float)x / (float)width) * 3.5f - 2.5f;
            float y0 = ((float)y / (float)height) * 2.0f - 1.0f;
            float xi = 0.0f;
            float yi = 0.0f;
            int iteration = 0;
            
            while (iteration < max_iterations && (xi * xi + yi * yi <= 4.0f)) {
                float xi_temp = xi * xi - yi * yi + x0;
                yi = 2.0f * xi * yi + y0;
                xi = xi_temp;
                iteration++;
            }
            mandelbrot_checksum += iteration;
        }
    }
    return mandelbrot_checksum;
}

// Mandelbrot using double precision without FPU (simulated with fixed-point)
uint64_t calculate_mandelbrot_double_no_fpu(int width, int height, int max_iterations) {
    // Simulate "no FPU" by using fixed-point arithmetic with high precision
    uint64_t mandelbrot_checksum = 0;
    int64_t scale_factor = 10000000;  // Higher precision for double simulation
    
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

// Mandelbrot using float precision without FPU (simulated with fixed-point)
uint64_t calculate_mandelbrot_float_no_fpu(int width, int height, int max_iterations) {
    // Simulate "no FPU" by using fixed-point arithmetic with lower precision
    uint64_t mandelbrot_checksum = 0;
    int32_t scale_factor = 1000000;  // Lower precision for float simulation
    
    for (int y = 0; y <= height - 1; y++) {
        for (int x = 0; x <= width - 1; x++) {
            int32_t x0 = (x * scale_factor / width) * 35 / 10 - 25 * scale_factor / 10;
            int32_t y0 = (y * scale_factor / height) * 2 - scale_factor;
            int32_t xi = 0;
            int32_t yi = 0;
            int32_t iteration = 0;
            
            while ((iteration < max_iterations) && (xi * xi + yi * yi <= 4 * scale_factor * scale_factor)) {
                int32_t xi_temp = (xi * xi - yi * yi) / scale_factor;
                yi = (2 * xi * yi) / scale_factor + y0;
                xi = xi_temp + x0;
                ++iteration;
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