/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Task 3 - Extended Execution Time Measurement
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

// Global variables for extended measurements
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time_ms[5];     // Wall-clock time in milliseconds
volatile uint32_t cpu_cycles[5];            // CPU clock cycles
volatile uint32_t pixels_per_second[5];     // Throughput in pixels/sec
volatile uint64_t checksum[5];              // Checksums for verification

// DWT cycle counter variables
volatile uint32_t cyc_start = 0;
volatile uint32_t cyc_end = 0;

// System clock frequency for calculations (120MHz)
#define SYSTEM_CLOCK_HZ 120000000

// Test mode: 0 = fixed-point, 1 = double
volatile int test_mode = 1; // Use double precision
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void DWT_Init(void);
uint32_t DWT_GetCycles(void);
void DWT_ResetCycles(void);
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    DWT_Init();

    /* USER CODE BEGIN 2 */
    
    // Run all 5 test cases with extended measurements
    for (int j = 0; j < 5; j++) {
        // Turn on LED 0 to signify the start of the operation
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

        // Reset and start cycle counter
        DWT_ResetCycles();
        cyc_start = DWT_GetCycles();
        
        // Record the start time (wall-clock)
        start_time = HAL_GetTick();

        // Call the Mandelbrot Function
        if (test_mode == 0) {
            // Fixed-point arithmetic
            checksum[j] = calculate_mandelbrot_fixed_point_arithmetic(width[j], height[j], MAX_ITER);
        } else {
            // Double precision
            checksum[j] = calculate_mandelbrot_double(width[j], height[j], MAX_ITER);
        }

        // Record the end time and cycles
        end_time = HAL_GetTick();
        cyc_end = DWT_GetCycles();

        // Calculate measurements
        execution_time_ms[j] = end_time - start_time;
        cpu_cycles[j] = cyc_end - cyc_start;
        
        // Calculate throughput (pixels per second)
        uint32_t total_pixels = width[j] * height[j];
        if (execution_time_ms[j] > 0) {
            pixels_per_second[j] = (total_pixels * 1000) / execution_time_ms[j];
        } else {
            // For very fast operations, use cycle-based calculation
            if (cpu_cycles[j] > 0) {
                uint32_t execution_time_us = (cpu_cycles[j] * 1000000) / SYSTEM_CLOCK_HZ;
                if (execution_time_us > 0) {
                    pixels_per_second[j] = (total_pixels * 1000000) / execution_time_us;
                } else {
                    pixels_per_second[j] = 0xFFFFFFFF; // Very high throughput
                }
            } else {
                pixels_per_second[j] = 0;
            }
        }

        // Turn on LED 1 to signify the end of the operation
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

        // Hold the LEDs on for a 1s delay
        HAL_Delay(1000);

        // Turn off the LEDs
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

        // Small delay between tests
        HAL_Delay(500);
    }

    // Flash both LEDs to indicate all tests complete
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(300);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(300);
    }

    // Set breakpoint here to examine all results
    volatile int debug_complete = 1; // Breakpoint marker
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // Extended measurement results are stored in:
        // execution_time_ms[0-4]  - Wall-clock time in milliseconds
        // cpu_cycles[0-4]         - CPU clock cycles
        // pixels_per_second[0-4]  - Throughput calculation
        // checksum[0-4]           - Verification checksums
        //
        // Additional calculations you can derive:
        // - Cycles per pixel = cpu_cycles[i] / (width[i] * height[i])
        // - MHz efficiency = (cpu_cycles[i] / 1000000) / (execution_time_ms[i] / 1000)
        // - Instructions per pixel (if you know the instruction count)
    }
}

/* DWT (Data Watchpoint and Trace) Functions for Precise Timing */
void DWT_Init(void) {
    // Enable trace
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    // Enable cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // Reset cycle counter
    DWT->CYCCNT = 0;
}

uint32_t DWT_GetCycles(void) {
    return DWT->CYCCNT;
}

void DWT_ResetCycles(void) {
    DWT->CYCCNT = 0;
}

/* System Clock Configuration for STM32F446RC - 120MHz */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;  // Target 120MHz as specified
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
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
// Mandelbrot using fixed point arithmetic
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

// Mandelbrot using double precision
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