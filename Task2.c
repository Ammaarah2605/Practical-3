/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Task 2 - Impact of Maximum Iteration Variable
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

// Image dimensions from Practical 1B
uint32_t height[5] = {128, 160, 192, 224, 256};
uint32_t width[5] = {128, 160, 192, 224, 256};

// MAX_ITER values to test (at least 5 values between 100 and 1000)
uint32_t max_iter_values[5] = {100, 250, 500, 750, 1000};

// Global variables for results (25 results: 5 MAX_ITER × 5 image sizes)
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time[25];
volatile uint64_t checksum[25];

// Current test parameters (for debugging)
volatile uint32_t current_max_iter = 0;
volatile uint32_t current_width = 0;
volatile uint32_t current_height = 0;
volatile uint32_t test_number = 0;

// Test mode: 0 = fixed-point, 1 = double
volatile int test_mode = 1; // Use double precision
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* USER CODE BEGIN 2 */
    
    test_number = 0;
    
    // Test each MAX_ITER value with each image size
    for (int iter_idx = 0; iter_idx < 5; iter_idx++) {
        for (int size_idx = 0; size_idx < 5; size_idx++) {
            
            // Update current test parameters for debugging
            current_max_iter = max_iter_values[iter_idx];
            current_width = width[size_idx];
            current_height = height[size_idx];
            
            // Turn on LED 0 to signify the start of the operation
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

            // Record the start time
            start_time = HAL_GetTick();

            // Call the Mandelbrot Function with current MAX_ITER and image size
            if (test_mode == 0) {
                // Fixed-point arithmetic
                checksum[test_number] = calculate_mandelbrot_fixed_point_arithmetic(
                    width[size_idx], height[size_idx], max_iter_values[iter_idx]);
            } else {
                // Double precision
                checksum[test_number] = calculate_mandelbrot_double(
                    width[size_idx], height[size_idx], max_iter_values[iter_idx]);
            }

            // Record the end time
            end_time = HAL_GetTick();

            // Calculate the execution time
            execution_time[test_number] = end_time - start_time;

            // Turn on LED 1 to signify the end of the operation
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

            // Brief delay to show completion
            HAL_Delay(200);

            // Turn off the LEDs
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

            // Small delay between tests
            HAL_Delay(300);
            
            test_number++;
        }
        
        // Longer delay between MAX_ITER groups
        HAL_Delay(1000);
    }

    // Flash both LEDs to indicate all tests complete
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(500);
    }

    // Set breakpoint here to examine all results
    volatile int debug_complete = 1; // Breakpoint marker
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // Results are stored in:
        // execution_time[0-24] and checksum[0-24]
        // 
        // Data layout:
        // Index 0-4:   MAX_ITER=100,  image sizes 128x128 to 256x256
        // Index 5-9:   MAX_ITER=250,  image sizes 128x128 to 256x256
        // Index 10-14: MAX_ITER=500,  image sizes 128x128 to 256x256
        // Index 15-19: MAX_ITER=750,  image sizes 128x128 to 256x256
        // Index 20-24: MAX_ITER=1000, image sizes 128x128 to 256x256
    }
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