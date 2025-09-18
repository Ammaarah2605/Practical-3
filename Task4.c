/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Task 4 - Scalability Test (up to Full HD)
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

// Scalability test: gradually increasing image sizes up to Full HD
typedef struct {
    uint32_t width;
    uint32_t height;
    const char* description;
} ImageSize_t;

// Test image sizes from small to Full HD
ImageSize_t test_sizes[] = {
    {128, 128, "128x128 (Baseline)"},
    {256, 256, "256x256 (Small)"},
    {512, 512, "512x512 (Medium)"},
    {640, 480, "640x480 (VGA)"},
    {800, 600, "800x600 (SVGA)"},
    {1024, 768, "1024x768 (XGA)"},
    {1280, 720, "1280x720 (HD)"},
    {1600, 900, "1600x900 (HD+)"},
    {1920, 1080, "1920x1080 (Full HD)"}
};

#define NUM_TEST_SIZES (sizeof(test_sizes)/sizeof(test_sizes[0]))

// Global variables for scalability results
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time_ms[NUM_TEST_SIZES];
volatile uint32_t pixels_per_second[NUM_TEST_SIZES];
volatile uint64_t checksum[NUM_TEST_SIZES];
volatile uint32_t memory_status[NUM_TEST_SIZES]; // 0=OK, 1=Chunked, 2=Failed

// Chunking parameters for large images
#define MAX_CHUNK_PIXELS 100000  // Maximum pixels per chunk
volatile uint32_t chunks_used[NUM_TEST_SIZES];

// Test mode: 0 = fixed-point, 1 = double
volatile int test_mode = 1; // Use double precision

// Current test status
volatile uint32_t current_test = 0;
volatile uint32_t tests_completed = 0;
volatile uint32_t tests_failed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_chunked(int width, int height, int max_iterations, int chunk_size);

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* USER CODE BEGIN 2 */
    
    // Initialize status arrays
    for (int i = 0; i < NUM_TEST_SIZES; i++) {
        execution_time_ms[i] = 0;
        pixels_per_second[i] = 0;
        checksum[i] = 0;
        memory_status[i] = 0;
        chunks_used[i] = 0;
    }
    
    // Run scalability tests
    for (int i = 0; i < NUM_TEST_SIZES; i++) {
        current_test = i;
        uint32_t width = test_sizes[i].width;
        uint32_t height = test_sizes[i].height;
        uint32_t total_pixels = width * height;
        
        // Turn on LED 0 to signify the start of the operation
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

        // Check if image is too large for direct processing
        if (total_pixels > MAX_CHUNK_PIXELS) {
            // Use chunked processing
            memory_status[i] = 1; // Chunked
            
            // Calculate chunk dimensions
            uint32_t chunk_height = MAX_CHUNK_PIXELS / width;
            if (chunk_height == 0) chunk_height = 1;
            if (chunk_height > height) chunk_height = height;
            
            chunks_used[i] = (height + chunk_height - 1) / chunk_height; // Ceiling division
            
            // Record the start time
            start_time = HAL_GetTick();
            
            // Process in chunks
            checksum[i] = calculate_mandelbrot_chunked(width, height, MAX_ITER, chunk_height);
            
        } else {
            // Direct processing
            memory_status[i] = 0; // OK
            chunks_used[i] = 1;
            
            // Record the start time
            start_time = HAL_GetTick();
            
            // Call the Mandelbrot Function directly
            if (test_mode == 0) {
                checksum[i] = calculate_mandelbrot_fixed_point_arithmetic(width, height, MAX_ITER);
            } else {
                checksum[i] = calculate_mandelbrot_double(width, height, MAX_ITER);
            }
        }

        // Record the end time
        end_time = HAL_GetTick();
        execution_time_ms[i] = end_time - start_time;

        // Calculate throughput
        if (execution_time_ms[i] > 0) {
            pixels_per_second[i] = (total_pixels * 1000) / execution_time_ms[i];
        } else {
            pixels_per_second[i] = 0xFFFFFFFF; // Very high throughput
        }

        // Turn on LED 1 to signify the end of the operation
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

        tests_completed++;

        // Variable delay based on test size (longer tests get shorter delays)
        uint32_t delay_time = 1000;
        if (execution_time_ms[i] > 5000) delay_time = 500;
        if (execution_time_ms[i] > 10000) delay_time = 200;
        HAL_Delay(delay_time);

        // Turn off the LEDs
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

        // Brief pause between tests
        HAL_Delay(300);
        
        // For very large images, give user feedback
        if (total_pixels > 500000) {
            // Flash both LEDs to show we're still alive
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        }
    }

    // Flash completion pattern (3 quick flashes)
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(200);
    }

    // Set breakpoint here to examine all results
    volatile int debug_complete = 1; // Breakpoint marker
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // Scalability test results are stored in:
        // execution_time_ms[0-8]   - Execution time for each image size
        // pixels_per_second[0-8]   - Throughput for each image size
        // checksum[0-8]            - Verification checksums
        // memory_status[0-8]       - 0=Direct, 1=Chunked, 2=Failed
        // chunks_used[0-8]         - Number of chunks used for processing
        //
        // test_sizes[] contains the image dimensions and descriptions
        //
        // Analysis suggestions:
        // - Plot throughput vs image size
        // - Check if throughput decreases with larger images
        // - Identify memory limitations
        // - Compare chunked vs direct processing efficiency
    }
}

/* Chunked Mandelbrot calculation for large images */
uint64_t calculate_mandelbrot_chunked(int width, int height, int max_iterations, int chunk_size) {
    uint64_t total_checksum = 0;
    
    // Process image in horizontal strips
    for (int y_start = 0; y_start < height; y_start += chunk_size) {
        int chunk_height = chunk_size;
        if (y_start + chunk_height > height) {
            chunk_height = height - y_start;
        }
        
        // Calculate this chunk
        for (int y = y_start; y < y_start + chunk_height; y++) {
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
                total_checksum += iteration;
            }
        }
        
        // Optional: Flash LED to show progress on very large images
        if (width * height > 1000000) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        }
    }
    
    return total_checksum;
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