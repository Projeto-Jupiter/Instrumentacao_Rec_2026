/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---- Calibration -----------------------------------------------------------
// HX711_SCALE is in counts per gram:
//   grams   = (filtered_counts - offset) / HX711_SCALE
//   force_N = grams / 1000 * g
#define HX711_SCALE          9.009f      // counts / gram
#define GRAVITY_MS2          9.80665f    // m/s^2
#define GRAM_TO_NEWTON       (GRAVITY_MS2 / 1000.0f)

// ---- Filtering --------------------------------------------------------------
#define MEDIAN_N             5       // sliding median window (odd): kills spikes
#define EMA_ALPHA            0.15f   // 0..1, smaller = smoother but slower

// ---- Zeroing ----------------------------------------------------------------
#define TARE_SAMPLES         20      // samples averaged for a manual/startup tare
#define ZERO_DEADBAND_N      0.05f   // |force| below this prints as exactly 0
#define AUTO_ZERO_ENABLE     1       // 1 = slowly track zero drift
#define AUTO_ZERO_WINDOW_N   0.20f   // only track if reading is within +-this of zero
#define AUTO_ZERO_RATE       0.02f   // how fast the offset follows (per sample)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
// Set by CDC_Receive_FS() in usbd_cdc_if.c (see instructions)
volatile uint8_t usb_rx_cmd = 0;

static int32_t median_buf[MEDIAN_N];
static int     median_count = 0;
static int     median_idx   = 0;

static float   offset_counts = 0.0f;
static float   ema_value     = 0.0f;
static int     ema_ready     = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    // CDC_Transmit_FS returns USBD_BUSY if the previous packet isn't done yet
    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY) {
        if (HAL_GetTick() - t0 > 20) break;
    }
    return len;
}

// Returns 1 on success, 0 on timeout (HX711 not responding)
int HX711_Read(int32_t *out) {
    uint32_t t0 = HAL_GetTick();

    // Wait for the HX711 to be ready (DT goes low)
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 1000) return 0;
    }

    uint32_t count = 0;

    // SCK held high > 60 us puts the HX711 into power-down, so an interrupt
    // (e.g. USB) in the middle of the read would corrupt it. Block IRQs briefly.
    __disable_irq();

    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // SCK High
        for (volatile int d = 0; d < 10; d++) {}

        count <<= 1;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // SCK Low
        for (volatile int d = 0; d < 10; d++) {}

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) {
            count++;
        }
    }

    // 25th pulse: next conversion = Channel A, gain 128
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    for (volatile int d = 0; d < 10; d++) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

    __enable_irq();

    // 24-bit two's complement -> 32-bit signed
    if (count & 0x00800000UL) {
        count |= 0xFF000000UL;
    }

    *out = (int32_t)count;
    return 1;
}

// Median of the last MEDIAN_N samples (removes single-sample spikes)
static int32_t Median_Push(int32_t sample) {
    median_buf[median_idx] = sample;
    median_idx = (median_idx + 1) % MEDIAN_N;
    if (median_count < MEDIAN_N) median_count++;

    int32_t tmp[MEDIAN_N];
    memcpy(tmp, median_buf, sizeof(int32_t) * median_count);

    // insertion sort (tiny array)
    for (int i = 1; i < median_count; i++) {
        int32_t key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    return tmp[median_count / 2];
}

// Median -> exponential moving average (low-pass)
static float Filter_Update(int32_t raw) {
    float med = (float)Median_Push(raw);
    if (!ema_ready) {
        ema_value = med;
        ema_ready = 1;
    } else {
        ema_value += EMA_ALPHA * (med - ema_value);
    }
    return ema_value;
}

static void Filter_Reset(void) {
    median_count = 0;
    median_idx   = 0;
    ema_ready    = 0;
}

// Average TARE_SAMPLES readings and use that as the new zero.
// Keep the load cell unloaded and still while this runs.
static void Tare(void) {
    int64_t sum = 0;
    int n = 0;
    for (int i = 0; i < TARE_SAMPLES; i++) {
        int32_t v;
        if (HX711_Read(&v)) {
            sum += v;
            n++;
        }
    }
    if (n > 0) {
        offset_counts = (float)((double)sum / (double)n);
    }
    Filter_Reset();
}

static float Force_Compute(float filtered_counts) {
    float grams = (filtered_counts - offset_counts) / HX711_SCALE;
    return grams * GRAM_TO_NEWTON;
}

static inline float absf_(float x) { return x < 0 ? -x : x; }
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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  // Make sure SCK starts low
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  // Give the USB port 1 second to connect to the PC before spamming data
  HAL_Delay(1000);

  // Initial zero (load cell must be empty at power-up)
  Tare();
  printf("Ready. Send 't' (or 'z') to tare.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Manual tare from the PC keyboard
    if (usb_rx_cmd == 't' || usb_rx_cmd == 'T' ||
        usb_rx_cmd == 'z' || usb_rx_cmd == 'Z') {
        usb_rx_cmd = 0;
        Tare();
        printf("Tared.\r\n");
    } else {
        usb_rx_cmd = 0;
    }

    // One HX711 sample (blocks until ready: ~10 Hz with the default RATE pin)
    int32_t raw;
    if (!HX711_Read(&raw)) {
        printf("HX711 timeout\r\n");
        continue;
    }

    float filtered = Filter_Update(raw);
    float force_N  = Force_Compute(filtered);

#if AUTO_ZERO_ENABLE
    // Zero tracking: if the reading is very close to zero, assume it's drift
    // and let the offset slowly follow it.
    if (absf_(force_N) < AUTO_ZERO_WINDOW_N) {
        offset_counts += AUTO_ZERO_RATE * (filtered - offset_counts);
        force_N = Force_Compute(filtered);
    }
#endif

    // Deadband: print a clean zero when unloaded
    if (absf_(force_N) < ZERO_DEADBAND_N) {
        force_N = 0.0f;
    }

    // Print without float printf (needs a linker flag on STM32CubeIDE):
    // convert to millinewtons and format manually.
    int32_t mN = (int32_t)(force_N * 1000.0f + (force_N >= 0 ? 0.5f : -0.5f));
    long a = (mN < 0) ? -(long)mN : (long)mN;
    printf("Force: %s%ld.%03ld N\r\n", (mN < 0) ? "-" : "", a / 1000, a % 1000);

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
