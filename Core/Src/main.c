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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AXIS_MAX 1023

//DMA buffer for the ADC channels (pitch, roll, throttle, hat)
volatile uint16_t adc_buf[4]; // 0=Pitch, 1=Roll, 2=Throttle, 3=Yaw

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    uint16_t pitch;    // X      — 0–1023  (2 bytes)
    uint16_t roll;     // Y      — 0–1023  (2 bytes)
    uint16_t throttle; // Slider — 0–1023  (2 bytes)
    uint16_t yaw;      // Rz     — 0–1023  (2 bytes)
    uint16_t buttons;  // 10 buttons + 6 bit padding  (2 bytes)
} __attribute__((packed)) HID_JoystickReport_t;  // total: 10 bytes

uint16_t read_hardware_buttons(void)
{
    uint16_t b = 0;

    // Hardware Buttons (bits 0-4)
    b |= (HAL_GPIO_ReadPin(BUTTON_WEAPON_GPIO_Port, BUTTON_WEAPON_Pin) == GPIO_PIN_RESET) << 0;
    b |= (HAL_GPIO_ReadPin(BUTTON_TRIGGER_GPIO_Port, BUTTON_TRIGGER_Pin) == GPIO_PIN_RESET) << 1;
    b |= (HAL_GPIO_ReadPin(BUTTON_FLARE_GPIO_Port, BUTTON_FLARE_Pin) == GPIO_PIN_RESET) << 2;
    b |= (HAL_GPIO_ReadPin(BUTTON_MISSILE_GPIO_Port, BUTTON_MISSILE_Pin) == GPIO_PIN_RESET) << 3;
    b |= (HAL_GPIO_ReadPin(BUTTON_PADDLE_GPIO_Port, BUTTON_PADDLE_Pin) == GPIO_PIN_RESET) << 4;

    return b;
}

volatile uint8_t adc_ready = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC1)
        adc_ready = 1;
}

static uint16_t fx = 512, fy = 512;

// Simple Moving Average — window size N
// Larger N = smoother but more lag
// At 100Hz: N=16 = 160ms lag, N=8 = 80ms, N=32 = 320ms

#define SMA_SIZE 16  // tune this one value for all axes

typedef struct {
    uint16_t buf[SMA_SIZE];
    uint8_t  idx;
    uint32_t sum;
    uint8_t  full;  // 0 until buffer is filled on startup
} SMA_t;

static inline void SMA_init(SMA_t *f, uint16_t initial)
{
    for (int i = 0; i < SMA_SIZE; i++) f->buf[i] = initial;
    f->sum  = (uint32_t)initial * SMA_SIZE;
    f->idx  = 0;
    f->full = 1;
}

static inline uint16_t SMA_update(SMA_t *f, uint16_t input)
{
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = input;
    f->sum += input;
    f->idx = (f->idx + 1) % SMA_SIZE;
    return (uint16_t)(f->sum / SMA_SIZE);
}

static inline uint16_t SMA_update(SMA_t *f, uint16_t input)
{
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = input;
    f->sum += input;
    f->idx = (f->idx + 1) % SMA_SIZE;
    return (uint16_t)(f->sum / SMA_SIZE);
}

// Using DMA on Both ADC1 and ADC2
#define ADC_HID_PITCH    adc_buf[0]
#define ADC_HID_ROLL     adc_buf[1]
#define ADC_HID_THROTTLE adc_buf[2]
#define ADC_HID_YAW      adc_buf[3]

void joystick_task(void) //build and send the HID report
{
//	if(!adc_ready) return;  // wait for full sequence
//	adc_ready = 0;          // consume sample

    static uint32_t last_tick = 0;

	// SMA filters
	static SMA_t sma_pitch = {0};
	static SMA_t sma_roll  = {0};
	static SMA_t sma_yaw   = {0};
	static uint8_t sma_ready = 0;

	if (HAL_GetTick() - last_tick < 10) //limits to 100Hz (10ms interval)
		return;

	last_tick = HAL_GetTick();

	//12 bit ADC precision for STM32G431
	uint16_t x = ADC_HID_PITCH >> 2; 		//pitch
	uint16_t y = ADC_HID_ROLL >> 2; 		//roll
	uint16_t z = ADC_HID_THROTTLE >> 2;   	// throttle
	uint16_t rz  = ADC_HID_YAW >> 2;  		// yaw (ADC2_IN2)

	// Init on first run with live values — avoids startup jump to zero
	if (!sma_ready) {
		SMA_init(&sma_pitch, x);
		SMA_init(&sma_roll,  y);
		SMA_init(&sma_yaw,   rz);
		sma_ready = 1;
	}

	 // Apply SMA
	uint16_t sx  = SMA_update(&sma_pitch, x);
	uint16_t sy  = SMA_update(&sma_roll, y);
	uint16_t srz = SMA_update(&sma_yaw, rz);

	 // Apply SMA
	uint16_t sx  = SMA_update(&sma_pitch, x);
	uint16_t sy  = SMA_update(&sma_roll,  y);
	uint16_t srz = SMA_update(&sma_yaw,   rz);

	uint16_t buttons = read_hardware_buttons();

	HID_JoystickReport_t report;

	report.roll = AXIS_MAX - sx; 		// roll
	report.pitch = sy;			 		// pitch
	report.throttle = z; 				// throttle
	report.yaw = srz;					// yaw
	report.buttons = buttons & 0x03FF;

    USBD_HID_SendReport(&hUsbDeviceFS,
                            (uint8_t*)&report,
                            sizeof(report));
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
  __HAL_RCC_ADC12_CLK_ENABLE();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // DMA Setup - ADC1 - Pitch and Roll
  ADC_ChannelConfTypeDef sConfig = {0};

  // Pitch → ADC1_IN1 → PA0 → rank 1
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Roll → ADC1_IN7 → PC1 → rank 2
  sConfig.Channel      = ADC_CHANNEL_7;
  sConfig.Rank         = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Throttle → rank 3
  sConfig.Channel      = ADC_CHANNEL_9;
  sConfig.Rank         = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Yaw → rank 4
  sConfig.Channel      = ADC_CHANNEL_2;
  sConfig.Rank         = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // Start DMA with 4 channels
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 4);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  joystick_task();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : BUTTON_FLARE_Pin BUTTON_WEAPON_Pin BUTTON_TRIGGER_Pin BUTTON_PADDLE_Pin */
  GPIO_InitStruct.Pin = BUTTON_FLARE_Pin|BUTTON_WEAPON_Pin|BUTTON_TRIGGER_Pin|BUTTON_PADDLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_MISSILE_Pin */
  GPIO_InitStruct.Pin = BUTTON_MISSILE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_MISSILE_GPIO_Port, &GPIO_InitStruct);

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
