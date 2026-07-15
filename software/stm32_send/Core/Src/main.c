/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  TX_STATE_IDLE = 0,
  TX_STATE_TONE,
  TX_STATE_GAP,
  TX_STATE_TEST_TONE
} TxState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_AUDIO_FS_HZ          32000U
#define TX_PWM_PERIOD           255U
#define TX_PWM_MID              128U
#define TX_SYMBOL_TONE_MS       120U
#define TX_SYMBOL_GAP_MS        60U
#define TX_REPEAT_COUNT         3U

#define TX_PREAMBLE_LEN         4U
#define TX_START_LEN            2U
#define TX_DATA_LEN             2U
#define TX_CHECK_LEN            2U
#define TX_END_LEN              2U
#define TX_FRAME_SYMBOLS        (TX_PREAMBLE_LEN + TX_START_LEN + TX_DATA_LEN + TX_CHECK_LEN + TX_END_LEN)

#define TX_TONE_SAMPLES         ((TX_AUDIO_FS_HZ * TX_SYMBOL_TONE_MS) / 1000U)
#define TX_GAP_SAMPLES          ((TX_AUDIO_FS_HZ * TX_SYMBOL_GAP_MS) / 1000U)
#define TX_SINE_TABLE_BITS      6U
#define TX_SINE_TABLE_SIZE      (1U << TX_SINE_TABLE_BITS)
#define TX_SINE_INDEX_SHIFT     (32U - TX_SINE_TABLE_BITS)
#define TX_BASE_SINE_AMP        80U
#define TX_MAX_SINE_AMP         126U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t tx_sine_table[TX_SINE_TABLE_SIZE] = {
  128U, 136U, 144U, 151U, 159U, 166U, 172U, 179U,
  185U, 190U, 195U, 199U, 202U, 205U, 206U, 208U,
  208U, 208U, 206U, 205U, 202U, 199U, 195U, 190U,
  185U, 179U, 172U, 166U, 159U, 151U, 144U, 136U,
  128U, 120U, 112U, 105U,  97U,  90U,  84U,  77U,
   71U,  66U,  61U,  57U,  54U,  51U,  50U,  48U,
   48U,  48U,  50U,  51U,  54U,  57U,  61U,  66U,
   71U,  77U,  84U,  90U,  97U, 105U, 112U, 120U
};

static const uint32_t tx_phase_inc[4] = {
  201326592UL, /* 1500Hz */
  335544320UL, /* 2500Hz */
  469762048UL, /* 3500Hz */
  603979776UL  /* 4500Hz */
};

static const uint16_t tx_freq_hz[4] = {1500U, 2500U, 3500U, 4500U};
static const uint8_t tx_amp_by_bits[4] = {
  50U,             /* 1500Hz: attenuate the strong low-frequency path */
  72U,             /* 2500Hz: moderate pre-emphasis */
  TX_MAX_SINE_AMP, /* 3500Hz: near maximum without PWM clipping */
  TX_MAX_SINE_AMP  /* 4500Hz: near maximum without PWM clipping */
};

static const uint8_t tx_digit_codebook[10][2] = {
  {0x00U, 0x01U}, /* 0 */
  {0x00U, 0x02U}, /* 1 */
  {0x00U, 0x03U}, /* 2 */
  {0x01U, 0x00U}, /* 3 */
  {0x01U, 0x02U}, /* 4 */
  {0x01U, 0x03U}, /* 5 */
  {0x02U, 0x00U}, /* 6 */
  {0x02U, 0x01U}, /* 7 */
  {0x02U, 0x03U}, /* 8 */
  {0x03U, 0x00U}  /* 9 */
};

static const uint8_t tx_preamble[TX_PREAMBLE_LEN] = {0x00U, 0x01U, 0x02U, 0x03U};
static const uint8_t tx_start[TX_START_LEN] = {0x01U, 0x02U};
static const uint8_t tx_end[TX_END_LEN] = {0x02U, 0x03U};

static volatile TxState tx_state = TX_STATE_IDLE;
static volatile uint8_t tx_active = 0U;
static volatile uint8_t tx_digit = 0U;
static volatile uint8_t tx_repeat_index = 0U;
static volatile uint8_t tx_symbol_index = 0U;
static volatile uint32_t tx_samples_left = 0U;
static volatile uint32_t tx_phase_acc = 0U;
static volatile uint32_t tx_current_inc = 0U;
static volatile uint8_t tx_current_amp = TX_BASE_SINE_AMP;
static uint8_t tx_frame[TX_FRAME_SYMBOLS];

static volatile uint8_t uart_rx_byte = 0U;
static volatile uint8_t uart_digit_pending = 0U;
static volatile uint8_t uart_pending_digit = 0U;
static volatile uint8_t uart_tone_pending = 0U;
static volatile uint8_t uart_pending_bits = 0U;
static volatile uint8_t uart_stop_pending = 0U;
static volatile uint8_t uart_busy_notice = 0U;
static volatile uint8_t uart_digit_prefix = 0U;
static volatile uint8_t uart_bit_waiting = 0U;
static volatile uint8_t uart_first_bit = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void TX_StartDigit(uint8_t digit);
static void TX_BuildFrame(uint8_t digit);
static void TX_LoadNextTone(void);
static void TX_Stop(void);
static void TX_StartTestTone(uint8_t bits);
static void TX_SendText(const char *text);
static const char *TX_BitsToString(uint8_t bits);
static uint8_t TX_ScaledSineSample(uint8_t index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void TX_SendText(const char *text)
{
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

static const char *TX_BitsToString(uint8_t bits)
{
  static const char *bit_text[4] = {"00", "01", "10", "11"};
  return bit_text[bits & 0x03U];
}

static uint8_t TX_ScaledSineSample(uint8_t index)
{
  int16_t delta = (int16_t)tx_sine_table[index & (TX_SINE_TABLE_SIZE - 1U)] - (int16_t)TX_PWM_MID;
  int16_t sample = (int16_t)TX_PWM_MID + (int16_t)((delta * (int16_t)tx_current_amp) / (int16_t)TX_BASE_SINE_AMP);

  if (sample < 1)
  {
    sample = 1;
  }
  else if (sample > (int16_t)(TX_PWM_PERIOD - 1U))
  {
    sample = (int16_t)(TX_PWM_PERIOD - 1U);
  }

  return (uint8_t)sample;
}

static void TX_BuildFrame(uint8_t digit)
{
  uint8_t pos = 0U;

  for (uint8_t i = 0U; i < TX_PREAMBLE_LEN; i++)
  {
    tx_frame[pos++] = tx_preamble[i];
  }

  for (uint8_t i = 0U; i < TX_START_LEN; i++)
  {
    tx_frame[pos++] = tx_start[i];
  }

  tx_frame[pos++] = tx_digit_codebook[digit][0];
  tx_frame[pos++] = tx_digit_codebook[digit][1];
  tx_frame[pos++] = tx_digit_codebook[digit][0] ^ 0x03U;
  tx_frame[pos++] = tx_digit_codebook[digit][1] ^ 0x03U;

  for (uint8_t i = 0U; i < TX_END_LEN; i++)
  {
    tx_frame[pos++] = tx_end[i];
  }
}

static void TX_StartDigit(uint8_t digit)
{
  TX_BuildFrame(digit);

  __disable_irq();
  tx_digit = digit;
  tx_repeat_index = 0U;
  tx_symbol_index = 0U;
  tx_samples_left = 0U;
  tx_phase_acc = 0U;
  tx_current_inc = 0U;
  tx_state = TX_STATE_GAP;
  tx_active = 1U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TX_PWM_MID);
  __enable_irq();
}

static void TX_Stop(void)
{
  tx_active = 0U;
  tx_state = TX_STATE_IDLE;
  tx_samples_left = 0U;
  tx_current_inc = 0U;
  tx_current_amp = TX_BASE_SINE_AMP;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TX_PWM_MID);
}

static void TX_StartTestTone(uint8_t bits)
{
  bits &= 0x03U;

  __disable_irq();
  tx_repeat_index = 0U;
  tx_symbol_index = 0U;
  tx_samples_left = 0U;
  tx_phase_acc = 0U;
  tx_current_inc = tx_phase_inc[bits];
  tx_current_amp = tx_amp_by_bits[bits];
  tx_state = TX_STATE_TEST_TONE;
  tx_active = 1U;
  __enable_irq();
}

static void TX_LoadNextTone(void)
{
  uint8_t bits;

  if (tx_repeat_index >= TX_REPEAT_COUNT)
  {
    TX_Stop();
    return;
  }

  bits = tx_frame[tx_symbol_index] & 0x03U;
  tx_current_inc = tx_phase_inc[bits];
  tx_current_amp = tx_amp_by_bits[bits];
  tx_phase_acc = 0U;
  tx_samples_left = TX_TONE_SAMPLES;
  tx_state = TX_STATE_TONE;

  tx_symbol_index++;
  if (tx_symbol_index >= TX_FRAME_SYMBOLS)
  {
    tx_symbol_index = 0U;
    tx_repeat_index++;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t last_tx_active = 0U;

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  TX_SendText("\r\n4FSK TX ready.\r\n");
  TX_SendText("Digit frame: d0-d9, or 2-9 direct.\r\n");
  TX_SendText("Test tone: 00=1500, 01=2500, 10=3500, 11=4500, s=stop.\r\n");
  TX_SendText("Pre-emphasis amp: 50/72/126/126.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (uart_stop_pending != 0U)
    {
      uart_stop_pending = 0U;
      uart_tone_pending = 0U;
      TX_Stop();
      TX_SendText("TX stop\r\n");
    }

    if (uart_tone_pending != 0U)
    {
      char msg[64];
      uint8_t bits = uart_pending_bits & 0x03U;

      uart_tone_pending = 0U;
      TX_StartTestTone(bits);
      (void)snprintf(msg, sizeof(msg), "TEST %s -> %uHz continuous, amp=%u\r\n",
                     TX_BitsToString(bits), tx_freq_hz[bits], tx_amp_by_bits[bits]);
      TX_SendText(msg);
    }

    if ((uart_digit_pending != 0U) && ((tx_active == 0U) || (tx_state == TX_STATE_TEST_TONE)))
    {
      char msg[64];
      uint8_t digit = uart_pending_digit;

      uart_digit_pending = 0U;
      TX_StartDigit(digit);
      (void)snprintf(msg, sizeof(msg), "TX digit %u, frame x%u\r\n", digit, TX_REPEAT_COUNT);
      TX_SendText(msg);
    }

    if (uart_busy_notice != 0U)
    {
      uart_busy_notice = 0U;
      TX_SendText("TX busy, queued latest digit\r\n");
    }

    if ((last_tx_active != 0U) && (tx_active == 0U))
    {
      TX_SendText("TX done\r\n");
    }
    last_tx_active = tx_active;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TX_PWM_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = TX_PWM_MID;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA8: TIM1_CH1 PWM audio to PAM8302 AIN+ through RC filter. */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA2/PA3: USART2 TX/RX for USB-TTL command input. */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TX_AudioTick(void)
{
  uint8_t sample;
  uint8_t index;

  if (tx_active == 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TX_PWM_MID);
    return;
  }

  if (tx_state == TX_STATE_TEST_TONE)
  {
    index = (uint8_t)(tx_phase_acc >> TX_SINE_INDEX_SHIFT);
    sample = TX_ScaledSineSample(index);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sample);
    tx_phase_acc += tx_current_inc;
    return;
  }

  if (tx_samples_left == 0U)
  {
    if (tx_state == TX_STATE_TONE)
    {
      tx_state = TX_STATE_GAP;
      tx_samples_left = TX_GAP_SAMPLES;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TX_PWM_MID);
    }
    else
    {
      TX_LoadNextTone();
    }
  }

  if (tx_state == TX_STATE_TONE)
  {
    index = (uint8_t)(tx_phase_acc >> TX_SINE_INDEX_SHIFT);
    sample = TX_ScaledSineSample(index);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sample);
    tx_phase_acc += tx_current_inc;
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TX_PWM_MID);
  }

  if (tx_samples_left > 0U)
  {
    tx_samples_left--;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    TX_AudioTick();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t ch;
  uint8_t bit;

  if (huart->Instance == USART2)
  {
    ch = uart_rx_byte;

    if ((ch == (uint8_t)'s') || (ch == (uint8_t)'S') || (ch == (uint8_t)'x') || (ch == (uint8_t)'X'))
    {
      uart_stop_pending = 1U;
      uart_digit_prefix = 0U;
      uart_bit_waiting = 0U;
    }
    else if ((ch == (uint8_t)'d') || (ch == (uint8_t)'D'))
    {
      uart_digit_prefix = 1U;
      uart_bit_waiting = 0U;
    }
    else if ((uart_digit_prefix != 0U) && (ch >= (uint8_t)'0') && (ch <= (uint8_t)'9'))
    {
      uart_pending_digit = ch - (uint8_t)'0';
      uart_digit_pending = 1U;
      uart_digit_prefix = 0U;
      if ((tx_active != 0U) && (tx_state != TX_STATE_TEST_TONE))
      {
        uart_busy_notice = 1U;
      }
    }
    else if ((ch == (uint8_t)'0') || (ch == (uint8_t)'1'))
    {
      bit = ch - (uint8_t)'0';
      if (uart_bit_waiting != 0U)
      {
        uart_pending_bits = (uint8_t)((uart_first_bit << 1) | bit);
        uart_tone_pending = 1U;
        uart_bit_waiting = 0U;
      }
      else
      {
        uart_first_bit = bit;
        uart_bit_waiting = 1U;
      }
    }
    else if ((ch >= (uint8_t)'2') && (ch <= (uint8_t)'9'))
    {
      uart_pending_digit = ch - (uint8_t)'0';
      uart_digit_pending = 1U;
      uart_digit_prefix = 0U;
      uart_bit_waiting = 0U;
      if ((tx_active != 0U) && (tx_state != TX_STATE_TEST_TONE))
      {
        uart_busy_notice = 1U;
      }
    }
    else if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n') || (ch == (uint8_t)' ') || (ch == (uint8_t)'\t') || (ch == (uint8_t)','))
    {
      if (uart_bit_waiting != 0U)
      {
        uart_pending_digit = uart_first_bit;
        uart_digit_pending = 1U;
        uart_bit_waiting = 0U;
      }
      uart_digit_prefix = 0U;
    }
    else
    {
      uart_digit_prefix = 0U;
      uart_bit_waiting = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx_byte, 1U);
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
