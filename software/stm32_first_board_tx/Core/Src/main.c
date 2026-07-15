/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : First board 4-FSK transmitter test.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

#define TX_AUDIO_FS_HZ          32000U
#define TX_PWM_PERIOD           255U
#define TX_PWM_MID              128U
#define TX_SINE_TABLE_SIZE      64U
#define TX_MAX_SINE_AMP         126U

#define TX_TONE_MS              120U
#define TX_GAP_MS               60U
#define TX_REPEAT_COUNT         3U
#define TX_FRAME_MAX_SYMBOLS    48U

#define TX_TONE_SAMPLES         ((TX_AUDIO_FS_HZ * TX_TONE_MS) / 1000U)
#define TX_GAP_SAMPLES          ((TX_AUDIO_FS_HZ * TX_GAP_MS) / 1000U)

typedef enum
{
  TX_MODE_IDLE = 0,
  TX_MODE_TEST_TONE,
  TX_MODE_FRAME
} TX_Mode;

typedef enum
{
  TX_FRAME_TONE = 0,
  TX_FRAME_GAP
} TX_FramePart;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_PWM_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

static void TX_StartTestTone(uint8_t bits);
static void TX_StartDigit(uint8_t digit);
static void TX_Stop(void);
static void TX_AudioTick(void);
static void TX_LoadFrameSymbol(uint8_t index);
static void TX_BuildDigitFrame(uint8_t digit);
static uint32_t TX_PhaseIncFromFreq(uint16_t freq_hz);
static uint8_t TX_ScaledSineSample(void);
static void TX_SendHelp(void);
static void TX_SendText(const char *text);
static void TX_HandleRxByte(uint8_t data);
static const char *TX_BitsToText(uint8_t bits);

static const uint8_t tx_sine_table[TX_SINE_TABLE_SIZE] = {
  128, 140, 152, 165, 176, 188, 198, 208,
  218, 226, 234, 240, 245, 250, 253, 255,
  255, 255, 253, 250, 245, 240, 234, 226,
  218, 208, 198, 188, 176, 165, 152, 140,
  128, 115, 103,  90,  79,  67,  57,  47,
   37,  29,  21,  15,  10,   5,   2,   0,
    0,   0,   2,   5,  10,  15,  21,  29,
   37,  47,  57,  67,  79,  90, 103, 115
};

static const uint16_t tx_fsk_freqs_hz[4] = {1500U, 2500U, 3500U, 4500U};
static const char *const tx_bits_text[4] = {"00", "01", "10", "11"};

/* Compensate the first-board analog chain: higher tones were measured smaller. */
static const uint8_t tx_fsk_amp[4] = {
  50U,              /* 1500Hz */
  72U,              /* 2500Hz */
  TX_MAX_SINE_AMP,  /* 3500Hz */
  TX_MAX_SINE_AMP   /* 4500Hz */
};

static const uint8_t tx_digit_symbols[10][2] = {
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

static const uint8_t tx_preamble_symbols[4] = {0x00U, 0x01U, 0x02U, 0x03U};
static const uint8_t tx_start_symbols[2] = {0x01U, 0x02U};
static const uint8_t tx_end_symbols[2] = {0x02U, 0x03U};

static volatile TX_Mode tx_mode = TX_MODE_IDLE;
static volatile TX_FramePart tx_frame_part = TX_FRAME_TONE;
static volatile uint32_t tx_phase = 0U;
static volatile uint32_t tx_phase_inc = 0U;
static volatile uint32_t tx_part_sample_count = 0U;
static volatile uint8_t tx_current_amp = 50U;
static volatile uint8_t tx_current_bits = 0U;
static volatile uint8_t tx_frame_index = 0U;
static volatile uint8_t tx_frame_len = 0U;
static uint8_t tx_frame_symbols[TX_FRAME_MAX_SYMBOLS];
static uint8_t uart_rx_byte;
static int8_t pending_bit = -1;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_PWM_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TX_PWM_MID);

  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);

  TX_SendText("\r\nFirst board 4FSK TX ready\r\n");
  TX_SendText("PA0=TIM2_CH1 PWM, PB12=AMP SD/mute, USART2=115200\r\n");
  TX_SendHelp();

  while (1)
  {
    static uint32_t last_blink_ms = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_blink_ms) >= 500U)
    {
      last_blink_ms = now;
      if (tx_mode == TX_MODE_IDLE)
      {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
      }
    }
  }
}

static void TX_StartTestTone(uint8_t bits)
{
  if (bits > 3U)
  {
    return;
  }

  __disable_irq();
  tx_current_bits = bits;
  tx_current_amp = tx_fsk_amp[bits];
  tx_phase = 0U;
  tx_phase_inc = TX_PhaseIncFromFreq(tx_fsk_freqs_hz[bits]);
  tx_mode = TX_MODE_TEST_TONE;
  __enable_irq();

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_ENABLE);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_ON);

  printf("tone: %s -> %uHz, amp=%u\r\n",
         TX_BitsToText(bits),
         tx_fsk_freqs_hz[bits],
         tx_fsk_amp[bits]);
}

static void TX_StartDigit(uint8_t digit)
{
  if (digit > 9U)
  {
    return;
  }

  TX_BuildDigitFrame(digit);

  __disable_irq();
  tx_frame_index = 0U;
  tx_frame_part = TX_FRAME_TONE;
  tx_part_sample_count = 0U;
  TX_LoadFrameSymbol(0U);
  tx_mode = TX_MODE_FRAME;
  __enable_irq();

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_ENABLE);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_ON);

  printf("digit: %u, symbols=%u\r\n", digit, tx_frame_len);
}

static void TX_Stop(void)
{
  __disable_irq();
  tx_mode = TX_MODE_IDLE;
  tx_phase = 0U;
  tx_part_sample_count = 0U;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TX_PWM_MID);
  __enable_irq();

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);
  TX_SendText("stop\r\n");
}

static void TX_AudioTick(void)
{
  TX_Mode mode = tx_mode;

  if (mode == TX_MODE_IDLE)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TX_PWM_MID);
    return;
  }

  if (mode == TX_MODE_TEST_TONE)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TX_ScaledSineSample());
    return;
  }

  if (tx_frame_part == TX_FRAME_TONE)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TX_ScaledSineSample());
    tx_part_sample_count++;

    if (tx_part_sample_count >= TX_TONE_SAMPLES)
    {
      tx_part_sample_count = 0U;
      tx_frame_part = TX_FRAME_GAP;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TX_PWM_MID);
    }
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TX_PWM_MID);
    tx_part_sample_count++;

    if (tx_part_sample_count >= TX_GAP_SAMPLES)
    {
      tx_part_sample_count = 0U;
      tx_frame_index++;

      if (tx_frame_index >= tx_frame_len)
      {
        tx_mode = TX_MODE_IDLE;
        HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);
      }
      else
      {
        TX_LoadFrameSymbol(tx_frame_index);
        tx_frame_part = TX_FRAME_TONE;
      }
    }
  }
}

static void TX_LoadFrameSymbol(uint8_t index)
{
  uint8_t bits = tx_frame_symbols[index] & 0x03U;

  tx_current_bits = bits;
  tx_current_amp = tx_fsk_amp[bits];
  tx_phase = 0U;
  tx_phase_inc = TX_PhaseIncFromFreq(tx_fsk_freqs_hz[bits]);
}

static void TX_BuildDigitFrame(uint8_t digit)
{
  uint8_t pos = 0U;
  uint8_t data0 = tx_digit_symbols[digit][0];
  uint8_t data1 = tx_digit_symbols[digit][1];

  for (uint8_t repeat = 0U; repeat < TX_REPEAT_COUNT; repeat++)
  {
    for (uint8_t i = 0U; i < sizeof(tx_preamble_symbols); i++)
    {
      tx_frame_symbols[pos++] = tx_preamble_symbols[i];
    }
    for (uint8_t i = 0U; i < sizeof(tx_start_symbols); i++)
    {
      tx_frame_symbols[pos++] = tx_start_symbols[i];
    }
    tx_frame_symbols[pos++] = data0;
    tx_frame_symbols[pos++] = data1;
    tx_frame_symbols[pos++] = (uint8_t)(data0 ^ 0x03U);
    tx_frame_symbols[pos++] = (uint8_t)(data1 ^ 0x03U);
    for (uint8_t i = 0U; i < sizeof(tx_end_symbols); i++)
    {
      tx_frame_symbols[pos++] = tx_end_symbols[i];
    }
  }

  tx_frame_len = pos;
}

static uint32_t TX_PhaseIncFromFreq(uint16_t freq_hz)
{
  return (uint32_t)(((uint64_t)freq_hz << 32) / TX_AUDIO_FS_HZ);
}

static uint8_t TX_ScaledSineSample(void)
{
  uint8_t index;
  int16_t delta;
  int16_t sample;

  tx_phase += tx_phase_inc;
  index = (uint8_t)(tx_phase >> 26);
  delta = (int16_t)tx_sine_table[index] - (int16_t)TX_PWM_MID;
  sample = (int16_t)TX_PWM_MID +
           (int16_t)((delta * (int16_t)tx_current_amp) / (int16_t)TX_MAX_SINE_AMP);

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

static void TX_SendHelp(void)
{
  TX_SendText("commands:\r\n");
  TX_SendText("  00/01/10/11 : continuous 1500/2500/3500/4500Hz\r\n");
  TX_SendText("  s            : stop\r\n");
  TX_SendText("  d0..d9       : send one digit frame\r\n");
}

static void TX_SendText(const char *text)
{
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void TX_HandleRxByte(uint8_t data)
{
  static bool digit_prefix = false;

  if ((data == '\r') || (data == '\n') || (data == ' ') || (data == '\t'))
  {
    return;
  }

  if ((data == 's') || (data == 'S'))
  {
    digit_prefix = false;
    pending_bit = -1;
    TX_Stop();
    return;
  }

  if ((data == 'h') || (data == 'H') || (data == '?'))
  {
    TX_SendHelp();
    return;
  }

  if ((data == 'd') || (data == 'D'))
  {
    digit_prefix = true;
    pending_bit = -1;
    return;
  }

  if (digit_prefix)
  {
    digit_prefix = false;
    if ((data >= '0') && (data <= '9'))
    {
      TX_StartDigit((uint8_t)(data - '0'));
    }
    else
    {
      TX_SendText("bad digit\r\n");
    }
    return;
  }

  if ((data == '0') || (data == '1'))
  {
    uint8_t bit = (uint8_t)(data - '0');

    if (pending_bit < 0)
    {
      pending_bit = (int8_t)bit;
    }
    else
    {
      uint8_t bits = (uint8_t)(((uint8_t)pending_bit << 1) | bit);
      pending_bit = -1;
      TX_StartTestTone(bits);
    }
    return;
  }

  if ((data >= '2') && (data <= '9'))
  {
    pending_bit = -1;
    TX_StartDigit((uint8_t)(data - '0'));
    return;
  }

  TX_SendText("unknown command, send h for help\r\n");
}

static const char *TX_BitsToText(uint8_t bits)
{
  return tx_bits_text[bits & 0x03U];
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    TX_AudioTick();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    TX_HandleRxByte(uart_rx_byte);
    (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);
  }
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_PWM_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TX_PWM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = TX_PWM_MID;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);
  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);

  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AMP_MUTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMP_MUTE_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
    HAL_Delay(80U);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
