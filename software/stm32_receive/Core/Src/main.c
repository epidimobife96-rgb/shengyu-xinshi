/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 4-FSK audio receiver demo.
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

/* USER CODE BEGIN Semihosting */
#if defined(__ARMCC_VERSION) && !defined(__CC_ARM)
__asm(".global __use_no_semihosting\n");
#endif

#if defined(__CC_ARM)
#pragma import(__use_no_semihosting)
#endif

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
void _ttywrch(int ch)
{
  (void)ch;
}

void _sys_exit(int status)
{
  (void)status;
  while (1)
  {
  }
}

int ferror(FILE *f)
{
  (void)f;
  return 0;
}
#endif
/* USER CODE END Semihosting */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t valid;
  uint16_t freq_hz;
  uint8_t bits;
  float max_energy;
  float second_energy;
} FSK_DetectResult;

typedef enum
{
  DIGIT_RX_SEARCH = 0,
  DIGIT_RX_START_0,
  DIGIT_RX_START_1,
  DIGIT_RX_DATA_0,
  DIGIT_RX_DATA_1,
  DIGIT_RX_CHECK_0,
  DIGIT_RX_CHECK_1,
  DIGIT_RX_END_0,
  DIGIT_RX_END_1
} DigitRxState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FSK_FS_HZ                 16000U
#define FSK_SYMBOL_MS             20U
#define FSK_SYMBOL_SAMPLES        ((FSK_FS_HZ * FSK_SYMBOL_MS) / 1000U)
#define ADC_DMA_BUFFER_SAMPLES    (FSK_SYMBOL_SAMPLES * 2U)

#define FSK_FREQ_COUNT            4U
#define FSK_ENERGY_THRESHOLD      5000000.0f
#define FSK_RATIO_THRESHOLD       1.3f
#define OLED_DEBUG_UPDATE_MS      200U
#define DEBUG_UART_ENABLE         0U
#define DIGIT_STABLE_WINDOWS      2U
#define DIGIT_FRAME_TIMEOUT_MS    1800U
#define DIGIT_NO_SYMBOL           0xFFU
#define DIGIT_PREAMBLE_LEN        4U
#define DIGIT_START_LEN           2U
#define DIGIT_END_LEN             2U

#define OLED_I2C_ADDR_3C          (0x3CU << 1)
#define OLED_I2C_ADDR_3D          (0x3DU << 1)
#define OLED_WIDTH                128U
#define OLED_HEIGHT               64U
#define OLED_PAGES                (OLED_HEIGHT / 8U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c3;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint16_t adc_dma_buf[ADC_DMA_BUFFER_SAMPLES];
static volatile uint8_t symbol_ready = 0U;
static volatile uint16_t symbol_start_index = 0U;

static const uint16_t fsk_freqs_hz[FSK_FREQ_COUNT] = {1500U, 2500U, 3500U, 4500U};
static const uint8_t fsk_bits[FSK_FREQ_COUNT] = {0x00U, 0x01U, 0x02U, 0x03U};
static const uint8_t digit_codebook[10][2] = {
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
static const uint8_t digit_preamble[DIGIT_PREAMBLE_LEN] = {0x00U, 0x01U, 0x02U, 0x03U};
static const uint8_t digit_start[DIGIT_START_LEN] = {0x03U, 0x02U};
static const uint8_t digit_end[DIGIT_END_LEN] = {0x02U, 0x03U};
static float fsk_goertzel_coeffs[FSK_FREQ_COUNT];
static uint16_t oled_i2c_addr = OLED_I2C_ADDR_3C;
static uint8_t oled_ready = 0U;
static uint8_t digit_candidate_bits = DIGIT_NO_SYMBOL;
static uint8_t digit_candidate_count = 0U;
static uint8_t digit_run_active = 0U;
static uint8_t digit_run_bits = DIGIT_NO_SYMBOL;
static DigitRxState digit_rx_state = DIGIT_RX_SEARCH;
static uint8_t digit_preamble_index = 0U;
static uint8_t digit_data_bits[2] = {DIGIT_NO_SYMBOL, DIGIT_NO_SYMBOL};
static uint8_t digit_frame_error = 0U;
static uint32_t digit_frame_tick = 0U;
static uint8_t digit_last_valid = 0U;
static uint8_t digit_last_value = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void FSK_Init(void);
static float Goertzel_Energy(const uint16_t *samples, uint16_t len, float coeff);
static FSK_DetectResult FSK_DetectSymbol(const uint16_t *samples, uint16_t len);
static void FSK_ProcessSymbol(const uint16_t *samples);
static const char *FSK_BitsToString(uint8_t bits);
static void DigitRx_Process(const FSK_DetectResult *result, uint32_t now);
static void DigitRx_AcceptSymbol(uint8_t bits, uint32_t now);
static int8_t DigitRx_DecodePair(uint8_t first_bits, uint8_t second_bits);
static void DigitRx_ResetFrame(void);

static uint8_t OLED_Init(void);
static void OLED_Clear(void);
static void OLED_WriteCommand(uint8_t command);
static void OLED_WriteData(const uint8_t *data, uint16_t len);
static void OLED_SetCursor(uint8_t col, uint8_t page);
static void OLED_PrintAt(uint8_t col, uint8_t page, const char *text);
static void OLED_PrintChar(char ch);
static const uint8_t *OLED_Font5x7(char ch);
static void OLED_PrintDetected(const FSK_DetectResult *result);
static void StatusLed_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void FSK_Init(void)
{
  /*
   * Goertzel coefficient = 2*cos(2*pi*f/Fs).
   * Fs is fixed at 16000 Hz for this first receiver version.
   */
  fsk_goertzel_coeffs[0] = 1.6629392f;   /* 1500 Hz */
  fsk_goertzel_coeffs[1] = 1.1111405f;   /* 2500 Hz */
  fsk_goertzel_coeffs[2] = 0.3901806f;   /* 3500 Hz */
  fsk_goertzel_coeffs[3] = -0.3901806f;  /* 4500 Hz */
}

static float Goertzel_Energy(const uint16_t *samples, uint16_t len, float coeff)
{
  uint32_t sum = 0U;
  float mean;
  float s0 = 0.0f;
  float s1 = 0.0f;
  float s2 = 0.0f;

  for (uint16_t i = 0U; i < len; i++)
  {
    sum += samples[i];
  }

  mean = (float)sum / (float)len;

  for (uint16_t i = 0U; i < len; i++)
  {
    float x = (float)samples[i] - mean;
    s0 = x + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }

  return (s1 * s1) + (s2 * s2) - (coeff * s1 * s2);
}

static FSK_DetectResult FSK_DetectSymbol(const uint16_t *samples, uint16_t len)
{
  FSK_DetectResult result = {0};
  float energies[FSK_FREQ_COUNT];
  uint8_t max_index = 0U;
  uint8_t second_index = 1U;

  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    energies[i] = Goertzel_Energy(samples, len, fsk_goertzel_coeffs[i]);
  }

  if (energies[second_index] > energies[max_index])
  {
    max_index = 1U;
    second_index = 0U;
  }

  for (uint8_t i = 2U; i < FSK_FREQ_COUNT; i++)
  {
    if (energies[i] > energies[max_index])
    {
      second_index = max_index;
      max_index = i;
    }
    else if (energies[i] > energies[second_index])
    {
      second_index = i;
    }
  }

  result.max_energy = energies[max_index];
  result.second_energy = energies[second_index];

  if ((result.max_energy > FSK_ENERGY_THRESHOLD) &&
      (result.max_energy > (result.second_energy * FSK_RATIO_THRESHOLD)))
  {
    result.valid = 1U;
    result.freq_hz = fsk_freqs_hz[max_index];
    result.bits = fsk_bits[max_index];
  }

  return result;
}

static void FSK_ProcessSymbol(const uint16_t *samples)
{
  FSK_DetectResult result = FSK_DetectSymbol(samples, FSK_SYMBOL_SAMPLES);
  static uint32_t last_oled_update = 0U;
  uint32_t now = HAL_GetTick();

  DigitRx_Process(&result, now);

#if DEBUG_UART_ENABLE
  if (result.valid != 0U)
  {
    printf("detected: %uHz -> %s, Emax=%lu, E2=%lu\r\n",
           result.freq_hz,
           FSK_BitsToString(result.bits),
           (uint32_t)result.max_energy,
           (uint32_t)result.second_energy);
  }
  else
  {
    printf("invalid/no signal, Emax=%lu, E2=%lu\r\n",
           (uint32_t)result.max_energy,
           (uint32_t)result.second_energy);
  }
#endif

  if ((now - last_oled_update) >= OLED_DEBUG_UPDATE_MS)
  {
    last_oled_update = now;
    OLED_PrintDetected(&result);
  }
}

static const char *FSK_BitsToString(uint8_t bits)
{
  static const char *bit_text[FSK_FREQ_COUNT] = {"00", "01", "10", "11"};
  return bit_text[bits & 0x03U];
}

static void DigitRx_Process(const FSK_DetectResult *result, uint32_t now)
{
  if ((digit_rx_state != DIGIT_RX_SEARCH) && ((now - digit_frame_tick) > DIGIT_FRAME_TIMEOUT_MS))
  {
    DigitRx_ResetFrame();
  }

  if (result->valid == 0U)
  {
    digit_candidate_bits = DIGIT_NO_SYMBOL;
    digit_candidate_count = 0U;
    digit_run_active = 0U;
    return;
  }

  if (digit_candidate_bits == result->bits)
  {
    if (digit_candidate_count < 255U)
    {
      digit_candidate_count++;
    }
  }
  else
  {
    digit_candidate_bits = result->bits;
    digit_candidate_count = 1U;
  }

  if (digit_candidate_count < DIGIT_STABLE_WINDOWS)
  {
    return;
  }

  if ((digit_run_active != 0U) && (digit_run_bits == result->bits))
  {
    return;
  }

  digit_run_active = 1U;
  digit_run_bits = result->bits;
  DigitRx_AcceptSymbol(result->bits, now);
}

static void DigitRx_AcceptSymbol(uint8_t bits, uint32_t now)
{
  digit_frame_tick = now;

  switch (digit_rx_state)
  {
    case DIGIT_RX_SEARCH:
      if (bits == digit_preamble[digit_preamble_index])
      {
        digit_preamble_index++;

        if (digit_preamble_index >= DIGIT_PREAMBLE_LEN)
        {
          digit_preamble_index = 0U;
          digit_rx_state = DIGIT_RX_START_0;
          digit_frame_error = 0U;
          digit_last_valid = 0U;
        }
      }
      else
      {
        digit_preamble_index = (bits == digit_preamble[0]) ? 1U : 0U;
      }
      break;

    case DIGIT_RX_START_0:
      digit_rx_state = (bits == digit_start[0]) ? DIGIT_RX_START_1 : DIGIT_RX_SEARCH;
      break;

    case DIGIT_RX_START_1:
      digit_rx_state = (bits == digit_start[1]) ? DIGIT_RX_DATA_0 : DIGIT_RX_SEARCH;
      break;

    case DIGIT_RX_DATA_0:
      digit_data_bits[0] = bits;
      digit_rx_state = DIGIT_RX_DATA_1;
      break;

    case DIGIT_RX_DATA_1:
      digit_data_bits[1] = bits;
      digit_rx_state = DIGIT_RX_CHECK_0;
      break;

    case DIGIT_RX_CHECK_0:
      if (bits == (uint8_t)(digit_data_bits[0] ^ 0x03U))
      {
        digit_rx_state = DIGIT_RX_CHECK_1;
      }
      else
      {
        digit_frame_error = 1U;
        DigitRx_ResetFrame();
      }
      break;

    case DIGIT_RX_CHECK_1:
      if (bits == (uint8_t)(digit_data_bits[1] ^ 0x03U))
      {
        digit_rx_state = DIGIT_RX_END_0;
      }
      else
      {
        digit_frame_error = 1U;
        DigitRx_ResetFrame();
      }
      break;

    case DIGIT_RX_END_0:
      if (bits == digit_end[0])
      {
        digit_rx_state = DIGIT_RX_END_1;
      }
      else
      {
        digit_frame_error = 1U;
        DigitRx_ResetFrame();
      }
      break;

    case DIGIT_RX_END_1:
      if (bits == digit_end[1])
      {
        int8_t digit = DigitRx_DecodePair(digit_data_bits[0], digit_data_bits[1]);
        if (digit >= 0)
        {
          digit_last_valid = 1U;
          digit_last_value = (uint8_t)digit;
          digit_frame_error = 0U;
        }
        else
        {
          digit_frame_error = 1U;
        }
      }
      else
      {
        digit_frame_error = 1U;
      }
      DigitRx_ResetFrame();
      break;

    default:
      DigitRx_ResetFrame();
      break;
  }
}

static int8_t DigitRx_DecodePair(uint8_t first_bits, uint8_t second_bits)
{
  for (uint8_t digit = 0U; digit < 10U; digit++)
  {
    if ((digit_codebook[digit][0] == first_bits) &&
        (digit_codebook[digit][1] == second_bits))
    {
      return (int8_t)digit;
    }
  }

  return -1;
}

static void DigitRx_ResetFrame(void)
{
  digit_rx_state = DIGIT_RX_SEARCH;
  digit_preamble_index = 0U;
  digit_data_bits[0] = DIGIT_NO_SYMBOL;
  digit_data_bits[1] = DIGIT_NO_SYMBOL;
  digit_frame_tick = 0U;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    symbol_start_index = 0U;
    symbol_ready = 1U;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    symbol_start_index = FSK_SYMBOL_SAMPLES;
    symbol_ready = 1U;
  }
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

int fputc(int ch, FILE *f)
{
  (void)f;
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
  return ch;
}

static uint8_t OLED_Init(void)
{
  HAL_Delay(100U);

  if (HAL_I2C_IsDeviceReady(&hi2c3, OLED_I2C_ADDR_3C, 3U, 50U) == HAL_OK)
  {
    oled_i2c_addr = OLED_I2C_ADDR_3C;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c3, OLED_I2C_ADDR_3D, 3U, 50U) == HAL_OK)
  {
    oled_i2c_addr = OLED_I2C_ADDR_3D;
  }
  else
  {
    oled_ready = 0U;
    return 0U;
  }

  oled_ready = 1U;
  OLED_WriteCommand(0xAEU); /* display off */
  OLED_WriteCommand(0x20U); /* memory mode */
  OLED_WriteCommand(0x00U); /* horizontal addressing */
  OLED_WriteCommand(0xB0U);
  OLED_WriteCommand(0xC8U);
  OLED_WriteCommand(0x00U);
  OLED_WriteCommand(0x10U);
  OLED_WriteCommand(0x40U);
  OLED_WriteCommand(0x81U);
  OLED_WriteCommand(0x7FU);
  OLED_WriteCommand(0xA1U);
  OLED_WriteCommand(0xA6U);
  OLED_WriteCommand(0xA8U);
  OLED_WriteCommand(0x3FU);
  OLED_WriteCommand(0xA4U);
  OLED_WriteCommand(0xD3U);
  OLED_WriteCommand(0x00U);
  OLED_WriteCommand(0xD5U);
  OLED_WriteCommand(0x80U);
  OLED_WriteCommand(0xD9U);
  OLED_WriteCommand(0xF1U);
  OLED_WriteCommand(0xDAU);
  OLED_WriteCommand(0x12U);
  OLED_WriteCommand(0xDBU);
  OLED_WriteCommand(0x40U);
  OLED_WriteCommand(0x8DU);
  OLED_WriteCommand(0x14U);
  OLED_WriteCommand(0xAFU); /* display on */
  OLED_Clear();
  OLED_PrintAt(0U, 0U, "DIGIT RX");
  OLED_PrintAt(0U, 2U, "WAIT FRAME");
  return 1U;
}

static void OLED_Clear(void)
{
  uint8_t blank[OLED_WIDTH];
  memset(blank, 0, sizeof(blank));

  for (uint8_t page = 0U; page < OLED_PAGES; page++)
  {
    OLED_SetCursor(0U, page);
    OLED_WriteData(blank, OLED_WIDTH);
  }
}

static void OLED_WriteCommand(uint8_t command)
{
  uint8_t data[2] = {0x00U, command};
  if (oled_ready != 0U)
  {
    (void)HAL_I2C_Master_Transmit(&hi2c3, oled_i2c_addr, data, sizeof(data), 100U);
  }
}

static void OLED_WriteData(const uint8_t *data, uint16_t len)
{
  uint8_t buffer[17];
  buffer[0] = 0x40U;

  if (oled_ready == 0U)
  {
    return;
  }

  while (len > 0U)
  {
    uint16_t chunk = (len > 16U) ? 16U : len;
    memcpy(&buffer[1], data, chunk);
    (void)HAL_I2C_Master_Transmit(&hi2c3, oled_i2c_addr, buffer, (uint16_t)(chunk + 1U), 100U);
    data += chunk;
    len -= chunk;
  }
}

static void OLED_SetCursor(uint8_t col, uint8_t page)
{
  OLED_WriteCommand((uint8_t)(0xB0U + page));
  OLED_WriteCommand((uint8_t)(0x00U + (col & 0x0FU)));
  OLED_WriteCommand((uint8_t)(0x10U + ((col >> 4U) & 0x0FU)));
}

static void OLED_PrintAt(uint8_t col, uint8_t page, const char *text)
{
  OLED_SetCursor(col, page);
  while (*text != '\0')
  {
    OLED_PrintChar(*text++);
  }
}

static void OLED_PrintChar(char ch)
{
  uint8_t glyph[6];
  const uint8_t *font = OLED_Font5x7(ch);

  memcpy(glyph, font, 5U);
  glyph[5] = 0x00U;
  OLED_WriteData(glyph, sizeof(glyph));
}

static const uint8_t *OLED_Font5x7(char ch)
{
  static const uint8_t blank[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t dash[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  static const uint8_t slash[5] = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
  static const uint8_t greater[5] = {0x00U, 0x41U, 0x22U, 0x14U, 0x08U};
  static const uint8_t num0[5] = {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU};
  static const uint8_t num1[5] = {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U};
  static const uint8_t num2[5] = {0x42U, 0x61U, 0x51U, 0x49U, 0x46U};
  static const uint8_t num3[5] = {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U};
  static const uint8_t num4[5] = {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U};
  static const uint8_t num5[5] = {0x27U, 0x45U, 0x45U, 0x45U, 0x39U};
  static const uint8_t num6[5] = {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U};
  static const uint8_t num7[5] = {0x01U, 0x71U, 0x09U, 0x05U, 0x03U};
  static const uint8_t num8[5] = {0x36U, 0x49U, 0x49U, 0x49U, 0x36U};
  static const uint8_t num9[5] = {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU};
  static const uint8_t A[5] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
  static const uint8_t C[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
  static const uint8_t D[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
  static const uint8_t E[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
  static const uint8_t F[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U};
  static const uint8_t G[5] = {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU};
  static const uint8_t H[5] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
  static const uint8_t I[5] = {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U};
  static const uint8_t K[5] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
  static const uint8_t L[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
  static const uint8_t M[5] = {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU};
  static const uint8_t N[5] = {0x7FU, 0x02U, 0x04U, 0x08U, 0x7FU};
  static const uint8_t O[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
  static const uint8_t R[5] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
  static const uint8_t S[5] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
  static const uint8_t T[5] = {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U};
  static const uint8_t U[5] = {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU};
  static const uint8_t V[5] = {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU};
  static const uint8_t W[5] = {0x7FU, 0x20U, 0x18U, 0x20U, 0x7FU};
  static const uint8_t X[5] = {0x63U, 0x14U, 0x08U, 0x14U, 0x63U};
  static const uint8_t Z[5] = {0x61U, 0x51U, 0x49U, 0x45U, 0x43U};

  switch (ch)
  {
    case ' ': return blank;
    case '-': return dash;
    case '/': return slash;
    case '>': return greater;
    case '0': return num0;
    case '1': return num1;
    case '2': return num2;
    case '3': return num3;
    case '4': return num4;
    case '5': return num5;
    case '6': return num6;
    case '7': return num7;
    case '8': return num8;
    case '9': return num9;
    case 'A': return A;
    case 'C': return C;
    case 'D': return D;
    case 'E': return E;
    case 'F': return F;
    case 'G': return G;
    case 'H': return H;
    case 'I': return I;
    case 'K': return K;
    case 'L': return L;
    case 'M': return M;
    case 'N': return N;
    case 'O': return O;
    case 'R': return R;
    case 'S': return S;
    case 'T': return T;
    case 'U': return U;
    case 'V': return V;
    case 'W': return W;
    case 'X': return X;
    case 'Z': return Z;
    default: return blank;
  }
}

static void OLED_PrintDetected(const FSK_DetectResult *result)
{
  char line[22];
  uint32_t max_k = (uint32_t)(result->max_energy / 1000.0f);
  uint32_t second_k = (uint32_t)(result->second_energy / 1000.0f);

  OLED_Clear();
  OLED_PrintAt(0U, 0U, "DIGIT RX");

  if (digit_last_valid != 0U)
  {
    (void)snprintf(line, sizeof(line), "DIGIT %u", digit_last_value);
    OLED_PrintAt(0U, 2U, line);
  }
  else if (digit_frame_error != 0U)
  {
    OLED_PrintAt(0U, 2U, "CHECK FAIL");
  }
  else if (digit_rx_state != DIGIT_RX_SEARCH)
  {
    if (result->valid != 0U)
    {
      (void)snprintf(line, sizeof(line), "F%u S%s", (uint8_t)digit_rx_state, FSK_BitsToString(result->bits));
    }
    else
    {
      (void)snprintf(line, sizeof(line), "FRAME %u", (uint8_t)digit_rx_state);
    }
    OLED_PrintAt(0U, 2U, line);
  }
  else if (result->valid != 0U)
  {
    (void)snprintf(line, sizeof(line), "SYM %s", FSK_BitsToString(result->bits));
    OLED_PrintAt(0U, 2U, line);
  }
  else
  {
    OLED_PrintAt(0U, 2U, "INVALID");
  }

  (void)snprintf(line, sizeof(line), "MAX %uK", (unsigned int)max_k);
  OLED_PrintAt(0U, 4U, line);

  (void)snprintf(line, sizeof(line), "SEC %uK", (unsigned int)second_k);
  OLED_PrintAt(0U, 6U, line);
}

static void StatusLed_Update(void)
{
  static uint32_t last_toggle = 0U;
  uint32_t now = HAL_GetTick();
  uint32_t interval = (oled_ready != 0U) ? 500U : 100U;

  if ((now - last_toggle) >= interval)
  {
    last_toggle = now;
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  FSK_Init();
  (void)OLED_Init();

#if DEBUG_UART_ENABLE
  printf("\r\n4-FSK receiver start\r\n");
  printf("ADC1_IN1 PA1, Fs=%uHz, symbol=%ums, samples=%u\r\n",
         FSK_FS_HZ, FSK_SYMBOL_MS, FSK_SYMBOL_SAMPLES);
#endif

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_BUFFER_SAMPLES) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    StatusLed_Update();

    if (symbol_ready != 0U)
    {
      uint16_t start;

      __disable_irq();
      start = symbol_start_index;
      symbol_ready = 0U;
      __enable_irq();

      FSK_ProcessSymbol(&adc_dma_buf[start]);
    }
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
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
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
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
