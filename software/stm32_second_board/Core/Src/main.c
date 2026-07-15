/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Second board 4-FSK RX/TX with SPI DAC and PGA.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#if defined(__ARMCC_VERSION)
#include <rt_sys.h>
#endif
/* USER CODE END Includes */

/* USER CODE BEGIN Semihosting */
extern UART_HandleTypeDef huart2;

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

FILEHANDLE _sys_open(const char *name, int openmode)
{
  (void)name;
  (void)openmode;
  return 1;
}

int _sys_close(FILEHANDLE fh)
{
  (void)fh;
  return 0;
}

int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode)
{
  (void)fh;
  (void)mode;
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
  return 0;
}

int _sys_read(FILEHANDLE fh, unsigned char *buf, unsigned len, int mode)
{
  (void)fh;
  (void)buf;
  (void)len;
  (void)mode;
  return 0;
}

int _sys_istty(FILEHANDLE fh)
{
  (void)fh;
  return 1;
}

int _sys_seek(FILEHANDLE fh, long pos)
{
  (void)fh;
  (void)pos;
  return -1;
}

long _sys_flen(FILEHANDLE fh)
{
  (void)fh;
  return -1;
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
  float data_energy[4U];
  float adc_mean;
  uint16_t adc_min;
  uint16_t adc_max;
} FSK_DetectResult;

typedef enum
{
  MSG_RX_SEARCH = 0,
  MSG_RX_START_0,
  MSG_RX_START_1,
  MSG_RX_RS_SYNC,
  MSG_RX_RS_CODEWORD,
  MSG_RX_END_0,
  MSG_RX_END_1
} MessageRxState;

typedef enum
{
  TX_MODE_IDLE = 0,
  TX_MODE_CALIBRATION,
  TX_MODE_FRAME
} TX_Mode;

typedef enum
{
  APP_MODE_RX = 0,
  APP_MODE_TX
} AppMode;

typedef enum
{
  TX_FRAME_TONE = 0,
  TX_FRAME_GAP
} TX_FramePart;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FSK_FS_HZ                 16000U
#define FSK_SYMBOL_MS             20U
#define FSK_SYMBOL_SAMPLES        ((FSK_FS_HZ * FSK_SYMBOL_MS) / 1000U)
#define ADC_DMA_BUFFER_SAMPLES    (FSK_SYMBOL_SAMPLES * 2U)

#define FSK_FREQ_COUNT            5U
#define FSK_DATA_FREQ_COUNT       4U
#define FSK_RX_FREQ_00_HZ         1500U
#define FSK_SYNC_FREQ_HZ          2000U
#define FSK_RX_FREQ_01_HZ         2500U
#define FSK_RX_FREQ_10_HZ         3500U
#define FSK_RX_FREQ_11_HZ         4500U
#define FSK_SYNC_SYMBOL           0x04U
#define FSK_ENERGY_THRESHOLD      5000000.0f
#define FSK_RATIO_THRESHOLD       1.3f
#define FSK_NOISE_MULTIPLIER      3.0f
#define FSK_NOISE_MIN_FLOOR       3000000.0f
#define FSK_SOFT_ENERGY_THRESHOLD 2500000.0f
#define FSK_SOFT_RATIO_THRESHOLD  1.05f
#define FSK_SOFT_NOISE_MULTIPLIER 1.15f
#define END1_SOFT_ACCEPT_MIN_MS    20U
#define END1_SOFT_ACCEPT_MAX_MS    210U

#define UART_IDLE_LOG_MS          1000U
#define OLED_IDLE_UPDATE_MS       1000U
#define OLED_VALID_UPDATE_MS      200U

#define DEBUG_UART_ENABLE         1U
#define FSK_PI                    3.14159265358979323846f
#define PREAMBLE_STABLE_WINDOWS   1U
#define START_STABLE_WINDOWS      2U
#define DIGIT_GAP_INVALID_WINDOWS 1U
#define RX_SAME_SYMBOL_REACCEPT_MS 65U
#define DIGIT_FRAME_TIMEOUT_MS    20000U
#define START_SEQUENCE_TIMEOUT_MS 400U
#define DIGIT_NO_SYMBOL           0xFFU
#define MSG_PREAMBLE_LEN          4U
#define MSG_START_LEN             2U
#define MSG_END_LEN               2U
#define RS_DATA_SYMBOLS           24U
#define RS_PARITY_SYMBOLS         10U
#define RS_CODEWORD_SYMBOLS       (RS_DATA_SYMBOLS + RS_PARITY_SYMBOLS)
#define RS_CORRECTABLE_SYMBOLS    (RS_PARITY_SYMBOLS / 2U)
#define RS_BLOCK_COUNT            2U
#define RS_TOTAL_DATA_SYMBOLS     (RS_DATA_SYMBOLS * RS_BLOCK_COUNT)
#define RS_TOTAL_PARITY_SYMBOLS   (RS_PARITY_SYMBOLS * RS_BLOCK_COUNT)
#define RS_TOTAL_CODEWORD_SYMBOLS (RS_CODEWORD_SYMBOLS * RS_BLOCK_COUNT)
#define RS_TOTAL_CORRECTABLE      (RS_CORRECTABLE_SYMBOLS * RS_BLOCK_COUNT)
#define RS_FSK_SYMBOLS_PER_SYMBOL 3U
#define RS_FIELD_ORDER            63U
#define RS_PRIMITIVE_POLY         0x43U
#define RS_PAD_VALUE              0x3FU
#define MESSAGE_MAX_LEN           RS_TOTAL_DATA_SYMBOLS
#define MESSAGE_CODE_INVALID      0xFFU
#define TEXT_TAP_TIMEOUT_MS       800U

#define TX_AUDIO_FS_HZ            32000U
#define TX_PWM_PERIOD             255U
#define TX_PWM_MID                128U
#define TX_SINE_TABLE_SIZE        64U
#define TX_MAX_SINE_AMP           126U
#define TX_DAC_COMMAND_ACTIVE_1X  0x3000U
#define TX_DAC_CODE_SHIFT         4U
#define TX_DAC_MID_CODE           ((uint16_t)TX_PWM_MID << TX_DAC_CODE_SHIFT)
#define RX_PGA_GAIN_CODE          0x04U
#define RX_PGA_GAIN_VALUE         8U
#define TX_TONE_MS                60U
#define TX_GAP_MS                 10U
#define TX_STEP_MS                (TX_TONE_MS + TX_GAP_MS)
#define TX_REPEAT_COUNT           1U
#define TX_FRAME_SYMBOLS_PER_REPEAT \
  (MSG_PREAMBLE_LEN + MSG_START_LEN + \
   (RS_TOTAL_CODEWORD_SYMBOLS * (RS_FSK_SYMBOLS_PER_SYMBOL + 1U)) + 1U + MSG_END_LEN)
#define TX_FRAME_MAX_SYMBOLS      (TX_FRAME_SYMBOLS_PER_REPEAT * TX_REPEAT_COUNT)
#define TX_FRAME_DURATION_MS      (TX_FRAME_MAX_SYMBOLS * TX_STEP_MS)
#define TX_TONE_SAMPLES           ((TX_AUDIO_FS_HZ * TX_TONE_MS) / 1000U)
#define TX_GAP_SAMPLES            ((TX_AUDIO_FS_HZ * TX_GAP_MS) / 1000U)
#define HALF_DUPLEX_RX_GUARD_MS   120U
#define RS_MARKER_INTERVAL_MS     ((RS_FSK_SYMBOLS_PER_SYMBOL + 1U) * TX_STEP_MS)
#define RS_SLOT_WIDTH_MS          TX_STEP_MS
#define RS_MARKER_ACCEPT_EARLY_MS 40U
#define RS_SLOT_CAPTURE_START_MS  40U
#define RS_SLOT_CAPTURE_END_MS    (RS_MARKER_INTERVAL_MS - 20U)
#define RS_SOFT_CAPTURE_START_MS  60U
#define RS_VOTE_RATIO_CAP         8.0f

#if TX_FRAME_DURATION_MS >= 20000U
#error "TX message frame must remain shorter than 20 seconds"
#endif

#define CALIBRATION_TONE_MS              1000U
#define CALIBRATION_TX_GUARD_MS          300U
#define CALIBRATION_TONE_SAMPLES         ((TX_AUDIO_FS_HZ * CALIBRATION_TONE_MS) / 1000U)
#define CALIBRATION_TX_GUARD_SAMPLES     ((TX_AUDIO_FS_HZ * CALIBRATION_TX_GUARD_MS) / 1000U)
#define CALIBRATION_STABLE_WINDOWS       5U
#define CALIBRATION_ENERGY_WINDOWS       12U
#define CALIBRATION_LOST_WINDOWS_MAX     3U
#define CALIBRATION_PROGRESS_TIMEOUT_MS  2500U
#define CALIBRATION_SCALE_MIN            0.005f
#define PREAMBLE_ENERGY_SCALE_MIN        0.005f
#define PREAMBLE_SCALE_NEW_WEIGHT        0.50f
#define PREAMBLE_RECOVERY_ENERGY_MIN     5000000.0f
#define PREAMBLE_RECOVERY_01_END_MS       70U
#define PREAMBLE_RECOVERY_10_START_MS     50U
#define PREAMBLE_RECOVERY_10_END_MS      140U
#define PREAMBLE_RECOVERY_11_START_MS    120U
#define PREAMBLE_RECOVERY_11_END_MS      210U
#define PREAMBLE_RECOVERY_START0_START_MS 190U
#define PREAMBLE_RECOVERY_START0_END_MS  300U
#define PREAMBLE_RECOVERY_START1_START_MS 260U
#define PREAMBLE_RECOVERY_START1_END_MS  330U
#define PREAMBLE_RECOVERY_MARKER_MIN_MS  300U
#define PREAMBLE_RECOVERY_MARKER_MAX_MS  400U

#define KEYPAD_ROWS               4U
#define KEYPAD_COLS               4U
#define KEYPAD_SCAN_MS            20U
#define KEYPAD_DEBOUNCE_SCANS     2U
#define KEY_NONE                  0U
#define POWER_OFF_HOLD_MS         600U

#define OLED_I2C_ADDR_3C          (0x3CU << 1)
#define OLED_I2C_ADDR_3D          (0x3DU << 1)
#define OLED_WIDTH                128U
#define OLED_HEIGHT               64U
#define OLED_PAGES                (OLED_HEIGHT / 8U)

#define MESSAGE_STORE_VISIBLE_COUNT 5U
#define MESSAGE_STORE_FLASH_START   0x08060000UL
#define MESSAGE_STORE_FLASH_END     0x08080000UL
#define MESSAGE_STORE_MAGIC         0x3147534DUL
#define MESSAGE_STORE_COMMIT        0xA55AC33CUL
#define RX_COMPLETE_LED_ON_MS       3000U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c3;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct
{
  uint32_t magic;
  uint32_t sequence;
  uint32_t length;
  uint8_t text[MESSAGE_MAX_LEN];
  uint32_t hash;
  uint32_t commit;
} MessageStoreRecord;

typedef struct
{
  uint32_t sequence;
  uint8_t len;
  char text[MESSAGE_MAX_LEN + 1U];
} StoredMessage;

static uint16_t adc_dma_buf[ADC_DMA_BUFFER_SAMPLES];
static volatile uint8_t symbol_ready = 0U;
static volatile uint16_t symbol_start_index = 0U;

static const uint16_t fsk_tx_freqs_hz[FSK_FREQ_COUNT] = {
  FSK_RX_FREQ_00_HZ,
  FSK_RX_FREQ_01_HZ,
  FSK_RX_FREQ_10_HZ,
  FSK_RX_FREQ_11_HZ,
  FSK_SYNC_FREQ_HZ
};
static uint16_t fsk_rx_freqs_hz[FSK_FREQ_COUNT];
static const uint8_t fsk_bits[FSK_FREQ_COUNT] = {0x00U, 0x01U, 0x02U, 0x03U, FSK_SYNC_SYMBOL};
static const uint8_t msg_preamble[MSG_PREAMBLE_LEN] = {0x00U, 0x01U, 0x02U, 0x03U};
static const uint8_t msg_start[MSG_START_LEN] = {0x01U, 0x02U};
static const uint8_t msg_end[MSG_END_LEN] = {0x02U, 0x03U};
static float fsk_goertzel_coeffs[FSK_FREQ_COUNT];
static float fsk_energy_scale[FSK_FREQ_COUNT];

static float calibration_energy[FSK_DATA_FREQ_COUNT];
static float calibration_energy_sum = 0.0f;
static uint8_t calibration_energy_windows = 0U;
static uint8_t calibration_lost_windows = 0U;
static uint8_t calibration_stage = 0U;
static uint8_t calibration_stable_windows = 0U;
static uint8_t calibration_capture_active = 0U;
static uint8_t calibration_complete = 0U;
static uint32_t calibration_progress_tick = 0U;
static uint8_t preamble_scale_initialized = 0U;

static uint16_t oled_i2c_addr = OLED_I2C_ADDR_3C;
static uint8_t oled_ready = 0U;
static float fsk_noise_floor[FSK_FREQ_COUNT] = {
  FSK_NOISE_MIN_FLOOR,
  FSK_NOISE_MIN_FLOOR,
  FSK_NOISE_MIN_FLOOR,
  FSK_NOISE_MIN_FLOOR,
  FSK_NOISE_MIN_FLOOR
};

static uint8_t digit_candidate_bits = DIGIT_NO_SYMBOL;
static uint8_t digit_candidate_count = 0U;
static uint8_t digit_invalid_count = 0U;
static uint8_t digit_run_active = 0U;
static uint8_t digit_run_bits = DIGIT_NO_SYMBOL;
static uint32_t digit_last_accept_ms = 0U;
static MessageRxState msg_rx_state = MSG_RX_SEARCH;
static uint8_t msg_preamble_index = 0U;
static uint32_t msg_start_entry_tick = 0U;
static float msg_preamble_energy[FSK_DATA_FREQ_COUNT];
static float msg_preamble_raw_top1[FSK_DATA_FREQ_COUNT];
static float msg_preamble_raw_top2[FSK_DATA_FREQ_COUNT];
static uint8_t msg_preamble_raw_count[FSK_DATA_FREQ_COUNT];
static uint32_t msg_preamble_raw_last_tick[FSK_DATA_FREQ_COUNT];
static uint8_t msg_preamble_capture_active = 0U;
static uint8_t msg_preamble_capture_stage = 0U;
static uint8_t msg_preamble_recovery_active = 0U;
static uint32_t msg_preamble_recovery_tick = 0U;
static float msg_preamble_recovery_energy[FSK_DATA_FREQ_COUNT];
static float msg_preamble_recovery_start_energy[MSG_START_LEN];
static uint8_t msg_rs_symbol_index = 0U;
static uint8_t msg_rs_fsk_index = 0U;
static uint8_t msg_rs_symbol = 0U;
static uint8_t msg_rs_codeword[RS_TOTAL_CODEWORD_SYMBOLS];
static uint8_t msg_rs_current_erased = 0U;
static uint8_t msg_rs_erasure_count = 0U;
static uint8_t msg_soft_fallback_count = 0U;
static uint8_t msg_vote_override_count = 0U;
static uint32_t msg_marker_tick = 0U;
static uint8_t msg_capture_bits[RS_FSK_SYMBOLS_PER_SYMBOL];
static uint8_t msg_capture_valid[RS_FSK_SYMBOLS_PER_SYMBOL];
static uint8_t msg_capture_distance[RS_FSK_SYMBOLS_PER_SYMBOL];
static float msg_capture_scores[RS_FSK_SYMBOLS_PER_SYMBOL][4];
static uint8_t msg_soft_bits[RS_FSK_SYMBOLS_PER_SYMBOL];
static uint8_t msg_soft_valid[RS_FSK_SYMBOLS_PER_SYMBOL];
static float msg_soft_score[RS_FSK_SYMBOLS_PER_SYMBOL];
static uint32_t msg_frame_tick = 0U;
static char rx_message[MESSAGE_MAX_LEN + 1U];
static uint8_t rx_message_len = 0U;
static uint8_t rx_message_valid = 0U;
static uint8_t rx_rs_corrected = 0U;
static uint32_t rx_display_seq = 0U;
static uint32_t uart_symbol_seq = 0U;

static StoredMessage message_store_cache[MESSAGE_STORE_VISIBLE_COUNT];
static uint8_t message_store_count = 0U;
static uint8_t message_store_view_index = 0U;
static uint32_t message_store_next_sequence = 1U;
static uint32_t message_store_write_address = MESSAGE_STORE_FLASH_START;
static uint8_t message_store_pending = 0U;
static uint8_t message_store_pending_len = 0U;
static char message_store_pending_text[MESSAGE_MAX_LEN + 1U];
static uint8_t message_store_last_save_failed = 0U;

static uint8_t rx_complete_led_active = 0U;
static uint32_t rx_complete_led_off_tick = 0U;

static uint8_t rs_gf_exp[RS_FIELD_ORDER * 2U];
static uint8_t rs_gf_log[RS_FIELD_ORDER + 1U];
static uint8_t rs_generator[RS_PARITY_SYMBOLS + 1U];

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
static const uint8_t tx_fsk_amp[FSK_FREQ_COUNT] = {
  30U,
  72U,
  TX_MAX_SINE_AMP,
  TX_MAX_SINE_AMP,
  60U
};
static const uint8_t tx_preamble_symbols[MSG_PREAMBLE_LEN] = {0x00U, 0x01U, 0x02U, 0x03U};
static const uint8_t tx_start_symbols[MSG_START_LEN] = {0x01U, 0x02U};
static const uint8_t tx_end_symbols[MSG_END_LEN] = {0x02U, 0x03U};

static volatile TX_Mode tx_mode = TX_MODE_IDLE;
static AppMode app_mode = APP_MODE_RX;
static volatile TX_FramePart tx_frame_part = TX_FRAME_TONE;
static volatile uint32_t tx_phase = 0U;
static volatile uint32_t tx_phase_inc = 0U;
static volatile uint32_t tx_part_sample_count = 0U;
static volatile uint8_t tx_current_amp = 50U;
static volatile uint8_t tx_current_bits = 0U;
static volatile uint16_t tx_frame_index = 0U;
static volatile uint16_t tx_frame_len = 0U;
static volatile uint8_t tx_done_pending = 0U;
static volatile uint8_t tx_done_is_calibration = 0U;
static volatile uint8_t tx_calibration_stage = 0U;
static volatile uint8_t tx_calibration_sent = 0U;
static uint8_t tx_frame_symbols[TX_FRAME_MAX_SYMBOLS];
static uint8_t tx_last_valid = 0U;
static char tx_last_message[MESSAGE_MAX_LEN + 1U];
static uint8_t tx_last_len = 0U;
static uint8_t tx_display_sending = 0U;
static volatile uint8_t rx_sampling_active = 0U;
static uint8_t rx_resume_pending = 0U;
static uint32_t rx_resume_tick = 0U;

static char tx_text[MESSAGE_MAX_LEN + 1U];
static uint8_t tx_text_len = 0U;
static uint8_t tx_cursor_pos = 0U;
static uint8_t pending_active = 0U;
static char pending_key = 0;
static uint8_t pending_index = 0U;
static uint32_t pending_last_tick = 0U;

static uint8_t keypad_last_raw = KEY_NONE;
static uint8_t keypad_stable_key = KEY_NONE;
static uint8_t keypad_debounce_count = 0U;
static uint32_t keypad_last_scan_ms = 0U;

static GPIO_TypeDef *const keypad_row_ports[KEYPAD_ROWS] = {GPIOB, GPIOB, GPIOB, GPIOD};
static const uint16_t keypad_row_pins[KEYPAD_ROWS] = {GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_2};
static GPIO_TypeDef *const keypad_col_ports[KEYPAD_COLS] = {GPIOA, GPIOC, GPIOC, GPIOC};
static const uint16_t keypad_col_pins[KEYPAD_COLS] = {GPIO_PIN_15, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12};
static const uint8_t keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void FSK_Init(void);
static void FSK_UpdateGoertzelCoefficient(uint8_t index);
static float FSK_WindowMean(const uint16_t *samples, uint16_t len);
static float Goertzel_EnergyWithMean(const uint16_t *samples, uint16_t len, float coeff, float mean);
static FSK_DetectResult FSK_DetectSymbol(const uint16_t *samples, uint16_t len);
static void FSK_ProcessSymbol(const uint16_t *samples);
static void Calibration_ResetProgress(void);
static void Calibration_Apply(void);
static void Calibration_ProcessBlock(const uint16_t *samples,
                                     const FSK_DetectResult *result,
                                     uint32_t now);
static const char *FSK_BitsToString(uint8_t bits);
static void MessageRx_Process(const FSK_DetectResult *result, uint32_t now);
static void MessageRx_AcceptSymbol(const FSK_DetectResult *result, uint32_t now);
static void MessageRx_ResetPreambleCapture(void);
static void MessageRx_AddPreambleRawWindow(uint8_t index,
                                           const FSK_DetectResult *result,
                                           uint32_t now);
static void MessageRx_CapturePreambleHardWindow(const FSK_DetectResult *result,
                                                 uint32_t now);
static void MessageRx_ApplyPreambleEnergyScale(void);
static void MessageRx_CaptureTimedPreamble(const FSK_DetectResult *result, uint32_t now);
static uint8_t MessageRx_TryTimedPreambleRecovery(uint32_t now);
static uint8_t MessageRx_DataWindowPosition(uint32_t now, uint8_t *slot,
                                             uint8_t *distance);
static uint8_t MessageRx_CaptureDataWindow(const FSK_DetectResult *result, uint32_t now);
static uint8_t MessageRx_SelectSoftDataCandidate(const FSK_DetectResult *result,
                                                  uint8_t *bits, float *max_energy,
                                                  float *second_energy, float *confidence);
static void MessageRx_CaptureSoftDataWindow(const FSK_DetectResult *result, uint32_t now);
static uint8_t MessageRx_TrySoftEnd1(const FSK_DetectResult *result, uint32_t now);
static void MessageRx_AppendRsBits(uint8_t decoded_bits, uint8_t erased);
static void MessageRx_ClearCapture(void);
static void MessageRx_FinalizeCapture(void);
static void MessageRx_ResetFrame(void);
static const char *MessageRx_StateName(MessageRxState state);
static void MessageRx_PrintSymbol(const FSK_DetectResult *result, uint32_t now);
static uint8_t Message_WhiteningMask(uint16_t fsk_symbol_index);
static uint8_t Message_CharToCode(char ch);
static char Message_CodeToChar(uint8_t code);
static uint32_t MessageStore_HashRecord(const MessageStoreRecord *record);
static uint8_t MessageStore_RecordIsErased(uint32_t address);
static uint8_t MessageStore_RecordValid(const MessageStoreRecord *record);
static void MessageStore_CacheAppend(uint32_t sequence, const char *text, uint8_t len);
static void MessageStore_BuildRecord(MessageStoreRecord *record, uint32_t sequence,
                                     const char *text, uint8_t len);
static uint8_t MessageStore_ProgramRecordUnlocked(uint32_t address,
                                                  const MessageStoreRecord *record);
static uint8_t MessageStore_CompactUnlocked(void);
static void MessageStore_Init(void);
static uint8_t MessageStore_Save(const char *text, uint8_t len);
static void MessageStore_Task(void);
static void MessageStore_HandleRxKey(char key);
static void RS_Init(void);
static uint8_t RS_GfMul(uint8_t a, uint8_t b);
static uint8_t RS_GfDiv(uint8_t a, uint8_t b);
static uint8_t RS_PolyEvalAscending(const uint8_t *poly, uint8_t degree, uint8_t x);
static void RS_CalculateSyndromes(const uint8_t *codeword, uint8_t *syndromes);
static void RS_Encode(const uint8_t *data, uint8_t *codeword);
static int8_t RS_Decode(uint8_t *codeword, uint8_t *corrected_count);
static uint8_t RS_SelfTest(void);
static uint8_t OLED_Init(void);
static void OLED_Clear(void);
static void OLED_ClearLine(uint8_t page);
static void OLED_WriteCommand(uint8_t command);
static void OLED_WriteData(const uint8_t *data, uint16_t len);
static void OLED_SetCursor(uint8_t col, uint8_t page);
static void OLED_PrintAt(uint8_t col, uint8_t page, const char *text);
static void OLED_PrintLine(uint8_t page, const char *text);
static void OLED_PrintChar(char ch);
static const uint8_t *OLED_Font5x7(char ch);
static void OLED_PrintDetected(const FSK_DetectResult *result);
static void OLED_PrintMode(void);
static void OLED_PrintTextRows(const char *text, uint8_t len);
static void OLED_PrintRxStatus(void);
static void OLED_PrintTxStatus(void);
static void TX_StartCalibration(void);
static void TX_StartMessage(void);
static void TX_Stop(void);
static void TX_AudioTick(void);
static void TX_LoadFrameSymbol(uint16_t index);
static void TX_LoadCalibrationTone(uint8_t stage);
static void TX_BuildMessageFrame(void);
static uint32_t TX_PhaseIncFromFreq(uint16_t freq_hz);
static uint8_t TX_ScaledSineSample(void);
static void SPI1_Write16(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t word);
static void DAC_Write12(uint16_t code);
static void DAC_WriteSample8(uint8_t sample);
static void PGA_SetGain(uint8_t gain_code);
static void TX_UI_Task(void);
static uint8_t RX_StartSampling(void);
static void RX_StopSampling(void);
static void RX_ResetDetector(void);
static void HalfDuplex_Task(void);
static void App_ToggleMode(void);
static void Editor_Task(void);
static void Editor_ProcessKey(char key);
static const char *Editor_KeyGroup(char key);
static uint8_t Editor_KeyGroupLen(const char *group);
static char Editor_PendingChar(void);
static void Editor_ConfirmPending(void);
static void Editor_ClearPending(void);
static void Editor_Insert(char ch);
static void Editor_Delete(void);
static void Keypad_Task(void);
static uint8_t Keypad_ScanRaw(void);
static void Keypad_HandlePress(uint8_t key);
static void StatusLed_Update(void);
static void StatusLed_NotifyRxComplete(void);
static void PowerHold_EarlyOn(void);
static void PowerKey_Task(void);
static void App_PowerOff(void);
static void OLED_RetryInitTask(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void PowerHold_EarlyOn(void)
{
  /* Keep Q1 on as soon as possible after reset, before HAL/system init. */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  __DSB();

  GPIOC->BSRR = POWER_HOLD_Pin;
  GPIOC->MODER = (GPIOC->MODER & ~(3UL << (1U * 2U))) | (1UL << (1U * 2U));
  GPIOC->OTYPER &= ~POWER_HOLD_Pin;
  GPIOC->OSPEEDR &= ~(3UL << (1U * 2U));
  GPIOC->PUPDR &= ~(3UL << (1U * 2U));
  GPIOC->BSRR = POWER_HOLD_Pin;
}

static void PowerKey_Task(void)
{
  static uint8_t armed_after_release = 0U;
  static uint8_t poweroff_started = 0U;
  static uint32_t press_start_ms = 0U;
  uint8_t pressed;
  uint32_t now = HAL_GetTick();

  pressed = (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == POWER_KEY_PRESSED) ? 1U : 0U;

  if (pressed == 0U)
  {
    armed_after_release = 1U;
    press_start_ms = 0U;
    return;
  }

  if (armed_after_release == 0U)
  {
    return;
  }

  if (press_start_ms == 0U)
  {
    press_start_ms = now;
  }

  if ((poweroff_started == 0U) && ((now - press_start_ms) >= POWER_OFF_HOLD_MS))
  {
    poweroff_started = 1U;
    App_PowerOff();
  }
}

static void App_PowerOff(void)
{
  printf("power key pressed: PC1 low, release SW2 to power off\r\n");

  TX_Stop();
  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);

  OLED_Clear();
  OLED_PrintAt(0U, 0U, "VOICE MSG");
  OLED_PrintAt(0U, 2U, "PWR OFF");
  OLED_PrintAt(0U, 4U, "LET GO");
  HAL_Delay(150U);
  OLED_WriteCommand(0xAEU);

  HAL_GPIO_WritePin(POWER_HOLD_GPIO_Port, POWER_HOLD_Pin, POWER_HOLD_OFF);

  while (1)
  {
    HAL_GPIO_WritePin(POWER_HOLD_GPIO_Port, POWER_HOLD_Pin, POWER_HOLD_OFF);
  }
}

static void OLED_RetryInitTask(void)
{
  static uint32_t last_try_ms = 0U;
  uint32_t now = HAL_GetTick();

  if (oled_ready != 0U)
  {
    return;
  }

  if ((now - last_try_ms) < 500U)
  {
    return;
  }
  last_try_ms = now;

  if (OLED_Init() != 0U)
  {
    OLED_PrintMode();
  }
}

static void FSK_Init(void)
{
  Calibration_ResetProgress();
}

static void FSK_UpdateGoertzelCoefficient(uint8_t index)
{
  if (index >= FSK_FREQ_COUNT)
  {
    return;
  }

  fsk_goertzel_coeffs[index] =
      2.0f * cosf((2.0f * FSK_PI * (float)fsk_rx_freqs_hz[index]) / (float)FSK_FS_HZ);
}

static void Calibration_ResetProgress(void)
{
  calibration_energy_sum = 0.0f;
  calibration_energy_windows = 0U;
  calibration_lost_windows = 0U;
  calibration_stage = 0U;
  calibration_stable_windows = 0U;
  calibration_capture_active = 0U;
  calibration_complete = 0U;
  calibration_progress_tick = HAL_GetTick();
  preamble_scale_initialized = 0U;
  memset(calibration_energy, 0, sizeof(calibration_energy));
  MessageRx_ResetPreambleCapture();

  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    fsk_rx_freqs_hz[i] = fsk_tx_freqs_hz[i];
    fsk_energy_scale[i] = 1.0f;
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
    FSK_UpdateGoertzelCoefficient(i);
  }
}

static float FSK_WindowMean(const uint16_t *samples, uint16_t len)
{
  uint32_t sum = 0U;

  for (uint16_t i = 0U; i < len; i++)
  {
    sum += samples[i];
  }

  return (float)sum / (float)len;
}

static float Goertzel_EnergyWithMean(const uint16_t *samples, uint16_t len, float coeff, float mean)
{
  float s0 = 0.0f;
  float s1 = 0.0f;
  float s2 = 0.0f;

  for (uint16_t i = 0U; i < len; i++)
  {
    float x = (float)samples[i] - mean;
    s0 = x + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }

  return (s1 * s1) + (s2 * s2) - (coeff * s1 * s2);
}

static void Calibration_Apply(void)
{
  float reference_energy = calibration_energy[0];

  for (uint8_t i = 1U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    if (calibration_energy[i] < reference_energy)
    {
      reference_energy = calibration_energy[i];
    }
  }
  if (reference_energy <= 0.0f)
  {
    printf("calibration apply failed: invalid reference energy\r\n");
    Calibration_ResetProgress();
    return;
  }

  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    float scale = reference_energy / calibration_energy[i];

    if (scale < CALIBRATION_SCALE_MIN)
    {
      scale = CALIBRATION_SCALE_MIN;
    }

    fsk_rx_freqs_hz[i] = fsk_tx_freqs_hz[i];
    fsk_energy_scale[i] = scale;
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
    FSK_UpdateGoertzelCoefficient(i);
  }

  fsk_rx_freqs_hz[FSK_DATA_FREQ_COUNT] = fsk_tx_freqs_hz[FSK_DATA_FREQ_COUNT];
  fsk_energy_scale[FSK_DATA_FREQ_COUNT] = 1.0f;
  fsk_noise_floor[FSK_DATA_FREQ_COUNT] = FSK_NOISE_MIN_FLOOR;
  FSK_UpdateGoertzelCoefficient(FSK_DATA_FREQ_COUNT);
  calibration_complete = 1U;
  preamble_scale_initialized = 0U;
  RX_ResetDetector();

  printf("calibration complete: fixed-frequency Goertzel reference=min_mean\r\n");
  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    printf("cal %s: G=%uHz meanE/1e6=%u scale=%lu.%03lu\r\n",
           FSK_BitsToString(i),
           fsk_rx_freqs_hz[i],
           (unsigned int)(calibration_energy[i] / 1000000.0f),
           (unsigned long)((uint32_t)(fsk_energy_scale[i] * 1000.0f) / 1000U),
           (unsigned long)((uint32_t)(fsk_energy_scale[i] * 1000.0f) % 1000U));
  }
  OLED_PrintRxStatus();
}

static void Calibration_ProcessBlock(const uint16_t *samples,
                                     const FSK_DetectResult *result,
                                     uint32_t now)
{
  (void)samples;

  if (calibration_complete != 0U)
  {
    return;
  }

  if ((calibration_stage > 0U) &&
      ((now - calibration_progress_tick) > CALIBRATION_PROGRESS_TIMEOUT_MS))
  {
    printf("calibration timeout after %u/4 tones, restart at 00\r\n", calibration_stage);
    Calibration_ResetProgress();
  }

  if (calibration_capture_active != 0U)
  {
    if ((result->valid == 0U) || (result->bits != calibration_stage))
    {
      if (calibration_lost_windows < 255U)
      {
        calibration_lost_windows++;
      }
      if (calibration_lost_windows >= CALIBRATION_LOST_WINDOWS_MAX)
      {
        calibration_capture_active = 0U;
        calibration_energy_sum = 0.0f;
        calibration_energy_windows = 0U;
        calibration_lost_windows = 0U;
        calibration_stable_windows = 0U;
        printf("calibration G tone %u lost, retry current tone\r\n",
               calibration_stage + 1U);
      }
      return;
    }

    calibration_lost_windows = 0U;
    calibration_energy_sum += result->data_energy[calibration_stage];
    calibration_energy_windows++;

    if (calibration_energy_windows >= CALIBRATION_ENERGY_WINDOWS)
    {
      calibration_capture_active = 0U;
      calibration_stable_windows = 0U;
      calibration_energy[calibration_stage] =
          calibration_energy_sum / (float)calibration_energy_windows;
      printf("calibration G %s: freq=%uHz windows=%u meanE/1e6=%u\r\n",
             FSK_BitsToString(calibration_stage),
             fsk_tx_freqs_hz[calibration_stage],
             calibration_energy_windows,
             (unsigned int)(calibration_energy[calibration_stage] / 1000000.0f));
      calibration_energy_sum = 0.0f;
      calibration_energy_windows = 0U;
      calibration_lost_windows = 0U;
      calibration_stage++;
      calibration_progress_tick = now;
      if (calibration_stage >= FSK_DATA_FREQ_COUNT)
      {
        Calibration_Apply();
      }
    }
    return;
  }

  if ((result->valid != 0U) && (result->bits == calibration_stage))
  {
    if (calibration_stable_windows < CALIBRATION_STABLE_WINDOWS)
    {
      calibration_stable_windows++;
    }
    if (calibration_stable_windows >= CALIBRATION_STABLE_WINDOWS)
    {
      calibration_capture_active = 1U;
      calibration_energy_sum = 0.0f;
      calibration_energy_windows = 0U;
      calibration_lost_windows = 0U;
      calibration_stable_windows = 0U;
      printf("calibration G capture %s: freq=%uHz windows=%u\r\n",
             FSK_BitsToString(calibration_stage),
             fsk_tx_freqs_hz[calibration_stage], CALIBRATION_ENERGY_WINDOWS);
    }
  }
  else
  {
    calibration_stable_windows = 0U;
  }
}

static FSK_DetectResult FSK_DetectSymbol(const uint16_t *samples, uint16_t len)
{
  FSK_DetectResult result = {0};
  float energies[FSK_FREQ_COUNT];
  float mean = FSK_WindowMean(samples, len);
  uint16_t adc_min = 0xFFFFU;
  uint16_t adc_max = 0U;
  uint8_t max_index = 0U;
  uint8_t second_index = 1U;

  for (uint16_t i = 0U; i < len; i++)
  {
    if (samples[i] < adc_min)
    {
      adc_min = samples[i];
    }
    if (samples[i] > adc_max)
    {
      adc_max = samples[i];
    }
  }

  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    energies[i] = Goertzel_EnergyWithMean(samples, len, fsk_goertzel_coeffs[i], mean) *
                  fsk_energy_scale[i];
  }
  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    result.data_energy[i] = energies[i];
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

  result.freq_hz = fsk_rx_freqs_hz[max_index];
  result.bits = fsk_bits[max_index];
  result.max_energy = energies[max_index];
  result.second_energy = energies[second_index];
  result.adc_mean = mean;
  result.adc_min = adc_min;
  result.adc_max = adc_max;

  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    float floor = fsk_noise_floor[i];

    if (energies[i] > floor)
    {
      floor = (0.995f * floor) + (0.005f * energies[i]);
    }
    else
    {
      floor = (0.95f * floor) + (0.05f * energies[i]);
    }

    if (floor < FSK_NOISE_MIN_FLOOR)
    {
      floor = FSK_NOISE_MIN_FLOOR;
    }

    fsk_noise_floor[i] = floor;
  }

  if ((result.max_energy > FSK_ENERGY_THRESHOLD) &&
      (result.max_energy > (result.second_energy * FSK_RATIO_THRESHOLD)) &&
      (result.max_energy > (fsk_noise_floor[max_index] * FSK_NOISE_MULTIPLIER)))
  {
    result.valid = 1U;
  }

  return result;
}

static void FSK_ProcessSymbol(const uint16_t *samples)
{
  FSK_DetectResult result = FSK_DetectSymbol(samples, FSK_SYMBOL_SAMPLES);
  static uint32_t last_idle_log = 0U;
  static uint32_t last_oled_rx_seq = 0U;
  uint32_t now = HAL_GetTick();

  if (calibration_complete == 0U)
  {
    Calibration_ProcessBlock(samples, &result, now);
  }
  else
  {
    MessageRx_Process(&result, now);
  }

#if DEBUG_UART_ENABLE
  if ((result.valid == 0U) && ((now - last_idle_log) >= UART_IDLE_LOG_MS))
  {
    last_idle_log = now;
    printf("idle: Emax=%u, E2=%u, ADC mean=%u min=%u max=%u\r\n",
           (unsigned int)result.max_energy,
           (unsigned int)result.second_energy,
           (unsigned int)result.adc_mean,
           result.adc_min,
           result.adc_max);
  }
#endif

  if (rx_display_seq != last_oled_rx_seq)
  {
    last_oled_rx_seq = rx_display_seq;
    OLED_PrintDetected(&result);
  }
}

static const char *FSK_BitsToString(uint8_t bits)
{
  if (bits == DIGIT_NO_SYMBOL)
  {
    return "--";
  }
  if (bits == FSK_SYNC_SYMBOL)
  {
    return "SYNC";
  }
  if (bits < 4U)
  {
    static const char *bit_text[4] = {"00", "01", "10", "11"};
    return bit_text[bits];
  }
  return "?";
}

static const char *MessageRx_StateName(MessageRxState state)
{
  switch (state)
  {
    case MSG_RX_SEARCH:
      return "SEARCH";
    case MSG_RX_START_0:
      return "START0";
    case MSG_RX_START_1:
      return "START1";
    case MSG_RX_RS_SYNC:
      return "RS_SYNC";
    case MSG_RX_RS_CODEWORD:
      return "RS_DATA";
    case MSG_RX_END_0:
      return "END0";
    case MSG_RX_END_1:
      return "END1";
    default:
      return "UNKNOWN";
  }
}

static void MessageRx_PrintSymbol(const FSK_DetectResult *result, uint32_t now)
{
  uint32_t ratio_x100 = 99999U;

  if (result->second_energy > 1.0f)
  {
    ratio_x100 = (uint32_t)((result->max_energy * 100.0f) / result->second_energy);
  }

  uart_symbol_seq++;
  printf("symbol #%lu: t=%lums state=%s, %uHz -> %s, Emax=%u, E2=%u, R=%lu.%02lu, N=%u, ADC mean=%u min=%u max=%u\r\n",
         (unsigned long)uart_symbol_seq,
         (unsigned long)now,
         MessageRx_StateName(msg_rx_state),
         result->freq_hz,
         FSK_BitsToString(result->bits),
         (unsigned int)result->max_energy,
         (unsigned int)result->second_energy,
         (unsigned long)(ratio_x100 / 100U),
         (unsigned long)(ratio_x100 % 100U),
         (unsigned int)fsk_noise_floor[result->bits],
         (unsigned int)result->adc_mean,
         result->adc_min,
         result->adc_max);
}

static void MessageRx_Process(const FSK_DetectResult *result, uint32_t now)
{
  uint8_t required_windows;

  if (msg_rx_state == MSG_RX_SEARCH)
  {
    MessageRx_CaptureTimedPreamble(result, now);
  }
  MessageRx_CapturePreambleHardWindow(result, now);

  if ((msg_rx_state != MSG_RX_SEARCH) && ((now - msg_frame_tick) > DIGIT_FRAME_TIMEOUT_MS))
  {
    printf("rx timeout\r\n");
    MessageRx_ResetFrame();
  }

  if (((msg_rx_state == MSG_RX_START_0) || (msg_rx_state == MSG_RX_START_1)) &&
      (msg_start_entry_tick != 0U) &&
      ((now - msg_start_entry_tick) > START_SEQUENCE_TIMEOUT_MS))
  {
    printf("rx start sequence timeout\r\n");
    MessageRx_ResetFrame();
  }

  if (msg_rx_state == MSG_RX_RS_CODEWORD)
  {
    if (result->valid == 0U)
    {
      MessageRx_CaptureSoftDataWindow(result, now);
    }
    else if (result->bits == FSK_SYNC_SYMBOL)
    {
      uint32_t marker_delta = now - msg_marker_tick;

      if (marker_delta >= (RS_MARKER_INTERVAL_MS - RS_MARKER_ACCEPT_EARLY_MS))
      {
        MessageRx_AcceptSymbol(result, now);
      }
      else
      {
        MessageRx_CaptureSoftDataWindow(result, now);
      }
    }
    else
    {
      msg_frame_tick = now;
      if (MessageRx_CaptureDataWindow(result, now) != 0U)
      {
        MessageRx_PrintSymbol(result, now);
      }
    }
    return;
  }

  if (result->valid == 0U)
  {
    if ((msg_rx_state == MSG_RX_END_1) && (MessageRx_TrySoftEnd1(result, now) != 0U))
    {
      return;
    }
    if (digit_invalid_count < 255U)
    {
      digit_invalid_count++;
    }

    if (digit_invalid_count >= DIGIT_GAP_INVALID_WINDOWS)
    {
      digit_candidate_bits = DIGIT_NO_SYMBOL;
      digit_candidate_count = 0U;
      digit_run_active = 0U;
    }

    return;
  }

  digit_invalid_count = 0U;

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

  if (msg_rx_state == MSG_RX_SEARCH)
  {
    /* A 60 ms PREAMBLE tone spans only three 20 ms detector windows.  Requiring
       two consecutive hard decisions occasionally drops a whole PREAMBLE tone
       when one of those windows straddles a tone/gap transition.  The complete
       00/01/10/11 order plus the two START tones still validates frame entry. */
    required_windows = PREAMBLE_STABLE_WINDOWS;
  }
  else if ((msg_rx_state == MSG_RX_START_0) ||
           (msg_rx_state == MSG_RX_START_1))
  {
    required_windows = START_STABLE_WINDOWS;
  }
  else
  {
    required_windows = 1U;
  }
  if (digit_candidate_count < required_windows)
  {
    return;
  }

  if ((digit_run_active != 0U) &&
      (digit_run_bits == result->bits) &&
      ((now - digit_last_accept_ms) < RX_SAME_SYMBOL_REACCEPT_MS))
  {
    return;
  }

  digit_run_active = 1U;
  digit_run_bits = result->bits;
  digit_last_accept_ms = now;
  MessageRx_AcceptSymbol(result, now);
}

static void MessageRx_ResetPreambleCapture(void)
{
  memset(msg_preamble_raw_top1, 0, sizeof(msg_preamble_raw_top1));
  memset(msg_preamble_raw_top2, 0, sizeof(msg_preamble_raw_top2));
  memset(msg_preamble_raw_count, 0, sizeof(msg_preamble_raw_count));
  memset(msg_preamble_raw_last_tick, 0, sizeof(msg_preamble_raw_last_tick));
  msg_preamble_capture_active = 0U;
  msg_preamble_capture_stage = 0U;
}

static void MessageRx_AddPreambleRawWindow(uint8_t index,
                                           const FSK_DetectResult *result,
                                           uint32_t now)
{
  float raw_energy;

  if ((index >= FSK_DATA_FREQ_COUNT) || (fsk_energy_scale[index] <= 0.0f))
  {
    return;
  }
  if ((msg_preamble_raw_count[index] != 0U) &&
      (msg_preamble_raw_last_tick[index] == now))
  {
    return;
  }

  raw_energy = result->data_energy[index] / fsk_energy_scale[index];
  if (raw_energy <= 0.0f)
  {
    return;
  }

  if (raw_energy >= msg_preamble_raw_top1[index])
  {
    msg_preamble_raw_top2[index] = msg_preamble_raw_top1[index];
    msg_preamble_raw_top1[index] = raw_energy;
  }
  else if (raw_energy > msg_preamble_raw_top2[index])
  {
    msg_preamble_raw_top2[index] = raw_energy;
  }
  if (msg_preamble_raw_count[index] < 255U)
  {
    msg_preamble_raw_count[index]++;
  }
  msg_preamble_raw_last_tick[index] = now;
}

static void MessageRx_CapturePreambleHardWindow(const FSK_DetectResult *result,
                                                 uint32_t now)
{
  if ((msg_preamble_capture_active != 0U) &&
      (msg_preamble_capture_stage < FSK_DATA_FREQ_COUNT) &&
      (result->valid != 0U) &&
      (result->bits == msg_preamble_capture_stage))
  {
    MessageRx_AddPreambleRawWindow(msg_preamble_capture_stage, result, now);
  }
}

static void MessageRx_ApplyPreambleEnergyScale(void)
{
  float raw_energy[FSK_DATA_FREQ_COUNT];
  float target_scale[FSK_DATA_FREQ_COUNT];
  float applied_scale[FSK_DATA_FREQ_COUNT];
  float reference_energy = 0.0f;
  float max_applied_scale = 0.0f;

  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    if (msg_preamble_raw_count[i] < 2U)
    {
      printf("preamble energy update skipped: %s has only %u raw window(s)\r\n",
             FSK_BitsToString(i), msg_preamble_raw_count[i]);
      return;
    }
    else
    {
      raw_energy[i] = 0.5f * (msg_preamble_raw_top1[i] + msg_preamble_raw_top2[i]);
    }

    if ((i == 0U) || (raw_energy[i] < reference_energy))
    {
      reference_energy = raw_energy[i];
    }
  }
  if (reference_energy <= 0.0f)
  {
    printf("preamble energy update skipped: invalid reference\r\n");
    return;
  }

  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    target_scale[i] = reference_energy / raw_energy[i];
    if (target_scale[i] < PREAMBLE_ENERGY_SCALE_MIN)
    {
      target_scale[i] = PREAMBLE_ENERGY_SCALE_MIN;
    }

    if (preamble_scale_initialized == 0U)
    {
      applied_scale[i] = target_scale[i];
    }
    else
    {
      applied_scale[i] = ((1.0f - PREAMBLE_SCALE_NEW_WEIGHT) * fsk_energy_scale[i]) +
                         (PREAMBLE_SCALE_NEW_WEIGHT * target_scale[i]);
    }
    if (applied_scale[i] > max_applied_scale)
    {
      max_applied_scale = applied_scale[i];
    }
  }
  if (max_applied_scale <= 0.0f)
  {
    printf("preamble energy update skipped: invalid applied scale\r\n");
    return;
  }

  printf("preamble energy update: top-2 raw windows, %s\r\n",
         (preamble_scale_initialized == 0U) ? "first absolute" : "50pct smoothed");
  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    uint32_t target_x1000 = (uint32_t)(target_scale[i] * 1000.0f);
    uint32_t scale_x1000;

    applied_scale[i] /= max_applied_scale;
    if (applied_scale[i] < PREAMBLE_ENERGY_SCALE_MIN)
    {
      applied_scale[i] = PREAMBLE_ENERGY_SCALE_MIN;
    }
    fsk_energy_scale[i] = applied_scale[i];
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
    scale_x1000 = (uint32_t)(applied_scale[i] * 1000.0f);
    printf("preamble %s: rawE/1e6=%u windows=%u target=%lu.%03lu scale=%lu.%03lu\r\n",
           FSK_BitsToString(i),
           (unsigned int)(raw_energy[i] / 1000000.0f),
           (unsigned int)msg_preamble_raw_count[i],
           (unsigned long)(target_x1000 / 1000U),
           (unsigned long)(target_x1000 % 1000U),
           (unsigned long)(scale_x1000 / 1000U),
           (unsigned long)(scale_x1000 % 1000U));
  }
  preamble_scale_initialized = 1U;
}

static void MessageRx_CaptureTimedPreamble(const FSK_DetectResult *result, uint32_t now)
{
  uint32_t delta;

  if (msg_preamble_recovery_active == 0U)
  {
    return;
  }

  delta = now - msg_preamble_recovery_tick;
  if (delta > PREAMBLE_RECOVERY_MARKER_MAX_MS)
  {
    msg_preamble_recovery_active = 0U;
    return;
  }

  if (delta <= PREAMBLE_RECOVERY_01_END_MS)
  {
    MessageRx_AddPreambleRawWindow(1U, result, now);
  }

  if ((delta >= PREAMBLE_RECOVERY_10_START_MS) &&
      (delta <= PREAMBLE_RECOVERY_10_END_MS) &&
      (result->data_energy[2] > msg_preamble_recovery_energy[2]))
  {
    msg_preamble_recovery_energy[2] = result->data_energy[2];
  }
  if ((delta >= PREAMBLE_RECOVERY_10_START_MS) &&
      (delta <= PREAMBLE_RECOVERY_10_END_MS))
  {
    MessageRx_AddPreambleRawWindow(2U, result, now);
  }
  if ((delta >= PREAMBLE_RECOVERY_11_START_MS) &&
      (delta <= PREAMBLE_RECOVERY_11_END_MS) &&
      (result->data_energy[3] > msg_preamble_recovery_energy[3]))
  {
    msg_preamble_recovery_energy[3] = result->data_energy[3];
  }
  if ((delta >= PREAMBLE_RECOVERY_11_START_MS) &&
      (delta <= PREAMBLE_RECOVERY_11_END_MS))
  {
    MessageRx_AddPreambleRawWindow(3U, result, now);
  }
  if ((delta >= PREAMBLE_RECOVERY_START0_START_MS) &&
      (delta <= PREAMBLE_RECOVERY_START0_END_MS) &&
      (result->data_energy[msg_start[0]] > msg_preamble_recovery_start_energy[0]))
  {
    msg_preamble_recovery_start_energy[0] = result->data_energy[msg_start[0]];
  }
  if ((delta >= PREAMBLE_RECOVERY_START1_START_MS) &&
      (delta <= PREAMBLE_RECOVERY_START1_END_MS) &&
      (result->data_energy[msg_start[1]] > msg_preamble_recovery_start_energy[1]))
  {
    msg_preamble_recovery_start_energy[1] = result->data_energy[msg_start[1]];
  }
}

static uint8_t MessageRx_TryTimedPreambleRecovery(uint32_t now)
{
  uint32_t delta;

  if (msg_preamble_recovery_active == 0U)
  {
    return 0U;
  }

  delta = now - msg_preamble_recovery_tick;
  if ((delta < PREAMBLE_RECOVERY_MARKER_MIN_MS) ||
      (delta > PREAMBLE_RECOVERY_MARKER_MAX_MS))
  {
    return 0U;
  }

  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    if (msg_preamble_recovery_energy[i] < PREAMBLE_RECOVERY_ENERGY_MIN)
    {
      printf("timed preamble recovery rejected: delta=%lums weak=%s E/1e6=%u\r\n",
             (unsigned long)delta, FSK_BitsToString(i),
             (unsigned int)(msg_preamble_recovery_energy[i] / 1000000.0f));
      msg_preamble_recovery_active = 0U;
      return 0U;
    }
  }
  for (uint8_t i = 0U; i < MSG_START_LEN; i++)
  {
    if (msg_preamble_recovery_start_energy[i] < PREAMBLE_RECOVERY_ENERGY_MIN)
    {
      printf("timed preamble recovery rejected: delta=%lums weak=start%u E/1e6=%u\r\n",
             (unsigned long)delta, i,
             (unsigned int)(msg_preamble_recovery_start_energy[i] / 1000000.0f));
      msg_preamble_recovery_active = 0U;
      return 0U;
    }
  }

  memcpy(msg_preamble_energy, msg_preamble_recovery_energy, sizeof(msg_preamble_energy));
  printf("rx timed preamble recovery: 00->01 anchor + marker delta=%lums\r\n",
         (unsigned long)delta);
  msg_preamble_capture_active = 0U;
  MessageRx_ApplyPreambleEnergyScale();
  msg_preamble_recovery_active = 0U;
  msg_preamble_index = 0U;
  msg_rs_symbol_index = 0U;
  msg_rs_fsk_index = 0U;
  msg_rs_symbol = 0U;
  msg_rs_current_erased = 0U;
  msg_rs_erasure_count = 0U;
  msg_soft_fallback_count = 0U;
  msg_vote_override_count = 0U;
  msg_marker_tick = now;
  MessageRx_ClearCapture();
  memset(msg_rs_codeword, 0, sizeof(msg_rs_codeword));
  msg_rx_state = MSG_RX_RS_CODEWORD;
  printf("rx start recovered by timed training; receiving %ux marker-framed RS(%u,%u) blocks\r\n",
         RS_BLOCK_COUNT, RS_CODEWORD_SYMBOLS, RS_DATA_SYMBOLS);
  printf("rx marker open: RS symbol 1/%u\r\n", RS_TOTAL_CODEWORD_SYMBOLS);
  return 1U;
}

static uint8_t MessageRx_DataWindowPosition(uint32_t now, uint8_t *slot,
                                             uint8_t *distance)
{
  uint32_t delta = now - msg_marker_tick;
  uint32_t target;

  if ((slot == NULL) || (distance == NULL) ||
      (delta < RS_SLOT_CAPTURE_START_MS) ||
      (delta >= RS_SLOT_CAPTURE_END_MS))
  {
    return 0U;
  }

  if (delta < (2U * RS_SLOT_WIDTH_MS))
  {
    *slot = 0U;
  }
  else if (delta < ((3U * RS_SLOT_WIDTH_MS) - (FSK_SYMBOL_MS / 2U)))
  {
    *slot = 1U;
  }
  else
  {
    *slot = 2U;
  }

  target = (uint32_t)(*slot + 1U) * RS_SLOT_WIDTH_MS;
  *distance = (uint8_t)((delta > target) ? (delta - target) : (target - delta));
  return 1U;
}

static uint8_t MessageRx_CaptureDataWindow(const FSK_DetectResult *result, uint32_t now)
{
  uint8_t slot;
  uint8_t distance;
  float vote_ratio;
  uint8_t nearest_updated = 0U;

  if ((result->bits >= FSK_DATA_FREQ_COUNT) ||
      (MessageRx_DataWindowPosition(now, &slot, &distance) == 0U))
  {
    return 0U;
  }

  /*
   * msg_marker_tick is the first accepted 20 ms marker window, not the
   * physical marker start.  A 70 ms step shifts by half a detector window
   * on alternate tones, so the capture ranges are 40..139, 140..199 and
   * 200..259 ms.  This keeps the first valid third-tone window out of slot 1.
   */
  vote_ratio = (result->second_energy > 1.0f) ?
               (result->max_energy / result->second_energy) : RS_VOTE_RATIO_CAP;
  if (vote_ratio > RS_VOTE_RATIO_CAP)
  {
    vote_ratio = RS_VOTE_RATIO_CAP;
  }

  msg_capture_scores[slot][result->bits] += vote_ratio;
  if ((msg_capture_valid[slot] == 0U) || (distance < msg_capture_distance[slot]))
  {
    msg_capture_bits[slot] = result->bits;
    msg_capture_valid[slot] = 1U;
    msg_capture_distance[slot] = distance;
    nearest_updated = 1U;
  }

  return nearest_updated;
}

static uint8_t MessageRx_SelectSoftDataCandidate(const FSK_DetectResult *result,
                                                  uint8_t *bits, float *max_energy,
                                                  float *second_energy, float *confidence)
{
  uint8_t max_index = 0U;
  uint8_t second_index = 1U;
  float ratio;
  float noise_ratio;

  if ((result == NULL) || (bits == NULL) || (max_energy == NULL) ||
      (second_energy == NULL) || (confidence == NULL))
  {
    return 0U;
  }

  if (result->data_energy[second_index] > result->data_energy[max_index])
  {
    max_index = 1U;
    second_index = 0U;
  }
  for (uint8_t i = 2U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    if (result->data_energy[i] > result->data_energy[max_index])
    {
      second_index = max_index;
      max_index = i;
    }
    else if (result->data_energy[i] > result->data_energy[second_index])
    {
      second_index = i;
    }
  }

  *bits = max_index;
  *max_energy = result->data_energy[max_index];
  *second_energy = result->data_energy[second_index];
  if ((*max_energy < FSK_SOFT_ENERGY_THRESHOLD) ||
      (*max_energy < (*second_energy * FSK_SOFT_RATIO_THRESHOLD)) ||
      (*max_energy < (fsk_noise_floor[max_index] * FSK_SOFT_NOISE_MULTIPLIER)))
  {
    return 0U;
  }

  ratio = (*second_energy > 1.0f) ? (*max_energy / *second_energy) : RS_VOTE_RATIO_CAP;
  noise_ratio = *max_energy / fsk_noise_floor[max_index];
  if (ratio > RS_VOTE_RATIO_CAP)
  {
    ratio = RS_VOTE_RATIO_CAP;
  }
  if (noise_ratio > RS_VOTE_RATIO_CAP)
  {
    noise_ratio = RS_VOTE_RATIO_CAP;
  }
  *confidence = ratio * noise_ratio;
  return 1U;
}

static void MessageRx_CaptureSoftDataWindow(const FSK_DetectResult *result, uint32_t now)
{
  uint32_t delta = now - msg_marker_tick;
  uint8_t slot;
  uint8_t distance;
  uint8_t candidate_bits;
  float candidate_energy;
  float second_energy;
  float confidence;
  float time_weight;
  float score;

  if ((delta < RS_SOFT_CAPTURE_START_MS) ||
      (MessageRx_DataWindowPosition(now, &slot, &distance) == 0U))
  {
    return;
  }
  if (MessageRx_SelectSoftDataCandidate(result, &candidate_bits, &candidate_energy,
                                        &second_energy, &confidence) == 0U)
  {
    return;
  }

  time_weight = 1.0f / (1.0f + ((float)distance / (float)FSK_SYMBOL_MS));
  score = confidence * time_weight;

  if ((msg_soft_valid[slot] == 0U) || (score > msg_soft_score[slot]))
  {
    msg_soft_bits[slot] = candidate_bits;
    msg_soft_valid[slot] = 1U;
    msg_soft_score[slot] = score;
  }
}

static uint8_t MessageRx_TrySoftEnd1(const FSK_DetectResult *result, uint32_t now)
{
  uint32_t delta = now - msg_frame_tick;
  uint8_t candidate_bits;
  float candidate_energy;
  float second_energy;
  float confidence;
  FSK_DetectResult soft_result;

  if ((delta < END1_SOFT_ACCEPT_MIN_MS) || (delta > END1_SOFT_ACCEPT_MAX_MS) ||
      (MessageRx_SelectSoftDataCandidate(result, &candidate_bits, &candidate_energy,
                                         &second_energy, &confidence) == 0U) ||
      (candidate_bits != msg_end[1]))
  {
    return 0U;
  }

  soft_result = *result;
  soft_result.valid = 1U;
  soft_result.bits = candidate_bits;
  soft_result.freq_hz = fsk_rx_freqs_hz[candidate_bits];
  soft_result.max_energy = candidate_energy;
  soft_result.second_energy = second_energy;
  printf("rx end1 soft accept: delta=%lums confidence=%u\r\n",
         (unsigned long)delta, (unsigned int)(confidence * 100.0f));
  MessageRx_AcceptSymbol(&soft_result, now);
  return 1U;
}

static void MessageRx_AppendRsBits(uint8_t decoded_bits, uint8_t erased)
{
  if (erased != 0U)
  {
    msg_rs_current_erased = 1U;
  }

  msg_rs_symbol = (uint8_t)((msg_rs_symbol << 2U) | (decoded_bits & 0x03U));
  msg_rs_fsk_index++;
  if (msg_rs_fsk_index < RS_FSK_SYMBOLS_PER_SYMBOL)
  {
    return;
  }

  {
    uint8_t wire_index = msg_rs_symbol_index;
    uint8_t block = (uint8_t)(wire_index % RS_BLOCK_COUNT);
    uint8_t block_index = (uint8_t)(wire_index / RS_BLOCK_COUNT);

    msg_rs_codeword[(block * RS_CODEWORD_SYMBOLS) + block_index] = msg_rs_symbol;
    msg_rs_symbol_index++;
  }
  if (msg_rs_current_erased != 0U)
  {
    msg_rs_erasure_count++;
    printf("rx rs wire %u/%u B%u S%u: 0x%02X [slot erasure]\r\n",
           msg_rs_symbol_index, RS_TOTAL_CODEWORD_SYMBOLS,
           ((msg_rs_symbol_index - 1U) % RS_BLOCK_COUNT) + 1U,
           ((msg_rs_symbol_index - 1U) / RS_BLOCK_COUNT) + 1U,
           msg_rs_symbol);
  }
  else
  {
    printf("rx rs wire %u/%u B%u S%u: 0x%02X\r\n",
           msg_rs_symbol_index, RS_TOTAL_CODEWORD_SYMBOLS,
           ((msg_rs_symbol_index - 1U) % RS_BLOCK_COUNT) + 1U,
           ((msg_rs_symbol_index - 1U) / RS_BLOCK_COUNT) + 1U,
           msg_rs_symbol);
  }

  msg_rs_symbol = 0U;
  msg_rs_fsk_index = 0U;
  msg_rs_current_erased = 0U;
}

static void MessageRx_ClearCapture(void)
{
  memset(msg_capture_bits, 0, sizeof(msg_capture_bits));
  memset(msg_capture_valid, 0, sizeof(msg_capture_valid));
  memset(msg_capture_distance, 0xFF, sizeof(msg_capture_distance));
  memset(msg_capture_scores, 0, sizeof(msg_capture_scores));
  memset(msg_soft_bits, 0, sizeof(msg_soft_bits));
  memset(msg_soft_valid, 0, sizeof(msg_soft_valid));
  memset(msg_soft_score, 0, sizeof(msg_soft_score));
}

static void MessageRx_FinalizeCapture(void)
{
  uint8_t rs_index = msg_rs_symbol_index;

  for (uint8_t slot = 0U; slot < RS_FSK_SYMBOLS_PER_SYMBOL; slot++)
  {
    uint16_t fsk_index = ((uint16_t)rs_index * RS_FSK_SYMBOLS_PER_SYMBOL) + slot;
    uint8_t voted_bits = 0U;
    float voted_score = msg_capture_scores[slot][0];
    float second_score = 0.0f;

    for (uint8_t candidate = 1U; candidate < 4U; candidate++)
    {
      float score = msg_capture_scores[slot][candidate];

      if (score > voted_score)
      {
        second_score = voted_score;
        voted_score = score;
        voted_bits = candidate;
      }
      else if (score > second_score)
      {
        second_score = score;
      }
    }

    if (voted_score > second_score)
    {
      if ((msg_capture_valid[slot] == 0U) || (voted_bits != msg_capture_bits[slot]))
      {
        msg_vote_override_count++;
      }
      uint8_t decoded_bits = (uint8_t)((voted_bits ^
                                        Message_WhiteningMask(fsk_index)) & 0x03U);
      MessageRx_AppendRsBits(decoded_bits, 0U);
    }
    else if (msg_capture_valid[slot] != 0U)
    {
      uint8_t decoded_bits = (uint8_t)((msg_capture_bits[slot] ^
                                        Message_WhiteningMask(fsk_index)) & 0x03U);
      MessageRx_AppendRsBits(decoded_bits, 0U);
    }
    else if (msg_soft_valid[slot] != 0U)
    {
      uint8_t decoded_bits = (uint8_t)((msg_soft_bits[slot] ^
                                        Message_WhiteningMask(fsk_index)) & 0x03U);
      msg_soft_fallback_count++;
      MessageRx_AppendRsBits(decoded_bits, 0U);
    }
    else
    {
      MessageRx_AppendRsBits(0U, 1U);
    }
  }

  MessageRx_ClearCapture();
  if (msg_rs_symbol_index >= RS_TOTAL_CODEWORD_SYMBOLS)
  {
    printf("rx RS codeword complete, slot erasures=%u, soft_fallbacks=%u, vote_overrides=%u\r\n",
           msg_rs_erasure_count, msg_soft_fallback_count, msg_vote_override_count);
  }
}

static void MessageRx_AcceptSymbol(const FSK_DetectResult *result, uint32_t now)
{
  uint8_t bits = result->bits;

  msg_frame_tick = now;
  MessageRx_PrintSymbol(result, now);

  switch (msg_rx_state)
  {
    case MSG_RX_SEARCH:
      if ((bits == FSK_SYNC_SYMBOL) && (MessageRx_TryTimedPreambleRecovery(now) != 0U))
      {
        break;
      }
      if (bits == msg_preamble[msg_preamble_index])
      {
        if (msg_preamble_index == 0U)
        {
          memset(msg_preamble_energy, 0, sizeof(msg_preamble_energy));
          MessageRx_ResetPreambleCapture();
          msg_preamble_capture_active = 1U;
          msg_preamble_capture_stage = 0U;
        }
        else
        {
          msg_preamble_capture_active = 1U;
          msg_preamble_capture_stage = msg_preamble_index;
        }
        MessageRx_AddPreambleRawWindow(msg_preamble_index, result, now);

        if ((msg_preamble_index == 1U) && (bits == msg_preamble[1]))
        {
          memset(msg_preamble_recovery_energy, 0, sizeof(msg_preamble_recovery_energy));
          memset(msg_preamble_recovery_start_energy, 0,
                 sizeof(msg_preamble_recovery_start_energy));
          msg_preamble_recovery_energy[0] = msg_preamble_energy[0];
          msg_preamble_recovery_energy[1] = result->data_energy[1];
          msg_preamble_recovery_tick = now;
          msg_preamble_recovery_active = 1U;
        }
        msg_preamble_energy[msg_preamble_index] = result->max_energy;
        msg_preamble_index++;

        if (msg_preamble_index >= MSG_PREAMBLE_LEN)
        {
          msg_preamble_index = 0U;
          msg_preamble_recovery_active = 0U;
          msg_start_entry_tick = now;
          msg_rx_state = MSG_RX_START_0;
          printf("rx preamble ok\r\n");
        }
      }
      else
      {
        memset(msg_preamble_energy, 0, sizeof(msg_preamble_energy));
        if (bits == msg_preamble[0])
        {
          if (msg_preamble_recovery_active == 0U)
          {
            MessageRx_ResetPreambleCapture();
            msg_preamble_capture_active = 1U;
            msg_preamble_capture_stage = 0U;
            MessageRx_AddPreambleRawWindow(0U, result, now);
          }
          msg_preamble_energy[0] = result->max_energy;
          msg_preamble_index = 1U;
        }
        else
        {
          if (msg_preamble_recovery_active == 0U)
          {
            MessageRx_ResetPreambleCapture();
          }
          msg_preamble_index = 0U;
        }
      }
      break;

    case MSG_RX_START_0:
      if (bits == msg_start[0])
      {
        msg_preamble_capture_active = 0U;
        msg_rx_state = MSG_RX_START_1;
        printf("rx start0 ok\r\n");
      }
      else if ((bits == msg_preamble[MSG_PREAMBLE_LEN - 1U]) ||
               (bits == FSK_SYNC_SYMBOL))
      {
        /* A strong 4500 Hz PREAMBLE tail can survive into the first START
           window.  Keep waiting for 2500 Hz instead of discarding a frame
           whose complete PREAMBLE has already been verified. */
        printf("rx start0 transition ignored: got=%s\r\n",
               FSK_BitsToString(bits));
      }
      else
      {
        printf("rx start0 fail: got=%s expected=%s\r\n",
               FSK_BitsToString(bits), FSK_BitsToString(msg_start[0]));
        MessageRx_ResetFrame();
      }
      break;

    case MSG_RX_START_1:
      if (bits == msg_start[1])
      {
        MessageRx_ApplyPreambleEnergyScale();
        msg_rs_symbol_index = 0U;
        msg_rs_fsk_index = 0U;
        msg_rs_symbol = 0U;
        msg_rs_current_erased = 0U;
        msg_rs_erasure_count = 0U;
        msg_soft_fallback_count = 0U;
        msg_vote_override_count = 0U;
        msg_marker_tick = 0U;
        msg_start_entry_tick = 0U;
        MessageRx_ClearCapture();
        memset(msg_rs_codeword, 0, sizeof(msg_rs_codeword));
        msg_rx_state = MSG_RX_RS_SYNC;
        printf("rx start ok, receiving %ux marker-framed RS(%u,%u) blocks\r\n",
               RS_BLOCK_COUNT, RS_CODEWORD_SYMBOLS, RS_DATA_SYMBOLS);
      }
      else if ((bits == msg_start[0]) || (bits == FSK_SYNC_SYMBOL))
      {
        /* Apply the same transition tolerance between START0 and START1. */
        printf("rx start1 transition ignored: got=%s\r\n",
               FSK_BitsToString(bits));
      }
      else
      {
        printf("rx start1 fail: got=%s expected=%s\r\n",
               FSK_BitsToString(bits), FSK_BitsToString(msg_start[1]));
        MessageRx_ResetFrame();
      }
      break;

    case MSG_RX_RS_SYNC:
      if (bits == FSK_SYNC_SYMBOL)
      {
        msg_marker_tick = now;
        MessageRx_ClearCapture();
        msg_rx_state = MSG_RX_RS_CODEWORD;
        printf("rx marker open: RS symbol %u/%u\r\n",
               msg_rs_symbol_index + 1U, RS_TOTAL_CODEWORD_SYMBOLS);
      }
      break;

    case MSG_RX_RS_CODEWORD:
      if (bits == FSK_SYNC_SYMBOL)
      {
        uint32_t marker_delta = now - msg_marker_tick;
        uint32_t marker_intervals = (marker_delta + (RS_MARKER_INTERVAL_MS / 2U)) /
                                    RS_MARKER_INTERVAL_MS;
        uint32_t missing_markers;

        if (marker_delta < (RS_MARKER_INTERVAL_MS - RS_MARKER_ACCEPT_EARLY_MS))
        {
          printf("rx early marker ignored: delta=%lums\r\n", (unsigned long)marker_delta);
          break;
        }

        if (marker_intervals == 0U)
        {
          marker_intervals = 1U;
        }
        missing_markers = marker_intervals - 1U;

        MessageRx_FinalizeCapture();
        while ((missing_markers > 0U) && (msg_rs_symbol_index < RS_TOTAL_CODEWORD_SYMBOLS))
        {
          printf("rx marker missing: erase RS symbol %u\r\n", msg_rs_symbol_index + 1U);
          MessageRx_ClearCapture();
          MessageRx_FinalizeCapture();
          missing_markers--;
        }

        if (msg_rs_symbol_index >= RS_TOTAL_CODEWORD_SYMBOLS)
        {
          msg_rx_state = MSG_RX_END_0;
          printf("rx closing marker ok\r\n");
        }
        else
        {
          msg_marker_tick = now;
          MessageRx_ClearCapture();
          msg_rx_state = MSG_RX_RS_CODEWORD;
          printf("rx marker open: RS symbol %u/%u\r\n",
                 msg_rs_symbol_index + 1U, RS_TOTAL_CODEWORD_SYMBOLS);
        }
      }
      else
      {
        (void)MessageRx_CaptureDataWindow(result, now);
      }
      break;

    case MSG_RX_END_0:
      if (bits == FSK_SYNC_SYMBOL)
      {
        printf("rx closing marker ok\r\n");
      }
      else if (bits == msg_end[0])
      {
        msg_rx_state = MSG_RX_END_1;
      }
      else
      {
        printf("rx end0 fail\r\n");
        MessageRx_ResetFrame();
      }
      break;

    case MSG_RX_END_1:
      if (bits == FSK_SYNC_SYMBOL)
      {
        printf("rx end1 transition marker ignored\r\n");
        break;
      }
      else if (bits == msg_end[1])
      {
        uint8_t total_corrected = 0U;
        uint8_t all_blocks_ok = 1U;

        printf("rx end ok\r\n");
        for (uint8_t block = 0U; block < RS_BLOCK_COUNT; block++)
        {
          uint8_t corrected = 0U;
          int8_t decode_status = RS_Decode(&msg_rs_codeword[block * RS_CODEWORD_SYMBOLS],
                                           &corrected);

          if (decode_status < 0)
          {
            all_blocks_ok = 0U;
            printf("rs block %u/%u fail: more than %u symbol errors\r\n",
                   block + 1U, RS_BLOCK_COUNT, RS_CORRECTABLE_SYMBOLS);
          }
          else
          {
            total_corrected = (uint8_t)(total_corrected + corrected);
            printf("rs block %u/%u ok: corrected=%u/%u\r\n",
                   block + 1U, RS_BLOCK_COUNT, corrected, RS_CORRECTABLE_SYMBOLS);
          }
        }

        if (all_blocks_ok != 0U)
        {
          uint8_t valid_text = 1U;
          uint8_t pad_seen = 0U;

          rx_message_len = 0U;
          for (uint8_t i = 0U; i < RS_TOTAL_DATA_SYMBOLS; i++)
          {
            uint8_t block = (uint8_t)(i / RS_DATA_SYMBOLS);
            uint8_t block_index = (uint8_t)(i % RS_DATA_SYMBOLS);
            uint8_t code = msg_rs_codeword[(block * RS_CODEWORD_SYMBOLS) + block_index];

            if (code == RS_PAD_VALUE)
            {
              pad_seen = 1U;
              continue;
            }
            if (pad_seen != 0U)
            {
              valid_text = 0U;
              printf("rx RS payload fail: data after padding at %u\r\n", i);
              break;
            }

            char ch = Message_CodeToChar(code);
            if (ch == 0)
            {
              valid_text = 0U;
              printf("rx RS payload fail: invalid character code %u at %u\r\n", code, i);
              break;
            }
            rx_message[rx_message_len++] = ch;
          }
          rx_message[rx_message_len] = '\0';

          if ((valid_text != 0U) && (rx_message_len > 0U))
          {
            rx_message_valid = 1U;
            rx_rs_corrected = total_corrected;
            memcpy(message_store_pending_text, rx_message, rx_message_len + 1U);
            message_store_pending_len = rx_message_len;
            message_store_pending = 1U;
            message_store_last_save_failed = 0U;
            StatusLed_NotifyRxComplete();
            printf("rs decode ok: corrected=%u/%u, slot_erasures=%u\r\n",
                    total_corrected, RS_TOTAL_CORRECTABLE, msg_rs_erasure_count);
            printf("message_ok: len=%u text=\"%s\"\r\n", rx_message_len, rx_message);
          }
          else
          {
            printf("message fail: invalid RS payload\r\n");
          }
        }
        else
        {
          printf("rs decode fail: one or more RS blocks exceeded t=%u\r\n",
                 RS_CORRECTABLE_SYMBOLS);
        }
      }
      else if (bits == msg_end[0])
      {
        printf("rx end1 repeated end0 ignored\r\n");
        break;
      }
      else
      {
        printf("rx end1 fail\r\n");
      }
      MessageRx_ResetFrame();
      break;

    default:
      MessageRx_ResetFrame();
      break;
  }
}

static void MessageRx_ResetFrame(void)
{
  msg_rx_state = MSG_RX_SEARCH;
  msg_preamble_index = 0U;
  msg_start_entry_tick = 0U;
  memset(msg_preamble_energy, 0, sizeof(msg_preamble_energy));
  MessageRx_ResetPreambleCapture();
  msg_preamble_recovery_active = 0U;
  msg_preamble_recovery_tick = 0U;
  memset(msg_preamble_recovery_energy, 0, sizeof(msg_preamble_recovery_energy));
  memset(msg_preamble_recovery_start_energy, 0,
         sizeof(msg_preamble_recovery_start_energy));
  msg_rs_symbol_index = 0U;
  msg_rs_fsk_index = 0U;
  msg_rs_symbol = 0U;
  msg_rs_current_erased = 0U;
  msg_rs_erasure_count = 0U;
  msg_soft_fallback_count = 0U;
  msg_vote_override_count = 0U;
  msg_marker_tick = 0U;
  MessageRx_ClearCapture();
  msg_frame_tick = 0U;
}

static uint8_t Message_WhiteningMask(uint16_t fsk_symbol_index)
{
  static const uint8_t mask_cycle[4] = {0x00U, 0x01U, 0x02U, 0x03U};
  return mask_cycle[fsk_symbol_index & 0x03U];
}

static uint8_t Message_CharToCode(char ch)
{
  if ((ch >= '0') && (ch <= '9'))
  {
    return (uint8_t)(ch - '0');
  }
  if ((ch >= 'A') && (ch <= 'Z'))
  {
    return (uint8_t)(10U + (uint8_t)(ch - 'A'));
  }

  switch (ch)
  {
    case ' ': return 36U;
    case '.': return 37U;
    case ',': return 38U;
    case '?': return 39U;
    case '!': return 40U;
    default: return MESSAGE_CODE_INVALID;
  }
}

static char Message_CodeToChar(uint8_t code)
{
  if (code <= 9U)
  {
    return (char)('0' + code);
  }
  if (code <= 35U)
  {
    return (char)('A' + (code - 10U));
  }

  switch (code)
  {
    case 36U: return ' ';
    case 37U: return '.';
    case 38U: return ',';
    case 39U: return '?';
    case 40U: return '!';
    default: return 0;
  }
}

static uint32_t MessageStore_HashRecord(const MessageStoreRecord *record)
{
  const uint8_t *data = (const uint8_t *)&record->sequence;
  const uint16_t data_len = (uint16_t)((2U * sizeof(uint32_t)) + MESSAGE_MAX_LEN);
  uint32_t hash = 2166136261UL;

  for (uint16_t i = 0U; i < data_len; i++)
  {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

static uint8_t MessageStore_RecordIsErased(uint32_t address)
{
  const uint32_t *words = (const uint32_t *)address;
  const uint16_t word_count = (uint16_t)(sizeof(MessageStoreRecord) / sizeof(uint32_t));

  for (uint16_t i = 0U; i < word_count; i++)
  {
    if (words[i] != 0xFFFFFFFFUL)
    {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t MessageStore_RecordValid(const MessageStoreRecord *record)
{
  if ((record->magic != MESSAGE_STORE_MAGIC) ||
      (record->commit != MESSAGE_STORE_COMMIT) ||
      (record->length == 0U) ||
      (record->length > MESSAGE_MAX_LEN) ||
      (record->hash != MessageStore_HashRecord(record)))
  {
    return 0U;
  }

  for (uint8_t i = 0U; i < (uint8_t)record->length; i++)
  {
    if (Message_CharToCode((char)record->text[i]) == MESSAGE_CODE_INVALID)
    {
      return 0U;
    }
  }
  return 1U;
}

static void MessageStore_CacheAppend(uint32_t sequence, const char *text, uint8_t len)
{
  uint8_t index;

  if (message_store_count < MESSAGE_STORE_VISIBLE_COUNT)
  {
    index = message_store_count;
    message_store_count++;
  }
  else
  {
    for (uint8_t i = 1U; i < MESSAGE_STORE_VISIBLE_COUNT; i++)
    {
      message_store_cache[i - 1U] = message_store_cache[i];
    }
    index = MESSAGE_STORE_VISIBLE_COUNT - 1U;
  }

  message_store_cache[index].sequence = sequence;
  message_store_cache[index].len = len;
  memcpy(message_store_cache[index].text, text, len);
  message_store_cache[index].text[len] = '\0';
  message_store_view_index = message_store_count - 1U;
}

static void MessageStore_BuildRecord(MessageStoreRecord *record, uint32_t sequence,
                                     const char *text, uint8_t len)
{
  memset(record, 0, sizeof(*record));
  record->magic = MESSAGE_STORE_MAGIC;
  record->sequence = sequence;
  record->length = len;
  memcpy(record->text, text, len);
  record->hash = MessageStore_HashRecord(record);
  record->commit = MESSAGE_STORE_COMMIT;
}

static uint8_t MessageStore_ProgramRecordUnlocked(uint32_t address,
                                                  const MessageStoreRecord *record)
{
  const uint8_t *raw = (const uint8_t *)record;
  const uint32_t commit_offset = sizeof(MessageStoreRecord) - sizeof(uint32_t);
  uint32_t word;

  if ((address + sizeof(MessageStoreRecord)) > MESSAGE_STORE_FLASH_END)
  {
    return 0U;
  }

  for (uint32_t offset = 0U; offset < commit_offset; offset += sizeof(uint32_t))
  {
    memcpy(&word, &raw[offset], sizeof(word));
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + offset, word) != HAL_OK)
    {
      return 0U;
    }
  }

  memcpy(&word, &raw[commit_offset], sizeof(word));
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + commit_offset, word) != HAL_OK)
  {
    return 0U;
  }
  return 1U;
}

static uint8_t MessageStore_CompactUnlocked(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  MessageStoreRecord record;
  uint32_t sector_error = 0U;

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_7;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    return 0U;
  }

  message_store_write_address = MESSAGE_STORE_FLASH_START;
  for (uint8_t i = 0U; i < message_store_count; i++)
  {
    MessageStore_BuildRecord(&record,
                             message_store_cache[i].sequence,
                             message_store_cache[i].text,
                             message_store_cache[i].len);
    if (MessageStore_ProgramRecordUnlocked(message_store_write_address, &record) == 0U)
    {
      return 0U;
    }
    message_store_write_address += sizeof(MessageStoreRecord);
  }
  return 1U;
}

static void MessageStore_Init(void)
{
  MessageStoreRecord record;
  uint32_t max_sequence = 0U;
  uint32_t address;

  message_store_count = 0U;
  message_store_view_index = 0U;
  message_store_write_address = MESSAGE_STORE_FLASH_END;

  for (address = MESSAGE_STORE_FLASH_START;
       (address + sizeof(MessageStoreRecord)) <= MESSAGE_STORE_FLASH_END;
       address += sizeof(MessageStoreRecord))
  {
    if (MessageStore_RecordIsErased(address) != 0U)
    {
      message_store_write_address = address;
      break;
    }

    memcpy(&record, (const void *)address, sizeof(record));
    if (MessageStore_RecordValid(&record) != 0U)
    {
      MessageStore_CacheAppend(record.sequence, (const char *)record.text,
                               (uint8_t)record.length);
      if (record.sequence > max_sequence)
      {
        max_sequence = record.sequence;
      }
    }
  }

  message_store_next_sequence = max_sequence + 1U;
  if (message_store_next_sequence == 0U)
  {
    message_store_next_sequence = 1U;
  }
  printf("message store: loaded=%u/5 next=%lu write=0x%08lX\r\n",
         message_store_count,
         (unsigned long)message_store_next_sequence,
         (unsigned long)message_store_write_address);
}

static uint8_t MessageStore_Save(const char *text, uint8_t len)
{
  MessageStoreRecord record;
  uint32_t sequence;
  uint8_t saved;

  if ((len == 0U) || (len > MESSAGE_MAX_LEN))
  {
    return 0U;
  }

  sequence = message_store_next_sequence;
  MessageStore_BuildRecord(&record, sequence, text, len);

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return 0U;
  }

  if ((message_store_write_address + sizeof(MessageStoreRecord)) > MESSAGE_STORE_FLASH_END)
  {
    if (MessageStore_CompactUnlocked() == 0U)
    {
      (void)HAL_FLASH_Lock();
      MessageStore_Init();
      return 0U;
    }
  }

  saved = MessageStore_ProgramRecordUnlocked(message_store_write_address, &record);
  (void)HAL_FLASH_Lock();
  if (saved == 0U)
  {
    MessageStore_Init();
    return 0U;
  }

  message_store_write_address += sizeof(MessageStoreRecord);
  MessageStore_CacheAppend(sequence, text, len);
  message_store_next_sequence = sequence + 1U;
  if (message_store_next_sequence == 0U)
  {
    message_store_next_sequence = 1U;
  }
  return 1U;
}

static void MessageStore_Task(void)
{
  uint8_t was_sampling;
  uint8_t saved;

  if (message_store_pending == 0U)
  {
    return;
  }

  was_sampling = rx_sampling_active;
  if (was_sampling != 0U)
  {
    RX_StopSampling();
  }

  saved = MessageStore_Save(message_store_pending_text, message_store_pending_len);
  message_store_pending = 0U;
  message_store_pending_len = 0U;

  message_store_last_save_failed = (saved != 0U) ? 0U : 1U;
  if (saved != 0U)
  {
    printf("message stored: item=%u/%u seq=%lu len=%u\r\n",
           message_store_view_index + 1U,
           message_store_count,
           (unsigned long)message_store_cache[message_store_view_index].sequence,
           message_store_cache[message_store_view_index].len);
  }
  else
  {
    printf("message store fail: received text remains in RAM only\r\n");
  }
  OLED_PrintRxStatus();

  if ((was_sampling != 0U) && (app_mode == APP_MODE_RX))
  {
    rx_resume_pending = 0U;
    if (RX_StartSampling() == 0U)
    {
      rx_resume_tick = HAL_GetTick() + 500U;
      rx_resume_pending = 1U;
    }
  }
}

static void MessageStore_HandleRxKey(char key)
{
  if (msg_rx_state != MSG_RX_SEARCH)
  {
    printf("RX memory key ignored while a frame is active\r\n");
    return;
  }

  if (message_store_count == 0U)
  {
    printf("RX memory empty; A=older B=newer C=latest\r\n");
    return;
  }

  if (key == 'A')
  {
    if (message_store_view_index > 0U)
    {
      message_store_view_index--;
    }
  }
  else if (key == 'B')
  {
    if ((message_store_view_index + 1U) < message_store_count)
    {
      message_store_view_index++;
    }
  }
  else if (key == 'C')
  {
    message_store_view_index = message_store_count - 1U;
  }
  else
  {
    printf("RX memory: A=older B=newer C=latest, *=TX\r\n");
    return;
  }

  message_store_last_save_failed = 0U;
  printf("memory view: item=%u/%u seq=%lu len=%u text=\"%s\"\r\n",
         message_store_view_index + 1U,
         message_store_count,
         (unsigned long)message_store_cache[message_store_view_index].sequence,
         message_store_cache[message_store_view_index].len,
         message_store_cache[message_store_view_index].text);
  OLED_PrintRxStatus();
}

static uint8_t RS_GfMul(uint8_t a, uint8_t b)
{
  if ((a == 0U) || (b == 0U))
  {
    return 0U;
  }
  return rs_gf_exp[(uint16_t)rs_gf_log[a] + (uint16_t)rs_gf_log[b]];
}

static uint8_t RS_GfDiv(uint8_t a, uint8_t b)
{
  int16_t exponent;

  if (a == 0U)
  {
    return 0U;
  }
  if (b == 0U)
  {
    return 0U;
  }

  exponent = (int16_t)rs_gf_log[a] - (int16_t)rs_gf_log[b];
  if (exponent < 0)
  {
    exponent += RS_FIELD_ORDER;
  }
  return rs_gf_exp[(uint16_t)exponent];
}

static uint8_t RS_PolyEvalAscending(const uint8_t *poly, uint8_t degree, uint8_t x)
{
  uint8_t value = poly[degree];

  while (degree > 0U)
  {
    degree--;
    value = (uint8_t)(RS_GfMul(value, x) ^ poly[degree]);
  }
  return value;
}

static void RS_Init(void)
{
  uint16_t value = 1U;
  uint8_t generator_len = 1U;

  memset(rs_gf_exp, 0, sizeof(rs_gf_exp));
  memset(rs_gf_log, 0, sizeof(rs_gf_log));

  for (uint16_t i = 0U; i < RS_FIELD_ORDER; i++)
  {
    rs_gf_exp[i] = (uint8_t)value;
    rs_gf_log[(uint8_t)value] = (uint8_t)i;
    value <<= 1U;
    if ((value & 0x40U) != 0U)
    {
      value ^= RS_PRIMITIVE_POLY;
    }
  }
  for (uint16_t i = RS_FIELD_ORDER; i < (RS_FIELD_ORDER * 2U); i++)
  {
    rs_gf_exp[i] = rs_gf_exp[i - RS_FIELD_ORDER];
  }

  memset(rs_generator, 0, sizeof(rs_generator));
  rs_generator[0] = 1U;

  for (uint8_t root = 0U; root < RS_PARITY_SYMBOLS; root++)
  {
    uint8_t next[RS_PARITY_SYMBOLS + 1U] = {0};

    for (uint8_t i = 0U; i < generator_len; i++)
    {
      next[i] ^= rs_generator[i];
      next[i + 1U] ^= RS_GfMul(rs_generator[i], rs_gf_exp[root]);
    }
    generator_len++;
    memcpy(rs_generator, next, generator_len);
  }
}

static void RS_CalculateSyndromes(const uint8_t *codeword, uint8_t *syndromes)
{
  for (uint8_t i = 0U; i < RS_PARITY_SYMBOLS; i++)
  {
    uint8_t x = rs_gf_exp[i];
    uint8_t value = 0U;

    for (uint8_t j = 0U; j < RS_CODEWORD_SYMBOLS; j++)
    {
      value = (uint8_t)(RS_GfMul(value, x) ^ codeword[j]);
    }
    syndromes[i] = value;
  }
}

static void RS_Encode(const uint8_t *data, uint8_t *codeword)
{
  uint8_t work[RS_CODEWORD_SYMBOLS] = {0};

  memcpy(work, data, RS_DATA_SYMBOLS);
  for (uint8_t i = 0U; i < RS_DATA_SYMBOLS; i++)
  {
    uint8_t factor = work[i];

    if (factor == 0U)
    {
      continue;
    }
    for (uint8_t j = 1U; j <= RS_PARITY_SYMBOLS; j++)
    {
      work[i + j] ^= RS_GfMul(rs_generator[j], factor);
    }
  }

  memcpy(codeword, data, RS_DATA_SYMBOLS);
  memcpy(&codeword[RS_DATA_SYMBOLS], &work[RS_DATA_SYMBOLS], RS_PARITY_SYMBOLS);
}

static int8_t RS_Decode(uint8_t *codeword, uint8_t *corrected_count)
{
  uint8_t syndromes[RS_PARITY_SYMBOLS] = {0};
  uint8_t locator[RS_PARITY_SYMBOLS + 1U] = {0};
  uint8_t previous[RS_PARITY_SYMBOLS + 1U] = {0};
  uint8_t saved[RS_PARITY_SYMBOLS + 1U];
  uint8_t error_positions[RS_CORRECTABLE_SYMBOLS];
  uint8_t error_locations[RS_CORRECTABLE_SYMBOLS];
  uint8_t matrix[RS_CORRECTABLE_SYMBOLS][RS_CORRECTABLE_SYMBOLS + 1U];
  uint8_t locator_degree = 0U;
  uint8_t shift = 1U;
  uint8_t previous_discrepancy = 1U;
  uint8_t error_count = 0U;
  uint8_t has_error = 0U;

  *corrected_count = 0U;
  RS_CalculateSyndromes(codeword, syndromes);
  for (uint8_t i = 0U; i < RS_PARITY_SYMBOLS; i++)
  {
    has_error |= syndromes[i];
  }
  if (has_error == 0U)
  {
    return 0;
  }

  locator[0] = 1U;
  previous[0] = 1U;

  for (uint8_t step = 0U; step < RS_PARITY_SYMBOLS; step++)
  {
    uint8_t discrepancy = syndromes[step];

    for (uint8_t i = 1U; i <= locator_degree; i++)
    {
      discrepancy ^= RS_GfMul(locator[i], syndromes[step - i]);
    }

    if (discrepancy == 0U)
    {
      shift++;
      continue;
    }

    memcpy(saved, locator, sizeof(locator));
    {
      uint8_t scale = RS_GfDiv(discrepancy, previous_discrepancy);
      for (uint8_t i = 0U; (uint16_t)i + shift <= RS_PARITY_SYMBOLS; i++)
      {
        locator[i + shift] ^= RS_GfMul(scale, previous[i]);
      }
    }

    if ((uint8_t)(2U * locator_degree) <= step)
    {
      locator_degree = (uint8_t)(step + 1U - locator_degree);
      memcpy(previous, saved, sizeof(previous));
      previous_discrepancy = discrepancy;
      shift = 1U;
    }
    else
    {
      shift++;
    }
  }

  if ((locator_degree == 0U) || (locator_degree > RS_CORRECTABLE_SYMBOLS))
  {
    return -1;
  }

  for (uint8_t pos = 0U; pos < RS_CODEWORD_SYMBOLS; pos++)
  {
    uint8_t location = (uint8_t)(RS_CODEWORD_SYMBOLS - 1U - pos);
    uint8_t x = rs_gf_exp[(RS_FIELD_ORDER - location) % RS_FIELD_ORDER];

    if (RS_PolyEvalAscending(locator, locator_degree, x) == 0U)
    {
      if (error_count >= RS_CORRECTABLE_SYMBOLS)
      {
        return -1;
      }
      error_positions[error_count] = pos;
      error_locations[error_count] = location;
      error_count++;
    }
  }

  if (error_count != locator_degree)
  {
    return -1;
  }

  memset(matrix, 0, sizeof(matrix));
  for (uint8_t row = 0U; row < error_count; row++)
  {
    for (uint8_t col = 0U; col < error_count; col++)
    {
      uint16_t exponent = (uint16_t)row * (uint16_t)error_locations[col];
      matrix[row][col] = rs_gf_exp[exponent % RS_FIELD_ORDER];
    }
    matrix[row][error_count] = syndromes[row];
  }

  for (uint8_t col = 0U; col < error_count; col++)
  {
    uint8_t pivot = col;

    while ((pivot < error_count) && (matrix[pivot][col] == 0U))
    {
      pivot++;
    }
    if (pivot >= error_count)
    {
      return -1;
    }
    if (pivot != col)
    {
      for (uint8_t j = col; j <= error_count; j++)
      {
        uint8_t temp = matrix[col][j];
        matrix[col][j] = matrix[pivot][j];
        matrix[pivot][j] = temp;
      }
    }

    {
      uint8_t inverse = RS_GfDiv(1U, matrix[col][col]);
      for (uint8_t j = col; j <= error_count; j++)
      {
        matrix[col][j] = RS_GfMul(matrix[col][j], inverse);
      }
    }

    for (uint8_t row = 0U; row < error_count; row++)
    {
      uint8_t factor;

      if (row == col)
      {
        continue;
      }
      factor = matrix[row][col];
      if (factor == 0U)
      {
        continue;
      }
      for (uint8_t j = col; j <= error_count; j++)
      {
        matrix[row][j] ^= RS_GfMul(factor, matrix[col][j]);
      }
    }
  }

  for (uint8_t i = 0U; i < error_count; i++)
  {
    codeword[error_positions[i]] ^= matrix[i][error_count];
  }

  RS_CalculateSyndromes(codeword, syndromes);
  has_error = 0U;
  for (uint8_t i = 0U; i < RS_PARITY_SYMBOLS; i++)
  {
    has_error |= syndromes[i];
  }
  if (has_error != 0U)
  {
    return -1;
  }

  *corrected_count = error_count;
  return (int8_t)error_count;
}

static uint8_t RS_SelfTest(void)
{
  static const uint8_t error_positions[RS_CORRECTABLE_SYMBOLS] = {0U, 7U, 15U, 25U, 33U};
  static const uint8_t error_values[RS_CORRECTABLE_SYMBOLS] = {1U, 3U, 7U, 15U, 31U};
  uint8_t data[RS_DATA_SYMBOLS];
  uint8_t expected[RS_CODEWORD_SYMBOLS];
  uint8_t damaged[RS_CODEWORD_SYMBOLS];
  uint8_t corrected = 0U;

  for (uint8_t i = 0U; i < RS_DATA_SYMBOLS; i++)
  {
    data[i] = (uint8_t)((i * 7U + 3U) & RS_FIELD_ORDER);
  }
  RS_Encode(data, expected);
  memcpy(damaged, expected, sizeof(damaged));

  for (uint8_t i = 0U; i < RS_CORRECTABLE_SYMBOLS; i++)
  {
    damaged[error_positions[i]] ^= error_values[i];
  }

  if (RS_Decode(damaged, &corrected) != (int8_t)RS_CORRECTABLE_SYMBOLS)
  {
    return 0U;
  }
  if (corrected != RS_CORRECTABLE_SYMBOLS)
  {
    return 0U;
  }
  return (memcmp(damaged, expected, sizeof(expected)) == 0) ? 1U : 0U;
}

static void RX_ResetDetector(void)
{
  __disable_irq();
  symbol_ready = 0U;
  symbol_start_index = 0U;
  __enable_irq();

  digit_candidate_bits = DIGIT_NO_SYMBOL;
  digit_candidate_count = 0U;
  digit_invalid_count = 0U;
  digit_run_active = 0U;
  digit_run_bits = DIGIT_NO_SYMBOL;
  digit_last_accept_ms = 0U;
  MessageRx_ResetFrame();
}

static uint8_t RX_StartSampling(void)
{
  if (rx_sampling_active != 0U)
  {
    return 1U;
  }

  RX_ResetDetector();
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_BUFFER_SAMPLES) != HAL_OK)
  {
    printf("half duplex: ADC DMA start fail\r\n");
    return 0U;
  }

  rx_sampling_active = 1U;
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    rx_sampling_active = 0U;
    (void)HAL_ADC_Stop_DMA(&hadc1);
    printf("half duplex: TIM2 start fail\r\n");
    return 0U;
  }
  return 1U;
}

static void RX_StopSampling(void)
{
  __disable_irq();
  rx_sampling_active = 0U;
  symbol_ready = 0U;
  __enable_irq();

  (void)HAL_TIM_Base_Stop(&htim2);
  (void)HAL_ADC_Stop_DMA(&hadc1);
  RX_ResetDetector();
  if (calibration_complete == 0U)
  {
    Calibration_ResetProgress();
  }
}

static void HalfDuplex_Task(void)
{
  uint32_t now;

  if ((rx_resume_pending == 0U) || (app_mode != APP_MODE_RX))
  {
    return;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - rx_resume_tick) < 0)
  {
    return;
  }

  if (RX_StartSampling() != 0U)
  {
    rx_resume_pending = 0U;
    printf("half duplex: RX listening\r\n");
    OLED_PrintRxStatus();
  }
  else
  {
    rx_resume_tick = now + 500U;
  }
}

static void App_ToggleMode(void)
{
  if (tx_mode != TX_MODE_IDLE)
  {
    printf("mode switch ignored: TX busy\r\n");
    return;
  }

  if (app_mode == APP_MODE_RX)
  {
    rx_complete_led_active = 0U;
    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);
    rx_resume_pending = 0U;
    RX_StopSampling();
    app_mode = APP_MODE_TX;
    if (tx_calibration_sent == 0U)
    {
      printf("mode: TX calibration required, #=CAL ONLY *=RX/TX\r\n");
    }
    else
    {
      printf("mode: TX editor, *=RX/TX #=SEND\r\n");
    }
  }
  else
  {
    Editor_ConfirmPending();
    app_mode = APP_MODE_RX;
    rx_resume_pending = 0U;
    if (RX_StartSampling() != 0U)
    {
      printf("mode: RX listening\r\n");
    }
    else
    {
      rx_resume_tick = HAL_GetTick() + 500U;
      rx_resume_pending = 1U;
      printf("mode: RX start retry pending\r\n");
    }
  }

  OLED_PrintMode();
}

static void TX_StartCalibration(void)
{
  if ((app_mode != APP_MODE_TX) ||
      (tx_mode != TX_MODE_IDLE) ||
      (tx_calibration_sent != 0U))
  {
    return;
  }

  rx_resume_pending = 0U;
  RX_StopSampling();
  tx_display_sending = 1U;

  __disable_irq();
  tx_done_pending = 0U;
  tx_done_is_calibration = 0U;
  tx_calibration_stage = 0U;
  tx_part_sample_count = 0U;
  TX_LoadCalibrationTone(0U);
  tx_mode = TX_MODE_CALIBRATION;
  __enable_irq();

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_ENABLE);
  printf("half duplex: RX/ADC disabled, TX calibration active\r\n");
  printf("tx calibration only: continuous 00/01/10/11, %ums each\r\n",
         CALIBRATION_TONE_MS);
  OLED_PrintTxStatus();
}

static void TX_StartMessage(void)
{
  if ((app_mode != APP_MODE_TX) || (tx_text_len == 0U) || (tx_mode != TX_MODE_IDLE))
  {
    return;
  }

  if (tx_calibration_sent == 0U)
  {
    printf("tx message blocked: press # to send calibration first\r\n");
    OLED_PrintTxStatus();
    return;
  }

  TX_BuildMessageFrame();

  memcpy(tx_last_message, tx_text, tx_text_len + 1U);
  tx_last_len = tx_text_len;
  tx_last_valid = 1U;
  tx_display_sending = 1U;

  rx_resume_pending = 0U;
  RX_StopSampling();
  tx_done_pending = 0U;

  printf("half duplex: RX paused, TX active\r\n");
  printf("tx start: len=%u FSK_symbols=%u RS=%ux(%u,%u) total_parity=%u text=\"%s\"\r\n",
         tx_last_len, tx_frame_len, RS_BLOCK_COUNT, RS_CODEWORD_SYMBOLS, RS_DATA_SYMBOLS,
         RS_TOTAL_PARITY_SYMBOLS, tx_last_message);
  OLED_PrintRxStatus();
  OLED_PrintTxStatus();

  __disable_irq();
  tx_done_is_calibration = 0U;
  tx_frame_index = 0U;
  tx_frame_part = TX_FRAME_TONE;
  tx_part_sample_count = 0U;
  TX_LoadFrameSymbol(0U);
  tx_mode = TX_MODE_FRAME;
  __enable_irq();

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_ENABLE);
}

static void TX_Stop(void)
{
  rx_resume_pending = 0U;

  __disable_irq();
  tx_mode = TX_MODE_IDLE;
  tx_phase = 0U;
  tx_part_sample_count = 0U;
  tx_calibration_stage = 0U;
  tx_done_pending = 0U;
  tx_done_is_calibration = 0U;
  DAC_Write12(TX_DAC_MID_CODE);
  __enable_irq();

  tx_display_sending = 0U;
  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
}

static void TX_UI_Task(void)
{
  uint8_t done = 0U;
  uint8_t done_is_calibration = 0U;

  __disable_irq();
  if (tx_done_pending != 0U)
  {
    tx_done_pending = 0U;
    done = 1U;
    done_is_calibration = tx_done_is_calibration;
    tx_done_is_calibration = 0U;
  }
  __enable_irq();

  if (done != 0U)
  {
    tx_display_sending = 0U;
    if (done_is_calibration != 0U)
    {
      printf("tx calibration done: local RX stayed disabled; press * for RX\r\n");
    }
    else
    {
      printf("tx done: len=%u text=\"%s\"\r\n", tx_last_len, tx_last_message);
    }
    if (app_mode == APP_MODE_RX)
    {
      rx_resume_tick = HAL_GetTick() + HALF_DUPLEX_RX_GUARD_MS;
      rx_resume_pending = 1U;
      printf("half duplex: RX resumes after %ums guard\r\n", HALF_DUPLEX_RX_GUARD_MS);
    }
    else
    {
      printf("mode: TX ready, press * for RX\r\n");
    }
    OLED_PrintRxStatus();
    OLED_PrintTxStatus();
  }
}

static void TX_AudioTick(void)
{
  if (tx_mode == TX_MODE_IDLE)
  {
    return;
  }

  if (tx_mode == TX_MODE_CALIBRATION)
  {
    if (tx_calibration_stage < FSK_DATA_FREQ_COUNT)
    {
      DAC_WriteSample8(TX_ScaledSineSample());
      tx_part_sample_count++;

      if (tx_part_sample_count >= CALIBRATION_TONE_SAMPLES)
      {
        tx_part_sample_count = 0U;
        tx_calibration_stage++;
        if (tx_calibration_stage < FSK_DATA_FREQ_COUNT)
        {
          TX_LoadCalibrationTone(tx_calibration_stage);
        }
        else
        {
          DAC_Write12(TX_DAC_MID_CODE);
        }
      }
    }
    else
    {
      tx_part_sample_count++;
      if (tx_part_sample_count >= CALIBRATION_TX_GUARD_SAMPLES)
      {
        tx_part_sample_count = 0U;
        tx_calibration_sent = 1U;
        tx_mode = TX_MODE_IDLE;
        tx_done_is_calibration = 1U;
        tx_done_pending = 1U;
        HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
      }
    }
    return;
  }

  if (tx_frame_part == TX_FRAME_TONE)
  {
    DAC_WriteSample8(TX_ScaledSineSample());
    tx_part_sample_count++;

    if (tx_part_sample_count >= TX_TONE_SAMPLES)
    {
      tx_part_sample_count = 0U;
      tx_frame_part = TX_FRAME_GAP;
      DAC_Write12(TX_DAC_MID_CODE);
    }
  }
  else
  {
    tx_part_sample_count++;

    if (tx_part_sample_count >= TX_GAP_SAMPLES)
    {
      tx_part_sample_count = 0U;
      tx_frame_index++;

      if (tx_frame_index >= tx_frame_len)
      {
        tx_mode = TX_MODE_IDLE;
        tx_done_is_calibration = 0U;
        tx_done_pending = 1U;
        HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
      }
      else
      {
        TX_LoadFrameSymbol(tx_frame_index);
        tx_frame_part = TX_FRAME_TONE;
      }
    }
  }
}

static void TX_LoadFrameSymbol(uint16_t index)
{
  uint8_t bits = tx_frame_symbols[index];

  if (bits >= FSK_FREQ_COUNT)
  {
    bits = 0U;
  }

  tx_current_bits = bits;
  tx_current_amp = tx_fsk_amp[bits];
  tx_phase = 0U;
  tx_phase_inc = TX_PhaseIncFromFreq(fsk_tx_freqs_hz[bits]);
}

static void TX_LoadCalibrationTone(uint8_t stage)
{
  if (stage >= FSK_DATA_FREQ_COUNT)
  {
    stage = 0U;
  }

  tx_current_bits = stage;
  tx_current_amp = tx_fsk_amp[stage];
  tx_phase = 0U;
  tx_phase_inc = TX_PhaseIncFromFreq(fsk_tx_freqs_hz[stage]);
}

static void TX_BuildMessageFrame(void)
{
  uint16_t pos = 0U;
  uint8_t rs_codewords[RS_BLOCK_COUNT][RS_CODEWORD_SYMBOLS];

  for (uint8_t block = 0U; block < RS_BLOCK_COUNT; block++)
  {
    uint8_t rs_data[RS_DATA_SYMBOLS];
    uint8_t text_offset = (uint8_t)(block * RS_DATA_SYMBOLS);

    memset(rs_data, RS_PAD_VALUE, sizeof(rs_data));
    for (uint8_t i = 0U; i < RS_DATA_SYMBOLS; i++)
    {
      uint8_t text_index = (uint8_t)(text_offset + i);

      if (text_index >= tx_text_len)
      {
        break;
      }
      rs_data[i] = Message_CharToCode(tx_text[text_index]);
    }
    RS_Encode(rs_data, rs_codewords[block]);
  }

  for (uint8_t repeat = 0U; repeat < TX_REPEAT_COUNT; repeat++)
  {
    uint16_t fsk_index = 0U;

    for (uint8_t i = 0U; i < MSG_PREAMBLE_LEN; i++)
    {
      tx_frame_symbols[pos++] = tx_preamble_symbols[i];
    }
    for (uint8_t i = 0U; i < MSG_START_LEN; i++)
    {
      tx_frame_symbols[pos++] = tx_start_symbols[i];
    }

    for (uint8_t i = 0U; i < RS_CODEWORD_SYMBOLS; i++)
    {
      for (uint8_t block = 0U; block < RS_BLOCK_COUNT; block++)
      {
        uint8_t raw_bits;

        tx_frame_symbols[pos++] = FSK_SYNC_SYMBOL;
        raw_bits = (uint8_t)((rs_codewords[block][i] >> 4U) & 0x03U);
        tx_frame_symbols[pos++] = (uint8_t)(raw_bits ^ Message_WhiteningMask(fsk_index++));
        raw_bits = (uint8_t)((rs_codewords[block][i] >> 2U) & 0x03U);
        tx_frame_symbols[pos++] = (uint8_t)(raw_bits ^ Message_WhiteningMask(fsk_index++));
        raw_bits = (uint8_t)(rs_codewords[block][i] & 0x03U);
        tx_frame_symbols[pos++] = (uint8_t)(raw_bits ^ Message_WhiteningMask(fsk_index++));
      }
    }

    tx_frame_symbols[pos++] = FSK_SYNC_SYMBOL;
    for (uint8_t i = 0U; i < MSG_END_LEN; i++)
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

static void SPI1_Write16(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t word)
{
  cs_port->BSRR = (uint32_t)cs_pin << 16U;

  while ((hspi1.Instance->SR & SPI_SR_TXE) == 0U)
  {
  }
  *(__IO uint8_t *)&hspi1.Instance->DR = (uint8_t)(word >> 8U);

  while ((hspi1.Instance->SR & SPI_SR_TXE) == 0U)
  {
  }
  *(__IO uint8_t *)&hspi1.Instance->DR = (uint8_t)word;

  while ((hspi1.Instance->SR & SPI_SR_TXE) == 0U)
  {
  }
  while ((hspi1.Instance->SR & SPI_SR_BSY) != 0U)
  {
  }
  cs_port->BSRR = cs_pin;
}

static void DAC_Write12(uint16_t code)
{
  SPI1_Write16(DAC_CS_GPIO_Port, DAC_CS_Pin,
               (uint16_t)(TX_DAC_COMMAND_ACTIVE_1X | (code & 0x0FFFU)));
}

static void DAC_WriteSample8(uint8_t sample)
{
  DAC_Write12((uint16_t)sample << TX_DAC_CODE_SHIFT);
}

static void PGA_SetGain(uint8_t gain_code)
{
  SPI1_Write16(PGA_CS_GPIO_Port, PGA_CS_Pin,
               (uint16_t)(0x4000U | (gain_code & 0x07U)));
}

static void Editor_Task(void)
{
  if ((app_mode == APP_MODE_TX) &&
      (pending_active != 0U) &&
      ((HAL_GetTick() - pending_last_tick) >= TEXT_TAP_TIMEOUT_MS))
  {
    Editor_ConfirmPending();
    printf("edit auto-commit: text=\"%s\"\r\n", tx_text);
    OLED_PrintTxStatus();
  }
}

static void Editor_ProcessKey(char key)
{
  const char *group;
  uint8_t group_len;

  if (app_mode != APP_MODE_TX)
  {
    printf("RX mode: key %c ignored, press * for TX\r\n", key);
    return;
  }

  if (tx_mode != TX_MODE_IDLE)
  {
    printf("tx busy, key %c ignored\r\n", key);
    return;
  }

  if (tx_calibration_sent == 0U)
  {
    if (key == '#')
    {
      TX_StartCalibration();
    }
    else
    {
      printf("tx calibration required: key %c ignored, press #\r\n", key);
      OLED_PrintTxStatus();
    }
    return;
  }

  if (key == 'A')
  {
    Editor_ConfirmPending();
    if (tx_cursor_pos > 0U)
    {
      tx_cursor_pos--;
    }
  }
  else if (key == 'B')
  {
    Editor_ConfirmPending();
    if (tx_cursor_pos < tx_text_len)
    {
      tx_cursor_pos++;
    }
  }
  else if (key == 'C')
  {
    if (pending_active != 0U)
    {
      Editor_ClearPending();
    }
    else
    {
      Editor_Delete();
    }
  }
  else if (key == 'D')
  {
    Editor_ClearPending();
    tx_text_len = 0U;
    tx_cursor_pos = 0U;
    tx_text[0] = '\0';
  }
  else if (key == '#')
  {
    Editor_ConfirmPending();
    if (tx_text_len > 0U)
    {
      TX_StartMessage();
      return;
    }
  }
  else
  {
    group = Editor_KeyGroup(key);
    if (group == NULL)
    {
      return;
    }

    group_len = Editor_KeyGroupLen(group);
    if ((pending_active != 0U) && (pending_key == key))
    {
      pending_index++;
      if (pending_index >= group_len)
      {
        pending_index = 0U;
      }
    }
    else
    {
      Editor_ConfirmPending();
      pending_active = 1U;
      pending_key = key;
      pending_index = 0U;
    }
    pending_last_tick = HAL_GetTick();
  }

  printf("edit key=%c text=\"%s\" pending=%c pos=%u\r\n",
         key,
         tx_text,
         (pending_active != 0U) ? Editor_PendingChar() : '-',
         tx_cursor_pos);
  OLED_PrintTxStatus();
}

static const char *Editor_KeyGroup(char key)
{
  switch (key)
  {
    case '1': return ".,?!1";
    case '2': return "ABC2";
    case '3': return "DEF3";
    case '4': return "GHI4";
    case '5': return "JKL5";
    case '6': return "MNO6";
    case '7': return "PQRS7";
    case '8': return "TUV8";
    case '9': return "WXYZ9";
    case '0': return " 0";
    default: return NULL;
  }
}

static uint8_t Editor_KeyGroupLen(const char *group)
{
  uint8_t len = 0U;

  while (group[len] != '\0')
  {
    len++;
  }
  return len;
}

static char Editor_PendingChar(void)
{
  const char *group;

  if (pending_active == 0U)
  {
    return 0;
  }

  group = Editor_KeyGroup(pending_key);
  return (group != NULL) ? group[pending_index] : 0;
}

static void Editor_ConfirmPending(void)
{
  char ch = Editor_PendingChar();

  if (ch != 0)
  {
    Editor_Insert(ch);
  }
  Editor_ClearPending();
}

static void Editor_ClearPending(void)
{
  pending_active = 0U;
  pending_key = 0;
  pending_index = 0U;
  pending_last_tick = 0U;
}

static void Editor_Insert(char ch)
{
  int16_t i;

  if (tx_text_len >= MESSAGE_MAX_LEN)
  {
    return;
  }

  for (i = (int16_t)tx_text_len; i >= (int16_t)tx_cursor_pos; i--)
  {
    tx_text[i + 1] = tx_text[i];
  }
  tx_text[tx_cursor_pos] = ch;
  tx_text_len++;
  tx_cursor_pos++;
  tx_text[tx_text_len] = '\0';
}

static void Editor_Delete(void)
{
  uint8_t i;

  if (tx_text_len == 0U)
  {
    return;
  }
  if (tx_cursor_pos >= tx_text_len)
  {
    tx_cursor_pos = (uint8_t)(tx_text_len - 1U);
  }

  for (i = tx_cursor_pos; i < tx_text_len; i++)
  {
    tx_text[i] = tx_text[i + 1U];
  }
  tx_text_len--;
  tx_text[tx_text_len] = '\0';
  if (tx_cursor_pos > tx_text_len)
  {
    tx_cursor_pos = tx_text_len;
  }
}

static void Keypad_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t raw;

  if ((now - keypad_last_scan_ms) < KEYPAD_SCAN_MS)
  {
    return;
  }
  keypad_last_scan_ms = now;

  raw = Keypad_ScanRaw();

  if (raw == keypad_last_raw)
  {
    if (keypad_debounce_count < 255U)
    {
      keypad_debounce_count++;
    }
  }
  else
  {
    keypad_last_raw = raw;
    keypad_debounce_count = 1U;
  }

  if ((keypad_debounce_count >= KEYPAD_DEBOUNCE_SCANS) && (raw != keypad_stable_key))
  {
    keypad_stable_key = raw;
    if (raw != KEY_NONE)
    {
      Keypad_HandlePress(raw);
    }
  }
}

static uint8_t Keypad_ScanRaw(void)
{
  for (uint8_t row = 0U; row < KEYPAD_ROWS; row++)
  {
    for (uint8_t i = 0U; i < KEYPAD_ROWS; i++)
    {
      HAL_GPIO_WritePin(keypad_row_ports[i], keypad_row_pins[i], GPIO_PIN_SET);
    }

    HAL_GPIO_WritePin(keypad_row_ports[row], keypad_row_pins[row], GPIO_PIN_RESET);
    for (volatile uint32_t delay = 0U; delay < 80U; delay++)
    {
    }

    for (uint8_t col = 0U; col < KEYPAD_COLS; col++)
    {
      if (HAL_GPIO_ReadPin(keypad_col_ports[col], keypad_col_pins[col]) == GPIO_PIN_RESET)
      {
        for (uint8_t i = 0U; i < KEYPAD_ROWS; i++)
        {
          HAL_GPIO_WritePin(keypad_row_ports[i], keypad_row_pins[i], GPIO_PIN_SET);
        }
        return keypad_map[row][col];
      }
    }
  }

  for (uint8_t i = 0U; i < KEYPAD_ROWS; i++)
  {
    HAL_GPIO_WritePin(keypad_row_ports[i], keypad_row_pins[i], GPIO_PIN_SET);
  }

  return KEY_NONE;
}

static void Keypad_HandlePress(uint8_t key)
{
  if (key == '*')
  {
    App_ToggleMode();
    return;
  }

  if (app_mode == APP_MODE_RX)
  {
    MessageStore_HandleRxKey((char)key);
    return;
  }

  Editor_ProcessKey((char)key);
}

static uint8_t OLED_Init(void)
{
  static const uint8_t init_cmds[] = {
    0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
    0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
    0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
    0x40, 0x8D, 0x14, 0xAF
  };

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
  HAL_Delay(50U);

  for (uint16_t i = 0U; i < sizeof(init_cmds); i++)
  {
    OLED_WriteCommand(init_cmds[i]);
  }

  OLED_Clear();
  return 1U;
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

  if (oled_ready == 0U)
  {
    return;
  }

  buffer[0] = 0x40U;
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
  OLED_WriteCommand((uint8_t)(0xB0U | (page & 0x07U)));
  OLED_WriteCommand((uint8_t)(0x00U | (col & 0x0FU)));
  OLED_WriteCommand((uint8_t)(0x10U | ((col >> 4U) & 0x0FU)));
}

static void OLED_Clear(void)
{
  uint8_t zeros[OLED_WIDTH];
  memset(zeros, 0, sizeof(zeros));

  for (uint8_t page = 0U; page < OLED_PAGES; page++)
  {
    OLED_SetCursor(0U, page);
    OLED_WriteData(zeros, sizeof(zeros));
  }
}

static void OLED_ClearLine(uint8_t page)
{
  uint8_t zeros[OLED_WIDTH];

  memset(zeros, 0, sizeof(zeros));
  OLED_SetCursor(0U, page);
  OLED_WriteData(zeros, sizeof(zeros));
}

static void OLED_PrintAt(uint8_t col, uint8_t page, const char *text)
{
  OLED_SetCursor(col, page);
  while (*text != '\0')
  {
    OLED_PrintChar(*text++);
  }
}

static void OLED_PrintLine(uint8_t page, const char *text)
{
  OLED_ClearLine(page);
  OLED_PrintAt(0U, page, text);
}

static void OLED_PrintChar(char ch)
{
  uint8_t data[6];
  const uint8_t *glyph = OLED_Font5x7(ch);

  memcpy(data, glyph, 5U);
  data[5] = 0x00U;
  OLED_WriteData(data, sizeof(data));
}

static const uint8_t *OLED_Font5x7(char ch)
{
  static const uint8_t blank[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  static const uint8_t dash[5]  = {0x08, 0x08, 0x08, 0x08, 0x08};
  static const uint8_t gt[5]    = {0x00, 0x41, 0x22, 0x14, 0x08};
  static const uint8_t eq[5]    = {0x14, 0x14, 0x14, 0x14, 0x14};
  static const uint8_t dot[5]   = {0x00, 0x60, 0x60, 0x00, 0x00};
  static const uint8_t comma[5] = {0x00, 0x50, 0x30, 0x00, 0x00};
  static const uint8_t ques[5]  = {0x02, 0x01, 0x51, 0x09, 0x06};
  static const uint8_t excl[5]  = {0x00, 0x00, 0x5F, 0x00, 0x00};
  static const uint8_t hash[5]  = {0x14, 0x7F, 0x14, 0x7F, 0x14};
  static const uint8_t star[5]  = {0x14, 0x08, 0x3E, 0x08, 0x14};
  static const uint8_t n0[5]    = {0x3E, 0x51, 0x49, 0x45, 0x3E};
  static const uint8_t n1[5]    = {0x00, 0x42, 0x7F, 0x40, 0x00};
  static const uint8_t n2[5]    = {0x42, 0x61, 0x51, 0x49, 0x46};
  static const uint8_t n3[5]    = {0x21, 0x41, 0x45, 0x4B, 0x31};
  static const uint8_t n4[5]    = {0x18, 0x14, 0x12, 0x7F, 0x10};
  static const uint8_t n5[5]    = {0x27, 0x45, 0x45, 0x45, 0x39};
  static const uint8_t n6[5]    = {0x3C, 0x4A, 0x49, 0x49, 0x30};
  static const uint8_t n7[5]    = {0x01, 0x71, 0x09, 0x05, 0x03};
  static const uint8_t n8[5]    = {0x36, 0x49, 0x49, 0x49, 0x36};
  static const uint8_t n9[5]    = {0x06, 0x49, 0x49, 0x29, 0x1E};
  static const uint8_t A[5]     = {0x7E, 0x11, 0x11, 0x11, 0x7E};
  static const uint8_t B[5]     = {0x7F, 0x49, 0x49, 0x49, 0x36};
  static const uint8_t C[5]     = {0x3E, 0x41, 0x41, 0x41, 0x22};
  static const uint8_t D[5]     = {0x7F, 0x41, 0x41, 0x22, 0x1C};
  static const uint8_t E[5]     = {0x7F, 0x49, 0x49, 0x49, 0x41};
  static const uint8_t F[5]     = {0x7F, 0x09, 0x09, 0x09, 0x01};
  static const uint8_t G[5]     = {0x3E, 0x41, 0x49, 0x49, 0x7A};
  static const uint8_t H[5]     = {0x7F, 0x08, 0x08, 0x08, 0x7F};
  static const uint8_t I[5]     = {0x00, 0x41, 0x7F, 0x41, 0x00};
  static const uint8_t J[5]     = {0x20, 0x40, 0x41, 0x3F, 0x01};
  static const uint8_t K[5]     = {0x7F, 0x08, 0x14, 0x22, 0x41};
  static const uint8_t L[5]     = {0x7F, 0x40, 0x40, 0x40, 0x40};
  static const uint8_t M[5]     = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
  static const uint8_t N[5]     = {0x7F, 0x04, 0x08, 0x10, 0x7F};
  static const uint8_t O[5]     = {0x3E, 0x41, 0x41, 0x41, 0x3E};
  static const uint8_t P[5]     = {0x7F, 0x09, 0x09, 0x09, 0x06};
  static const uint8_t Q[5]     = {0x3E, 0x41, 0x51, 0x21, 0x5E};
  static const uint8_t R[5]     = {0x7F, 0x09, 0x19, 0x29, 0x46};
  static const uint8_t S[5]     = {0x46, 0x49, 0x49, 0x49, 0x31};
  static const uint8_t T[5]     = {0x01, 0x01, 0x7F, 0x01, 0x01};
  static const uint8_t U[5]     = {0x3F, 0x40, 0x40, 0x40, 0x3F};
  static const uint8_t V[5]     = {0x1F, 0x20, 0x40, 0x20, 0x1F};
  static const uint8_t W[5]     = {0x7F, 0x20, 0x18, 0x20, 0x7F};
  static const uint8_t X[5]     = {0x63, 0x14, 0x08, 0x14, 0x63};
  static const uint8_t Y[5]     = {0x07, 0x08, 0x70, 0x08, 0x07};
  static const uint8_t Z[5]     = {0x61, 0x51, 0x49, 0x45, 0x43};

  switch (ch)
  {
    case ' ': return blank;
    case '-': return dash;
    case '>': return gt;
    case '=': return eq;
    case '.': return dot;
    case ',': return comma;
    case '?': return ques;
    case '!': return excl;
    case '#': return hash;
    case '*': return star;
    case '0': return n0;
    case '1': return n1;
    case '2': return n2;
    case '3': return n3;
    case '4': return n4;
    case '5': return n5;
    case '6': return n6;
    case '7': return n7;
    case '8': return n8;
    case '9': return n9;
    case 'A': return A;
    case 'B': return B;
    case 'C': return C;
    case 'D': return D;
    case 'E': return E;
    case 'F': return F;
    case 'G': return G;
    case 'H': return H;
    case 'I': return I;
    case 'J': return J;
    case 'K': return K;
    case 'L': return L;
    case 'M': return M;
    case 'N': return N;
    case 'O': return O;
    case 'P': return P;
    case 'Q': return Q;
    case 'R': return R;
    case 'S': return S;
    case 'T': return T;
    case 'U': return U;
    case 'V': return V;
    case 'W': return W;
    case 'X': return X;
    case 'Y': return Y;
    case 'Z': return Z;
    default: return blank;
  }
}

static void OLED_PrintDetected(const FSK_DetectResult *result)
{
  (void)result;
  OLED_PrintRxStatus();
}

static void OLED_PrintMode(void)
{
  OLED_Clear();
  OLED_PrintRxStatus();
  OLED_PrintTxStatus();
}

static void OLED_PrintTextRows(const char *text, uint8_t len)
{
  char row[22];

  for (uint8_t line_index = 0U; line_index < 3U; line_index++)
  {
    uint8_t offset = (uint8_t)(line_index * 21U);
    uint8_t count = 0U;

    if (len > offset)
    {
      count = (uint8_t)(len - offset);
      if (count > 21U)
      {
        count = 21U;
      }
      memcpy(row, &text[offset], count);
    }
    row[count] = '\0';
    OLED_PrintLine((uint8_t)(2U + (line_index * 2U)), row);
  }
}

static void OLED_PrintRxStatus(void)
{
  char line[22];
  const char *display_text = "";
  uint8_t display_len = 0U;

  if (app_mode != APP_MODE_RX)
  {
    return;
  }

  if (calibration_complete == 0U)
  {
    if (message_store_count > 0U)
    {
      const StoredMessage *stored = &message_store_cache[message_store_view_index];

      display_text = stored->text;
      display_len = stored->len;
    }
    (void)snprintf(line, sizeof(line), "RX CAL %u/4 %s",
                   calibration_stage + 1U,
                   (calibration_capture_active != 0U) ? "GAVG" : "WAIT");
  }
  else if ((message_store_last_save_failed != 0U) && (rx_message_valid != 0U))
  {
    (void)snprintf(line, sizeof(line), "RX NOSAVE L%u", rx_message_len);
    display_text = rx_message;
    display_len = rx_message_len;
  }
  else if (message_store_count > 0U)
  {
    const StoredMessage *stored = &message_store_cache[message_store_view_index];

    (void)snprintf(line, sizeof(line), "RX MEM %u/%u L%u",
                   message_store_view_index + 1U,
                   message_store_count,
                   stored->len);
    display_text = stored->text;
    display_len = stored->len;
  }
  else if (rx_message_valid != 0U)
  {
    (void)snprintf(line, sizeof(line), "RX OK L%u RS+%u", rx_message_len, rx_rs_corrected);
    display_text = rx_message;
    display_len = rx_message_len;
  }
  else if (rx_sampling_active == 0U)
  {
    (void)snprintf(line, sizeof(line), "RX PAUSED MEM0");
  }
  else
  {
    (void)snprintf(line, sizeof(line), "RX LISTEN MEM0");
  }

  OLED_PrintLine(0U, line);
  OLED_PrintTextRows(display_text, display_len);
}

static void OLED_PrintTxStatus(void)
{
  char line[22];
  char preview[MESSAGE_MAX_LEN + 2U];
  uint8_t preview_len = tx_text_len;

  if (app_mode != APP_MODE_TX)
  {
    return;
  }

  if (tx_calibration_sent == 0U)
  {
    if (tx_mode == TX_MODE_CALIBRATION)
    {
      (void)snprintf(line, sizeof(line), "TX CAL SENDING");
    }
    else
    {
      (void)snprintf(line, sizeof(line), "TX CAL PRESS #");
    }
    OLED_PrintLine(0U, line);
    OLED_PrintTextRows("", 0U);
    return;
  }

  memcpy(preview, tx_text, tx_text_len + 1U);
  if ((pending_active != 0U) && (preview_len < MESSAGE_MAX_LEN))
  {
    for (int16_t i = (int16_t)preview_len; i >= (int16_t)tx_cursor_pos; i--)
    {
      preview[i + 1] = preview[i];
    }
    preview[tx_cursor_pos] = Editor_PendingChar();
    preview_len++;
    preview[preview_len] = '\0';
  }

  (void)snprintf(line, sizeof(line), "TX %s L%u/%u",
                 (tx_display_sending != 0U) ? "SEND" : "EDIT",
                 preview_len, MESSAGE_MAX_LEN);
  OLED_PrintLine(0U, line);
  OLED_PrintTextRows(preview, preview_len);
}

static void StatusLed_Update(void)
{
  if ((rx_complete_led_active != 0U) &&
      ((int32_t)(HAL_GetTick() - rx_complete_led_off_tick) >= 0))
  {
    rx_complete_led_active = 0U;
    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);
  }
}

static void StatusLed_NotifyRxComplete(void)
{
  rx_complete_led_active = 1U;
  rx_complete_led_off_tick = HAL_GetTick() + RX_COMPLETE_LED_ON_MS;
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_ON);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    TX_AudioTick();
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (rx_sampling_active != 0U))
  {
    symbol_start_index = 0U;
    symbol_ready = 1U;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (rx_sampling_active != 0U))
  {
    symbol_start_index = FSK_SYMBOL_SAMPLES;
    symbol_ready = 1U;
  }
}
/* USER CODE END 0 */

int main(void)
{
  PowerHold_EarlyOn();

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  DAC_Write12(TX_DAC_MID_CODE);
  PGA_SetGain(RX_PGA_GAIN_CODE);

  FSK_Init();
  RS_Init();
  MessageStore_Init();
  HAL_Delay(300U);
  (void)OLED_Init();
  OLED_PrintMode();

  printf("\r\nSecond board 4-FSK RX/TX start\r\n");
  printf("HALF DUPLEX: RX PC4 via MCP6S21 x%u, TX MCP4921 DAC, *=RX/TX\r\n",
         RX_PGA_GAIN_VALUE);
  printf("SPI1 MODE0 4MHz: SCK PA5, MOSI PA7, DAC_CS PB0, PGA_CS PA6\r\n");
  printf("POWER: SW2 PB9, HOLD PC1; AMP_MUTE PB12 active high\r\n");
  printf("KEYPAD: 1 2 3 A / 4 5 6 B / 7 8 9 C / * 0 # D\r\n");
  printf("EDITOR: phone multi-tap, A=LEFT B=RIGHT C=DELETE D=CLEAR *=RX/TX #=CAL-FIRST/SEND\r\n");
  printf("RX MEMORY: A=OLDER B=NEWER C=LATEST, saved=%u/5; BLUE LED=RX COMPLETE\r\n",
         message_store_count);
  printf("Fs=%uHz, symbol=%ums, samples=%u\r\n",
         FSK_FS_HZ, FSK_SYMBOL_MS, FSK_SYMBOL_SAMPLES);
  printf("FEC: %ux RS(%u,%u) GF(64), payload=%u chars parity=%u, t=%u per block/%u total\r\n",
         RS_BLOCK_COUNT, RS_CODEWORD_SYMBOLS, RS_DATA_SYMBOLS, RS_TOTAL_DATA_SYMBOLS,
         RS_TOTAL_PARITY_SYMBOLS, RS_CORRECTABLE_SYMBOLS, RS_TOTAL_CORRECTABLE);
  printf("FSK whitening: ON, mask cycle=00/01/10/11\r\n");
  printf("RS framing marker: %uHz before every 6-bit RS symbol\r\n", FSK_SYNC_FREQ_HZ);
  printf("TX timing: %u tones, %u+%u=%ums each, frame=%ums\r\n",
         TX_FRAME_MAX_SYMBOLS, TX_TONE_MS, TX_GAP_MS, TX_STEP_MS, TX_FRAME_DURATION_MS);
  printf("RS capture: all valid 20ms windows; slots=40..139/140..199/200..259ms\r\n");
  printf("RX adapt: timed PREAMBLE recovery + per-data-bin soft fallback + END soft/repeat guard\r\n");
  printf("RX fix: PREAMBLE top-2 energy + 50pct scale smoothing\r\n");
  printf("MANUAL CAL: enter TX and press #; first # sends only 00/01/10/11 for %ums each\r\n",
         CALIBRATION_TONE_MS);
  printf("CAL RX: fixed 1500/2500/3500/4500Hz Goertzel, %u-window mean energy\r\n",
         CALIBRATION_ENERGY_WINDOWS);
  printf("RS self-test: %s\r\n", (RS_SelfTest() != 0U) ? "PASS" : "FAIL");

  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);

  if (RX_StartSampling() == 0U)
  {
    Error_Handler();
  }
  printf("half duplex: RX listening\r\n");
  OLED_PrintRxStatus();

  while (1)
  {
    StatusLed_Update();
    Keypad_Task();
    Editor_Task();
    PowerKey_Task();
    OLED_RetryInitTask();

    if ((symbol_ready != 0U) && (rx_sampling_active != 0U))
    {
      uint16_t start;

      __disable_irq();
      start = symbol_start_index;
      symbol_ready = 0U;
      __enable_irq();

      FSK_ProcessSymbol(&adc_dma_buf[start]);
    }

    MessageStore_Task();
    TX_UI_Task();
    HalfDuplex_Task();
  }
}

void SystemClock_Config(void)
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

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

  sConfig.Channel = ADC_CHANNEL_14; /* PC4 */
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

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

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10U;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  SPI_1LINE_TX(&hspi1);
  __HAL_SPI_ENABLE(&hspi1);
}

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

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);
  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
  HAL_GPIO_WritePin(POWER_HOLD_GPIO_Port, POWER_HOLD_Pin, POWER_HOLD_ON);
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, DAC_CS_INACTIVE);
  HAL_GPIO_WritePin(PGA_CS_GPIO_Port, PGA_CS_Pin, PGA_CS_INACTIVE);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

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

  GPIO_InitStruct.Pin = DAC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DAC_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PGA_CS_Pin;
  HAL_GPIO_Init(PGA_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = POWER_HOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_HOLD_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = POWER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_KEY_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
    HAL_Delay(100);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
