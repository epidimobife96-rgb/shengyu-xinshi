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

typedef struct
{
  uint16_t fsk_index;
  uint8_t source;
  uint8_t selected_bits;
  float confidence;
  float energy[4U];
} RxDecisionDiagnostic;

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
  COMM_MODE_UNSELECTED = 0,
  COMM_MODE_STANDARD,
  COMM_MODE_MULTI_NODE,
  COMM_MODE_HIDDEN
} CommunicationMode;

typedef enum
{
  TX_FRAME_TONE = 0,
  TX_FRAME_GAP
} TX_FramePart;

typedef enum
{
  MESSAGE_PAYLOAD_OK = 0,
  MESSAGE_PAYLOAD_CRC_FAIL,
  MESSAGE_PAYLOAD_DATA_AFTER_PAD,
  MESSAGE_PAYLOAD_INVALID_CODE,
  MESSAGE_PAYLOAD_EMPTY
} MessagePayloadStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FSK_NORMAL_FS_HZ          16000U
#define FSK_HIDDEN_FS_HZ          20000U
#define FSK_SYMBOL_MS             20U
#define FSK_NORMAL_SYMBOL_SAMPLES ((FSK_NORMAL_FS_HZ * FSK_SYMBOL_MS) / 1000U)
#define FSK_HIDDEN_SYMBOL_SAMPLES ((FSK_HIDDEN_FS_HZ * FSK_SYMBOL_MS) / 1000U)
#define FSK_MAX_SYMBOL_SAMPLES    FSK_HIDDEN_SYMBOL_SAMPLES
#define ADC_DMA_BUFFER_SAMPLES    (FSK_MAX_SYMBOL_SAMPLES * 2U)
#define TIM2_COUNTER_CLOCK_HZ     16000000U

#define FSK_FREQ_COUNT            5U
#define FSK_DATA_FREQ_COUNT       4U
#define FSK_RX_FREQ_00_HZ         1500U
#define FSK_SYNC_FREQ_HZ          2000U
#define FSK_RX_FREQ_01_HZ         2500U
#define FSK_RX_FREQ_10_HZ         3500U
#define FSK_RX_FREQ_11_HZ         4500U
#define FSK_HIGH_FREQ_00_HZ       6400U
#define FSK_HIGH_FREQ_01_HZ       6550U
#define FSK_HIGH_SYNC_FREQ_HZ     6700U
#define FSK_HIGH_FREQ_10_HZ       6850U
#define FSK_HIGH_FREQ_11_HZ       7000U
#define TX_HIGH_PROFILE_AMP       96U
/* Hidden mode uses the restored high-frequency profile. */
#define FSK_HIGH_PROFILE_ENABLE   1U
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
#define DIGIT_FRAME_TIMEOUT_MS    22000U
#define START_SEQUENCE_TIMEOUT_MS 400U
#define DIGIT_NO_SYMBOL           0xFFU
#define MSG_PREAMBLE_LEN          4U
#define MSG_START_LEN             2U
#define MSG_END_LEN               2U
#define RS_DATA_SYMBOLS           25U
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
#define MESSAGE_CRC_SYMBOLS       2U
#define MESSAGE_MAX_LEN           (RS_TOTAL_DATA_SYMBOLS - MESSAGE_CRC_SYMBOLS)
#define MESSAGE_CRC12_POLY        0x080FU
#define MESSAGE_CRC12_MASK        0x0FFFU
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
#define RX_PGA_DEFAULT_GAIN_CODE  0x07U  /* MCP6S21 gain code 7 = x32 */
#define RX_PGA_MULTI_GAIN_CODE    0x01U  /* MCP6S21 gain code 1 = x2 */
#define TX_TONE_MS                60U
#define TX_GAP_MS                 9U
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
#define RX_DIAG_WORST_COUNT       8U
#define RX_DECISION_VOTE          0U
#define RX_DECISION_NEAREST       1U
#define RX_DECISION_SOFT          2U
#define RX_DECISION_ERASURE       3U

#if TX_FRAME_DURATION_MS > 20100U
#error "TX message frame must remain no longer than 20.10 seconds"
#endif

#define CALIBRATION_STAGE_MS             1000U
#define CALIBRATION_PASS_COUNT            2U
#define CALIBRATION_BOUNDARY_MS            300U
#define CALIBRATION_BOUNDARY_STABLE_WINDOWS 5U
#define CALIBRATION_INTERPASS_GAP_MS      2000U
#define CALIBRATION_GAP_DETECT_MS         1200U
#define CALIBRATION_TX_GUARD_MS          300U
#define CALIBRATION_STAGE_SAMPLES        ((TX_AUDIO_FS_HZ * CALIBRATION_STAGE_MS) / 1000U)
#define CALIBRATION_BOUNDARY_SAMPLES     ((TX_AUDIO_FS_HZ * CALIBRATION_BOUNDARY_MS) / 1000U)
#define CALIBRATION_INTERPASS_GAP_SAMPLES ((TX_AUDIO_FS_HZ * CALIBRATION_INTERPASS_GAP_MS) / 1000U)
#define CALIBRATION_TX_GUARD_SAMPLES     ((TX_AUDIO_FS_HZ * CALIBRATION_TX_GUARD_MS) / 1000U)
#define CALIBRATION_STABLE_WINDOWS       5U
#define CALIBRATION_ENERGY_WINDOWS       12U
#define CALIBRATION_LOST_WINDOWS_MAX     3U
#define CALIBRATION_PROGRESS_TIMEOUT_MS  2500U
#define CALIBRATION_SCALE_MIN            0.005f
#define RX_PGA_ADC_LOW_PEAK              350U
#define RX_PGA_ADC_HIGH_PEAK             1650U
#define RX_PGA_ADC_CLIP_MARGIN            128U
#define RX_PGA_AGC_SETTLE_MS               60U
#define RX_PGA_AGC_HIGH_WINDOWS             4U
#define RX_PGA_AGC_CLIP_WINDOWS             3U
#define RX_PGA_AGC_LOW_WINDOWS              5U
#define RX_PGA_AGC_TONE_RATIO              1.8f
#define PREAMBLE_RECOVERY_ENERGY_MIN     5000000.0f
#define PREAMBLE_FRAME_SCALE_MIN_CORR     0.50f
#define PREAMBLE_FRAME_SCALE_MAX_CORR     2.00f
#define PREAMBLE_FRAME_MAX_4500_TO_3500   2.50f
#define PREAMBLE_RECOVERY_01_END_MS       70U
#define PREAMBLE_RECOVERY_10_START_MS     50U
#define PREAMBLE_RECOVERY_10_END_MS      140U
#define PREAMBLE_RECOVERY_11_START_MS    120U
#define PREAMBLE_RECOVERY_11_END_MS      210U
#define PREAMBLE_RECOVERY_START0_START_MS 190U
#define PREAMBLE_RECOVERY_START0_END_MS  255U
#define PREAMBLE_RECOVERY_START1_START_MS 260U
#define PREAMBLE_RECOVERY_START1_END_MS  330U
#define PREAMBLE_RECOVERY_MARKER_MIN_MS  300U
#define PREAMBLE_RECOVERY_MARKER_MAX_MS  400U

#define KEYPAD_ROWS               4U
#define KEYPAD_COLS               4U
#define KEYPAD_SCAN_MS            20U
#define KEYPAD_DEBOUNCE_SCANS     2U
#define KEYPAD_LONG_PRESS_MS      800U
#define KEY_NONE                  0U

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
#define MESSAGE_STORE_LENGTH_MASK   0x000000FFUL
#define MESSAGE_STORE_SOURCE_SHIFT  8U
#define MESSAGE_STORE_DEST_SHIFT    10U
#define MESSAGE_STORE_ROUTE_MASK    0x00000F00UL
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
  uint8_t source;
  uint8_t destination;
  char text[MESSAGE_MAX_LEN + 1U];
} StoredMessage;

static uint16_t adc_dma_buf[ADC_DMA_BUFFER_SAMPLES];
static volatile uint8_t symbol_ready = 0U;
static volatile uint16_t symbol_start_index = 0U;
static uint32_t fsk_sample_rate_hz = FSK_NORMAL_FS_HZ;
static uint16_t fsk_symbol_samples = FSK_NORMAL_SYMBOL_SAMPLES;

static const uint16_t fsk_normal_freqs_hz[FSK_FREQ_COUNT] = {
  FSK_RX_FREQ_00_HZ,
  FSK_RX_FREQ_01_HZ,
  FSK_RX_FREQ_10_HZ,
  FSK_RX_FREQ_11_HZ,
  FSK_SYNC_FREQ_HZ
};
#if FSK_HIGH_PROFILE_ENABLE != 0U
static const uint16_t fsk_high_freqs_hz[FSK_FREQ_COUNT] = {
  FSK_HIGH_FREQ_00_HZ,
  FSK_HIGH_FREQ_01_HZ,
  FSK_HIGH_FREQ_10_HZ,
  FSK_HIGH_FREQ_11_HZ,
  FSK_HIGH_SYNC_FREQ_HZ
};
#endif
static uint16_t fsk_tx_freqs_hz[FSK_FREQ_COUNT] = {
  FSK_RX_FREQ_00_HZ,
  FSK_RX_FREQ_01_HZ,
  FSK_RX_FREQ_10_HZ,
  FSK_RX_FREQ_11_HZ,
  FSK_SYNC_FREQ_HZ
};
static uint16_t fsk_rx_freqs_hz[FSK_FREQ_COUNT];
#if FSK_HIGH_PROFILE_ENABLE != 0U
static uint8_t fsk_high_profile_enabled = 0U;
#endif
static const uint8_t fsk_bits[FSK_FREQ_COUNT] = {0x00U, 0x01U, 0x02U, 0x03U, FSK_SYNC_SYMBOL};
static const uint8_t msg_preamble[MSG_PREAMBLE_LEN] = {0x00U, 0x01U, 0x02U, 0x03U};
static const uint8_t msg_end[MSG_END_LEN] = {0x02U, 0x03U};
static float fsk_goertzel_coeffs[FSK_FREQ_COUNT];
static float fsk_energy_scale[FSK_FREQ_COUNT];
static float calibration_energy_scale[FSK_DATA_FREQ_COUNT];

static float calibration_energy[FSK_DATA_FREQ_COUNT];
static float calibration_energy_sum = 0.0f;
static uint8_t calibration_energy_windows = 0U;
static uint8_t calibration_lost_windows = 0U;
static uint8_t calibration_stage = 0U;
static uint8_t calibration_stable_windows = 0U;
static uint8_t calibration_capture_active = 0U;
static uint8_t calibration_complete = 0U;
static uint8_t calibration_failed = 0U;
static uint32_t calibration_progress_tick = 0U;
static uint8_t calibration_pass = 0U;
static uint8_t calibration_coarse_activity = 0U;
static uint8_t calibration_boundary_windows = 0U;
static uint32_t calibration_coarse_last_signal_tick = 0U;
static uint16_t calibration_adc_min_seen = 0xFFFFU;
static uint16_t calibration_adc_max_seen = 0U;
static uint16_t calibration_adc_mean_last = 2048U;
static uint32_t rx_pga_agc_last_adjust_tick = 0U;
static uint8_t rx_pga_agc_high_windows = 0U;
static uint8_t rx_pga_agc_clip_windows = 0U;
static uint8_t rx_pga_agc_low_windows = 0U;
static const uint8_t rx_pga_gain_values[8U] = {1U, 2U, 4U, 5U, 8U, 10U, 16U, 32U};
static uint8_t rx_pga_gain_code = RX_PGA_DEFAULT_GAIN_CODE;

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
static uint8_t msg_preamble_recovery_start_bits[MSG_START_LEN];
static uint8_t local_node_id = 0U;
static uint8_t tx_destination_id = 0U;
static uint8_t msg_rx_source_id = 0U;
static uint8_t msg_rx_destination_id = 0U;
static uint8_t msg_rs_symbol_index = 0U;
static uint8_t msg_rs_fsk_index = 0U;
static uint8_t msg_rs_symbol = 0U;
static uint8_t msg_rs_codeword[RS_TOTAL_CODEWORD_SYMBOLS];
static uint8_t msg_rs_codeword_erased[RS_TOTAL_CODEWORD_SYMBOLS];
static float msg_rs_codeword_reliability[RS_TOTAL_CODEWORD_SYMBOLS];
static uint8_t msg_rs_current_erased = 0U;
static float msg_rs_current_reliability = RS_VOTE_RATIO_CAP;
static uint8_t msg_rs_erasure_count = 0U;
static uint8_t msg_soft_fallback_count = 0U;
static uint8_t msg_vote_override_count = 0U;
static uint32_t msg_marker_tick = 0U;
static uint8_t msg_capture_bits[RS_FSK_SYMBOLS_PER_SYMBOL];
static uint8_t msg_capture_valid[RS_FSK_SYMBOLS_PER_SYMBOL];
static uint8_t msg_capture_distance[RS_FSK_SYMBOLS_PER_SYMBOL];
static float msg_capture_scores[RS_FSK_SYMBOLS_PER_SYMBOL][4];
static float msg_capture_best_ratio[RS_FSK_SYMBOLS_PER_SYMBOL][FSK_DATA_FREQ_COUNT];
static float msg_capture_best_energy[RS_FSK_SYMBOLS_PER_SYMBOL][FSK_DATA_FREQ_COUNT][FSK_DATA_FREQ_COUNT];
static uint8_t msg_soft_bits[RS_FSK_SYMBOLS_PER_SYMBOL];
static uint8_t msg_soft_valid[RS_FSK_SYMBOLS_PER_SYMBOL];
static float msg_soft_score[RS_FSK_SYMBOLS_PER_SYMBOL];
static float msg_soft_ratio[RS_FSK_SYMBOLS_PER_SYMBOL];
static float msg_soft_energy[RS_FSK_SYMBOLS_PER_SYMBOL][FSK_DATA_FREQ_COUNT];
static RxDecisionDiagnostic msg_worst_decisions[RX_DIAG_WORST_COUNT];
static uint8_t msg_worst_decision_count = 0U;
static uint8_t msg_low_confidence_count = 0U;
static uint32_t msg_frame_tick = 0U;
static char rx_message[MESSAGE_MAX_LEN + 1U];
static uint8_t rx_message_len = 0U;
static uint8_t rx_message_valid = 0U;
static uint8_t rx_rs_corrected = 0U;
static uint8_t rx_message_source_id = 0U;
static uint8_t rx_message_destination_id = 0U;
static uint32_t rx_display_seq = 0U;
static uint32_t uart_symbol_seq = 0U;

static StoredMessage message_store_cache[MESSAGE_STORE_VISIBLE_COUNT];
static uint8_t message_store_count = 0U;
static uint8_t message_store_view_index = 0U;
static uint32_t message_store_next_sequence = 1U;
static uint32_t message_store_write_address = MESSAGE_STORE_FLASH_START;
static uint8_t message_store_pending = 0U;
static uint8_t message_store_pending_len = 0U;
static uint8_t message_store_pending_source = 0U;
static uint8_t message_store_pending_destination = 0U;
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
#if FSK_HIGH_PROFILE_ENABLE != 0U
static const uint8_t tx_normal_fsk_amp[FSK_FREQ_COUNT] = {
  30U,
  72U,
  TX_MAX_SINE_AMP,
  TX_MAX_SINE_AMP,
  60U
};
static const uint8_t tx_high_fsk_amp[FSK_FREQ_COUNT] = {
  TX_HIGH_PROFILE_AMP,
  TX_HIGH_PROFILE_AMP,
  TX_HIGH_PROFILE_AMP,
  TX_HIGH_PROFILE_AMP,
  TX_HIGH_PROFILE_AMP
};
#endif
static uint8_t tx_fsk_amp[FSK_FREQ_COUNT] = {
  30U,
  72U,
  TX_MAX_SINE_AMP,
  TX_MAX_SINE_AMP,
  60U
};
static const uint8_t tx_preamble_symbols[MSG_PREAMBLE_LEN] = {0x00U, 0x01U, 0x02U, 0x03U};
static const uint8_t tx_end_symbols[MSG_END_LEN] = {0x02U, 0x03U};

static volatile TX_Mode tx_mode = TX_MODE_IDLE;
static AppMode app_mode = APP_MODE_RX;
static CommunicationMode communication_mode = COMM_MODE_UNSELECTED;
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
static volatile uint8_t tx_calibration_pass = 0U;
static volatile uint8_t tx_calibration_boundary_active = 0U;
static volatile uint32_t tx_calibration_stage_sample_count = 0U;
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
static uint32_t keypad_press_start_ms = 0U;
static uint8_t keypad_defer_short_press = 0U;
static uint8_t keypad_long_press_handled = 0U;

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
#if FSK_HIGH_PROFILE_ENABLE != 0U
static void FSK_ApplyProfile(uint8_t high_profile);
#endif
static void FSK_UpdateGoertzelCoefficient(uint8_t index);
static float FSK_WindowMean(const uint16_t *samples, uint16_t len);
static float Goertzel_EnergyWithMean(const uint16_t *samples, uint16_t len, float coeff, float mean);
static FSK_DetectResult FSK_DetectSymbol(const uint16_t *samples, uint16_t len);
static void FSK_ProcessSymbol(const uint16_t *samples);
static void Calibration_ResetProgress(void);
static void Calibration_StartVerificationPass(uint32_t now);
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
static void MessageRx_RestoreCalibrationEnergyScale(void);
static void MessageRx_ApplyPreambleFrameScale(void);
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
static void MessageRx_AppendRsBits(uint8_t decoded_bits, uint8_t erased,
                                   float confidence);
static void MessageRx_RecordDecision(uint16_t fsk_index, uint8_t source,
                                     uint8_t selected_bits, float confidence,
                                     const float *energy);
static void MessageRx_PrintDecisionDiagnostics(void);
static void MessageRx_ClearCapture(void);
static void MessageRx_FinalizeCapture(void);
static void MessageRx_ResetFrame(void);
static const char *MessageRx_StateName(MessageRxState state);
static void MessageRx_PrintSymbol(const FSK_DetectResult *result, uint32_t now);
static uint8_t Message_WhiteningMask(uint16_t fsk_symbol_index);
static uint8_t Node_SourceBitsFromId(uint8_t node_id);
static uint8_t Node_IdFromSourceBits(uint8_t bits);
static const char *Node_DestinationLabel(uint8_t destination);
static void Node_SetLocalId(uint8_t node_id);
static void Node_CycleDestination(void);
static uint8_t Message_CharToCode(char ch);
static char Message_CodeToChar(uint8_t code);
static uint16_t Message_Crc12Symbols(const uint8_t *symbols, uint8_t count);
static uint16_t Message_Crc12Addressed(uint8_t source, uint8_t destination,
                                      const uint8_t *symbols, uint8_t count);
static uint8_t Message_CrcSelfTest(void);
static MessagePayloadStatus Message_ValidateRsPayload(const uint8_t *codewords,
                                                       uint8_t source,
                                                       uint8_t destination,
                                                       char *text,
                                                       uint8_t *text_len,
                                                       uint16_t *received_crc,
                                                       uint16_t *calculated_crc,
                                                       uint8_t *failure_index,
                                                       uint8_t *failure_code);
static uint32_t MessageStore_HashRecord(const MessageStoreRecord *record);
static uint8_t MessageStore_RecordLength(const MessageStoreRecord *record);
static uint8_t MessageStore_RecordSource(const MessageStoreRecord *record);
static uint8_t MessageStore_RecordDestination(const MessageStoreRecord *record);
static uint8_t MessageStore_RecordIsErased(uint32_t address);
static uint8_t MessageStore_RecordValid(const MessageStoreRecord *record);
static void MessageStore_CacheAppend(uint32_t sequence, const char *text, uint8_t len,
                                     uint8_t source, uint8_t destination);
static void MessageStore_BuildRecord(MessageStoreRecord *record, uint32_t sequence,
                                     const char *text, uint8_t len,
                                     uint8_t source, uint8_t destination);
static uint8_t MessageStore_ProgramRecordUnlocked(uint32_t address,
                                                  const MessageStoreRecord *record);
static uint8_t MessageStore_CompactUnlocked(void);
static void MessageStore_Init(void);
static uint8_t MessageStore_Save(const char *text, uint8_t len,
                                 uint8_t source, uint8_t destination);
static void MessageStore_Task(void);
static void MessageStore_HandleRxKey(char key);
static void RS_Init(void);
static uint8_t RS_GfMul(uint8_t a, uint8_t b);
static uint8_t RS_GfDiv(uint8_t a, uint8_t b);
static uint8_t RS_PolyEvalAscending(const uint8_t *poly, uint8_t degree, uint8_t x);
static void RS_CalculateSyndromes(const uint8_t *codeword, uint8_t *syndromes);
static void RS_Encode(const uint8_t *data, uint8_t *codeword);
static int8_t RS_Decode(uint8_t *codeword, const uint8_t *erasure_positions,
                        uint8_t erasure_count, uint8_t *corrected_count);
static int8_t RS_DecodeAdaptive(uint8_t *codeword,
                                const uint8_t *erased_flags,
                                const float *reliability,
                                uint8_t *corrected_count,
                                uint8_t *used_erasure_count,
                                uint8_t *adaptive_erasure_count);
static int8_t RS_DecodeAdaptiveAttempt(const uint8_t *received,
                                       const uint8_t *erased_flags,
                                       const float *reliability,
                                       uint8_t adaptive_erasure_count,
                                       uint8_t *decoded,
                                       uint8_t *corrected_count,
                                       uint8_t *used_erasure_count);
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
static void OLED_ShowStartupScreen(void);
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
static void TX_LoadCalibrationBoundaryTone(void);
static void TX_BuildMessageFrame(void);
static uint32_t TX_PhaseIncFromFreq(uint16_t freq_hz);
static uint8_t TX_ScaledSineSample(void);
static void SPI1_Write16(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t word);
static void DAC_Write12(uint16_t code);
static void DAC_WriteSample8(uint8_t sample);
static void PGA_SetGain(uint8_t gain_code);
static uint8_t PGA_IsCalibrationToneQualified(const FSK_DetectResult *result);
static uint8_t PGA_AutoGainControl(const FSK_DetectResult *result, uint32_t now);
static uint8_t PGA_ManualAdjust(int8_t direction);
static const char *PGA_AdcQuality(void);
static void TX_UI_Task(void);
static uint8_t RX_StartSampling(void);
static void RX_StopSampling(void);
static void RX_ResetDetector(void);
static void HalfDuplex_Task(void);
static void App_ToggleMode(void);
static void CommunicationMode_Select(CommunicationMode mode);
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
static void Keypad_HandleLongPress(uint8_t key);
static void StatusLed_Update(void);
static void StatusLed_NotifyRxComplete(void);
static void PowerKill_EarlyKeepAlive(void);
static void PowerInt_Task(void);
static void App_PowerOff(void);
static void OLED_RetryInitTask(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void PowerKill_EarlyKeepAlive(void)
{
  /* Keep LTC2950 KILL high as soon as possible, before HAL/system init. */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  __DSB();

  GPIOC->BSRR = POWER_KILL_Pin;
  GPIOC->MODER = (GPIOC->MODER & ~(3UL << (1U * 2U))) | (1UL << (1U * 2U));
  GPIOC->OTYPER &= ~POWER_KILL_Pin;
  GPIOC->OSPEEDR &= ~(3UL << (1U * 2U));
  GPIOC->PUPDR &= ~(3UL << (1U * 2U));
  GPIOC->BSRR = POWER_KILL_Pin;
}

static void PowerInt_Task(void)
{
  static uint8_t poweroff_started = 0U;

  if ((poweroff_started == 0U) &&
      (HAL_GPIO_ReadPin(POWER_INT_GPIO_Port, POWER_INT_Pin) == POWER_INT_ASSERTED))
  {
    poweroff_started = 1U;
    App_PowerOff();
  }
}

static void App_PowerOff(void)
{
  printf("power INT# asserted: stopping, PC1/KILL low\r\n");

  TX_Stop();
  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, BOARD_LED_OFF);

  OLED_WriteCommand(0xAEU);

  HAL_GPIO_WritePin(POWER_KILL_GPIO_Port, POWER_KILL_Pin, POWER_KILL_POWER_OFF);

  while (1)
  {
    HAL_GPIO_WritePin(POWER_KILL_GPIO_Port, POWER_KILL_Pin, POWER_KILL_POWER_OFF);
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

#if FSK_HIGH_PROFILE_ENABLE != 0U
static void FSK_ApplyProfile(uint8_t high_profile)
{
  const uint16_t *profile_freqs;
  const uint8_t *profile_amps;
  uint32_t tim2_period;

  fsk_high_profile_enabled = (high_profile != 0U) ? 1U : 0U;
  profile_freqs = (fsk_high_profile_enabled != 0U) ? fsk_high_freqs_hz : fsk_normal_freqs_hz;
  profile_amps = (fsk_high_profile_enabled != 0U) ? tx_high_fsk_amp : tx_normal_fsk_amp;
  fsk_sample_rate_hz = (fsk_high_profile_enabled != 0U) ? FSK_HIDDEN_FS_HZ : FSK_NORMAL_FS_HZ;
  fsk_symbol_samples = (fsk_high_profile_enabled != 0U) ?
                       FSK_HIDDEN_SYMBOL_SAMPLES : FSK_NORMAL_SYMBOL_SAMPLES;

  /* Mode selection happens before RX starts, so TIM2 and DMA can be sized as
     one complete 20 ms detector window per half-buffer. */
  tim2_period = TIM2_COUNTER_CLOCK_HZ / fsk_sample_rate_hz;
  htim2.Init.Period = tim2_period - 1U;
  __HAL_TIM_SET_AUTORELOAD(&htim2, htim2.Init.Period);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  htim2.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    fsk_tx_freqs_hz[i] = profile_freqs[i];
    tx_fsk_amp[i] = profile_amps[i];
  }

  printf("FSK profile selected: %s, data=%u/%u/%u/%uHz sync=%uHz amp=%u/%u/%u/%u/%u ADC=%luHz/%u samples\r\n",
         (fsk_high_profile_enabled != 0U) ? "HIGH" : "NORMAL",
         fsk_tx_freqs_hz[0], fsk_tx_freqs_hz[1], fsk_tx_freqs_hz[2], fsk_tx_freqs_hz[3],
         fsk_tx_freqs_hz[FSK_SYNC_SYMBOL],
         tx_fsk_amp[0], tx_fsk_amp[1], tx_fsk_amp[2], tx_fsk_amp[3],
         tx_fsk_amp[FSK_SYNC_SYMBOL],
         (unsigned long)fsk_sample_rate_hz, fsk_symbol_samples);
}
#endif

static void FSK_UpdateGoertzelCoefficient(uint8_t index)
{
  if (index >= FSK_FREQ_COUNT)
  {
    return;
  }

  fsk_goertzel_coeffs[index] =
      2.0f * cosf((2.0f * FSK_PI * (float)fsk_rx_freqs_hz[index]) /
                  (float)fsk_sample_rate_hz);
}

static void Calibration_ResetProgress(void)
{
  calibration_energy_sum = 0.0f;
  calibration_energy_windows = 0U;
  calibration_lost_windows = 0U;
  calibration_stage = 0U;
  calibration_stable_windows = 0U;
  calibration_capture_active = 0U;
  calibration_complete = (communication_mode == COMM_MODE_MULTI_NODE) ? 1U : 0U;
  calibration_failed = 0U;
  calibration_progress_tick = HAL_GetTick();
  calibration_pass = 0U;
  calibration_coarse_activity = 0U;
  calibration_boundary_windows = 0U;
  calibration_coarse_last_signal_tick = 0U;
  calibration_adc_min_seen = 0xFFFFU;
  calibration_adc_max_seen = 0U;
  calibration_adc_mean_last = 2048U;
  rx_pga_agc_last_adjust_tick = 0U;
  rx_pga_agc_high_windows = 0U;
  rx_pga_agc_clip_windows = 0U;
  rx_pga_agc_low_windows = 0U;
  rx_pga_gain_code = (communication_mode == COMM_MODE_MULTI_NODE) ?
                     RX_PGA_MULTI_GAIN_CODE : RX_PGA_DEFAULT_GAIN_CODE;
  PGA_SetGain(rx_pga_gain_code);
  memset(calibration_energy, 0, sizeof(calibration_energy));
  MessageRx_ResetPreambleCapture();

  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    fsk_rx_freqs_hz[i] = fsk_tx_freqs_hz[i];
    fsk_energy_scale[i] = 1.0f;
    if (i < FSK_DATA_FREQ_COUNT)
    {
      calibration_energy_scale[i] = 1.0f;
    }
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
    FSK_UpdateGoertzelCoefficient(i);
  }
}

static void Calibration_StartVerificationPass(uint32_t now)
{
  calibration_pass = 1U;
  calibration_energy_sum = 0.0f;
  calibration_energy_windows = 0U;
  calibration_lost_windows = 0U;
  calibration_stage = 0U;
  calibration_stable_windows = 0U;
  calibration_capture_active = 0U;
  calibration_progress_tick = now;
  calibration_coarse_activity = 0U;
  calibration_boundary_windows = 0U;
  calibration_coarse_last_signal_tick = 0U;
  calibration_adc_min_seen = 0xFFFFU;
  calibration_adc_max_seen = 0U;
  calibration_adc_mean_last = 2048U;
  rx_pga_agc_high_windows = 0U;
  rx_pga_agc_clip_windows = 0U;
  rx_pga_agc_low_windows = 0U;
  memset(calibration_energy, 0, sizeof(calibration_energy));
  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
  }
  RX_ResetDetector();
  printf("calibration pass 1/2 AGC up/down complete: pass 2/2 verify with automatic down-only, PGA=x%u\r\n",
         rx_pga_gain_values[rx_pga_gain_code]);
  OLED_PrintRxStatus();
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

  if ((calibration_adc_min_seen <= RX_PGA_ADC_CLIP_MARGIN) ||
      (calibration_adc_max_seen >= (4095U - RX_PGA_ADC_CLIP_MARGIN)))
  {
    calibration_capture_active = 0U;
    calibration_complete = 0U;
    calibration_failed = 1U;
    printf("calibration failed: pass 2 ADC CLIP at PGA x%u min=%u max=%u; retry farther away or reduce analog gain\r\n",
           rx_pga_gain_values[rx_pga_gain_code],
           calibration_adc_min_seen, calibration_adc_max_seen);
    OLED_PrintRxStatus();
    return;
  }

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
    calibration_energy_scale[i] = scale;
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
    FSK_UpdateGoertzelCoefficient(i);
  }

  fsk_rx_freqs_hz[FSK_DATA_FREQ_COUNT] = fsk_tx_freqs_hz[FSK_DATA_FREQ_COUNT];
  fsk_energy_scale[FSK_DATA_FREQ_COUNT] = 1.0f;
  fsk_noise_floor[FSK_DATA_FREQ_COUNT] = FSK_NOISE_MIN_FLOOR;
  FSK_UpdateGoertzelCoefficient(FSK_DATA_FREQ_COUNT);
  calibration_failed = 0U;
  calibration_complete = 1U;
  RX_ResetDetector();

  printf("calibration complete: pass 2/2 verified, MCP6S21 x%u, fixed-frequency Goertzel reference=min_mean\r\n",
         rx_pga_gain_values[rx_pga_gain_code]);
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
  uint8_t calibration_tone_qualified;

  (void)samples;

  if ((communication_mode != COMM_MODE_STANDARD) &&
      (communication_mode != COMM_MODE_HIDDEN))
  {
    return;
  }

  if ((calibration_complete != 0U) || (calibration_failed != 0U))
  {
    return;
  }

  calibration_tone_qualified = PGA_IsCalibrationToneQualified(result);
  if (PGA_AutoGainControl(result, now) != 0U)
  {
    return;
  }
  if ((calibration_tone_qualified != 0U) &&
      (result->adc_min > RX_PGA_ADC_CLIP_MARGIN) &&
      (result->adc_max < (4095U - RX_PGA_ADC_CLIP_MARGIN)))
  {
    calibration_adc_mean_last = (uint16_t)result->adc_mean;
    if (result->adc_min < calibration_adc_min_seen)
    {
      calibration_adc_min_seen = result->adc_min;
    }
    if (result->adc_max > calibration_adc_max_seen)
    {
      calibration_adc_max_seen = result->adc_max;
    }
  }

  if (calibration_pass == 0U)
  {
    if ((result->valid != 0U) && (result->bits == FSK_SYNC_SYMBOL))
    {
      if (calibration_boundary_windows < CALIBRATION_BOUNDARY_STABLE_WINDOWS)
      {
        calibration_boundary_windows++;
      }
      if (calibration_boundary_windows >= CALIBRATION_BOUNDARY_STABLE_WINDOWS)
      {
        printf("calibration interpass boundary: 2000Hz SYNC accepted after %u windows\r\n",
               CALIBRATION_BOUNDARY_STABLE_WINDOWS);
        Calibration_StartVerificationPass(now);
      }
      return;
    }

    calibration_boundary_windows = 0U;
    if ((result->valid != 0U) && (result->bits < FSK_DATA_FREQ_COUNT))
    {
      calibration_coarse_activity = 1U;
      calibration_coarse_last_signal_tick = now;
    }
    else if ((calibration_coarse_activity != 0U) &&
             ((now - calibration_coarse_last_signal_tick) >= CALIBRATION_GAP_DETECT_MS))
    {
      Calibration_StartVerificationPass(now);
    }
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
  FSK_DetectResult result = FSK_DetectSymbol(samples, fsk_symbol_samples);
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
    printf("idle: PGA=x%u, Emax=%u, E2=%u, ADC mean=%u min=%u max=%u\r\n",
           rx_pga_gain_values[rx_pga_gain_code],
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

static void MessageRx_RestoreCalibrationEnergyScale(void)
{
  if (calibration_complete == 0U)
  {
    return;
  }

  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    fsk_energy_scale[i] = calibration_energy_scale[i];
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
  }
  fsk_energy_scale[FSK_DATA_FREQ_COUNT] = 1.0f;
  fsk_noise_floor[FSK_DATA_FREQ_COUNT] = FSK_NOISE_MIN_FLOOR;
}

static void MessageRx_ApplyPreambleFrameScale(void)
{
  float frame_energy[FSK_DATA_FREQ_COUNT];
  float correction[FSK_DATA_FREQ_COUNT];
  float applied_scale[FSK_DATA_FREQ_COUNT];
  uint8_t energy_windows_used[FSK_DATA_FREQ_COUNT];
  uint8_t single_window_fallbacks = 0U;
  float pair_01;
  float pair_23;
  float reference_energy;
  float max_applied_scale = 0.0f;

  MessageRx_RestoreCalibrationEnergyScale();
  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    if (msg_preamble_raw_count[i] == 0U)
    {
      printf("preamble frame scale skipped: %s has no hard window\r\n",
             FSK_BitsToString(i));
      return;
    }

    /* Capture stores raw (unscaled) Goertzel energy.  Prefer the mean of the
       two strongest windows.  A 60+10 ms PREAMBLE can occasionally provide
       only one hard-valid 20 ms window for 00; using that already-qualified
       window is safer than discarding all four frame measurements and falling
       back to a highly unequal startup calibration.  The existing energy,
       square-root correction and ratio guards still bound this fallback. */
    if (msg_preamble_raw_count[i] >= 2U)
    {
      frame_energy[i] =
          0.5f * (msg_preamble_raw_top1[i] + msg_preamble_raw_top2[i]) *
          calibration_energy_scale[i];
      energy_windows_used[i] = 2U;
    }
    else
    {
      frame_energy[i] = msg_preamble_raw_top1[i] * calibration_energy_scale[i];
      energy_windows_used[i] = 1U;
      single_window_fallbacks++;
    }
    if (frame_energy[i] < PREAMBLE_RECOVERY_ENERGY_MIN)
    {
      printf("preamble frame scale skipped: weak %s E/1e6=%u captured=%u used=%u\r\n",
             FSK_BitsToString(i),
             (unsigned int)(frame_energy[i] / 1000000.0f),
             (unsigned int)msg_preamble_raw_count[i],
             (unsigned int)energy_windows_used[i]);
      return;
    }
  }

  /* Correct only half of the measured imbalance (square root), then clamp
     each bin relative to the startup calibration.  The top-two mean avoids
     making the decision from one 20 ms window that may land on tone attack or
     inter-symbol ringing. */
  pair_01 = sqrtf(frame_energy[0] * frame_energy[1]);
  pair_23 = sqrtf(frame_energy[2] * frame_energy[3]);
  reference_energy = sqrtf(pair_01 * pair_23);
  if (reference_energy <= 0.0f)
  {
    printf("preamble frame scale skipped: invalid reference\r\n");
    return;
  }

  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    correction[i] = sqrtf(reference_energy / frame_energy[i]);
    if (correction[i] < PREAMBLE_FRAME_SCALE_MIN_CORR)
    {
      correction[i] = PREAMBLE_FRAME_SCALE_MIN_CORR;
    }
    else if (correction[i] > PREAMBLE_FRAME_SCALE_MAX_CORR)
    {
      correction[i] = PREAMBLE_FRAME_SCALE_MAX_CORR;
    }

    applied_scale[i] = calibration_energy_scale[i] * correction[i];
  }

  /* This acoustic path can make a weak 4500 Hz PREAMBLE sample boost the
     4500 Hz bin enough to absorb following 3500 Hz symbols.  Successful
     captures stayed below about 1.8x while failed captures were 4x..13x, so
     keep the frame-local 4500/3500 ratio inside a conservative 2.5x guard. */
  if (applied_scale[3] > (applied_scale[2] * PREAMBLE_FRAME_MAX_4500_TO_3500))
  {
    uint32_t ratio_x1000 =
        (uint32_t)((applied_scale[3] * 1000.0f) / applied_scale[2]);

    applied_scale[3] = applied_scale[2] * PREAMBLE_FRAME_MAX_4500_TO_3500;
    printf("preamble 4500/3500 guard: ratio=%lu.%03lu capped=2.500\r\n",
           (unsigned long)(ratio_x1000 / 1000U),
           (unsigned long)(ratio_x1000 % 1000U));
  }

  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    if (applied_scale[i] > max_applied_scale)
    {
      max_applied_scale = applied_scale[i];
    }
  }
  if (max_applied_scale <= 0.0f)
  {
    printf("preamble frame scale skipped: invalid applied scale\r\n");
    return;
  }

  printf("preamble frame scale: top2 mean + qualified top1 fallback, bounded sqrt, current frame only; top1=%u\r\n",
         (unsigned int)single_window_fallbacks);
  for (uint8_t i = 0U; i < FSK_DATA_FREQ_COUNT; i++)
  {
    uint32_t base_x1000 = (uint32_t)(calibration_energy_scale[i] * 1000.0f);
    uint32_t correction_x1000 = (uint32_t)(correction[i] * 1000.0f);
    uint32_t frame_x1000;

    applied_scale[i] /= max_applied_scale;
    if (applied_scale[i] < CALIBRATION_SCALE_MIN)
    {
      applied_scale[i] = CALIBRATION_SCALE_MIN;
    }
    fsk_energy_scale[i] = applied_scale[i];
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
    frame_x1000 = (uint32_t)(applied_scale[i] * 1000.0f);
    printf("frame %s: E/1e6=%u captured=%u used=%u base=%lu.%03lu corr=%lu.%03lu scale=%lu.%03lu\r\n",
           FSK_BitsToString(i),
           (unsigned int)(frame_energy[i] / 1000000.0f),
           (unsigned int)msg_preamble_raw_count[i],
           (unsigned int)energy_windows_used[i],
           (unsigned long)(base_x1000 / 1000U),
           (unsigned long)(base_x1000 % 1000U),
           (unsigned long)(correction_x1000 / 1000U),
           (unsigned long)(correction_x1000 % 1000U),
           (unsigned long)(frame_x1000 / 1000U),
           (unsigned long)(frame_x1000 % 1000U));
  }
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
      (delta <= PREAMBLE_RECOVERY_START0_END_MS))
  {
    for (uint8_t bits = 0U; bits < 3U; bits++)
    {
      if (result->data_energy[bits] > msg_preamble_recovery_start_energy[0])
      {
        msg_preamble_recovery_start_energy[0] = result->data_energy[bits];
        msg_preamble_recovery_start_bits[0] = bits;
      }
    }
  }
  if ((delta >= PREAMBLE_RECOVERY_START1_START_MS) &&
      (delta <= PREAMBLE_RECOVERY_START1_END_MS))
  {
    for (uint8_t bits = 0U; bits < FSK_DATA_FREQ_COUNT; bits++)
    {
      if (result->data_energy[bits] > msg_preamble_recovery_start_energy[1])
      {
        msg_preamble_recovery_start_energy[1] = result->data_energy[bits];
        msg_preamble_recovery_start_bits[1] = bits;
      }
    }
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

  msg_rx_source_id = Node_IdFromSourceBits(msg_preamble_recovery_start_bits[0]);
  msg_rx_destination_id = msg_preamble_recovery_start_bits[1];
  if (msg_rx_source_id == 0U)
  {
    printf("timed preamble recovery rejected: invalid source bits=%s\r\n",
           FSK_BitsToString(msg_preamble_recovery_start_bits[0]));
    msg_preamble_recovery_active = 0U;
    return 0U;
  }

  memcpy(msg_preamble_energy, msg_preamble_recovery_energy, sizeof(msg_preamble_energy));
  if (msg_rx_destination_id == 0U)
  {
    printf("rx timed preamble recovery: N%u->ALL, marker delta=%lums\r\n",
           msg_rx_source_id, (unsigned long)delta);
  }
  else
  {
    printf("rx timed preamble recovery: N%u->N%u, marker delta=%lums\r\n",
           msg_rx_source_id, msg_rx_destination_id, (unsigned long)delta);
  }
  msg_preamble_capture_active = 0U;
  MessageRx_ApplyPreambleFrameScale();
  msg_preamble_recovery_active = 0U;
  msg_preamble_index = 0U;
  msg_rs_symbol_index = 0U;
  msg_rs_fsk_index = 0U;
  msg_rs_symbol = 0U;
  msg_rs_current_erased = 0U;
  msg_rs_current_reliability = RS_VOTE_RATIO_CAP;
  msg_rs_erasure_count = 0U;
  msg_soft_fallback_count = 0U;
  msg_low_confidence_count = 0U;
  msg_vote_override_count = 0U;
  msg_worst_decision_count = 0U;
  memset(msg_worst_decisions, 0, sizeof(msg_worst_decisions));
  msg_marker_tick = now;
  MessageRx_ClearCapture();
  memset(msg_rs_codeword, 0, sizeof(msg_rs_codeword));
  memset(msg_rs_codeword_erased, 0, sizeof(msg_rs_codeword_erased));
  memset(msg_rs_codeword_reliability, 0, sizeof(msg_rs_codeword_reliability));
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
  if (vote_ratio > msg_capture_best_ratio[slot][result->bits])
  {
    msg_capture_best_ratio[slot][result->bits] = vote_ratio;
    memcpy(msg_capture_best_energy[slot][result->bits], result->data_energy,
           sizeof(msg_capture_best_energy[slot][result->bits]));
  }
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
    msg_soft_ratio[slot] = (second_energy > 1.0f) ?
                           (candidate_energy / second_energy) : RS_VOTE_RATIO_CAP;
    if (msg_soft_ratio[slot] > RS_VOTE_RATIO_CAP)
    {
      msg_soft_ratio[slot] = RS_VOTE_RATIO_CAP;
    }
    memcpy(msg_soft_energy[slot], result->data_energy,
           sizeof(msg_soft_energy[slot]));
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

static void MessageRx_AppendRsBits(uint8_t decoded_bits, uint8_t erased,
                                   float confidence)
{
  if (msg_rs_fsk_index == 0U)
  {
    msg_rs_current_reliability = RS_VOTE_RATIO_CAP;
  }
  if (erased != 0U)
  {
    msg_rs_current_erased = 1U;
    confidence = 0.0f;
  }
  if (confidence < msg_rs_current_reliability)
  {
    msg_rs_current_reliability = confidence;
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
    uint16_t codeword_index = (uint16_t)(block * RS_CODEWORD_SYMBOLS) + block_index;

    msg_rs_codeword[codeword_index] = msg_rs_symbol;
    msg_rs_codeword_erased[codeword_index] = msg_rs_current_erased;
    msg_rs_codeword_reliability[codeword_index] = msg_rs_current_reliability;
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
  msg_rs_current_reliability = RS_VOTE_RATIO_CAP;
}

static void MessageRx_RecordDecision(uint16_t fsk_index, uint8_t source,
                                     uint8_t selected_bits, float confidence,
                                     const float *energy)
{
  uint8_t target;

  if ((source != RX_DECISION_ERASURE) &&
      (confidence < FSK_RATIO_THRESHOLD))
  {
    msg_low_confidence_count++;
  }

  if (msg_worst_decision_count < RX_DIAG_WORST_COUNT)
  {
    target = msg_worst_decision_count;
    msg_worst_decision_count++;
  }
  else
  {
    target = 0U;
    for (uint8_t i = 1U; i < RX_DIAG_WORST_COUNT; i++)
    {
      if (msg_worst_decisions[i].confidence >
          msg_worst_decisions[target].confidence)
      {
        target = i;
      }
    }
    if (confidence >= msg_worst_decisions[target].confidence)
    {
      return;
    }
  }

  msg_worst_decisions[target].fsk_index = fsk_index;
  msg_worst_decisions[target].source = source;
  msg_worst_decisions[target].selected_bits = selected_bits;
  msg_worst_decisions[target].confidence = confidence;
  if (energy != NULL)
  {
    memcpy(msg_worst_decisions[target].energy, energy,
           sizeof(msg_worst_decisions[target].energy));
  }
  else
  {
    memset(msg_worst_decisions[target].energy, 0,
           sizeof(msg_worst_decisions[target].energy));
  }
}

static void MessageRx_PrintDecisionDiagnostics(void)
{
  uint8_t printed[RX_DIAG_WORST_COUNT] = {0};

  printf("rx weak decisions: lowest %u of %u data slots\r\n",
         msg_worst_decision_count,
         RS_TOTAL_CODEWORD_SYMBOLS * RS_FSK_SYMBOLS_PER_SYMBOL);
  for (uint8_t rank = 0U; rank < msg_worst_decision_count; rank++)
  {
    uint8_t selected = 0xFFU;
    const RxDecisionDiagnostic *diag;
    const char *source_name;
    uint16_t wire_index;
    uint8_t decoded_bits;

    for (uint8_t i = 0U; i < msg_worst_decision_count; i++)
    {
      if ((printed[i] == 0U) &&
          ((selected == 0xFFU) ||
           (msg_worst_decisions[i].confidence <
            msg_worst_decisions[selected].confidence)))
      {
        selected = i;
      }
    }
    if (selected == 0xFFU)
    {
      break;
    }
    printed[selected] = 1U;
    diag = &msg_worst_decisions[selected];
    wire_index = (uint16_t)(diag->fsk_index / RS_FSK_SYMBOLS_PER_SYMBOL);
    decoded_bits = (uint8_t)((diag->selected_bits ^
                              Message_WhiteningMask(diag->fsk_index)) & 0x03U);

    switch (diag->source)
    {
      case RX_DECISION_VOTE: source_name = "vote"; break;
      case RX_DECISION_NEAREST: source_name = "nearest"; break;
      case RX_DECISION_SOFT: source_name = "soft"; break;
      default: source_name = "erasure"; break;
    }

    if (diag->source == RX_DECISION_ERASURE)
    {
      printf("rx weak B%u S%u slot%u src=%s ratio=0\r\n",
             (wire_index % RS_BLOCK_COUNT) + 1U,
             (wire_index / RS_BLOCK_COUNT) + 1U,
             (diag->fsk_index % RS_FSK_SYMBOLS_PER_SYMBOL) + 1U,
             source_name);
    }
    else
    {
      printf("rx weak B%u S%u slot%u src=%s tone=%s data=%s ratio_x100=%u E/1e6=%u/%u/%u/%u\r\n",
             (wire_index % RS_BLOCK_COUNT) + 1U,
             (wire_index / RS_BLOCK_COUNT) + 1U,
             (diag->fsk_index % RS_FSK_SYMBOLS_PER_SYMBOL) + 1U,
             source_name, FSK_BitsToString(diag->selected_bits),
             FSK_BitsToString(decoded_bits),
             (unsigned int)(diag->confidence * 100.0f),
             (unsigned int)(diag->energy[0] / 1000000.0f),
             (unsigned int)(diag->energy[1] / 1000000.0f),
             (unsigned int)(diag->energy[2] / 1000000.0f),
             (unsigned int)(diag->energy[3] / 1000000.0f));
    }
  }
}

static void MessageRx_ClearCapture(void)
{
  memset(msg_capture_bits, 0, sizeof(msg_capture_bits));
  memset(msg_capture_valid, 0, sizeof(msg_capture_valid));
  memset(msg_capture_distance, 0xFF, sizeof(msg_capture_distance));
  memset(msg_capture_scores, 0, sizeof(msg_capture_scores));
  memset(msg_capture_best_ratio, 0, sizeof(msg_capture_best_ratio));
  memset(msg_capture_best_energy, 0, sizeof(msg_capture_best_energy));
  memset(msg_soft_bits, 0, sizeof(msg_soft_bits));
  memset(msg_soft_valid, 0, sizeof(msg_soft_valid));
  memset(msg_soft_score, 0, sizeof(msg_soft_score));
  memset(msg_soft_ratio, 0, sizeof(msg_soft_ratio));
  memset(msg_soft_energy, 0, sizeof(msg_soft_energy));
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
      float confidence = (second_score > 0.0f) ?
                         (voted_score / second_score) : RS_VOTE_RATIO_CAP;

      if (confidence > RS_VOTE_RATIO_CAP)
      {
        confidence = RS_VOTE_RATIO_CAP;
      }
      if ((msg_capture_valid[slot] == 0U) || (voted_bits != msg_capture_bits[slot]))
      {
        msg_vote_override_count++;
      }
      uint8_t decoded_bits = (uint8_t)((voted_bits ^
                                        Message_WhiteningMask(fsk_index)) & 0x03U);
      MessageRx_RecordDecision(fsk_index, RX_DECISION_VOTE, voted_bits,
                               confidence,
                               msg_capture_best_energy[slot][voted_bits]);
      MessageRx_AppendRsBits(decoded_bits, 0U, confidence);
    }
    else if (msg_capture_valid[slot] != 0U)
    {
      uint8_t decoded_bits = (uint8_t)((msg_capture_bits[slot] ^
                                        Message_WhiteningMask(fsk_index)) & 0x03U);
      MessageRx_RecordDecision(fsk_index, RX_DECISION_NEAREST,
                               msg_capture_bits[slot],
                               msg_capture_best_ratio[slot][msg_capture_bits[slot]],
                               msg_capture_best_energy[slot][msg_capture_bits[slot]]);
      MessageRx_AppendRsBits(decoded_bits, 0U,
                             msg_capture_best_ratio[slot][msg_capture_bits[slot]]);
    }
    else if (msg_soft_valid[slot] != 0U)
    {
      uint8_t decoded_bits = (uint8_t)((msg_soft_bits[slot] ^
                                        Message_WhiteningMask(fsk_index)) & 0x03U);
      msg_soft_fallback_count++;
      MessageRx_RecordDecision(fsk_index, RX_DECISION_SOFT,
                               msg_soft_bits[slot], msg_soft_ratio[slot],
                               msg_soft_energy[slot]);
      MessageRx_AppendRsBits(decoded_bits, 0U, msg_soft_ratio[slot]);
    }
    else
    {
      MessageRx_RecordDecision(fsk_index, RX_DECISION_ERASURE, 0U, 0.0f, NULL);
      MessageRx_AppendRsBits(0U, 1U, 0.0f);
    }
  }

  MessageRx_ClearCapture();
  if (msg_rs_symbol_index >= RS_TOTAL_CODEWORD_SYMBOLS)
  {
    printf("rx RS codeword complete, slot erasures=%u, soft_fallbacks=%u, low_confidence=%u, vote_overrides=%u\r\n",
           msg_rs_erasure_count, msg_soft_fallback_count,
           msg_low_confidence_count, msg_vote_override_count);
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
          memset(msg_preamble_recovery_start_bits, 0,
                 sizeof(msg_preamble_recovery_start_bits));
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
          /* Multi-node mode has no startup calibration.  Apply the four-tone
             PREAMBLE normalization before decoding source/destination, or a
             weak address frequency can be missed and RS data can be consumed
             as the address.  Keep this scale for the remainder of the frame. */
          msg_preamble_capture_active = 0U;
          MessageRx_ApplyPreambleFrameScale();
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
      if (bits < 3U)
      {
        msg_rx_source_id = Node_IdFromSourceBits(bits);
        msg_preamble_capture_active = 0U;
        msg_rx_state = MSG_RX_START_1;
        printf("rx address source: bits=%s -> N%u\r\n",
               FSK_BitsToString(bits), msg_rx_source_id);
      }
      else if ((bits == msg_preamble[MSG_PREAMBLE_LEN - 1U]) ||
               (bits == FSK_SYNC_SYMBOL))
      {
        /* A strong 4500 Hz PREAMBLE tail can survive into the first START
           window.  Source IDs deliberately use only 00/01/10, so 11 remains
           available as an unambiguous transition guard. */
        printf("rx start0 transition ignored: got=%s\r\n",
               FSK_BitsToString(bits));
      }
      else
      {
        printf("rx source address fail: got=%s\r\n", FSK_BitsToString(bits));
        MessageRx_ResetFrame();
      }
      break;

    case MSG_RX_START_1:
      if (bits < 4U)
      {
        msg_rx_destination_id = bits;
        msg_rs_symbol_index = 0U;
        msg_rs_fsk_index = 0U;
        msg_rs_symbol = 0U;
        msg_rs_current_erased = 0U;
        msg_rs_current_reliability = RS_VOTE_RATIO_CAP;
        msg_rs_erasure_count = 0U;
        msg_soft_fallback_count = 0U;
        msg_low_confidence_count = 0U;
        msg_vote_override_count = 0U;
        msg_worst_decision_count = 0U;
        memset(msg_worst_decisions, 0, sizeof(msg_worst_decisions));
        msg_marker_tick = 0U;
        msg_start_entry_tick = 0U;
        MessageRx_ClearCapture();
        memset(msg_rs_codeword, 0, sizeof(msg_rs_codeword));
        memset(msg_rs_codeword_erased, 0, sizeof(msg_rs_codeword_erased));
        memset(msg_rs_codeword_reliability, 0, sizeof(msg_rs_codeword_reliability));
        msg_rx_state = MSG_RX_RS_SYNC;
        if (msg_rx_destination_id == 0U)
        {
          printf("rx route: N%u->ALL, receiving %ux marker-framed RS(%u,%u) blocks\r\n",
                 msg_rx_source_id, RS_BLOCK_COUNT, RS_CODEWORD_SYMBOLS,
                 RS_DATA_SYMBOLS);
        }
        else
        {
          printf("rx route: N%u->N%u, receiving %ux marker-framed RS(%u,%u) blocks\r\n",
                 msg_rx_source_id, msg_rx_destination_id, RS_BLOCK_COUNT,
                 RS_CODEWORD_SYMBOLS, RS_DATA_SYMBOLS);
        }
      }
      else if (bits == FSK_SYNC_SYMBOL)
      {
        printf("rx destination transition marker ignored\r\n");
      }
      else
      {
        printf("rx destination address fail: got=%s\r\n", FSK_BitsToString(bits));
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
        uint8_t received_codewords[RS_TOTAL_CODEWORD_SYMBOLS];
        uint8_t candidate0[RS_CODEWORD_SYMBOLS];
        uint8_t candidate1[RS_CODEWORD_SYMBOLS];
        uint8_t selected_corrected[RS_BLOCK_COUNT] = {0};
        uint8_t selected_erasures[RS_BLOCK_COUNT] = {0};
        uint8_t selected_adaptive[RS_BLOCK_COUNT] = {0};
        uint8_t selected = 0U;
        uint8_t rs_pair_seen = 0U;
        uint8_t crc_failure_seen = 0U;
        uint8_t format_failure_seen = 0U;
        uint8_t failure_index = 0U;
        uint8_t failure_code = 0U;
        uint8_t saved_failure_index = 0U;
        uint8_t saved_failure_code = 0U;
        uint16_t received_crc = 0U;
        uint16_t calculated_crc = 0U;
        uint16_t saved_received_crc = 0U;
        uint16_t saved_calculated_crc = 0U;
        MessagePayloadStatus payload_status = MESSAGE_PAYLOAD_CRC_FAIL;
        MessagePayloadStatus saved_format_status = MESSAGE_PAYLOAD_OK;

        printf("rx end ok\r\n");
        memcpy(received_codewords, msg_rs_codeword,
               sizeof(received_codewords));

        /* Search in increasing total added erasures.  A syndrome-valid pair
           is accepted only if its CRC and payload structure both pass. */
        for (uint8_t total_add = 0U;
             (total_add <= (RS_PARITY_SYMBOLS * RS_BLOCK_COUNT)) &&
             (selected == 0U);
             total_add++)
        {
          for (uint8_t add0 = 0U;
               (add0 <= RS_PARITY_SYMBOLS) && (add0 <= total_add);
               add0++)
          {
            uint8_t add1 = (uint8_t)(total_add - add0);
            uint8_t corrected0 = 0U;
            uint8_t corrected1 = 0U;
            uint8_t erasures0 = 0U;
            uint8_t erasures1 = 0U;

            if (add1 > RS_PARITY_SYMBOLS)
            {
              continue;
            }
            if (RS_DecodeAdaptiveAttempt(&received_codewords[0],
                                         &msg_rs_codeword_erased[0],
                                         &msg_rs_codeword_reliability[0],
                                         add0, candidate0, &corrected0,
                                         &erasures0) < 0)
            {
              continue;
            }
            if (RS_DecodeAdaptiveAttempt(
                    &received_codewords[RS_CODEWORD_SYMBOLS],
                    &msg_rs_codeword_erased[RS_CODEWORD_SYMBOLS],
                    &msg_rs_codeword_reliability[RS_CODEWORD_SYMBOLS],
                    add1, candidate1, &corrected1, &erasures1) < 0)
            {
              continue;
            }

            rs_pair_seen = 1U;
            memcpy(&msg_rs_codeword[0], candidate0, RS_CODEWORD_SYMBOLS);
            memcpy(&msg_rs_codeword[RS_CODEWORD_SYMBOLS], candidate1,
                   RS_CODEWORD_SYMBOLS);
            payload_status = Message_ValidateRsPayload(
                msg_rs_codeword, msg_rx_source_id, msg_rx_destination_id,
                rx_message, &rx_message_len,
                &received_crc, &calculated_crc, &failure_index,
                &failure_code);
            if (payload_status == MESSAGE_PAYLOAD_OK)
            {
              selected_corrected[0] = corrected0;
              selected_corrected[1] = corrected1;
              selected_erasures[0] = erasures0;
              selected_erasures[1] = erasures1;
              selected_adaptive[0] = add0;
              selected_adaptive[1] = add1;
              selected = 1U;
              break;
            }
            if (payload_status == MESSAGE_PAYLOAD_CRC_FAIL)
            {
              crc_failure_seen = 1U;
              saved_received_crc = received_crc;
              saved_calculated_crc = calculated_crc;
            }
            else
            {
              format_failure_seen = 1U;
              saved_format_status = payload_status;
              saved_failure_index = failure_index;
              saved_failure_code = failure_code;
            }
          }
        }

        if (selected != 0U)
        {
          uint8_t total_corrected =
              (uint8_t)(selected_corrected[0] + selected_corrected[1]);
          uint8_t total_decode_erasures =
              (uint8_t)(selected_erasures[0] + selected_erasures[1]);
          uint8_t total_adaptive_erasures =
              (uint8_t)(selected_adaptive[0] + selected_adaptive[1]);

          for (uint8_t block = 0U; block < RS_BLOCK_COUNT; block++)
          {
            printf("rs block %u/%u ok: erasures=%u (GMD +%u) corrected_total=%u capacity=2e+s<=%u\r\n",
                   block + 1U, RS_BLOCK_COUNT, selected_erasures[block],
                   selected_adaptive[block], selected_corrected[block],
                   RS_PARITY_SYMBOLS);
          }
          printf("CRC12 ok: route=N%u->%s received=calculated=0x%03X\r\n",
                 msg_rx_source_id,
                 Node_DestinationLabel(msg_rx_destination_id), received_crc);
          printf("rs decode ok: corrected_locations=%u, slot_erasures=%u, decode_erasures=%u (GMD +%u), rule=2e+s<=%u/block\r\n",
                 total_corrected, msg_rs_erasure_count, total_decode_erasures,
                 total_adaptive_erasures, RS_PARITY_SYMBOLS);
          if (local_node_id == 0U)
          {
            printf("message ignored: local node ID unset; press RX key 1/2/3\r\n");
          }
          else if ((msg_rx_destination_id != 0U) &&
                   (msg_rx_destination_id != local_node_id))
          {
            printf("message ignored: addressed to N%u, local=N%u\r\n",
                   msg_rx_destination_id, local_node_id);
          }
          else
          {
            rx_message_valid = 1U;
            rx_message_source_id = msg_rx_source_id;
            rx_message_destination_id = msg_rx_destination_id;
            rx_rs_corrected = total_corrected;
            memcpy(message_store_pending_text, rx_message, rx_message_len + 1U);
            message_store_pending_len = rx_message_len;
            message_store_pending_source = msg_rx_source_id;
            message_store_pending_destination = msg_rx_destination_id;
            message_store_pending = 1U;
            message_store_last_save_failed = 0U;
            StatusLed_NotifyRxComplete();
            printf("message_ok: N%u->%s len=%u text=\"%s\"\r\n",
                   msg_rx_source_id,
                   Node_DestinationLabel(msg_rx_destination_id),
                   rx_message_len, rx_message);
          }
        }
        else if (rs_pair_seen == 0U)
        {
          printf("rs decode fail: no block pair satisfied 2e+s<=%u\r\n",
                 RS_PARITY_SYMBOLS);
          MessageRx_PrintDecisionDiagnostics();
        }
        else
        {
          if (crc_failure_seen != 0U)
          {
            printf("message CRC12 fail: no GMD candidate matched, last received=0x%03X calculated=0x%03X\r\n",
                   saved_received_crc, saved_calculated_crc);
          }
          if (format_failure_seen != 0U)
          {
            if (saved_format_status == MESSAGE_PAYLOAD_DATA_AFTER_PAD)
            {
              printf("rx payload format fail: data 0x%02X after padding at %u\r\n",
                     saved_failure_code, saved_failure_index);
            }
            else if (saved_format_status == MESSAGE_PAYLOAD_INVALID_CODE)
            {
              printf("rx payload format fail: invalid code %u at %u\r\n",
                     saved_failure_code, saved_failure_index);
            }
            else
            {
              printf("rx payload format fail: empty message\r\n");
            }
          }
          printf("message fail: all syndrome-valid GMD candidates rejected by CRC/payload\r\n");
          MessageRx_PrintDecisionDiagnostics();
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
  MessageRx_RestoreCalibrationEnergyScale();
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
  memset(msg_preamble_recovery_start_bits, 0,
         sizeof(msg_preamble_recovery_start_bits));
  msg_rx_source_id = 0U;
  msg_rx_destination_id = 0U;
  msg_rs_symbol_index = 0U;
  msg_rs_fsk_index = 0U;
  msg_rs_symbol = 0U;
  msg_rs_current_erased = 0U;
  msg_rs_current_reliability = RS_VOTE_RATIO_CAP;
  msg_rs_erasure_count = 0U;
  msg_soft_fallback_count = 0U;
  msg_low_confidence_count = 0U;
  msg_vote_override_count = 0U;
  msg_worst_decision_count = 0U;
  memset(msg_worst_decisions, 0, sizeof(msg_worst_decisions));
  msg_marker_tick = 0U;
  MessageRx_ClearCapture();
  memset(msg_rs_codeword, 0, sizeof(msg_rs_codeword));
  memset(msg_rs_codeword_erased, 0, sizeof(msg_rs_codeword_erased));
  memset(msg_rs_codeword_reliability, 0, sizeof(msg_rs_codeword_reliability));
  msg_frame_tick = 0U;
}

static uint8_t Message_WhiteningMask(uint16_t fsk_symbol_index)
{
  static const uint8_t mask_cycle[4] = {0x00U, 0x01U, 0x02U, 0x03U};
  return mask_cycle[fsk_symbol_index & 0x03U];
}

static uint8_t Node_SourceBitsFromId(uint8_t node_id)
{
  return ((node_id >= 1U) && (node_id <= 3U)) ?
         (uint8_t)(node_id - 1U) : 0x03U;
}

static uint8_t Node_IdFromSourceBits(uint8_t bits)
{
  return (bits < 3U) ? (uint8_t)(bits + 1U) : 0U;
}

static const char *Node_DestinationLabel(uint8_t destination)
{
  static const char *labels[4U] = {"ALL", "N1", "N2", "N3"};
  return (destination < 4U) ? labels[destination] : "?";
}

static void Node_SetLocalId(uint8_t node_id)
{
  if ((node_id < 1U) || (node_id > 3U))
  {
    return;
  }
  local_node_id = node_id;
  if (tx_destination_id == local_node_id)
  {
    tx_destination_id = 0U;
  }
  printf("node configured: local=N%u, destination=%s; RX keys 1/2/3=set local, D=cycle destination\r\n",
         local_node_id, Node_DestinationLabel(tx_destination_id));
  OLED_PrintRxStatus();
}

static void Node_CycleDestination(void)
{
  do
  {
    tx_destination_id = (uint8_t)((tx_destination_id + 1U) & 0x03U);
  }
  while ((local_node_id != 0U) && (tx_destination_id == local_node_id));

  if (tx_destination_id == 0U)
  {
    printf("destination: ALL (broadcast)\r\n");
  }
  else
  {
    printf("destination: N%u\r\n", tx_destination_id);
  }
  OLED_PrintRxStatus();
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

/* CRC-12/3GPP polynomial x^12+x^11+x^3+x^2+x+1 (0x80F),
   initialized to zero and fed MSB-first over each 6-bit payload symbol. */
static uint16_t Message_Crc12Symbols(const uint8_t *symbols, uint8_t count)
{
  uint16_t crc = 0U;

  for (uint8_t i = 0U; i < count; i++)
  {
    uint8_t symbol = (uint8_t)(symbols[i] & RS_FIELD_ORDER);

    for (uint8_t mask = 0x20U; mask != 0U; mask >>= 1U)
    {
      uint8_t feedback = (uint8_t)(((crc >> 11U) ^
                                    ((symbol & mask) != 0U ? 1U : 0U)) & 1U);
      crc = (uint16_t)((crc << 1U) & MESSAGE_CRC12_MASK);
      if (feedback != 0U)
      {
        crc ^= MESSAGE_CRC12_POLY;
      }
    }
  }
  return (uint16_t)(crc & MESSAGE_CRC12_MASK);
}

static uint16_t Message_Crc12Addressed(uint8_t source, uint8_t destination,
                                      const uint8_t *symbols, uint8_t count)
{
  uint8_t addressed_symbols[MESSAGE_MAX_LEN + 2U];

  addressed_symbols[0] = (uint8_t)(source & 0x03U);
  addressed_symbols[1] = (uint8_t)(destination & 0x03U);
  memcpy(&addressed_symbols[2], symbols, count);
  return Message_Crc12Symbols(addressed_symbols, (uint8_t)(count + 2U));
}

static uint8_t Message_CrcSelfTest(void)
{
  uint8_t symbols[MESSAGE_MAX_LEN];

  for (uint8_t i = 0U; i < MESSAGE_MAX_LEN; i++)
  {
    symbols[i] = i;
  }
  return ((Message_Crc12Symbols(symbols, MESSAGE_MAX_LEN) == 0x0708U) &&
          (Message_Crc12Addressed(1U, 2U, symbols, MESSAGE_MAX_LEN) == 0x0780U)) ?
         1U : 0U;
}

static MessagePayloadStatus Message_ValidateRsPayload(const uint8_t *codewords,
                                                       uint8_t source,
                                                       uint8_t destination,
                                                       char *text,
                                                       uint8_t *text_len,
                                                       uint16_t *received_crc,
                                                       uint16_t *calculated_crc,
                                                       uint8_t *failure_index,
                                                       uint8_t *failure_code)
{
  uint8_t data[RS_TOTAL_DATA_SYMBOLS];
  uint8_t pad_seen = 0U;

  *text_len = 0U;
  *failure_index = 0U;
  *failure_code = 0U;
  for (uint8_t i = 0U; i < RS_TOTAL_DATA_SYMBOLS; i++)
  {
    uint8_t block = (uint8_t)(i / RS_DATA_SYMBOLS);
    uint8_t block_index = (uint8_t)(i % RS_DATA_SYMBOLS);
    data[i] = codewords[(block * RS_CODEWORD_SYMBOLS) + block_index];
  }

  *calculated_crc = Message_Crc12Addressed(source, destination,
                                           data, MESSAGE_MAX_LEN);
  *received_crc = (uint16_t)(((uint16_t)data[MESSAGE_MAX_LEN] << 6U) |
                             data[MESSAGE_MAX_LEN + 1U]);
  if (*received_crc != *calculated_crc)
  {
    text[0] = '\0';
    return MESSAGE_PAYLOAD_CRC_FAIL;
  }

  for (uint8_t i = 0U; i < MESSAGE_MAX_LEN; i++)
  {
    uint8_t code = data[i];

    if (code == RS_PAD_VALUE)
    {
      pad_seen = 1U;
      continue;
    }
    if (pad_seen != 0U)
    {
      *failure_index = i;
      *failure_code = code;
      text[0] = '\0';
      return MESSAGE_PAYLOAD_DATA_AFTER_PAD;
    }

    {
      char ch = Message_CodeToChar(code);
      if (ch == 0)
      {
        *failure_index = i;
        *failure_code = code;
        text[0] = '\0';
        return MESSAGE_PAYLOAD_INVALID_CODE;
      }
      text[(*text_len)++] = ch;
    }
  }
  text[*text_len] = '\0';
  if (*text_len == 0U)
  {
    return MESSAGE_PAYLOAD_EMPTY;
  }
  return MESSAGE_PAYLOAD_OK;
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

static uint8_t MessageStore_RecordLength(const MessageStoreRecord *record)
{
  return (uint8_t)(record->length & MESSAGE_STORE_LENGTH_MASK);
}

static uint8_t MessageStore_RecordSource(const MessageStoreRecord *record)
{
  return (uint8_t)((record->length >> MESSAGE_STORE_SOURCE_SHIFT) & 0x03U);
}

static uint8_t MessageStore_RecordDestination(const MessageStoreRecord *record)
{
  return (uint8_t)((record->length >> MESSAGE_STORE_DEST_SHIFT) & 0x03U);
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
  uint8_t len = MessageStore_RecordLength(record);
  uint8_t source = MessageStore_RecordSource(record);

  if ((record->magic != MESSAGE_STORE_MAGIC) ||
      (record->commit != MESSAGE_STORE_COMMIT) ||
      (len == 0U) || (len > MESSAGE_MAX_LEN) ||
      (source > 3U) ||
      ((record->length & ~(MESSAGE_STORE_LENGTH_MASK |
                           MESSAGE_STORE_ROUTE_MASK)) != 0U) ||
      (record->hash != MessageStore_HashRecord(record)))
  {
    return 0U;
  }

  for (uint8_t i = 0U; i < len; i++)
  {
    if (Message_CharToCode((char)record->text[i]) == MESSAGE_CODE_INVALID)
    {
      return 0U;
    }
  }
  return 1U;
}

static void MessageStore_CacheAppend(uint32_t sequence, const char *text, uint8_t len,
                                     uint8_t source, uint8_t destination)
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
  message_store_cache[index].source = source;
  message_store_cache[index].destination = destination;
  memcpy(message_store_cache[index].text, text, len);
  message_store_cache[index].text[len] = '\0';
  message_store_view_index = message_store_count - 1U;
}

static void MessageStore_BuildRecord(MessageStoreRecord *record, uint32_t sequence,
                                     const char *text, uint8_t len,
                                     uint8_t source, uint8_t destination)
{
  memset(record, 0, sizeof(*record));
  record->magic = MESSAGE_STORE_MAGIC;
  record->sequence = sequence;
  record->length = (uint32_t)len |
                   ((uint32_t)(source & 0x03U) << MESSAGE_STORE_SOURCE_SHIFT) |
                   ((uint32_t)(destination & 0x03U) << MESSAGE_STORE_DEST_SHIFT);
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
                             message_store_cache[i].len,
                             message_store_cache[i].source,
                             message_store_cache[i].destination);
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
                               MessageStore_RecordLength(&record),
                               MessageStore_RecordSource(&record),
                               MessageStore_RecordDestination(&record));
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

static uint8_t MessageStore_Save(const char *text, uint8_t len,
                                 uint8_t source, uint8_t destination)
{
  MessageStoreRecord record;
  uint32_t sequence;
  uint8_t saved;

  if ((len == 0U) || (len > MESSAGE_MAX_LEN) ||
      (source < 1U) || (source > 3U) || (destination > 3U))
  {
    return 0U;
  }

  sequence = message_store_next_sequence;
  MessageStore_BuildRecord(&record, sequence, text, len, source, destination);

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
  MessageStore_CacheAppend(sequence, text, len, source, destination);
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

  saved = MessageStore_Save(message_store_pending_text, message_store_pending_len,
                            message_store_pending_source,
                            message_store_pending_destination);
  message_store_pending = 0U;
  message_store_pending_len = 0U;
  message_store_pending_source = 0U;
  message_store_pending_destination = 0U;

  message_store_last_save_failed = (saved != 0U) ? 0U : 1U;
  if (saved != 0U)
  {
    printf("message stored: item=%u/%u seq=%lu route=N%u->%s len=%u\r\n",
           message_store_view_index + 1U,
           message_store_count,
           (unsigned long)message_store_cache[message_store_view_index].sequence,
           message_store_cache[message_store_view_index].source,
           Node_DestinationLabel(message_store_cache[message_store_view_index].destination),
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
  printf("memory view: item=%u/%u seq=%lu route=%s%u->%s len=%u text=\"%s\"\r\n",
         message_store_view_index + 1U,
         message_store_count,
         (unsigned long)message_store_cache[message_store_view_index].sequence,
         (message_store_cache[message_store_view_index].source == 0U) ? "OLD" : "N",
         message_store_cache[message_store_view_index].source,
         Node_DestinationLabel(message_store_cache[message_store_view_index].destination),
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

static uint8_t RS_FindErrorLocator(const uint8_t *syndromes, uint8_t syndrome_count,
                                   uint8_t *locator, uint8_t *locator_degree)
{
  uint8_t previous[RS_PARITY_SYMBOLS + 1U] = {0};
  uint8_t saved[RS_PARITY_SYMBOLS + 1U];
  uint8_t degree = 0U;
  uint8_t shift = 1U;
  uint8_t previous_discrepancy = 1U;

  memset(locator, 0, RS_PARITY_SYMBOLS + 1U);
  locator[0] = 1U;
  previous[0] = 1U;

  for (uint8_t step = 0U; step < syndrome_count; step++)
  {
    uint8_t discrepancy = syndromes[step];

    for (uint8_t i = 1U; i <= degree; i++)
    {
      discrepancy ^= RS_GfMul(locator[i], syndromes[step - i]);
    }
    if (discrepancy == 0U)
    {
      shift++;
      continue;
    }

    memcpy(saved, locator, RS_PARITY_SYMBOLS + 1U);
    {
      uint8_t scale = RS_GfDiv(discrepancy, previous_discrepancy);
      for (uint8_t i = 0U; (uint16_t)i + shift <= RS_PARITY_SYMBOLS; i++)
      {
        locator[i + shift] ^= RS_GfMul(scale, previous[i]);
      }
    }

    if ((uint8_t)(2U * degree) <= step)
    {
      degree = (uint8_t)(step + 1U - degree);
      memcpy(previous, saved, RS_PARITY_SYMBOLS + 1U);
      previous_discrepancy = discrepancy;
      shift = 1U;
    }
    else
    {
      shift++;
    }
  }

  *locator_degree = degree;
  return 1U;
}

static int8_t RS_Decode(uint8_t *codeword, const uint8_t *erasure_positions,
                        uint8_t erasure_count, uint8_t *corrected_count)
{
  uint8_t syndromes[RS_PARITY_SYMBOLS] = {0};
  uint8_t forney_syndromes[RS_PARITY_SYMBOLS] = {0};
  uint8_t erasure_locator[RS_PARITY_SYMBOLS + 1U] = {0};
  uint8_t error_locator[RS_PARITY_SYMBOLS + 1U] = {0};
  uint8_t locator[RS_PARITY_SYMBOLS + 1U] = {0};
  uint8_t error_positions[RS_PARITY_SYMBOLS];
  uint8_t error_locations[RS_PARITY_SYMBOLS];
  uint8_t matrix[RS_PARITY_SYMBOLS][RS_PARITY_SYMBOLS + 1U];
  uint8_t forney_count = RS_PARITY_SYMBOLS;
  uint8_t erasure_degree = 0U;
  uint8_t unknown_error_degree = 0U;
  uint8_t locator_degree;
  uint8_t error_count = 0U;
  uint8_t has_error = 0U;

  if ((corrected_count == NULL) ||
      (erasure_count > RS_PARITY_SYMBOLS) ||
      ((erasure_count > 0U) && (erasure_positions == NULL)))
  {
    return -1;
  }
  *corrected_count = 0U;

  for (uint8_t i = 0U; i < erasure_count; i++)
  {
    if (erasure_positions[i] >= RS_CODEWORD_SYMBOLS)
    {
      return -1;
    }
    for (uint8_t j = 0U; j < i; j++)
    {
      if (erasure_positions[i] == erasure_positions[j])
      {
        return -1;
      }
    }
  }

  RS_CalculateSyndromes(codeword, syndromes);
  for (uint8_t i = 0U; i < RS_PARITY_SYMBOLS; i++)
  {
    has_error |= syndromes[i];
  }
  if (has_error == 0U)
  {
    return 0;
  }

  memcpy(forney_syndromes, syndromes, sizeof(forney_syndromes));
  erasure_locator[0] = 1U;
  for (uint8_t erasure = 0U; erasure < erasure_count; erasure++)
  {
    uint8_t location = (uint8_t)(RS_CODEWORD_SYMBOLS - 1U - erasure_positions[erasure]);
    uint8_t x = rs_gf_exp[location];

    for (int16_t i = (int16_t)erasure_degree; i >= 0; i--)
    {
      erasure_locator[i + 1] ^= RS_GfMul(erasure_locator[i], x);
    }
    erasure_degree++;

    /* Remove this known erasure from the syndrome sequence.  The remaining
       sequence contains only unknown errors, so ordinary Berlekamp-Massey
       can find their locator without spending two parity symbols per erasure. */
    for (uint8_t i = 0U; (uint8_t)(i + 1U) < forney_count; i++)
    {
      forney_syndromes[i] =
          forney_syndromes[i + 1U] ^ RS_GfMul(x, forney_syndromes[i]);
    }
    forney_count--;
  }

  if (forney_count > 0U)
  {
    if (RS_FindErrorLocator(forney_syndromes, forney_count,
                            error_locator, &unknown_error_degree) == 0U)
    {
      return -1;
    }
  }
  else
  {
    error_locator[0] = 1U;
  }

  if (((uint16_t)(2U * unknown_error_degree) + erasure_count) > RS_PARITY_SYMBOLS)
  {
    return -1;
  }

  for (uint8_t i = 0U; i <= unknown_error_degree; i++)
  {
    for (uint8_t j = 0U; j <= erasure_degree; j++)
    {
      locator[i + j] ^= RS_GfMul(error_locator[i], erasure_locator[j]);
    }
  }
  locator_degree = (uint8_t)(unknown_error_degree + erasure_degree);
  if ((locator_degree == 0U) || (locator_degree > RS_PARITY_SYMBOLS))
  {
    return -1;
  }

  for (uint8_t pos = 0U; pos < RS_CODEWORD_SYMBOLS; pos++)
  {
    uint8_t location = (uint8_t)(RS_CODEWORD_SYMBOLS - 1U - pos);
    uint8_t x = rs_gf_exp[(RS_FIELD_ORDER - location) % RS_FIELD_ORDER];

    if (RS_PolyEvalAscending(locator, locator_degree, x) == 0U)
    {
      if (error_count >= RS_PARITY_SYMBOLS)
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

  /* Solve all known-erasure and unknown-error magnitudes together from the
     first locator_degree syndromes using a GF(64) Vandermonde system. */
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

/* Generalized minimum-distance retry: preserve any true slot erasures, then
   only after ordinary errors+erasures decoding fails, progressively mark the
   least-reliable remaining symbols as erasures.  Every attempt starts from
   the unmodified received codeword and RS_Decode performs a full syndrome
   verification before a candidate is accepted. */
static int8_t RS_DecodeAdaptive(uint8_t *codeword,
                                const uint8_t *erased_flags,
                                const float *reliability,
                                uint8_t *corrected_count,
                                uint8_t *used_erasure_count,
                                uint8_t *adaptive_erasure_count)
{
  uint8_t original[RS_CODEWORD_SYMBOLS];
  uint8_t working[RS_CODEWORD_SYMBOLS];
  uint8_t erasure_positions[RS_PARITY_SYMBOLS];
  uint8_t candidates[RS_CODEWORD_SYMBOLS];
  uint8_t explicit_count = 0U;
  uint8_t candidate_count = 0U;
  uint8_t max_adaptive;

  if ((codeword == NULL) || (erased_flags == NULL) || (reliability == NULL) ||
      (corrected_count == NULL) || (used_erasure_count == NULL) ||
      (adaptive_erasure_count == NULL))
  {
    return -1;
  }

  *corrected_count = 0U;
  *used_erasure_count = 0U;
  *adaptive_erasure_count = 0U;
  memcpy(original, codeword, sizeof(original));

  for (uint8_t pos = 0U; pos < RS_CODEWORD_SYMBOLS; pos++)
  {
    if (erased_flags[pos] != 0U)
    {
      explicit_count++;
    }
    else
    {
      candidates[candidate_count++] = pos;
    }
  }
  *used_erasure_count = explicit_count;
  if (explicit_count > RS_PARITY_SYMBOLS)
  {
    return -1;
  }

  {
    uint8_t index = 0U;
    for (uint8_t pos = 0U; pos < RS_CODEWORD_SYMBOLS; pos++)
    {
      if (erased_flags[pos] != 0U)
      {
        erasure_positions[index++] = pos;
      }
    }
  }

  /* Stable insertion sort keeps equal-confidence symbols in wire order. */
  for (uint8_t i = 1U; i < candidate_count; i++)
  {
    uint8_t candidate = candidates[i];
    uint8_t j = i;

    while ((j > 0U) &&
           (reliability[candidate] < reliability[candidates[j - 1U]]))
    {
      candidates[j] = candidates[j - 1U];
      j--;
    }
    candidates[j] = candidate;
  }

  max_adaptive = (uint8_t)(RS_PARITY_SYMBOLS - explicit_count);
  if (max_adaptive > candidate_count)
  {
    max_adaptive = candidate_count;
  }

  for (uint8_t add_count = 0U; add_count <= max_adaptive; add_count++)
  {
    uint8_t attempt_corrected = 0U;
    uint8_t total_erasures = (uint8_t)(explicit_count + add_count);
    int8_t status;

    memcpy(working, original, sizeof(working));
    for (uint8_t i = 0U; i < add_count; i++)
    {
      erasure_positions[explicit_count + i] = candidates[i];
    }
    status = RS_Decode(working, erasure_positions, total_erasures,
                       &attempt_corrected);
    if (status >= 0)
    {
      memcpy(codeword, working, sizeof(working));
      *corrected_count = attempt_corrected;
      *used_erasure_count = total_erasures;
      *adaptive_erasure_count = add_count;
      return status;
    }
  }

  /* On failure report the explicit count and how far the GMD search ran. */
  *adaptive_erasure_count = max_adaptive;
  return -1;
}

/* Decode one exact GMD hypothesis.  This is used by CRC-guided fallback to
   continue past a syndrome-valid but CRC-invalid candidate. */
static int8_t RS_DecodeAdaptiveAttempt(const uint8_t *received,
                                       const uint8_t *erased_flags,
                                       const float *reliability,
                                       uint8_t adaptive_erasure_count,
                                       uint8_t *decoded,
                                       uint8_t *corrected_count,
                                       uint8_t *used_erasure_count)
{
  uint8_t erasure_positions[RS_PARITY_SYMBOLS];
  uint8_t candidates[RS_CODEWORD_SYMBOLS];
  uint8_t explicit_count = 0U;
  uint8_t candidate_count = 0U;

  if ((received == NULL) || (erased_flags == NULL) || (reliability == NULL) ||
      (decoded == NULL) || (corrected_count == NULL) ||
      (used_erasure_count == NULL))
  {
    return -1;
  }

  for (uint8_t pos = 0U; pos < RS_CODEWORD_SYMBOLS; pos++)
  {
    if (erased_flags[pos] != 0U)
    {
      if (explicit_count < RS_PARITY_SYMBOLS)
      {
        erasure_positions[explicit_count] = pos;
      }
      explicit_count++;
    }
    else
    {
      candidates[candidate_count++] = pos;
    }
  }
  *corrected_count = 0U;
  *used_erasure_count = explicit_count;
  if ((explicit_count > RS_PARITY_SYMBOLS) ||
      (adaptive_erasure_count > candidate_count) ||
      (((uint16_t)explicit_count + adaptive_erasure_count) > RS_PARITY_SYMBOLS))
  {
    return -1;
  }

  for (uint8_t i = 1U; i < candidate_count; i++)
  {
    uint8_t candidate = candidates[i];
    uint8_t j = i;

    while ((j > 0U) &&
           (reliability[candidate] < reliability[candidates[j - 1U]]))
    {
      candidates[j] = candidates[j - 1U];
      j--;
    }
    candidates[j] = candidate;
  }
  for (uint8_t i = 0U; i < adaptive_erasure_count; i++)
  {
    erasure_positions[explicit_count + i] = candidates[i];
  }

  *used_erasure_count = (uint8_t)(explicit_count + adaptive_erasure_count);
  memcpy(decoded, received, RS_CODEWORD_SYMBOLS);
  return RS_Decode(decoded, erasure_positions, *used_erasure_count,
                   corrected_count);
}

static uint8_t RS_SelfTest(void)
{
  static const uint8_t error_positions[RS_CORRECTABLE_SYMBOLS] = {0U, 7U, 15U, 25U, 33U};
  static const uint8_t error_values[RS_CORRECTABLE_SYMBOLS] = {1U, 3U, 7U, 15U, 31U};
  static const uint8_t erasure_positions[RS_PARITY_SYMBOLS] = {
    0U, 3U, 6U, 9U, 12U, 15U, 18U, 21U, 24U, 33U
  };
  static const uint8_t mixed_erasure_positions[4U] = {1U, 8U, 17U, 29U};
  static const uint8_t mixed_error_positions[3U] = {4U, 14U, 32U};
  static const uint8_t gmd_error_positions[6U] = {2U, 5U, 11U, 19U, 27U, 31U};
  uint8_t data[RS_DATA_SYMBOLS];
  uint8_t expected[RS_CODEWORD_SYMBOLS];
  uint8_t damaged[RS_CODEWORD_SYMBOLS];
  uint8_t gmd_erased_flags[RS_CODEWORD_SYMBOLS] = {0};
  float gmd_reliability[RS_CODEWORD_SYMBOLS];
  uint8_t corrected = 0U;
  uint8_t used_erasures = 0U;
  uint8_t adaptive_erasures = 0U;

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

  if (RS_Decode(damaged, NULL, 0U, &corrected) != (int8_t)RS_CORRECTABLE_SYMBOLS)
  {
    return 0U;
  }
  if (corrected != RS_CORRECTABLE_SYMBOLS)
  {
    return 0U;
  }
  if (memcmp(damaged, expected, sizeof(expected)) != 0)
  {
    return 0U;
  }

  memcpy(damaged, expected, sizeof(damaged));
  for (uint8_t i = 0U; i < RS_PARITY_SYMBOLS; i++)
  {
    damaged[erasure_positions[i]] ^= (uint8_t)(i + 1U);
  }
  corrected = 0U;
  if ((RS_Decode(damaged, erasure_positions, RS_PARITY_SYMBOLS, &corrected) !=
       (int8_t)RS_PARITY_SYMBOLS) ||
      (corrected != RS_PARITY_SYMBOLS) ||
      (memcmp(damaged, expected, sizeof(expected)) != 0))
  {
    return 0U;
  }

  memcpy(damaged, expected, sizeof(damaged));
  for (uint8_t i = 0U; i < sizeof(mixed_erasure_positions); i++)
  {
    damaged[mixed_erasure_positions[i]] ^= (uint8_t)(3U + (i * 5U));
  }
  for (uint8_t i = 0U; i < sizeof(mixed_error_positions); i++)
  {
    damaged[mixed_error_positions[i]] ^= (uint8_t)(7U + (i * 9U));
  }
  corrected = 0U;
  if ((RS_Decode(damaged, mixed_erasure_positions,
                  (uint8_t)sizeof(mixed_erasure_positions), &corrected) != 7) ||
      (corrected != 7U) ||
      (memcmp(damaged, expected, sizeof(expected)) != 0))
  {
    return 0U;
  }

  /* Six unknown errors exceed t=5.  Marking the two least-reliable damaged
     positions as erasures converts the case to 4 errors + 2 erasures, exactly
     satisfying 2e+s=10. */
  memcpy(damaged, expected, sizeof(damaged));
  for (uint8_t i = 0U; i < RS_CODEWORD_SYMBOLS; i++)
  {
    gmd_reliability[i] = RS_VOTE_RATIO_CAP;
  }
  for (uint8_t i = 0U; i < sizeof(gmd_error_positions); i++)
  {
    damaged[gmd_error_positions[i]] ^= (uint8_t)(1U << i);
  }
  gmd_reliability[gmd_error_positions[0]] = 1.01f;
  gmd_reliability[gmd_error_positions[1]] = 1.02f;
  corrected = 0U;
  used_erasures = 0U;
  adaptive_erasures = 0U;
  if ((RS_DecodeAdaptive(damaged, gmd_erased_flags, gmd_reliability,
                         &corrected, &used_erasures,
                         &adaptive_erasures) != 6) ||
      (corrected != 6U) || (used_erasures != 2U) ||
      (adaptive_erasures != 2U) ||
      (memcmp(damaged, expected, sizeof(expected)) != 0))
  {
    return 0U;
  }

  return 1U;
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
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf,
                        (uint32_t)fsk_symbol_samples * 2U) != HAL_OK)
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
    printf("half duplex: RX listening, PGA=x%u\r\n",
           rx_pga_gain_values[rx_pga_gain_code]);
    OLED_PrintRxStatus();
  }
  else
  {
    rx_resume_tick = now + 500U;
  }
}

static void CommunicationMode_Select(CommunicationMode mode)
{
  if ((communication_mode != COMM_MODE_UNSELECTED) ||
      ((mode != COMM_MODE_STANDARD) &&
       (mode != COMM_MODE_MULTI_NODE) &&
       (mode != COMM_MODE_HIDDEN)))
  {
    return;
  }

  communication_mode = mode;
  app_mode = APP_MODE_RX;
  tx_destination_id = 0U;
  rx_resume_pending = 0U;

#if FSK_HIGH_PROFILE_ENABLE != 0U
  FSK_ApplyProfile((communication_mode == COMM_MODE_HIDDEN) ? 1U : 0U);
#endif

  if (communication_mode != COMM_MODE_MULTI_NODE)
  {
    /* Standard and hidden modes use the addressed frame format as N1 broadcast. */
    local_node_id = 1U;
    tx_calibration_sent = 0U;
  }
  else
  {
    local_node_id = 0U;
    /* Multi-node mode intentionally has no calibration transmission stage. */
    tx_calibration_sent = 1U;
  }

  Calibration_ResetProgress();
  RX_ResetDetector();

  if (RX_StartSampling() == 0U)
  {
    Error_Handler();
  }

  if (communication_mode == COMM_MODE_STANDARD)
  {
    printf("communication mode: STANDARD, calibration enabled, PGA automatic from x%u; reset to change mode\r\n",
           rx_pga_gain_values[rx_pga_gain_code]);
    printf("standard route: internal N1->ALL; receive the other board's calibration before messages\r\n");
    printf("standard RX keys after calibration: short A/B=older/newer message, hold A/B %ums=PGA down/up\r\n",
           KEYPAD_LONG_PRESS_MS);
  }
  else if (communication_mode == COMM_MODE_HIDDEN)
  {
    printf("communication mode: HIDDEN, high-frequency profile with calibration, PGA automatic from x%u; reset to change mode\r\n",
           rx_pga_gain_values[rx_pga_gain_code]);
    printf("hidden profile: data=%u/%u/%u/%uHz sync=%uHz; internal N1->ALL\r\n",
           fsk_tx_freqs_hz[0], fsk_tx_freqs_hz[1],
           fsk_tx_freqs_hz[2], fsk_tx_freqs_hz[3],
           fsk_tx_freqs_hz[FSK_SYNC_SYMBOL]);
    printf("hidden RX keys after calibration: short A/B=older/newer message, hold A/B %ums=PGA down/up\r\n",
           KEYPAD_LONG_PRESS_MS);
  }
  else
  {
    printf("communication mode: MULTI-NODE, calibration skipped, PGA locked x%u; reset to change mode\r\n",
           rx_pga_gain_values[rx_pga_gain_code]);
    printf("multi-node setup: press 1/2/3 for local ID, D cycles ALL/N1/N2/N3 destination\r\n");
  }
  printf("half duplex: RX listening, PGA=x%u\r\n",
         rx_pga_gain_values[rx_pga_gain_code]);
  OLED_PrintMode();
}

static void App_ToggleMode(void)
{
  if (communication_mode == COMM_MODE_UNSELECTED)
  {
    printf("select communication mode first: A=STANDARD B=MULTI-NODE C=HIDDEN\r\n");
    OLED_PrintMode();
    return;
  }

  if (tx_mode != TX_MODE_IDLE)
  {
    printf("mode switch ignored: TX busy\r\n");
    return;
  }

  if (app_mode == APP_MODE_RX)
  {
    if (local_node_id == 0U)
    {
      printf("mode switch blocked: press RX key 1/2/3 to set local node ID first\r\n");
      OLED_PrintRxStatus();
      return;
    }
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
      printf("mode: RX listening, PGA=x%u\r\n",
             rx_pga_gain_values[rx_pga_gain_code]);
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
  if ((communication_mode != COMM_MODE_STANDARD) &&
      (communication_mode != COMM_MODE_HIDDEN))
  {
    printf("multi-node mode: calibration is disabled and PGA is locked x%u\r\n",
           rx_pga_gain_values[RX_PGA_MULTI_GAIN_CODE]);
    return;
  }

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
  tx_calibration_pass = 0U;
  tx_calibration_boundary_active = 0U;
  tx_calibration_stage = 0U;
  tx_calibration_stage_sample_count = 0U;
  tx_frame_part = TX_FRAME_TONE;
  tx_part_sample_count = 0U;
  TX_LoadCalibrationTone(0U);
  tx_mode = TX_MODE_CALIBRATION;
  __enable_irq();

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_ENABLE);
  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, TX_LED_ON);
  printf("half duplex: RX/ADC disabled, TX calibration active\r\n");
  printf("tx calibration only: 2 passes of 00/01/10/11, pass1=RX PGA AGC up/down, %ums %uHz boundary + %ums gap, pass2=AGC down-only verify\r\n",
         CALIBRATION_BOUNDARY_MS, fsk_tx_freqs_hz[FSK_SYNC_SYMBOL], CALIBRATION_INTERPASS_GAP_MS);
  printf("tx calibration tone timing: %u+%ums bursts for %ums each tone; no manual replay\r\n",
         TX_TONE_MS, TX_GAP_MS, CALIBRATION_STAGE_MS);
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
  if ((local_node_id < 1U) || (local_node_id > 3U))
  {
    printf("tx message blocked: return to RX and press 1/2/3 to set local node ID\r\n");
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
  printf("tx start: N%u->%s len=%u FSK_symbols=%u RS=%ux(%u,%u) total_parity=%u text=\"%s\"\r\n",
         local_node_id, Node_DestinationLabel(tx_destination_id),
         tx_last_len, tx_frame_len, RS_BLOCK_COUNT,
         RS_CODEWORD_SYMBOLS, RS_DATA_SYMBOLS, RS_TOTAL_PARITY_SYMBOLS,
         tx_last_message);
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
  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, TX_LED_ON);
}

static void TX_Stop(void)
{
  rx_resume_pending = 0U;

  __disable_irq();
  tx_mode = TX_MODE_IDLE;
  tx_phase = 0U;
  tx_part_sample_count = 0U;
  tx_calibration_pass = 0U;
  tx_calibration_boundary_active = 0U;
  tx_calibration_stage = 0U;
  tx_calibration_stage_sample_count = 0U;
  tx_done_pending = 0U;
  tx_done_is_calibration = 0U;
  DAC_Write12(TX_DAC_MID_CODE);
  __enable_irq();

  tx_display_sending = 0U;
  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, TX_LED_OFF);
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
      printf("tx calibration done: 2 passes sent once, local RX stayed disabled; press * for RX\r\n");
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
    if (tx_calibration_boundary_active != 0U)
    {
      DAC_WriteSample8(TX_ScaledSineSample());
      tx_calibration_stage_sample_count++;
      if (tx_calibration_stage_sample_count >= CALIBRATION_BOUNDARY_SAMPLES)
      {
        tx_calibration_boundary_active = 0U;
        tx_calibration_stage_sample_count = 0U;
        tx_part_sample_count = 0U;
        tx_frame_part = TX_FRAME_GAP;
        DAC_Write12(TX_DAC_MID_CODE);
        HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
      }
      return;
    }

    if (tx_calibration_stage < FSK_DATA_FREQ_COUNT)
    {
      if (tx_frame_part == TX_FRAME_TONE)
      {
        DAC_WriteSample8(TX_ScaledSineSample());
      }
      tx_part_sample_count++;
      tx_calibration_stage_sample_count++;

      if (tx_calibration_stage_sample_count >= CALIBRATION_STAGE_SAMPLES)
      {
        tx_part_sample_count = 0U;
        tx_calibration_stage_sample_count = 0U;
        tx_calibration_stage++;
        if (tx_calibration_stage < FSK_DATA_FREQ_COUNT)
        {
          tx_frame_part = TX_FRAME_TONE;
          TX_LoadCalibrationTone(tx_calibration_stage);
        }
        else
        {
          if ((tx_calibration_pass + 1U) < CALIBRATION_PASS_COUNT)
          {
            tx_calibration_boundary_active = 1U;
            tx_frame_part = TX_FRAME_TONE;
            TX_LoadCalibrationBoundaryTone();
          }
          else
          {
            tx_frame_part = TX_FRAME_GAP;
            DAC_Write12(TX_DAC_MID_CODE);
            HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
          }
        }
      }
      else if ((tx_frame_part == TX_FRAME_TONE) &&
               (tx_part_sample_count >= TX_TONE_SAMPLES))
      {
        tx_frame_part = TX_FRAME_GAP;
        tx_part_sample_count = 0U;
        DAC_Write12(TX_DAC_MID_CODE);
      }
      else if ((tx_frame_part == TX_FRAME_GAP) &&
               (tx_part_sample_count >= TX_GAP_SAMPLES))
      {
        tx_frame_part = TX_FRAME_TONE;
        tx_part_sample_count = 0U;
        TX_LoadCalibrationTone(tx_calibration_stage);
      }
    }
    else
    {
      tx_part_sample_count++;
      if ((tx_calibration_pass + 1U) < CALIBRATION_PASS_COUNT)
      {
        if (tx_part_sample_count >= CALIBRATION_INTERPASS_GAP_SAMPLES)
        {
          tx_part_sample_count = 0U;
          tx_calibration_stage_sample_count = 0U;
          tx_calibration_pass++;
          tx_calibration_boundary_active = 0U;
          tx_calibration_stage = 0U;
          tx_frame_part = TX_FRAME_TONE;
          TX_LoadCalibrationTone(0U);
          HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_ENABLE);
        }
      }
      else if (tx_part_sample_count >= CALIBRATION_TX_GUARD_SAMPLES)
      {
        tx_part_sample_count = 0U;
        tx_calibration_stage_sample_count = 0U;
        tx_calibration_sent = 1U;
        tx_mode = TX_MODE_IDLE;
        tx_done_is_calibration = 1U;
        tx_done_pending = 1U;
        HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
        HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, TX_LED_OFF);
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
        HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, TX_LED_OFF);
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

static void TX_LoadCalibrationBoundaryTone(void)
{
  tx_current_bits = FSK_SYNC_SYMBOL;
  tx_current_amp = tx_fsk_amp[FSK_SYNC_SYMBOL];
  tx_phase = 0U;
  tx_phase_inc = TX_PhaseIncFromFreq(fsk_tx_freqs_hz[FSK_SYNC_SYMBOL]);
}

static void TX_BuildMessageFrame(void)
{
  uint16_t pos = 0U;
  uint8_t rs_payload[RS_TOTAL_DATA_SYMBOLS];
  uint8_t rs_codewords[RS_BLOCK_COUNT][RS_CODEWORD_SYMBOLS];
  uint16_t crc;

  memset(rs_payload, RS_PAD_VALUE, sizeof(rs_payload));
  for (uint8_t i = 0U; i < tx_text_len; i++)
  {
    rs_payload[i] = Message_CharToCode(tx_text[i]);
  }
  crc = Message_Crc12Addressed(local_node_id, tx_destination_id,
                               rs_payload, MESSAGE_MAX_LEN);
  rs_payload[MESSAGE_MAX_LEN] = (uint8_t)((crc >> 6U) & RS_FIELD_ORDER);
  rs_payload[MESSAGE_MAX_LEN + 1U] = (uint8_t)(crc & RS_FIELD_ORDER);

  for (uint8_t block = 0U; block < RS_BLOCK_COUNT; block++)
  {
    RS_Encode(&rs_payload[block * RS_DATA_SYMBOLS], rs_codewords[block]);
  }
  printf("tx payload: N%u->%s text=%u/%u CRC12=0x%03X\r\n",
         local_node_id, Node_DestinationLabel(tx_destination_id),
         tx_text_len, MESSAGE_MAX_LEN, crc);

  for (uint8_t repeat = 0U; repeat < TX_REPEAT_COUNT; repeat++)
  {
    uint16_t fsk_index = 0U;

    for (uint8_t i = 0U; i < MSG_PREAMBLE_LEN; i++)
    {
      tx_frame_symbols[pos++] = tx_preamble_symbols[i];
    }
    tx_frame_symbols[pos++] = Node_SourceBitsFromId(local_node_id);
    tx_frame_symbols[pos++] = tx_destination_id;

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

static uint8_t PGA_IsCalibrationToneQualified(const FSK_DetectResult *result)
{
  if ((result->valid == 0U) ||
      (result->bits >= FSK_DATA_FREQ_COUNT) ||
      (result->max_energy <= (result->second_energy * RX_PGA_AGC_TONE_RATIO)))
  {
    return 0U;
  }

  if ((calibration_pass != 0U) && (result->bits != calibration_stage))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t PGA_AutoGainControl(const FSK_DetectResult *result, uint32_t now)
{
  uint16_t low_peak;
  uint16_t high_peak;
  uint16_t peak;
  uint8_t clipped;
  int8_t direction = 0;
  const char *reason = "OK";
  uint8_t old_code;
  uint8_t old_gain;
  uint8_t new_gain;
  uint8_t calibration_tone_valid;

  if ((communication_mode != COMM_MODE_STANDARD) &&
      (communication_mode != COMM_MODE_HIDDEN))
  {
    rx_pga_agc_high_windows = 0U;
    rx_pga_agc_clip_windows = 0U;
    rx_pga_agc_low_windows = 0U;
    return 0U;
  }

  if ((calibration_complete != 0U) || (calibration_failed != 0U))
  {
    rx_pga_agc_high_windows = 0U;
    rx_pga_agc_clip_windows = 0U;
    rx_pga_agc_low_windows = 0U;
    return 0U;
  }
  if ((now - rx_pga_agc_last_adjust_tick) < RX_PGA_AGC_SETTLE_MS)
  {
    return 0U;
  }

  low_peak = (result->adc_mean > result->adc_min) ?
             (uint16_t)(result->adc_mean - result->adc_min) : 0U;
  high_peak = (result->adc_max > result->adc_mean) ?
              (uint16_t)(result->adc_max - result->adc_mean) : 0U;
  peak = (low_peak > high_peak) ? low_peak : high_peak;
  clipped = ((result->adc_min <= RX_PGA_ADC_CLIP_MARGIN) ||
             (result->adc_max >= (4095U - RX_PGA_ADC_CLIP_MARGIN))) ? 1U : 0U;
  calibration_tone_valid = PGA_IsCalibrationToneQualified(result);

  if (calibration_tone_valid == 0U)
  {
    rx_pga_agc_high_windows = 0U;
    rx_pga_agc_clip_windows = 0U;
    rx_pga_agc_low_windows = 0U;
    return 0U;
  }

  if (clipped != 0U)
  {
    rx_pga_agc_high_windows = 0U;
    rx_pga_agc_low_windows = 0U;
    if (rx_pga_agc_clip_windows < RX_PGA_AGC_CLIP_WINDOWS)
    {
      rx_pga_agc_clip_windows++;
    }
    if (rx_pga_agc_clip_windows < RX_PGA_AGC_CLIP_WINDOWS)
    {
      return 0U;
    }
    direction = -1;
    reason = "CLIP";
  }
  else if (peak > RX_PGA_ADC_HIGH_PEAK)
  {
    rx_pga_agc_clip_windows = 0U;
    rx_pga_agc_low_windows = 0U;
    if (rx_pga_agc_high_windows < RX_PGA_AGC_HIGH_WINDOWS)
    {
      rx_pga_agc_high_windows++;
    }
    if (rx_pga_agc_high_windows < RX_PGA_AGC_HIGH_WINDOWS)
    {
      return 0U;
    }
    direction = -1;
    reason = "HIGH";
  }
  else if ((calibration_pass == 0U) &&
           (peak < RX_PGA_ADC_LOW_PEAK) &&
           (result->bits < FSK_DATA_FREQ_COUNT))
  {
    rx_pga_agc_high_windows = 0U;
    rx_pga_agc_clip_windows = 0U;
    if (rx_pga_agc_low_windows < RX_PGA_AGC_LOW_WINDOWS)
    {
      rx_pga_agc_low_windows++;
    }
    if (rx_pga_agc_low_windows < RX_PGA_AGC_LOW_WINDOWS)
    {
      return 0U;
    }
    direction = 1;
    reason = "LOW";
  }
  else
  {
    rx_pga_agc_high_windows = 0U;
    rx_pga_agc_clip_windows = 0U;
    rx_pga_agc_low_windows = 0U;
    return 0U;
  }

  if ((direction < 0) && (rx_pga_gain_code == 0U))
  {
    rx_pga_agc_high_windows = 0U;
    rx_pga_agc_clip_windows = 0U;
    rx_pga_agc_low_windows = 0U;
    if (clipped != 0U)
    {
      calibration_capture_active = 0U;
      calibration_complete = 0U;
      calibration_failed = 1U;
      printf("calibration failed: ADC CLIP remains at PGA x1 pass=%u/2 mean=%u min=%u max=%u; reduce analog gain or increase distance\r\n",
             calibration_pass + 1U, (unsigned int)result->adc_mean,
             result->adc_min, result->adc_max);
      OLED_PrintRxStatus();
      return 1U;
    }
    return 0U;
  }
  if ((direction > 0) && (rx_pga_gain_code >= 7U))
  {
    rx_pga_agc_low_windows = 0U;
    return 0U;
  }

  old_code = rx_pga_gain_code;
  old_gain = rx_pga_gain_values[old_code];
  rx_pga_gain_code = (direction < 0) ?
                     (uint8_t)(rx_pga_gain_code - 1U) :
                     (uint8_t)(rx_pga_gain_code + 1U);
  new_gain = rx_pga_gain_values[rx_pga_gain_code];

  if (calibration_pass != 0U)
  {
    float energy_scale = (float)new_gain / (float)old_gain;
    energy_scale *= energy_scale;
    for (uint8_t i = 0U; i < calibration_stage; i++)
    {
      calibration_energy[i] *= energy_scale;
    }
  }

  PGA_SetGain(rx_pga_gain_code);
  rx_pga_agc_last_adjust_tick = now;
  rx_pga_agc_high_windows = 0U;
  rx_pga_agc_clip_windows = 0U;
  rx_pga_agc_low_windows = 0U;
  calibration_capture_active = 0U;
  calibration_energy_sum = 0.0f;
  calibration_energy_windows = 0U;
  calibration_boundary_windows = 0U;
  calibration_stable_windows = 0U;
  calibration_lost_windows = 0U;
  calibration_progress_tick = now;
  if (calibration_pass == 0U)
  {
    calibration_coarse_activity = 0U;
    calibration_coarse_last_signal_tick = 0U;
  }
  calibration_adc_min_seen = 0xFFFFU;
  calibration_adc_max_seen = 0U;
  calibration_adc_mean_last = 2048U;
  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
  }
  RX_ResetDetector();
  printf("PGA AGC adjust: pass=%u/2 reason=%s x%u -> x%u ADC mean=%u min=%u max=%u peak=%u%s\r\n",
         calibration_pass + 1U, reason,
         rx_pga_gain_values[old_code], rx_pga_gain_values[rx_pga_gain_code],
         (unsigned int)result->adc_mean, result->adc_min, result->adc_max, peak,
         (calibration_pass != 0U) ? "; current tone restarted" : "");
  OLED_PrintRxStatus();
  return 1U;
}

static uint8_t PGA_ManualAdjust(int8_t direction)
{
  uint8_t old_code;
  uint8_t old_gain;
  uint8_t new_gain;
  uint8_t calibrating;
  uint32_t now = HAL_GetTick();

  if (((communication_mode != COMM_MODE_STANDARD) &&
       (communication_mode != COMM_MODE_HIDDEN)) ||
      (app_mode != APP_MODE_RX) ||
      (calibration_failed != 0U) ||
      (direction == 0))
  {
    return 0U;
  }

  calibrating = (calibration_complete == 0U) ? 1U : 0U;
  if ((calibrating == 0U) && (msg_rx_state != MSG_RX_SEARCH))
  {
    printf("PGA manual adjust ignored while an RX frame is active\r\n");
    return 1U;
  }

  if (((direction < 0) && (rx_pga_gain_code == 0U)) ||
      ((direction > 0) && (rx_pga_gain_code >= 7U)))
  {
    printf("PGA manual limit: already x%u\r\n",
           rx_pga_gain_values[rx_pga_gain_code]);
    OLED_PrintRxStatus();
    return 1U;
  }

  old_code = rx_pga_gain_code;
  old_gain = rx_pga_gain_values[old_code];
  rx_pga_gain_code = (direction < 0) ?
                     (uint8_t)(rx_pga_gain_code - 1U) :
                     (uint8_t)(rx_pga_gain_code + 1U);
  new_gain = rx_pga_gain_values[rx_pga_gain_code];

  /* Keep already captured pass-2 tones in the same gain domain. */
  if ((calibrating != 0U) && (calibration_pass != 0U))
  {
    float energy_scale = (float)new_gain / (float)old_gain;
    energy_scale *= energy_scale;
    for (uint8_t i = 0U; i < calibration_stage; i++)
    {
      calibration_energy[i] *= energy_scale;
    }
  }

  PGA_SetGain(rx_pga_gain_code);
  rx_pga_agc_last_adjust_tick = now;
  rx_pga_agc_high_windows = 0U;
  rx_pga_agc_clip_windows = 0U;
  rx_pga_agc_low_windows = 0U;

  if (calibrating == 0U)
  {
    for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
    {
      fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
    }
    RX_ResetDetector();
    printf("PGA RX manual adjust: %s x%u -> x%u; calibration energy scales retained\r\n",
           (direction < 0) ? "A DOWN" : "B UP", old_gain, new_gain);
    OLED_PrintRxStatus();
    return 1U;
  }

  calibration_capture_active = 0U;
  calibration_energy_sum = 0.0f;
  calibration_energy_windows = 0U;
  calibration_boundary_windows = 0U;
  calibration_stable_windows = 0U;
  calibration_lost_windows = 0U;
  calibration_progress_tick = now;
  if (calibration_pass == 0U)
  {
    calibration_coarse_activity = 0U;
    calibration_coarse_last_signal_tick = 0U;
  }
  calibration_adc_min_seen = 0xFFFFU;
  calibration_adc_max_seen = 0U;
  calibration_adc_mean_last = 2048U;
  for (uint8_t i = 0U; i < FSK_FREQ_COUNT; i++)
  {
    fsk_noise_floor[i] = FSK_NOISE_MIN_FLOOR;
  }
  RX_ResetDetector();

  printf("PGA manual adjust: pass=%u/2 %s x%u -> x%u; current tone restarted, automatic AGC remains enabled\r\n",
         calibration_pass + 1U, (direction < 0) ? "A DOWN" : "B UP",
         old_gain, new_gain);
  OLED_PrintRxStatus();
  return 1U;
}

static const char *PGA_AdcQuality(void)
{
  uint16_t low_peak;
  uint16_t high_peak;
  uint16_t peak;

  if (calibration_adc_min_seen == 0xFFFFU)
  {
    return "WAIT";
  }
  if ((calibration_adc_min_seen <= RX_PGA_ADC_CLIP_MARGIN) ||
      (calibration_adc_max_seen >= (4095U - RX_PGA_ADC_CLIP_MARGIN)))
  {
    return "CLIP";
  }

  low_peak = (calibration_adc_mean_last > calibration_adc_min_seen) ?
             (calibration_adc_mean_last - calibration_adc_min_seen) : 0U;
  high_peak = (calibration_adc_max_seen > calibration_adc_mean_last) ?
              (calibration_adc_max_seen - calibration_adc_mean_last) : 0U;
  peak = (low_peak > high_peak) ? low_peak : high_peak;
  if (peak < RX_PGA_ADC_LOW_PEAK)
  {
    return "LOW";
  }
  if (peak > RX_PGA_ADC_HIGH_PEAK)
  {
    return "HIGH";
  }
  return "OK";
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
  uint8_t previous_key;

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
    previous_key = keypad_stable_key;

    if ((previous_key != KEY_NONE) &&
        (keypad_defer_short_press != 0U) &&
        (keypad_long_press_handled == 0U))
    {
      Keypad_HandlePress(previous_key);
    }

    keypad_stable_key = raw;
    keypad_press_start_ms = now;
    keypad_defer_short_press = 0U;
    keypad_long_press_handled = 0U;

    if (raw != KEY_NONE)
    {
      if (((communication_mode == COMM_MODE_STANDARD) ||
           (communication_mode == COMM_MODE_HIDDEN)) &&
          (app_mode == APP_MODE_RX) &&
          (calibration_complete != 0U) &&
          ((raw == 'A') || (raw == 'B')))
      {
        /* Defer A/B until release so a long press does not also page memory. */
        keypad_defer_short_press = 1U;
      }
      else
      {
        Keypad_HandlePress(raw);
      }
    }
  }

  if ((keypad_defer_short_press != 0U) &&
      (keypad_long_press_handled == 0U) &&
      (keypad_stable_key != KEY_NONE) &&
      ((now - keypad_press_start_ms) >= KEYPAD_LONG_PRESS_MS))
  {
    keypad_long_press_handled = 1U;
    Keypad_HandleLongPress(keypad_stable_key);
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
  if (communication_mode == COMM_MODE_UNSELECTED)
  {
    if (key == 'A')
    {
      CommunicationMode_Select(COMM_MODE_STANDARD);
    }
    else if (key == 'B')
    {
      CommunicationMode_Select(COMM_MODE_MULTI_NODE);
    }
    else if (key == 'C')
    {
      CommunicationMode_Select(COMM_MODE_HIDDEN);
    }
    else
    {
      printf("select communication mode: A=STANDARD B=MULTI-NODE C=HIDDEN\r\n");
      OLED_PrintMode();
    }
    return;
  }

  if (key == '*')
  {
    App_ToggleMode();
    return;
  }

  if (app_mode == APP_MODE_RX)
  {
    if ((communication_mode == COMM_MODE_MULTI_NODE) &&
        (key >= '1') && (key <= '3'))
    {
      if (msg_rx_state == MSG_RX_SEARCH)
      {
        Node_SetLocalId((uint8_t)(key - '0'));
      }
      else
      {
        printf("node ID change ignored while a frame is active\r\n");
      }
      return;
    }
    if ((communication_mode == COMM_MODE_MULTI_NODE) && (key == 'D'))
    {
      if (msg_rx_state == MSG_RX_SEARCH)
      {
        Node_CycleDestination();
      }
      else
      {
        printf("destination change ignored while a frame is active\r\n");
      }
      return;
    }
    if (calibration_failed != 0U)
    {
      printf("RX calibration failed at PGA=x%u ADC=%s min=%u max=%u; toggle mode and resend calibration\r\n",
             rx_pga_gain_values[rx_pga_gain_code], PGA_AdcQuality(),
             calibration_adc_min_seen, calibration_adc_max_seen);
      OLED_PrintRxStatus();
      return;
    }
    if (calibration_complete == 0U)
    {
      if (((communication_mode == COMM_MODE_STANDARD) ||
           (communication_mode == COMM_MODE_HIDDEN)) &&
          ((key == 'A') || (key == 'B')))
      {
        (void)PGA_ManualAdjust((key == 'A') ? -1 : 1);
        return;
      }
      printf("RX calibration PGA: automatic AGC plus manual A=down B=up; pass 1 auto up/down, pass 2 auto down-only; current=x%u ADC=%s min=%u max=%u\r\n",
             rx_pga_gain_values[rx_pga_gain_code], PGA_AdcQuality(),
             calibration_adc_min_seen, calibration_adc_max_seen);
      OLED_PrintRxStatus();
      return;
    }
    MessageStore_HandleRxKey((char)key);
    return;
  }

  Editor_ProcessKey((char)key);
}

static void Keypad_HandleLongPress(uint8_t key)
{
  if (((communication_mode != COMM_MODE_STANDARD) &&
       (communication_mode != COMM_MODE_HIDDEN)) ||
      (app_mode != APP_MODE_RX) ||
      (calibration_complete == 0U) ||
      ((key != 'A') && (key != 'B')))
  {
    return;
  }

  (void)PGA_ManualAdjust((key == 'A') ? -1 : 1);
}

/* 16x16 page-oriented glyphs used only by the required startup screen. */
static const uint8_t splash_sheng[32] = {
  0x04, 0x14, 0xD4, 0x54, 0x54, 0x54, 0x54, 0xDF, 0x54, 0x54, 0x54, 0x54, 0xD4, 0x14, 0x04, 0x00,
  0x80, 0x60, 0x1F, 0x02, 0x02, 0x02, 0x02, 0x03, 0x02, 0x02, 0x02, 0x02, 0x03, 0x00, 0x00, 0x00
};
static const uint8_t splash_yu[32] = {
  0x40, 0x42, 0xCC, 0x00, 0x00, 0x82, 0x92, 0x92, 0xF2, 0x9E, 0x92, 0x92, 0xF2, 0x82, 0x80, 0x00,
  0x00, 0x00, 0x7F, 0x20, 0x10, 0x00, 0xFC, 0x44, 0x44, 0x44, 0x44, 0x44, 0xFC, 0x00, 0x00, 0x00
};
static const uint8_t splash_xin[32] = {
  0x00, 0x80, 0x60, 0xF8, 0x07, 0x00, 0x04, 0x24, 0x24, 0x25, 0x26, 0x24, 0x24, 0x24, 0x04, 0x00,
  0x01, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xF9, 0x49, 0x49, 0x49, 0x49, 0x49, 0xF9, 0x00, 0x00
};
static const uint8_t splash_shi[32] = {
  0x80, 0x60, 0xF8, 0x07, 0x04, 0xE4, 0x24, 0x24, 0x24, 0xFF, 0x24, 0x24, 0x24, 0xE4, 0x04, 0x00,
  0x00, 0x00, 0xFF, 0x00, 0x80, 0x81, 0x45, 0x29, 0x11, 0x2F, 0x41, 0x41, 0x81, 0x81, 0x80, 0x00
};
static const uint8_t splash_ding[32] = {
  0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0xFE, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t splash_cong[32] = {
  0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x40, 0x30, 0x0E, 0x01, 0x82, 0x4C, 0x20, 0x18, 0x07, 0x00, 0x07, 0x18, 0x60, 0x80, 0x00
};
static const uint8_t splash_zhe[32] = {
  0x00, 0x24, 0xA4, 0x24, 0xFF, 0x14, 0x14, 0x80, 0x7E, 0x12, 0x12, 0x12, 0xF1, 0x11, 0x10, 0x00,
  0x00, 0x00, 0x00, 0xFD, 0x44, 0x44, 0x45, 0x44, 0x44, 0x44, 0x44, 0xFC, 0x01, 0x00, 0x00, 0x00
};
static const uint8_t splash_fan[32] = {
  0x04, 0x44, 0x84, 0x14, 0x64, 0x0F, 0x04, 0xE4, 0x24, 0x2F, 0x24, 0x24, 0xE4, 0x04, 0x04, 0x00,
  0x00, 0x08, 0x09, 0x78, 0x04, 0x03, 0x00, 0x3F, 0x40, 0x40, 0x42, 0x44, 0x43, 0x40, 0x78, 0x00
};
static const uint8_t splash_jia[32] = {
  0x02, 0x02, 0x0A, 0xEA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAA, 0xAA, 0xAA, 0xEA, 0x0A, 0x02, 0x02, 0x00,
  0x82, 0x4A, 0x2A, 0x1E, 0x0B, 0x4A, 0x8A, 0x7A, 0x02, 0xFA, 0x4B, 0x4A, 0x4A, 0xFA, 0x02, 0x00
};
static const uint8_t splash_yi[32] = {
  0x20, 0x24, 0xAC, 0x75, 0xA6, 0x34, 0x2C, 0xA4, 0xA0, 0x9E, 0x82, 0x82, 0x9E, 0xA0, 0x20, 0x00,
  0x49, 0x49, 0x24, 0x52, 0x89, 0x7F, 0x05, 0x98, 0x80, 0x43, 0x2C, 0x10, 0x2C, 0x43, 0x80, 0x00
};
static const uint8_t splash_song[32] = {
  0x10, 0x8C, 0x84, 0x84, 0x84, 0x84, 0x85, 0xF6, 0x84, 0x84, 0x84, 0x84, 0x84, 0x94, 0x0C, 0x00,
  0x20, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0xFF, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x20, 0x00
};
static const uint8_t splash_heng[32] = {
  0x00, 0xE0, 0x00, 0xFF, 0x10, 0x20, 0x02, 0xF2, 0x92, 0x92, 0x92, 0x92, 0x92, 0xF2, 0x02, 0x00,
  0x01, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x40, 0x4F, 0x44, 0x44, 0x44, 0x44, 0x44, 0x4F, 0x40, 0x00
};

static void OLED_SplashBlitGlyph16(uint8_t *row, uint8_t col,
                                   const uint8_t *glyph)
{
  if (col > (OLED_WIDTH - 16U))
  {
    return;
  }
  memcpy(&row[col], glyph, 16U);
  memcpy(&row[OLED_WIDTH + col], &glyph[16], 16U);
}

static void OLED_SplashBlitAscii(uint8_t *row, uint8_t col, const char *text)
{
  while ((*text != '\0') && (col <= (OLED_WIDTH - 6U)))
  {
    const uint8_t *glyph = OLED_Font5x7(*text++);

    for (uint8_t i = 0U; i < 5U; i++)
    {
      uint16_t shifted = (uint16_t)glyph[i] << 4U;
      row[col + i] = (uint8_t)(shifted & 0xFFU);
      row[OLED_WIDTH + col + i] = (uint8_t)(shifted >> 8U);
    }
    col += 6U;
  }
}

static void OLED_SplashWriteRow(uint8_t first_page, const uint8_t *row)
{
  OLED_SetCursor(0U, first_page);
  OLED_WriteData(row, OLED_WIDTH);
  OLED_SetCursor(0U, (uint8_t)(first_page + 1U));
  OLED_WriteData(&row[OLED_WIDTH], OLED_WIDTH);
}

static void OLED_SplashWriteMember(uint8_t first_page, const char *student_id,
                                   const uint8_t *name0, const uint8_t *name1,
                                   const uint8_t *name2)
{
  uint8_t row[OLED_WIDTH * 2U];

  memset(row, 0, sizeof(row));
  OLED_SplashBlitAscii(row, 14U, student_id);
  OLED_SplashBlitGlyph16(row, 66U, name0);
  OLED_SplashBlitGlyph16(row, 82U, name1);
  OLED_SplashBlitGlyph16(row, 98U, name2);
  OLED_SplashWriteRow(first_page, row);
}

static void OLED_ShowStartupScreen(void)
{
  uint8_t title[OLED_WIDTH * 2U];

  if (oled_ready == 0U)
  {
    return;
  }

  OLED_Clear();
  memset(title, 0, sizeof(title));
  OLED_SplashBlitGlyph16(title, 32U, splash_sheng);
  OLED_SplashBlitGlyph16(title, 48U, splash_yu);
  OLED_SplashBlitGlyph16(title, 64U, splash_xin);
  OLED_SplashBlitGlyph16(title, 80U, splash_shi);
  OLED_SplashWriteRow(0U, title);

  OLED_SplashWriteMember(2U, "24361004", splash_ding, splash_cong, splash_zhe);
  OLED_SplashWriteMember(4U, "24271097", splash_fan, splash_jia, splash_yi);
  OLED_SplashWriteMember(6U, "24211218", splash_song, splash_jia, splash_heng);
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
  if (communication_mode == COMM_MODE_UNSELECTED)
  {
    OLED_PrintLine(0U, "SELECT COMM MODE");
    OLED_PrintLine(2U, "A=STANDARD B=MULTI");
    OLED_PrintLine(4U, "C=HIDDEN HIGH FSK");
    OLED_PrintLine(6U, "B:NO CAL C:CAL");
    return;
  }
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
  char endpoint[5];
  const char *display_text = "";
  uint8_t display_len = 0U;

  if (app_mode != APP_MODE_RX)
  {
    return;
  }
  if (communication_mode == COMM_MODE_UNSELECTED)
  {
    OLED_PrintMode();
    return;
  }
  if (local_node_id == 0U)
  {
    OLED_PrintLine(0U, "SET NODE: PRESS 1/2/3");
    OLED_PrintLine(2U, "THEN D=DESTINATION");
    OLED_PrintLine(4U, "0=ALL 1/2/3=NODE");
    OLED_PrintLine(6U, "MULTI NO CAL PGAx2");
    return;
  }

  if (communication_mode == COMM_MODE_STANDARD)
  {
    (void)snprintf(endpoint, sizeof(endpoint), "STD");
  }
  else if (communication_mode == COMM_MODE_HIDDEN)
  {
    (void)snprintf(endpoint, sizeof(endpoint), "HID");
  }
  else
  {
    (void)snprintf(endpoint, sizeof(endpoint), "N%u", local_node_id);
  }

  if (calibration_failed != 0U)
  {
    (void)snprintf(line, sizeof(line), "%s RX CAL FAIL Gx%u",
                   endpoint,
                   rx_pga_gain_values[rx_pga_gain_code]);
    OLED_PrintLine(0U, line);
    (void)snprintf(line, sizeof(line), "ADC %u-%u",
                   calibration_adc_min_seen, calibration_adc_max_seen);
    OLED_PrintLine(2U, line);
    OLED_PrintLine(4U, "CLIP AT MIN GAIN");
    OLED_PrintLine(6U, "* RETRY CAL");
    return;
  }

  if (calibration_complete == 0U)
  {
    if (calibration_pass == 0U)
    {
      (void)snprintf(line, sizeof(line), "%s RX CAL1 Gx%u",
                     endpoint,
                     rx_pga_gain_values[rx_pga_gain_code]);
    }
    else
    {
      (void)snprintf(line, sizeof(line), "%s RX CAL2 %u/4 Gx%u",
                     endpoint,
                     calibration_stage + 1U,
                     rx_pga_gain_values[rx_pga_gain_code]);
    }
    OLED_PrintLine(0U, line);
    if (calibration_adc_min_seen == 0xFFFFU)
    {
      OLED_PrintLine(2U, "ADC WAIT SIGNAL");
    }
    else
    {
      (void)snprintf(line, sizeof(line), "ADC %u-%u",
                     calibration_adc_min_seen, calibration_adc_max_seen);
      OLED_PrintLine(2U, line);
    }
    OLED_PrintLine(4U, (calibration_pass == 0U) ?
                   "AUTO+A/B UP/DOWN" : "AUTO DOWN A/B BOTH");
    (void)snprintf(line, sizeof(line), "LEVEL %s", PGA_AdcQuality());
    OLED_PrintLine(6U, line);
    return;
  }
  else if ((message_store_last_save_failed != 0U) && (rx_message_valid != 0U))
  {
    (void)snprintf(line, sizeof(line), "%s Gx%u RX<%u NOSAVE",
                   endpoint, rx_pga_gain_values[rx_pga_gain_code],
                   rx_message_source_id);
    display_text = rx_message;
    display_len = rx_message_len;
  }
  else if (message_store_count > 0U)
  {
    const StoredMessage *stored = &message_store_cache[message_store_view_index];

    (void)snprintf(line, sizeof(line), "%s Gx%u RX<%u M%u/%u",
                   endpoint, rx_pga_gain_values[rx_pga_gain_code], stored->source,
                   message_store_view_index + 1U,
                   message_store_count);
    display_text = stored->text;
    display_len = stored->len;
  }
  else if (rx_message_valid != 0U)
  {
    (void)snprintf(line, sizeof(line), "%s Gx%u RX<%u OK L%u",
                   endpoint, rx_pga_gain_values[rx_pga_gain_code],
                   rx_message_source_id, rx_message_len);
    display_text = rx_message;
    display_len = rx_message_len;
  }
  else if (rx_sampling_active == 0U)
  {
    (void)snprintf(line, sizeof(line), "%s Gx%u >%s PAUSE",
                   endpoint, rx_pga_gain_values[rx_pga_gain_code],
                   Node_DestinationLabel(tx_destination_id));
  }
  else
  {
    (void)snprintf(line, sizeof(line), "%s Gx%u >%s LISTEN",
                   endpoint, rx_pga_gain_values[rx_pga_gain_code],
                   Node_DestinationLabel(tx_destination_id));
  }

  OLED_PrintLine(0U, line);
  OLED_PrintTextRows(display_text, display_len);
}

static void OLED_PrintTxStatus(void)
{
  char line[22];
  char endpoint[5];
  char preview[MESSAGE_MAX_LEN + 2U];
  uint8_t preview_len = tx_text_len;

  if (app_mode != APP_MODE_TX)
  {
    return;
  }

  if (communication_mode == COMM_MODE_STANDARD)
  {
    (void)snprintf(endpoint, sizeof(endpoint), "STD");
  }
  else if (communication_mode == COMM_MODE_HIDDEN)
  {
    (void)snprintf(endpoint, sizeof(endpoint), "HID");
  }
  else
  {
    (void)snprintf(endpoint, sizeof(endpoint), "N%u", local_node_id);
  }

  if (tx_calibration_sent == 0U)
  {
    if (tx_mode == TX_MODE_CALIBRATION)
    {
      (void)snprintf(line, sizeof(line), "%s>%s CAL SEND",
                     endpoint, Node_DestinationLabel(tx_destination_id));
    }
    else
    {
      (void)snprintf(line, sizeof(line), "%s>%s CAL PRESS #",
                     endpoint, Node_DestinationLabel(tx_destination_id));
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

  (void)snprintf(line, sizeof(line), "%s>%s %s L%u/%u",
                 endpoint, Node_DestinationLabel(tx_destination_id),
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
    symbol_start_index = fsk_symbol_samples;
    symbol_ready = 1U;
  }
}
/* USER CODE END 0 */

int main(void)
{
  PowerKill_EarlyKeepAlive();

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
  rx_pga_gain_code = RX_PGA_DEFAULT_GAIN_CODE;
  PGA_SetGain(rx_pga_gain_code);

  FSK_Init();
  RS_Init();
  MessageStore_Init();
  HAL_Delay(300U);
  if (OLED_Init() != 0U)
  {
    OLED_ShowStartupScreen();
    HAL_Delay(2000U);
  }
  OLED_PrintMode();

  printf("\r\nThree-mode 4-FSK RX/TX start\r\n");
  printf("SELECT MODE: A=STANDARD, B=MULTI-NODE (no calibration + PGA x2), C=HIDDEN (high-frequency + calibration)\r\n");
  printf("HALF DUPLEX: RX PC4 via MCP6S21, TX MCP4921 DAC; select mode first, then *=RX/TX\r\n");
  printf("CLOCK: 25MHz HSE -> 96MHz PLL, TIM2 ADC=16kHz NORMAL/20kHz HIDDEN, TIM3 DAC=32kHz\r\n");
  printf("SPI1 MODE0 6MHz: SCK PA5, MOSI PA7, DAC_CS PB0, PGA_CS PA6\r\n");
  printf("POWER: LTC2950 PB9=INT# active low, PC1=KILL; AMP_MUTE PB12 active high\r\n");
  printf("KEYPAD: 1 2 3 A / 4 5 6 B / 7 8 9 C / * 0 # D\r\n");
  printf("TX EDITOR: phone multi-tap, A=LEFT B=RIGHT C=DELETE D=CLEAR *=RX/TX; STANDARD/HIDDEN first #=CAL, MULTI #=SEND\r\n");
  printf("MULTI-NODE: no calibration, PGA fixed x2; in RX press 1/2/3=set local ID, D=cycle destination ALL/N1/N2/N3\r\n");
  printf("ROUTING: START0=source ID encoded as 00/01/10, START1=destination 00=ALL 01/10/11=N1/N2/N3\r\n");
  printf("RX PROFILES: STANDARD/MULTI=NORMAL, HIDDEN=HIGH; both communicating boards must select the same mode\r\n");
  printf("STANDARD/HIDDEN CAL PGA: automatic plus RX keys A=down/B=up manual override; auto down requires valid tone (HIGH %u, CLIP %u windows); pass1 auto up/down, pass2 auto down-only\r\n",
         RX_PGA_AGC_HIGH_WINDOWS, RX_PGA_AGC_CLIP_WINDOWS);
  printf("LED3 PB7 active low: on for calibration/message TX, off when idle\r\n");
  printf("RX MEMORY: A=OLDER B=NEWER C=LATEST, saved=%u/5; BLUE LED=RX COMPLETE\r\n",
         message_store_count);
  printf("ADC windows: NORMAL=%uHz/%u samples, HIDDEN=%uHz/%u samples, symbol=%ums\r\n",
         FSK_NORMAL_FS_HZ, FSK_NORMAL_SYMBOL_SAMPLES,
         FSK_HIDDEN_FS_HZ, FSK_HIDDEN_SYMBOL_SAMPLES, FSK_SYMBOL_MS);
  printf("FEC: %ux RS(%u,%u) GF(64), payload=%u chars + CRC12/%u symbols, parity=%u, t=%u per block/%u total\r\n",
         RS_BLOCK_COUNT, RS_CODEWORD_SYMBOLS, RS_DATA_SYMBOLS, MESSAGE_MAX_LEN,
         MESSAGE_CRC_SYMBOLS,
         RS_TOTAL_PARITY_SYMBOLS, RS_CORRECTABLE_SYMBOLS, RS_TOTAL_CORRECTABLE);
  printf("FEC erasures: known codeword positions retained; each block decodes 2*errors+erasures<=%u\r\n",
         RS_PARITY_SYMBOLS);
  printf("FEC GMD: failed blocks retry with least-reliable RS symbols, up to %u total erasures/block\r\n",
         RS_PARITY_SYMBOLS);
  printf("CRC12: 0x80F over source+destination+all %u padded text symbols; CRC guides GMD candidate selection\r\n",
         MESSAGE_MAX_LEN);
  printf("FSK whitening: ON, mask cycle=00/01/10/11\r\n");
  printf("FSK NORMAL: data=%u/%u/%u/%uHz sync=%uHz\r\n",
         fsk_normal_freqs_hz[0], fsk_normal_freqs_hz[1], fsk_normal_freqs_hz[2], fsk_normal_freqs_hz[3],
         fsk_normal_freqs_hz[FSK_SYNC_SYMBOL]);
  printf("FSK HIDDEN: data=%u/%u/%u/%uHz sync=%uHz\r\n",
         fsk_high_freqs_hz[0], fsk_high_freqs_hz[1], fsk_high_freqs_hz[2], fsk_high_freqs_hz[3],
         fsk_high_freqs_hz[FSK_SYNC_SYMBOL]);
  printf("RS framing marker: %uHz before every 6-bit RS symbol\r\n",
         fsk_tx_freqs_hz[FSK_SYNC_SYMBOL]);
  printf("TX timing: %u tones, %u+%u=%ums each, frame=%ums\r\n",
         TX_FRAME_MAX_SYMBOLS, TX_TONE_MS, TX_GAP_MS, TX_STEP_MS, TX_FRAME_DURATION_MS);
  printf("RS capture: all valid 20ms windows; slots=40..139/140..199/200..259ms\r\n");
  printf("RX adapt: timed PREAMBLE recovery + per-data-bin soft fallback + END soft/repeat guard\r\n");
  printf("RX scale: PREAMBLE top2 mean / qualified top1 fallback + bounded frame sqrt correction (0.5x..2.0x)\r\n");
  printf("STANDARD/HIDDEN CAL: first # sends 00/01/10/11 as %u+%ums bursts for %ums each\r\n",
         TX_TONE_MS, TX_GAP_MS, CALIBRATION_STAGE_MS);
  printf("CAL RX: profile-fixed %u/%u/%u/%uHz Goertzel, %u-window mean energy\r\n",
         fsk_tx_freqs_hz[0], fsk_tx_freqs_hz[1], fsk_tx_freqs_hz[2], fsk_tx_freqs_hz[3],
         CALIBRATION_ENERGY_WINDOWS);
  printf("RS errors+erasures+GMD self-test (5e, 10s, 3e+4s, 6e->2s): %s\r\n",
         (RS_SelfTest() != 0U) ? "PASS" : "FAIL");
  printf("CRC12 self-test (text=0x708, N1->N2 addressed=0x780): %s\r\n",
         (Message_CrcSelfTest() != 0U) ? "PASS" : "FAIL");

  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);

  printf("receiver paused until mode selection\r\n");
  OLED_PrintMode();

  while (1)
  {
    StatusLed_Update();
    Keypad_Task();
    Editor_Task();
    PowerInt_Task();
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

  /* X5 is a 25 MHz crystal.  Run the core and both audio timers from the
     crystal-derived 96 MHz PLL so separate boards share a ppm-class timebase. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25U;
  RCC_OscInitStruct.PLL.PLLN = 192U;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4U;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim2.Init.Prescaler = 5U;
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
  htim3.Init.Prescaler = 5U;
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
  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, TX_LED_OFF);
  HAL_GPIO_WritePin(AMP_MUTE_GPIO_Port, AMP_MUTE_Pin, AMP_MUTE_ON);
  HAL_GPIO_WritePin(POWER_KILL_GPIO_Port, POWER_KILL_Pin, POWER_KILL_KEEP_ON);
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, DAC_CS_INACTIVE);
  HAL_GPIO_WritePin(PGA_CS_GPIO_Port, PGA_CS_Pin, PGA_CS_INACTIVE);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TX_LED_Pin;
  HAL_GPIO_Init(TX_LED_GPIO_Port, &GPIO_InitStruct);

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

  GPIO_InitStruct.Pin = POWER_KILL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_KILL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = POWER_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_INT_GPIO_Port, &GPIO_InitStruct);

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
