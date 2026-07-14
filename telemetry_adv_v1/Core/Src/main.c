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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "telemetry_protocol.h"

/*
 * Sensor feature switches
 * -----------------------
 * The working BMI270 is enabled. Keep an uncommissioned device at 0; the
 * preprocessor then removes its include, initialization, and service code so
 * that device produces no I2C/SPI traffic and cannot raise an error LED.
 *
 * Before changing a switch to 1, first configure the matching bus and pins in
 * telemetry_adv_v1.ioc and add the Bosch .c files to the CubeIDE build:
 *   BMI270: SPI1 with IMU_CS (bmi270.h includes bmi2.h/bmi2_defs.h)
 *   BME280: SPI1 with BME_CS (bme280.h includes bme280_defs.h)
 *   RTC:    I2C1 for the external PCF85263A driver
 *
 * SPI1 is shared by the BMI270 and BME280 at 4 MHz. Short polling transfers
 * keep chip-select asserted across each transaction; CAN can preempt them.
 */
#define ENABLE_BMI270_IMU              1U
#define ENABLE_BME280_SENSOR           0U
#define ENABLE_STM32_INTERNAL_RTC       1U
#define ENABLE_EXTERNAL_PCF85263A_RTC  0U

/*
 * Set to 1U while using CubeIDE's SWV ITM Data Console; leave at 0U for normal
 * operation.  Disabling this prevents printf traffic from entering ITM even
 * when the debugger accidentally leaves tracing enabled.  USART1/RS232,
 * CAN, GPS, SD, RTC, and ESP32 operation are not disabled by this switch.
 */
#define ENABLE_SWV_DEBUG_OUTPUT        1U
/* Keep the complete raw_data block out of SWV unless explicitly needed. */
#define ENABLE_SWV_RS232_BLOCK_ECHO    0U

#if ENABLE_BMI270_IMU
#include "bmi270.h"
#endif

#if ENABLE_BME280_SENSOR
#define BME280_FLOAT_ENABLE
#include "bme280.h"
#endif

/*
 * Application overview
 * --------------------
 * - SPI2 reads the u-blox GPS and accepts only checksum-valid RMC/GGA/VTG data.
 * - SPI3 and FatFS append time-stamped CAN/GPS/speed snapshots to the SD card.
 * - SPI4 exchanges 768-byte ASCII telemetry/command frames with the ESP32.
 * - CAN1 receives all bus traffic and can queue telemetry commands on ID 0x5C0.
 * - USART1 sends the legacy raw_data block at 115200 baud.
 * - Optional SWV/ITM printf diagnostics never mix with USART1 telemetry.
 *
 * The STM32 internal RTC is active.  A valid GPS RMC sentence sets it from
 * UTC after boot and periodically corrects crystal drift; SET_RTC from the
 * ESP32/app can also set it. The BMI270 is calibrated and sampled at 100 Hz.
 * External PCF85263A and BME280 access remains disabled until ready.
 *
 * Transfer/interrupt policy
 * -------------------------
 * - CAN RX is interrupt-driven at NVIC priority 0 and can preempt every
 *   SPI/I2C task.
 * - BMI270/BME280 SPI1 uses the polling sequence proven by the IMU test code.
 * - GPS SPI2 reads 256 bytes in about 1.024 ms at 2 MHz.  Polling this short
 *   burst is cheaper and more deterministic than roughly 256 HAL byte IRQs
 *   every 20 ms when DMA is not configured; CAN can still preempt it.
 * - FatFS requires synchronous disk operations, so SD SPI3 stays polling.
 *   Interrupting every SD byte would add thousands of IRQs per sector.
 * - ESP32 SPI4 moves 768 bytes per transaction (about 1.536 ms at 4 MHz).
 *   Polling avoids hundreds of byte interrupts while priority-0 CAN remains
 *   able to preempt the transfer.
 *
 * If fully asynchronous SPI2/3/4 operation is needed later, DMA is preferable
 * to byte-level HAL interrupts for maintaining deterministic CPU timing.
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t tick_ms;     /* HAL tick when this frame was received. */
  uint32_t count;       /* Total frames received since boot. */
  uint32_t id;          /* 11-bit standard or 29-bit extended CAN ID. */
  uint8_t extended;     /* 0=standard ID, 1=extended ID. */
  uint8_t dlc;          /* Number of valid payload bytes, limited to 0..8. */
  uint8_t data[8];      /* Unmodified CAN payload bytes. */
} CAN_Snapshot_t;

typedef struct
{
  uint32_t can_id;      /* Standard CAN ID that fills this row. */
  const char *name;     /* Short label expected by the legacy parser. */
  volatile uint32_t high_word; /* CAN bytes 4..7, decoded little-endian. */
  volatile uint32_t low_word;  /* CAN bytes 0..3, decoded little-endian. */
  volatile uint8_t valid;      /* 1 after first matching 8-byte frame. */
} SunRawEntry_t;

/* Shared date/time representation for RTC commands, SD, RS232, ESP32, and SWV. */
typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t weekday;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t valid;
} TelemetryDateTime_t;

/* Latest BMI270 sample and processed vehicle-motion values written to SD. */
typedef struct
{
  int16_t acc_x_raw;
  int16_t acc_y_raw;
  int16_t acc_z_raw;
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;
  int32_t acc_x_mg;
  int32_t acc_y_mg;
  int32_t acc_z_mg;
  int32_t gyro_x_mdps;
  int32_t gyro_y_mdps;
  int32_t gyro_z_mdps;
  int32_t linear_x_mg;
  int32_t linear_y_mg;
  int32_t linear_z_mg;
  int32_t forward_accel_mg;
  uint32_t total_g_mg;
  uint32_t dynamic_g_mg;
  uint32_t peak_window_g_mg;
  uint32_t peak_boot_g_mg;
  uint32_t sample_ms;
  uint32_t sample_count;
  uint32_t read_error_count;
  uint8_t valid;
} IMU_Telemetry_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Status LEDs are active-low on this PCB. */
#define LED_ON                         GPIO_PIN_RESET
#define LED_OFF                        GPIO_PIN_SET
#define HEARTBEAT_PERIOD_MS            2000U
#define HEARTBEAT_ON_MS                1000U
#define ACTIVITY_PULSE_MS              120U
#define TELEMETRY_PERIOD_MS            1000U
#define SD_RETRY_PERIOD_MS             2000U
#define GPS_POLL_PERIOD_MS             20U
#define GPS_VALID_TIMEOUT_MS           3000U
#define GPS_SPI_BLOCK_SIZE             256U
#define GPS_NMEA_SIZE                  160U
#define GPS_SPEED_VALID_MS             5000U
#define GPS_FIX_VALID_MS               5000U
#define GPS_NAV_PRINT_PERIOD_MS        1000U
#define GPS_RTC_SYNC_PERIOD_MS         3600000U
#define KNOTS_TO_MPH                   1.15077945f
#define KMH_TO_MPH                     0.62137119f
#define MPS_TO_MPH                     2.23693629f
#define CAN_SPEED_VALID_MS             1000U
#define CAN_SPEED_MAX_ABS_MPH          120.0f
#define MC1_VELOCITY_CAN_ID            0x403U
#define MC2_VELOCITY_CAN_ID            0x423U
#define IMU_SAMPLE_PERIOD_MS           10U
#define IMU_SPEED_VALID_MS             500U
#define IMU_MAX_DT_MS                  2000U
#define IMU_STILL_GYRO_LIMIT_MDPS      5000L
#define IMU_STILL_ACC_TOL_MG           120.0f
#define IMU_ACCEL_DEADBAND_MG          35L
#define IMU_MOUNT_VERTICAL_MIN_MG      800L
#define IMU_MOUNT_FORWARD_MAX_MG       350L
#define IMU_MOUNT_LATERAL_MAX_MG       350L
#define IMU_AXIS_X                     0U
#define IMU_AXIS_Y                     1U
#define IMU_AXIS_Z                     2U
/* Pin-1/dot side faces the car's front, which is BMI270 +Y in top view. */
#define IMU_FORWARD_AXIS               IMU_AXIS_Y
#define IMU_FORWARD_SIGN               1L
#define IMU_SPEED_COAST_RETENTION      0.9998f
#define IMU_ZERO_SPEED_THRESHOLD_MPH   0.50f
#define IMU_ANCHOR_NONE                0U
#define IMU_ANCHOR_CAN                 1U
#define IMU_ANCHOR_GPS                 2U
#if ((IMU_FORWARD_AXIS != IMU_AXIS_X) && (IMU_FORWARD_AXIS != IMU_AXIS_Y) && \
     (IMU_FORWARD_AXIS != IMU_AXIS_Z))
#error "IMU_FORWARD_AXIS must be IMU_AXIS_X, IMU_AXIS_Y, or IMU_AXIS_Z"
#endif
#if ((IMU_FORWARD_SIGN != 1L) && (IMU_FORWARD_SIGN != -1L))
#error "IMU_FORWARD_SIGN must be 1L or -1L"
#endif
#define IMU_CALIBRATION_SAMPLES        32U
#define IMU_CALIBRATION_SAMPLE_MS      10U
#define BMI270_SPI_TIMEOUT_MS          100U
#define BMI270_STARTUP_DELAY_MS        10U
#define BMI270_CONFIG_BURST_LENGTH     64U
#define BMI270_RAW_PREFLIGHT_READS     5U
#define BMI270_RETRY_PERIOD_MS         5000U
#define BMI270_CHIP_ID_VALUE           0x24U
#define ESP32_READY_TIMEOUT_MS         100U
#define ESP32_SPI_TIMEOUT_MS           100U
#define ESP32_ASCII_FRAME_SIZE         768U
#define ESP32_COMMAND_SIZE             160U
#define ESP32_RESPONSE_SIZE            160U
#define CAN_SWV_MIN_PERIOD_MS          100U
#define CAN_SWV_STATUS_PERIOD_MS       5000U
#define RS232_SWV_ECHO_BLOCK           (ENABLE_SWV_DEBUG_OUTPUT && \
                                        ENABLE_SWV_RS232_BLOCK_ECHO)
#define STM32_RTC_BACKUP_MAGIC         0x544CU
#define STM32_RTC_DEFAULT_YEAR         2026U
#define STM32_RTC_DEFAULT_MONTH        7U
#define STM32_RTC_DEFAULT_DAY          12U
#define STM32_RTC_DEFAULT_WEEKDAY      RTC_WEEKDAY_SUNDAY
/* Red LED bitmask: any active subsystem error turns LED 10 on. */
#define ERROR_SD                       (1UL << 0)
#define ERROR_CAN                      (1UL << 1)
#define ERROR_RS232                    (1UL << 2)
#define ERROR_ESP32                    (1UL << 3)
#define ERROR_IMU                      (1UL << 4)

#define SUN_RAW_TABLE_COUNT            (sizeof(sun_raw_table) / sizeof(sun_raw_table[0]))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static volatile CAN_Snapshot_t can_latest;
/* "until" values are HAL tick deadlines used to make short activity pulses. */
static volatile uint32_t can_led_until;
static uint32_t rs232_led_until;
static volatile uint8_t rs232_tx_busy;
static uint32_t sd_led_until;
static volatile uint32_t error_flags;

/* FatFS is used only after a successful mount sets sd_ready. */
static uint8_t sd_ready;
static uint8_t sd_logging_enabled = 1U;
static uint32_t sd_last_attempt_ms;

/* GPS parser state and the most recent filtered navigation solution. */
static uint8_t gps_tx[GPS_SPI_BLOCK_SIZE];
static uint8_t gps_rx[GPS_SPI_BLOCK_SIZE];
static char gps_nmea[GPS_NMEA_SIZE];
static uint16_t gps_nmea_index;
static uint32_t gps_last_poll_ms;
static uint32_t gps_last_valid_ms;
static uint32_t gps_valid_count;
static uint32_t gps_filtered_count;
static uint32_t gps_checksum_error_count;
static uint32_t gps_last_nav_print_ms;
static uint32_t gps_rtc_last_sync_ms;
static uint32_t gps_rtc_sync_count;
static float latest_gps_lat_deg;
static float latest_gps_lon_deg;
static float latest_gps_speed_mph;
/* GGA altitude above mean sea level, retained as millimetres for exact ASCII output. */
static int32_t latest_gps_altitude_mm;
static uint32_t latest_gps_fix_ms;
static uint32_t latest_gps_speed_ms;
static uint32_t latest_gps_altitude_ms;
static uint8_t latest_gps_fix_valid;
static uint8_t latest_gps_speed_valid;
static uint8_t latest_gps_altitude_valid;

/* Vehicle-speed arbitration: fresh CAN wins, then GPS, then IMU estimate. */
static float latest_imu_speed_mph;
static uint32_t latest_imu_speed_ms;
static IMU_Telemetry_t imu_telemetry;
static uint8_t imu_ready;
static uint8_t imu_calibrated;
static uint8_t imu_calibration_active;
static uint8_t imu_mount_valid;
static uint8_t imu_calibration_wrong_mount;
static int8_t imu_last_result = -2; /* BMI2_E_COM_FAIL when the IMU is enabled. */
static uint32_t imu_last_retry_ms;
static float latest_can_speed_mph;
static uint32_t latest_can_speed_ms;
static uint8_t latest_can_speed_valid;
static const char *latest_can_speed_source = "NONE";
static float latest_vehicle_speed_mph;
static const char *latest_vehicle_speed_source = "NONE";

/*
 * The matching ESP32 bridge exchanges fixed 768-byte ASCII frames.  Static
 * buffers avoid placing 1536 bytes on the main-loop stack once per second.
 */
static uint8_t esp32_tx_frame[ESP32_ASCII_FRAME_SIZE];
static uint8_t esp32_rx_frame[ESP32_ASCII_FRAME_SIZE];
static char esp32_response[ESP32_RESPONSE_SIZE];
static uint8_t esp32_response_pending;
static char esp32_last_command_id[16];
static char esp32_last_command[ESP32_COMMAND_SIZE];
static uint32_t esp32_sequence;
static uint32_t esp32_pass_count;
static uint32_t esp32_fail_count;
static uint32_t esp32_last_pass_ms;
static volatile int32_t esp32_last_result;

/*
 * The internal RTC is the active calendar source while the external RTC is
 * disabled.  The source label is included in SWV, ESP32, and RS232 status so
 * the operator can distinguish the default, app-set, and GPS-set calendars.
 */
static uint8_t stm32_rtc_ready;
#if ENABLE_SWV_DEBUG_OUTPUT
static uint32_t stm32_rtc_last_print_ms;
#endif
static const char *stm32_rtc_time_source = "DEFAULT";

/*
 * Legacy Sunseeker raw_data table
 * --------------------------------
 * The table is not an arbitrary CAN filter.  It is the ordered set of rows
 * used by Telemetry_Reference and by the older computer/display parser.
 *
 * CAN IDs follow the Sunseeker network convention:
 *   0x400 + offset = motor controller 1
 *   0x420 + offset = motor controller 2
 *   0x500 + offset = driver controls
 *   0x580 + offset = battery protection
 *   0x5C0          = telemetry-board command transmission
 *
 * The order is deliberately fixed so the receiving program gets the same
 * packet layout on every transmission, regardless of CAN arrival order.  A
 * row displays 0xHHHHHHHH placeholders until its first complete 8-byte frame.
 * The Python DataProcessor maps the first value to the CAN LOW field and the
 * second value to the CAN HIGH field.  With its endianness set to "little",
 * bytes.fromhex() expects each eight-digit value in raw CAN byte order.  Thus
 * numeric IEEE-754 word 0x4305999A is sent as text 0x9A990543, which Python's
 * struct.unpack("<f", ...) decodes as about 133.6.
 * When adding a new ID, also update the downstream parser if it depends on
 * this row order.
 */
static SunRawEntry_t sun_raw_table[] =
{
  /* Motor controller 1 (base ID 0x400). */
  { 0x402U, "MC1BUS", 0U, 0U, 0U }, /* DC bus current and voltage. */
  { 0x403U, "MC1VEL", 0U, 0U, 0U }, /* Vehicle velocity and motor RPM. */
  { 0x40BU, "MC1TP1", 0U, 0U, 0U }, /* Heatsink and motor temperature. */
  { 0x40CU, "MC1TP2", 0U, 0U, 0U }, /* DSP temperature. */
  { 0x404U, "MC1PHA", 0U, 0U, 0U }, /* Motor phase currents. */
  { 0x40EU, "MC1CUM", 0U, 0U, 0U }, /* Amp-hours and odometer. */
  { 0x405U, "MC1VVC", 0U, 0U, 0U }, /* Motor voltage vector. */
  { 0x406U, "MC1IVC", 0U, 0U, 0U }, /* Motor current vector. */
  { 0x407U, "MC1BEM", 0U, 0U, 0U }, /* Back-EMF vector. */

  /* Motor controller 2 uses the same offsets with base ID 0x420. */
  { 0x422U, "MC2BUS", 0U, 0U, 0U },
  { 0x423U, "MC2VEL", 0U, 0U, 0U },
  { 0x42BU, "MC2TP1", 0U, 0U, 0U },
  { 0x42CU, "MC2TP2", 0U, 0U, 0U },
  { 0x424U, "MC2PHA", 0U, 0U, 0U },
  { 0x42EU, "MC2CUM", 0U, 0U, 0U },
  { 0x425U, "MC2VVC", 0U, 0U, 0U },
  { 0x426U, "MC2IVC", 0U, 0U, 0U },
  { 0x427U, "MC2BEM", 0U, 0U, 0U },

  /* Driver controls. */
  { 0x501U, "DC_DRV",  0U, 0U, 0U }, /* Current/velocity setpoints. */
  { 0x504U, "DC_SWC",  0U, 0U, 0U }, /* Switch position and changes. */

  /* Battery protection system. */
  { 0x581U, "BP_VMX",  0U, 0U, 0U }, /* Maximum cell voltage. */
  { 0x582U, "BP_VMN",  0U, 0U, 0U }, /* Minimum cell voltage. */
  { 0x583U, "BP_TMX",  0U, 0U, 0U }, /* Maximum cell temperature. */
  { 0x585U, "BP_ISH",  0U, 0U, 0U }, /* Shunt current and state of charge. */
  { 0x586U, "BP_PVS",  0U, 0U, 0U }, /* Pack voltage and shunt sum. */

  /* Motor error/limit flags are kept last for legacy packet compatibility. */
  { 0x401U, "MC1LIM", 0U, 0U, 0U },
  { 0x421U, "MC2LIM", 0U, 0U, 0U }
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void LED_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t on);
static HAL_StatusTypeDef CAN_StartListenAll(void);
static void CAN_Drain(void);
#if ENABLE_SWV_DEBUG_OUTPUT
static void CAN_SWV_Task(void);
#endif
static void SD_Task(void);
static void SD_LogSnapshot(void);
static void GPS_Task(void);
static uint8_t GPS_GetField(const char *sentence, uint8_t field_index,
                            char *out, size_t out_len);
static uint8_t GPS_IsNavSentence(const char *sentence);
static uint8_t GPS_ParseCoordDeg(const char *value, const char *hemisphere,
                                 float *coord_deg);
static uint8_t GPS_ParseSpeedMph(const char *sentence, float *speed_mph);
static uint8_t GPS_ParseDateTimeUtc(const char *sentence,
                                    TelemetryDateTime_t *date_time);
static void GPS_UpdateNavFromSentence(const char *sentence);
static void VehicleSpeed_UpdateFromCAN(uint32_t id, uint32_t velocity_word);
static void VehicleSpeed_UpdateFromSources(void);
static void FormatSignedFixed6(float value, char *out, size_t out_len);
static void FormatSignedMilli(int32_t milli_value, char *out, size_t out_len);
static void ESP32_Task(void);
static void ESP32_ProcessRxFrame(const uint8_t *rx, size_t len);
static void ESP32_HandleCommand(char *command);
static void ESP32_ExecuteCommand(const char *id, const char *verb);
static void ESP32_QueueResponse(const char *id, const char *status,
                                const char *message);
static void RS232_Task(void);
static void StatusLED_Task(void);
#if ENABLE_SWV_DEBUG_OUTPUT
static void ErrorFlags_SWV_Task(void);
#endif
static void SetError(uint32_t mask, uint8_t active);
static uint8_t STM32_RTC_Read(TelemetryDateTime_t *date_time);
static uint8_t STM32_RTC_Set(const TelemetryDateTime_t *date_time);
static uint8_t TelemetryDateTime_IsValid(const TelemetryDateTime_t *date_time);
static uint8_t TelemetryDateTime_CalcWeekday(uint16_t year, uint8_t month,
                                             uint8_t day);
static void TelemetryDateTime_Format(const TelemetryDateTime_t *date_time,
                                     char *out, size_t out_len);
static void BoardUptime_Format(uint32_t uptime_ms, char *out, size_t out_len);
#if ENABLE_SWV_DEBUG_OUTPUT
static void STM32_RTC_Task(void);
#endif

#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR || ENABLE_EXTERNAL_PCF85263A_RTC
static void FutureSensors_Init(void);
static void FutureSensors_Task(void);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  /*
   * Newlib printf() eventually calls this function.  The compile-time switch
   * is checked before the ITM registers, so a connected debugger cannot
   * accidentally receive enough trace traffic to overwhelm CubeIDE.  Output
   * is always nonblocking and discarded when SWV is disabled or unavailable.
   */
#if ENABLE_SWV_DEBUG_OUTPUT
  if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U) &&
      ((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0U) && ((ITM->TER & 1UL) != 0U))
  {
    ITM_SendChar((uint32_t)(uint8_t)ch);
  }
#else
  (void)ch;
#endif
  return ch;
}

static void LED_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
  /* Every LED on this board is active-low. */
  HAL_GPIO_WritePin(port, pin, (on != 0U) ? LED_ON : LED_OFF);
}

static void SetError(uint32_t mask, uint8_t active)
{
  /*
   * Each subsystem owns one bit.  Clearing one subsystem therefore cannot
   * accidentally hide an unrelated failure; StatusLED_Task turns red LED 10
   * on whenever at least one bit remains set.
   */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (active != 0U) error_flags |= mask;
  else error_flags &= ~mask;
  __set_PRIMASK(primask);
}

static uint8_t TelemetryDateTime_IsValid(const TelemetryDateTime_t *date_time)
{
  static const uint8_t days_per_month[12] =
      {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
  uint8_t max_day;

  if (date_time == NULL) return 0U;
  if ((date_time->year < 2000U) || (date_time->year > 2099U)) return 0U;
  if ((date_time->month < 1U) || (date_time->month > 12U)) return 0U;
  if (date_time->hour > 23U || date_time->minute > 59U ||
      date_time->second > 59U) return 0U;

  max_day = days_per_month[date_time->month - 1U];
  if ((date_time->month == 2U) && ((date_time->year % 4U) == 0U)) max_day = 29U;
  return ((date_time->day >= 1U) && (date_time->day <= max_day)) ? 1U : 0U;
}

static uint8_t TelemetryDateTime_CalcWeekday(uint16_t year, uint8_t month,
                                             uint8_t day)
{
  /* Sakamoto: result 0=Sunday..6=Saturday; STM32 HAL uses Monday=1..Sunday=7. */
  static const uint8_t table[12] = {0U, 3U, 2U, 5U, 0U, 3U,
                                    5U, 1U, 4U, 6U, 2U, 4U};
  if (month < 3U) year--;
  uint8_t weekday = (uint8_t)((year + year / 4U - year / 100U + year / 400U +
                               table[month - 1U] + day) % 7U);
  return (weekday == 0U) ? RTC_WEEKDAY_SUNDAY : weekday;
}

static uint8_t STM32_RTC_Set(const TelemetryDateTime_t *date_time)
{
#if ENABLE_STM32_INTERNAL_RTC
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};

  if (TelemetryDateTime_IsValid(date_time) == 0U) return 0U;
  time.Hours = date_time->hour;
  time.Minutes = date_time->minute;
  time.Seconds = date_time->second;
  time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time.StoreOperation = RTC_STOREOPERATION_RESET;
  date.WeekDay = TelemetryDateTime_CalcWeekday(date_time->year,
                                               date_time->month,
                                               date_time->day);
  date.Month = date_time->month;
  date.Date = date_time->day;
  date.Year = (uint8_t)(date_time->year - 2000U);

  if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) return 0U;
  if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) return 0U;

  /* With no VBAT this marker and calendar are lost on complete power removal. */
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, STM32_RTC_BACKUP_MAGIC);
  stm32_rtc_ready = 1U;
  return 1U;
#else
  (void)date_time;
  return 0U;
#endif
}

static uint8_t STM32_RTC_Read(TelemetryDateTime_t *date_time)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  if ((date_time == NULL) || (stm32_rtc_ready == 0U)) return 0U;

  /* STM32 requires reading time first and date second to unlock shadow data. */
  if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) return 0U;
  if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) return 0U;

  date_time->year = 2000U + date.Year;
  date_time->month = date.Month;
  date_time->day = date.Date;
  date_time->weekday = date.WeekDay;
  date_time->hour = time.Hours;
  date_time->minute = time.Minutes;
  date_time->second = time.Seconds;
  date_time->valid = 1U;
  return 1U;
}

static void TelemetryDateTime_Format(const TelemetryDateTime_t *date_time,
                                     char *out, size_t out_len)
{
  if ((date_time == NULL) || (date_time->valid == 0U))
  {
    (void)snprintf(out, out_len, "RTC_READ_FAIL");
    return;
  }
  (void)snprintf(out, out_len, "%04u-%02u-%02uT%02u:%02u:%02u",
                 date_time->year, date_time->month, date_time->day,
                 date_time->hour, date_time->minute, date_time->second);
}

static void BoardUptime_Format(uint32_t uptime_ms, char *out, size_t out_len)
{
  uint32_t total_seconds = uptime_ms / 1000U;
  uint32_t milliseconds = uptime_ms % 1000U;
  uint32_t days = total_seconds / 86400U;
  uint32_t hours = (total_seconds / 3600U) % 24U;
  uint32_t minutes = (total_seconds / 60U) % 60U;
  uint32_t seconds = total_seconds % 60U;
  (void)snprintf(out, out_len, "%lu:%02lu:%02lu:%02lu.%03lu",
                 days, hours, minutes, seconds, milliseconds);
}

#if ENABLE_SWV_DEBUG_OUTPUT
static void STM32_RTC_Task(void)
{
#if ENABLE_STM32_INTERNAL_RTC
  uint32_t now = HAL_GetTick();
  if ((now - stm32_rtc_last_print_ms) < 1000U) return;
  stm32_rtc_last_print_ms = now;

  TelemetryDateTime_t date_time = {0};
  char text[24];
  char uptime[24];
  BoardUptime_Format(now, uptime, sizeof(uptime));
  if (STM32_RTC_Read(&date_time) != 0U)
  {
    TelemetryDateTime_Format(&date_time, text, sizeof(text));
    printf("[STM32 RTC] %s clock=HSE_DIV16 time_source=%s gps_syncs=%lu uptime=%s\r\n",
           text, stm32_rtc_time_source, gps_rtc_sync_count, uptime);
  }
  else
  {
    printf("[STM32 RTC] read failed\r\n");
  }
#endif
}
#endif

static uint32_t MakeU32LE(const uint8_t *data)
{
  /*
   * Sunseeker CAN values are carried as two little-endian 32-bit words.
   * For bytes AA BB CC DD this returns 0xDDCCBBAA.
   */
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint32_t ByteSwapU32(uint32_t value)
{
  /*
   * snprintf prints the numeric word most-significant digit first.  Swapping
   * it makes the ASCII byte pairs preserve little-endian CAN wire order for
   * Python's bytes.fromhex() followed by struct.unpack("<f").
   */
  return ((value & 0x000000FFUL) << 24) |
         ((value & 0x0000FF00UL) << 8)  |
         ((value & 0x00FF0000UL) >> 8)  |
         ((value & 0xFF000000UL) >> 24);
}

static void VehicleSpeed_UpdateFromCAN(uint32_t id, uint32_t velocity_word)
{
  /* MC1VEL/MC2VEL bytes 4..7 contain an IEEE-754 velocity in metres/second. */
  float velocity_mps;
  memcpy(&velocity_mps, &velocity_word, sizeof(velocity_mps));
  if (!isfinite(velocity_mps)) return;

  float speed_mph = fabsf(velocity_mps) * MPS_TO_MPH;
  if (speed_mph > CAN_SPEED_MAX_ABS_MPH) return;

  latest_can_speed_mph = speed_mph;
  latest_can_speed_ms = HAL_GetTick();
  latest_can_speed_valid = 1U;
  latest_can_speed_source = (id == MC1_VELOCITY_CAN_ID) ? "CAN_MC1" : "CAN_MC2";
}

static void VehicleSpeed_UpdateFromSources(void)
{
  uint32_t now = HAL_GetTick();

  if (latest_can_speed_valid &&
      ((now - latest_can_speed_ms) > CAN_SPEED_VALID_MS))
  {
    latest_can_speed_valid = 0U;
    latest_can_speed_source = "NONE";
  }
  if (latest_gps_speed_valid &&
      ((now - latest_gps_speed_ms) > GPS_SPEED_VALID_MS))
    latest_gps_speed_valid = 0U;
  if (latest_gps_fix_valid &&
      ((now - latest_gps_fix_ms) > GPS_FIX_VALID_MS))
    latest_gps_fix_valid = 0U;
  if (latest_gps_altitude_valid &&
      ((now - latest_gps_altitude_ms) > GPS_FIX_VALID_MS))
    latest_gps_altitude_valid = 0U;

  if (latest_can_speed_valid)
  {
    latest_vehicle_speed_mph = latest_can_speed_mph;
    latest_vehicle_speed_source = latest_can_speed_source;
  }
  else if (latest_gps_speed_valid)
  {
    latest_vehicle_speed_mph = latest_gps_speed_mph;
    latest_vehicle_speed_source = "GPS";
  }
  else if ((latest_imu_speed_mph > 0.05f) &&
           ((now - latest_imu_speed_ms) <= IMU_SPEED_VALID_MS))
  {
    latest_vehicle_speed_mph = latest_imu_speed_mph;
    latest_vehicle_speed_source = "IMU_INTEGRATED";
  }
  else
  {
    latest_vehicle_speed_mph = 0.0f;
    latest_vehicle_speed_source = "NONE";
  }
}

static void CAN_Store(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
  /* Cache the newest frame for diagnostics/logging and update the legacy table. */
  uint32_t id = (header->IDE == CAN_ID_STD) ? header->StdId : header->ExtId;
  can_latest.tick_ms = HAL_GetTick();
  can_latest.count++;
  can_latest.id = id;
  can_latest.extended = (header->IDE == CAN_ID_EXT) ? 1U : 0U;
  can_latest.dlc = (header->DLC > 8U) ? 8U : (uint8_t)header->DLC;
  memcpy((void *)can_latest.data, data, can_latest.dlc);
  can_led_until = HAL_GetTick() + ACTIVITY_PULSE_MS;
  SetError(ERROR_CAN, 0U);

  if ((header->IDE == CAN_ID_STD) && (header->DLC >= 8U))
  {
    if ((id == MC1_VELOCITY_CAN_ID) || (id == MC2_VELOCITY_CAN_ID))
      VehicleSpeed_UpdateFromCAN(id, MakeU32LE(&data[4]));

    /* Extended IDs and short frames can still be logged, but cannot fill the
       legacy table because its rows require an exact 8-byte standard frame. */
    for (uint32_t i = 0U; i < SUN_RAW_TABLE_COUNT; i++)
    {
      if (sun_raw_table[i].can_id == id)
      {
        /* Store each 32-bit half as little-endian: byte 0 is the least
           significant byte of LOW and byte 4 is the least significant of HIGH. */
        sun_raw_table[i].low_word = MakeU32LE(&data[0]);
        sun_raw_table[i].high_word = MakeU32LE(&data[4]);
        sun_raw_table[i].valid = 1U;
        break;
      }
    }
  }
}

/*
 * Feature-gated future-sensor integration
 * =======================================
 * This section contains the enabled BMI270 implementation plus feature-gated
 * interfaces for the BME280 and external RTC.  The bus handles and pin names
 * document what CubeMX must provide for each device.
 */

#if ENABLE_BME280_SENSOR
/*
 * The feature-gated BME280 callbacks use the same bounded polling policy as
 * the proven BMI270 path. Priority-0 CAN can preempt these short transfers.
 */
static HAL_StatusTypeDef Future_SPI1_Transmit(uint8_t *data, uint16_t len)
{
  return HAL_SPI_Transmit(&hspi1, data, len, BMI270_SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef Future_SPI1_Receive(uint8_t *data, uint16_t len)
{
  return HAL_SPI_Receive(&hspi1, data, len, BMI270_SPI_TIMEOUT_MS);
}
#endif

#if ENABLE_BMI270_IMU
/* Board connection: BMI270 on SPI1 with active-low IMU_CS. */
static struct bmi2_dev future_bmi270;
static float imu_gravity_x_mg;
static float imu_gravity_y_mg;
static float imu_gravity_z_mg = 1000.0f;
static float imu_forward_velocity_mps;
static uint32_t imu_last_velocity_ms;
static uint32_t imu_last_anchor_ms;
static uint8_t imu_last_anchor_source;
static const char *imu_init_stage = "not-started";
static HAL_StatusTypeDef imu_last_hal_status = HAL_ERROR;
static uint32_t imu_last_hal_error;
static uint32_t imu_last_spi_sr;

typedef struct
{
  int32_t ax_mg;
  int32_t ay_mg;
  int32_t az_mg;
  int32_t gx_mdps;
  int32_t gy_mdps;
  int32_t gz_mdps;
} IMU_ConvertedSample_t;

static int8_t Future_BMI270_SPI_Read(uint8_t reg, uint8_t *data,
                                    uint32_t len, void *intf_ptr)
{
  SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)intf_ptr;
  if ((spi != &hspi1) || (data == NULL) || (len == 0U) ||
      (len > UINT16_MAX)) return BMI2_E_COM_FAIL;

  /* The Bosch driver has already set the SPI read bit in reg. */
  uint8_t address = reg;
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_Transmit(spi, &address, 1U,
                                               BMI270_SPI_TIMEOUT_MS);
  if (result == HAL_OK)
    result = HAL_SPI_Receive(spi, data, (uint16_t)len,
                             BMI270_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  imu_last_hal_status = result;
  imu_last_hal_error = HAL_SPI_GetError(spi);
  imu_last_spi_sr = spi->Instance->SR;
  return (result == HAL_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static int8_t Future_BMI270_SPI_Write(uint8_t reg, const uint8_t *data,
                                     uint32_t len, void *intf_ptr)
{
  SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)intf_ptr;
  if ((spi != &hspi1) || (data == NULL) || (len == 0U) ||
      (len > UINT16_MAX)) return BMI2_E_COM_FAIL;

  /* The Bosch driver has already cleared the SPI read bit in reg. */
  uint8_t address = reg;
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_Transmit(spi, &address, 1U,
                                               BMI270_SPI_TIMEOUT_MS);
  if (result == HAL_OK)
    result = HAL_SPI_Transmit(spi, (uint8_t *)data, (uint16_t)len,
                              BMI270_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  imu_last_hal_status = result;
  imu_last_hal_error = HAL_SPI_GetError(spi);
  imu_last_spi_sr = spi->Instance->SR;
  return (result == HAL_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static void Future_BMI270_DelayUs(uint32_t period_us, void *intf_ptr)
{
  (void)intf_ptr;
  if (imu_calibration_active != 0U)
  {
    /* Show progress while Bosch FOC repeatedly invokes this delay callback. */
    LED_Write(Green_LED11_GPIO_Port, Green_LED11_Pin,
              ((HAL_GetTick() % 500U) < 250U) ? 1U : 0U);
  }
  uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  uint32_t start = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - start) < (period_us * cycles_per_us))
  {
  }
}

static uint8_t Future_BMI270_RawRead(uint8_t reg, uint8_t *value)
{
  /* A BMI270 SPI read returns one dummy byte before the register value. */
  uint8_t address = reg | 0x80U;
  uint8_t rx[2] = {0U, 0U};
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, &address, 1U,
                                               BMI270_SPI_TIMEOUT_MS);
  if (result == HAL_OK)
    result = HAL_SPI_Receive(&hspi1, rx, sizeof(rx), BMI270_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  imu_last_hal_status = result;
  imu_last_hal_error = HAL_SPI_GetError(&hspi1);
  imu_last_spi_sr = hspi1.Instance->SR;
  if (result != HAL_OK) return 0U;
  *value = rx[1];
  return 1U;
}

static uint8_t Future_BMI270_Preflight(void)
{
  uint8_t chip_id = 0U;
  uint8_t valid_reads = 0U;
  imu_init_stage = "raw-spi-wakeup";
  (void)Future_BMI270_RawRead(BMI2_CHIP_ID_ADDR, &chip_id);
  imu_init_stage = "raw-chip-id";
  for (uint8_t i = 0U; i < BMI270_RAW_PREFLIGHT_READS; i++)
  {
    if (Future_BMI270_RawRead(BMI2_CHIP_ID_ADDR, &chip_id) &&
        (chip_id == BMI270_CHIP_ID_VALUE)) valid_reads++;
  }
  printf("[IMU PREFLIGHT] valid=%u/%u chip=0x%02X expected=0x%02X HAL=%u HAL_ERR=0x%08lX SR=0x%08lX\r\n",
         valid_reads, BMI270_RAW_PREFLIGHT_READS, chip_id,
         BMI270_CHIP_ID_VALUE, (unsigned int)imu_last_hal_status,
         imu_last_hal_error, imu_last_spi_sr);
  return (valid_reads == BMI270_RAW_PREFLIGHT_READS) ? 1U : 0U;
}

static int8_t Future_BMI270_Init(void)
{
  uint8_t sensors[2] = { BMI2_ACCEL, BMI2_GYRO };
  struct bmi2_sens_config config[2] = {0};
  memset(&future_bmi270, 0, sizeof(future_bmi270));
  future_bmi270.intf = BMI2_SPI_INTF;
  future_bmi270.read = Future_BMI270_SPI_Read;
  future_bmi270.write = Future_BMI270_SPI_Write;
  future_bmi270.delay_us = Future_BMI270_DelayUs;
  future_bmi270.intf_ptr = &hspi1;
  future_bmi270.read_write_len = BMI270_CONFIG_BURST_LENGTH;

  imu_init_stage = "bmi270-init";
  int8_t result = bmi270_init(&future_bmi270);
  printf("[IMU INIT] stage=%s result=%d chip=0x%02X load=0x%02X\r\n",
         imu_init_stage, result, future_bmi270.chip_id,
         future_bmi270.load_status);
  if (result != BMI2_OK) goto init_failed;

  config[0].type = BMI2_ACCEL;
  config[1].type = BMI2_GYRO;
  config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
  config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  imu_init_stage = "set-config";
  result = bmi2_set_sensor_config(config, 2U, &future_bmi270);
  printf("[IMU INIT] stage=%s result=%d\r\n", imu_init_stage, result);
  if (result != BMI2_OK) goto init_failed;

  imu_init_stage = "enable-accel-gyro";
  result = bmi270_sensor_enable(sensors, 2U, &future_bmi270);
  printf("[IMU INIT] stage=%s result=%d\r\n", imu_init_stage, result);
  if (result != BMI2_OK) goto init_failed;

  HAL_Delay(50U);
  imu_init_stage = "active";
  return BMI2_OK;

init_failed:
  printf("[IMU INIT FAIL] stage=%s result=%d HAL=%u HAL_ERR=0x%08lX SR=0x%08lX\r\n",
         imu_init_stage, result, (unsigned int)imu_last_hal_status,
         imu_last_hal_error, imu_last_spi_sr);
  return result;
}

static int32_t Future_BMI270_AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static void Future_BMI270_ConvertSample(const struct bmi2_sens_data *sample,
                                        IMU_ConvertedSample_t *converted)
{
  /* BMI270 scale factors for the configured +/-4 g and +/-2000 deg/s ranges. */
  converted->ax_mg = ((int32_t)sample->acc.x * 4000L) / 32768L;
  converted->ay_mg = ((int32_t)sample->acc.y * 4000L) / 32768L;
  converted->az_mg = ((int32_t)sample->acc.z * 4000L) / 32768L;
  converted->gx_mdps =
      (int32_t)(((int64_t)sample->gyr.x * 2000000LL) / 32768LL);
  converted->gy_mdps =
      (int32_t)(((int64_t)sample->gyr.y * 2000000LL) / 32768LL);
  converted->gz_mdps =
      (int32_t)(((int64_t)sample->gyr.z * 2000000LL) / 32768LL);
}

static int8_t Future_BMI270_CaptureRestGravity(void)
{
  /*
   * Average one stationary window to learn the gravity vector in board axes.
   * Reject the window if rotation is present or acceleration is not close to
   * 1 g. The installed board is vertical with its left/pin-1 side forward:
   * X is vertical, +Y is vehicle-forward, and Z is vehicle-lateral.
   */
  imu_mount_valid = 0U;
  int64_t sum_x_mg = 0;
  int64_t sum_y_mg = 0;
  int64_t sum_z_mg = 0;

  for (uint32_t i = 0U; i < IMU_CALIBRATION_SAMPLES; i++)
  {
    struct bmi2_sens_data sample = {0};
    IMU_ConvertedSample_t converted;
    int8_t result = bmi2_get_sensor_data(&sample, &future_bmi270);
    if (result != BMI2_OK) return result;
    Future_BMI270_ConvertSample(&sample, &converted);

    float magnitude_mg = sqrtf((float)converted.ax_mg * converted.ax_mg +
                                (float)converted.ay_mg * converted.ay_mg +
                                (float)converted.az_mg * converted.az_mg);
    int32_t gyro_motion_mdps = Future_BMI270_AbsI32(converted.gx_mdps) +
                               Future_BMI270_AbsI32(converted.gy_mdps) +
                               Future_BMI270_AbsI32(converted.gz_mdps);
    if ((gyro_motion_mdps >= IMU_STILL_GYRO_LIMIT_MDPS) ||
        (fabsf(magnitude_mg - 1000.0f) >= IMU_STILL_ACC_TOL_MG))
      return BMI2_E_PRECON_ERROR;

    sum_x_mg += converted.ax_mg;
    sum_y_mg += converted.ay_mg;
    sum_z_mg += converted.az_mg;
    if ((i + 1U) < IMU_CALIBRATION_SAMPLES)
      HAL_Delay(IMU_CALIBRATION_SAMPLE_MS);
  }

  float gravity_x_mg = (float)sum_x_mg / (float)IMU_CALIBRATION_SAMPLES;
  float gravity_y_mg = (float)sum_y_mg / (float)IMU_CALIBRATION_SAMPLES;
  float gravity_z_mg = (float)sum_z_mg / (float)IMU_CALIBRATION_SAMPLES;

  /*
   * Refuse a flat-on-the-bench calibration: it would learn roughly 1 g on Z
   * and later report about 1 g of false dynamic acceleration when installed.
   * The limits allow approximately 20 degrees of mounting/vehicle inclination.
   */
  if ((Future_BMI270_AbsI32((int32_t)gravity_x_mg) <
       IMU_MOUNT_VERTICAL_MIN_MG) ||
      (Future_BMI270_AbsI32((int32_t)gravity_y_mg) >
       IMU_MOUNT_FORWARD_MAX_MG) ||
      (Future_BMI270_AbsI32((int32_t)gravity_z_mg) >
       IMU_MOUNT_LATERAL_MAX_MG))
  {
    imu_calibration_wrong_mount = 1U;
    printf("[IMU CAL] wrong mount: X=%ld Y=%ld Z=%ld mg; board must be vertical, left edge forward\r\n",
           (long)gravity_x_mg, (long)gravity_y_mg, (long)gravity_z_mg);
    return BMI2_E_PRECON_ERROR;
  }

  imu_gravity_x_mg = gravity_x_mg;
  imu_gravity_y_mg = gravity_y_mg;
  imu_gravity_z_mg = gravity_z_mg;
  imu_mount_valid = 1U;
  return BMI2_OK;
}

static int8_t Future_BMI270_CalibrateAtRest(void)
{
  /*
   * Bosch gyro FOC collects 128 samples at 25 Hz and takes about 6.4 seconds.
   * A rest check is performed before it, and a second stationary window after
   * it establishes the accelerometer gravity baseline used for dynamic g.
  */
  imu_calibrated = 0U;
  imu_mount_valid = 0U;
  imu_calibration_wrong_mount = 0U;
  imu_telemetry.valid = 0U;
  imu_calibration_active = 1U;
  HAL_Delay(50U);
  int8_t result = Future_BMI270_CaptureRestGravity();
  if (result == BMI2_OK) result = bmi2_perform_gyro_foc(&future_bmi270);
  if (result == BMI2_OK)
  {
    HAL_Delay(50U);
    result = Future_BMI270_CaptureRestGravity();
  }
  if (result == BMI2_OK)
  {
    imu_calibrated = 1U;
    latest_imu_speed_mph = 0.0f;
    imu_forward_velocity_mps = 0.0f;
    imu_last_velocity_ms = HAL_GetTick();
    imu_last_anchor_ms = 0U;
    imu_last_anchor_source = IMU_ANCHOR_NONE;
    imu_telemetry.peak_window_g_mg = 0U;
    imu_telemetry.peak_boot_g_mg = 0U;
  }
  imu_calibration_active = 0U;
  LED_Write(Green_LED11_GPIO_Port, Green_LED11_Pin, 0U);
  return result;
}

static int8_t Future_BMI270_StartAndCalibrate(void)
{
  /* Follow the known-working power-up, raw-ID, Bosch-init, then FOC sequence. */
  imu_ready = 0U;
  imu_calibrated = 0U;
  HAL_Delay(BMI270_STARTUP_DELAY_MS);
  if (Future_BMI270_Preflight() == 0U) return BMI2_E_DEV_NOT_FOUND;

  int8_t result = Future_BMI270_Init();
  if (result != BMI2_OK) return result;
  imu_ready = 1U;

  /* The vehicle must remain stationary during the roughly 7-second FOC. */
  result = Future_BMI270_CalibrateAtRest();
  return result;
}

static void Future_BMI270_UpdateSpeed(const struct bmi2_sens_data *sample)
{
  /*
   * Integrate only the configured forward axis for fast short-term speed
   * response. Fresh CAN/GPS samples periodically re-anchor this estimate;
   * pure accelerometer integration is not trusted over long distances.
   */
  IMU_ConvertedSample_t converted;
  Future_BMI270_ConvertSample(sample, &converted);
  float magnitude_mg = sqrtf((float)converted.ax_mg * converted.ax_mg +
                              (float)converted.ay_mg * converted.ay_mg +
                              (float)converted.az_mg * converted.az_mg);
  uint8_t still = ((Future_BMI270_AbsI32(converted.gx_mdps) +
                    Future_BMI270_AbsI32(converted.gy_mdps) +
                    Future_BMI270_AbsI32(converted.gz_mdps)) <
                   IMU_STILL_GYRO_LIMIT_MDPS) &&
                  (fabsf(magnitude_mg - 1000.0f) < IMU_STILL_ACC_TOL_MG);
  uint32_t now = HAL_GetTick();
  uint32_t dt_ms = now - imu_last_velocity_ms;
  if ((imu_last_velocity_ms == 0U) || (dt_ms > IMU_MAX_DT_MS)) dt_ms = 0U;
  imu_last_velocity_ms = now;

  int32_t linear_x_mg = (int32_t)((float)converted.ax_mg - imu_gravity_x_mg);
  int32_t linear_y_mg = (int32_t)((float)converted.ay_mg - imu_gravity_y_mg);
  int32_t linear_z_mg = (int32_t)((float)converted.az_mg - imu_gravity_z_mg);
  int32_t filtered_x_mg =
      (Future_BMI270_AbsI32(linear_x_mg) > IMU_ACCEL_DEADBAND_MG) ?
      linear_x_mg : 0L;
  int32_t filtered_y_mg =
      (Future_BMI270_AbsI32(linear_y_mg) > IMU_ACCEL_DEADBAND_MG) ?
      linear_y_mg : 0L;
  int32_t filtered_z_mg =
      (Future_BMI270_AbsI32(linear_z_mg) > IMU_ACCEL_DEADBAND_MG) ?
      linear_z_mg : 0L;

#if IMU_FORWARD_AXIS == IMU_AXIS_X
  int32_t forward_accel_mg = IMU_FORWARD_SIGN * filtered_x_mg;
#elif IMU_FORWARD_AXIS == IMU_AXIS_Y
  int32_t forward_accel_mg = IMU_FORWARD_SIGN * filtered_y_mg;
#else
  int32_t forward_accel_mg = IMU_FORWARD_SIGN * filtered_z_mg;
#endif

  /* Select the freshest authoritative source and apply each sample once. */
  uint8_t anchor_source = IMU_ANCHOR_NONE;
  uint32_t anchor_ms = 0U;
  float anchor_speed_mph = 0.0f;
  if (latest_can_speed_valid &&
      ((now - latest_can_speed_ms) <= CAN_SPEED_VALID_MS))
  {
    anchor_source = IMU_ANCHOR_CAN;
    anchor_ms = latest_can_speed_ms;
    anchor_speed_mph = latest_can_speed_mph;
  }
  else if (latest_gps_speed_valid &&
           ((now - latest_gps_speed_ms) <= GPS_SPEED_VALID_MS))
  {
    anchor_source = IMU_ANCHOR_GPS;
    anchor_ms = latest_gps_speed_ms;
    anchor_speed_mph = latest_gps_speed_mph;
  }

  uint8_t new_anchor = 0U;
  if ((imu_calibrated != 0U) && (anchor_source != IMU_ANCHOR_NONE) &&
      ((imu_last_anchor_source == IMU_ANCHOR_NONE) ||
       ((int32_t)(anchor_ms - imu_last_anchor_ms) > 0)))
  {
    imu_forward_velocity_mps = anchor_speed_mph / MPS_TO_MPH;
    imu_last_anchor_ms = anchor_ms;
    imu_last_anchor_source = anchor_source;
    new_anchor = 1U;
  }

  if ((imu_calibrated != 0U) && (dt_ms != 0U))
  {
    float dt_s = (float)dt_ms / 1000.0f;
    imu_forward_velocity_mps +=
        (float)forward_accel_mg * 0.00980665f * dt_s;

    /* A gentle leak limits drift only while no forward acceleration is seen. */
    if ((forward_accel_mg == 0L) && (new_anchor == 0U))
      imu_forward_velocity_mps *= IMU_SPEED_COAST_RETENTION;
  }

  if ((new_anchor != 0U) && still &&
      (anchor_speed_mph < IMU_ZERO_SPEED_THRESHOLD_MPH))
    imu_forward_velocity_mps = 0.0f;
  if (imu_forward_velocity_mps < 0.0f) imu_forward_velocity_mps = 0.0f;

  latest_imu_speed_mph = imu_forward_velocity_mps * MPS_TO_MPH;
  latest_imu_speed_ms = now;
  if (latest_imu_speed_mph > CAN_SPEED_MAX_ABS_MPH)
  {
    latest_imu_speed_mph = CAN_SPEED_MAX_ABS_MPH;
    imu_forward_velocity_mps = CAN_SPEED_MAX_ABS_MPH / MPS_TO_MPH;
  }

  float dynamic_mg = sqrtf((float)filtered_x_mg * filtered_x_mg +
                            (float)filtered_y_mg * filtered_y_mg +
                            (float)filtered_z_mg * filtered_z_mg);
  imu_telemetry.acc_x_raw = sample->acc.x;
  imu_telemetry.acc_y_raw = sample->acc.y;
  imu_telemetry.acc_z_raw = sample->acc.z;
  imu_telemetry.gyro_x_raw = sample->gyr.x;
  imu_telemetry.gyro_y_raw = sample->gyr.y;
  imu_telemetry.gyro_z_raw = sample->gyr.z;
  imu_telemetry.acc_x_mg = converted.ax_mg;
  imu_telemetry.acc_y_mg = converted.ay_mg;
  imu_telemetry.acc_z_mg = converted.az_mg;
  imu_telemetry.gyro_x_mdps = converted.gx_mdps;
  imu_telemetry.gyro_y_mdps = converted.gy_mdps;
  imu_telemetry.gyro_z_mdps = converted.gz_mdps;
  imu_telemetry.linear_x_mg = linear_x_mg;
  imu_telemetry.linear_y_mg = linear_y_mg;
  imu_telemetry.linear_z_mg = linear_z_mg;
  imu_telemetry.forward_accel_mg = forward_accel_mg;
  imu_telemetry.total_g_mg = (uint32_t)(magnitude_mg + 0.5f);
  imu_telemetry.dynamic_g_mg = (uint32_t)(dynamic_mg + 0.5f);
  if (imu_telemetry.dynamic_g_mg > imu_telemetry.peak_window_g_mg)
    imu_telemetry.peak_window_g_mg = imu_telemetry.dynamic_g_mg;
  if (imu_telemetry.dynamic_g_mg > imu_telemetry.peak_boot_g_mg)
    imu_telemetry.peak_boot_g_mg = imu_telemetry.dynamic_g_mg;
  imu_telemetry.sample_ms = now;
  imu_telemetry.sample_count++;
  imu_telemetry.valid = 1U;
}
#endif /* ENABLE_BMI270_IMU */

#if ENABLE_BME280_SENSOR
/* Board connection: BME280 shares SPI1 and has its own active-low BME_CS. */
static struct bme280_dev future_bme280;

static int8_t Future_BME280_SPI_Read(uint8_t reg, uint8_t *data,
                                     uint32_t len, void *intf_ptr)
{
  (void)intf_ptr;
  uint8_t address = reg | 0x80U;
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = Future_SPI1_Transmit(&address, 1U);
  if (result == HAL_OK)
    result = Future_SPI1_Receive(data, (uint16_t)len);
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  return (result == HAL_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

static int8_t Future_BME280_SPI_Write(uint8_t reg, const uint8_t *data,
                                      uint32_t len, void *intf_ptr)
{
  (void)intf_ptr;
  uint8_t address = reg & 0x7FU;
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = Future_SPI1_Transmit(&address, 1U);
  if (result == HAL_OK)
    result = Future_SPI1_Transmit((uint8_t *)data, (uint16_t)len);
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  return (result == HAL_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

static void Future_BME280_DelayUs(uint32_t period_us, void *intf_ptr)
{
  (void)intf_ptr;
  HAL_Delay((period_us + 999U) / 1000U);
}

static int8_t Future_BME280_Init(void)
{
  struct bme280_settings settings = {0};
  memset(&future_bme280, 0, sizeof(future_bme280));
  future_bme280.intf = BME280_SPI_INTF;
  future_bme280.read = Future_BME280_SPI_Read;
  future_bme280.write = Future_BME280_SPI_Write;
  future_bme280.delay_us = Future_BME280_DelayUs;

  int8_t result = bme280_init(&future_bme280);
  if (result != BME280_OK) return result;
  settings.osr_h = BME280_OVERSAMPLING_1X;
  settings.osr_p = BME280_OVERSAMPLING_1X;
  settings.osr_t = BME280_OVERSAMPLING_1X;
  settings.filter = BME280_FILTER_COEFF_OFF;
  settings.standby_time = BME280_STANDBY_TIME_1000_MS;
  result = bme280_set_sensor_settings(BME280_SEL_OSR_TEMP |
                                      BME280_SEL_OSR_PRESS |
                                      BME280_SEL_OSR_HUM |
                                      BME280_SEL_FILTER |
                                      BME280_SEL_STANDBY,
                                      &settings, &future_bme280);
  return (result == BME280_OK) ?
         bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &future_bme280) : result;
}
#endif /* ENABLE_BME280_SENSOR */

#if ENABLE_EXTERNAL_PCF85263A_RTC
/*
 * PCF85263A RTC path using interrupt-driven I2C1 transfers.  STM32 HAL expects
 * the 7-bit 0x51 target address shifted left by one.  The device remains
 * feature-gated until the I2C bus, backup supply, and 32.768 kHz crystal have
 * been validated on hardware.
 */
#define FUTURE_PCF85263A_ADDRESS       (0x51U << 1)
#define FUTURE_PCF85263A_TIME_START    0x00U
#define FUTURE_PCF85263A_OSCILLATOR    0x25U
#define FUTURE_PCF85263A_FUNCTION      0x28U
#define FUTURE_PCF85263A_STOP_ENABLE   0x2EU
#define FUTURE_PCF85263A_CLEAR_PRESCALER 0xA4U
#define FUTURE_PCF85263A_RTCM          (1U << 4)
#define FUTURE_PCF85263A_STOPM         (1U << 3)
#define FUTURE_PCF85263A_12_HOUR       (1U << 5)
static volatile uint8_t future_i2c1_done;
static volatile uint8_t future_i2c1_error;

static HAL_StatusTypeDef Future_I2C1_Wait(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while ((future_i2c1_done == 0U) && (future_i2c1_error == 0U))
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      (void)HAL_I2C_Master_Abort_IT(&hi2c1, FUTURE_PCF85263A_ADDRESS);
      return HAL_TIMEOUT;
    }
    __WFI();
  }
  return (future_i2c1_error == 0U) ? HAL_OK : HAL_ERROR;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) future_i2c1_done = 1U;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) future_i2c1_done = 1U;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) future_i2c1_error = 1U;
}

static HAL_StatusTypeDef Future_PCF85263A_Write(uint8_t reg,
                                                uint8_t *data,
                                                uint16_t length)
{
  future_i2c1_done = 0U;
  future_i2c1_error = 0U;
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write_IT(&hi2c1,
      FUTURE_PCF85263A_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length);
  if (result == HAL_OK) result = Future_I2C1_Wait(100U);
  return result;
}

static HAL_StatusTypeDef Future_PCF85263A_Read(uint8_t reg,
                                               uint8_t *data,
                                               uint16_t length)
{
  future_i2c1_done = 0U;
  future_i2c1_error = 0U;
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read_IT(&hi2c1,
      FUTURE_PCF85263A_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length);
  if (result == HAL_OK) result = Future_I2C1_Wait(100U);
  return result;
}

static uint8_t Future_PCF85263A_Init(void)
{
  uint8_t function;
  uint8_t oscillator;
  uint8_t run = 0U;

  if (HAL_I2C_IsDeviceReady(&hi2c1, FUTURE_PCF85263A_ADDRESS,
                            3U, 100U) != HAL_OK) return 0U;

  /* Select calendar RTC mode, STOP controlled only by register, and 24 hours. */
  if (Future_PCF85263A_Read(FUTURE_PCF85263A_FUNCTION,
                            &function, 1U) != HAL_OK) return 0U;
  function &= (uint8_t)~(FUTURE_PCF85263A_RTCM |
                         FUTURE_PCF85263A_STOPM);
  if (Future_PCF85263A_Write(FUTURE_PCF85263A_FUNCTION,
                             &function, 1U) != HAL_OK) return 0U;

  if (Future_PCF85263A_Read(FUTURE_PCF85263A_OSCILLATOR,
                            &oscillator, 1U) != HAL_OK) return 0U;
  oscillator &= (uint8_t)~FUTURE_PCF85263A_12_HOUR;
  if (Future_PCF85263A_Write(FUTURE_PCF85263A_OSCILLATOR,
                             &oscillator, 1U) != HAL_OK) return 0U;

  /* Do not reset a retained calendar; only ensure its counter is running. */
  return (Future_PCF85263A_Write(FUTURE_PCF85263A_STOP_ENABLE,
                                 &run, 1U) == HAL_OK) ? 1U : 0U;
}

static uint8_t Future_PCF85263A_ReadRaw(uint8_t registers[8])
{
  /* One coherent read: 100ths, seconds, minutes, hours, day, weekday, month, year. */
  HAL_StatusTypeDef result = Future_PCF85263A_Read(
      FUTURE_PCF85263A_TIME_START, registers, 8U);
  return (result == HAL_OK) ? 1U : 0U;
}

static uint8_t Future_PCF85263A_BinToBcd(uint8_t value)
{
  return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

static uint8_t Future_PCF85263A_Set(const TelemetryDateTime_t *date_time)
{
  /*
   * The PCF85263A requires STOP, clear-prescaler, and all eight clock/calendar
   * bytes to be written as one access. Starting at 0x2E uses the documented
   * address rollover 0x2E, 0x2F, 0x00..0x07. A final write clears STOP and
   * starts time counting. This remains compiled out while the feature is 0.
   */
  uint8_t registers[10];
  uint8_t stm_weekday;
  uint8_t run = 0U;

  if (TelemetryDateTime_IsValid(date_time) == 0U) return 0U;
  stm_weekday = TelemetryDateTime_CalcWeekday(date_time->year,
                                               date_time->month,
                                               date_time->day);
  registers[0] = 1U; /* STOP */
  registers[1] = FUTURE_PCF85263A_CLEAR_PRESCALER;
  registers[2] = 0U; /* hundredths */
  registers[3] = Future_PCF85263A_BinToBcd(date_time->second); /* OS cleared */
  registers[4] = Future_PCF85263A_BinToBcd(date_time->minute);
  registers[5] = Future_PCF85263A_BinToBcd(date_time->hour);
  registers[6] = Future_PCF85263A_BinToBcd(date_time->day);
  registers[7] = (stm_weekday == RTC_WEEKDAY_SUNDAY) ? 0U : stm_weekday;
  registers[8] = Future_PCF85263A_BinToBcd(date_time->month);
  registers[9] = Future_PCF85263A_BinToBcd(
      (uint8_t)(date_time->year - 2000U));

  if (Future_PCF85263A_Write(FUTURE_PCF85263A_STOP_ENABLE,
                             registers, sizeof(registers)) != HAL_OK) return 0U;
  return (Future_PCF85263A_Write(FUTURE_PCF85263A_STOP_ENABLE,
                                 &run, 1U) == HAL_OK) ? 1U : 0U;
}
#endif /* ENABLE_EXTERNAL_PCF85263A_RTC */

#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR || ENABLE_EXTERNAL_PCF85263A_RTC
static void FutureSensors_Init(void)
{
#if ENABLE_BMI270_IMU
  imu_last_result = Future_BMI270_StartAndCalibrate();
  imu_last_retry_ms = HAL_GetTick();
  SetError(ERROR_IMU, (imu_ready && imu_calibrated) ? 0U : 1U);
  printf("[IMU BOOT] ready=%u calibrated=%u mount_valid=%u result=%d\r\n",
         imu_ready, imu_calibrated, imu_mount_valid, imu_last_result);
#endif
#if ENABLE_BME280_SENSOR
  printf("[FUTURE BME] init result=%d\r\n", Future_BME280_Init());
#endif
#if ENABLE_EXTERNAL_PCF85263A_RTC
  printf("[FUTURE RTC] PCF85263A init result=%u\r\n",
         Future_PCF85263A_Init());
#endif
}

static void FutureSensors_Task(void)
{
  static uint32_t last_slow_ms;
  uint32_t now = HAL_GetTick();

#if ENABLE_BMI270_IMU
  static uint32_t last_imu_ms;
  static uint32_t last_imu_print_ms;
  if ((imu_ready == 0U) &&
      ((now - imu_last_retry_ms) >= BMI270_RETRY_PERIOD_MS))
  {
    imu_last_retry_ms = now;
    imu_last_result = Future_BMI270_StartAndCalibrate();
    SetError(ERROR_IMU, (imu_ready && imu_calibrated) ? 0U : 1U);
    printf("[IMU RETRY] ready=%u calibrated=%u mount_valid=%u result=%d\r\n",
           imu_ready, imu_calibrated, imu_mount_valid, imu_last_result);
  }
  if ((now - last_imu_ms) >= IMU_SAMPLE_PERIOD_MS)
  {
    last_imu_ms = now;
    struct bmi2_sens_data imu_data = {0};
    int8_t sample_result = imu_ready ?
        bmi2_get_sensor_data(&imu_data, &future_bmi270) : BMI2_E_COM_FAIL;
    if (imu_ready && (sample_result == BMI2_OK))
    {
      Future_BMI270_UpdateSpeed(&imu_data);
      SetError(ERROR_IMU, (imu_calibrated != 0U) ? 0U : 1U);
      if ((now - last_imu_print_ms) >= 1000U)
      {
        last_imu_print_ms = now;
        uint32_t imu_hundredths =
            (uint32_t)(latest_imu_speed_mph * 100.0f + 0.5f);
        printf("[IMU] cal=%u acc=%d,%d,%d gyro=%d,%d,%d forward_mg=%ld dynamic_g=%lu.%03lu speed=%lu.%02lu mph\r\n",
               imu_calibrated,
               imu_data.acc.x, imu_data.acc.y, imu_data.acc.z,
               imu_data.gyr.x, imu_data.gyr.y, imu_data.gyr.z,
               (long)imu_telemetry.forward_accel_mg,
               imu_telemetry.dynamic_g_mg / 1000U,
               imu_telemetry.dynamic_g_mg % 1000U,
               imu_hundredths / 100U, imu_hundredths % 100U);
      }
    }
    else if (imu_ready != 0U)
    {
      imu_last_result = sample_result;
      imu_ready = 0U;
      imu_calibrated = 0U;
      imu_telemetry.valid = 0U;
      imu_telemetry.read_error_count++;
      SetError(ERROR_IMU, 1U);
      printf("[IMU SAMPLE FAIL] result=%d HAL=%u HAL_ERR=0x%08lX SR=0x%08lX\r\n",
             sample_result, (unsigned int)imu_last_hal_status,
             imu_last_hal_error, imu_last_spi_sr);
    }
  }
#endif

  if ((now - last_slow_ms) < 1000U) return;
  last_slow_ms = now;
#if ENABLE_BME280_SENSOR
  struct bme280_data bme_data = {0};
  if (bme280_get_sensor_data(BME280_ALL, &bme_data, &future_bme280) == BME280_OK)
    printf("[FUTURE BME] sample received\r\n");
#endif
#if ENABLE_EXTERNAL_PCF85263A_RTC
  uint8_t registers[8];
  if (Future_PCF85263A_ReadRaw(registers))
    printf("[FUTURE RTC] PCF85263A raw=%02X:%02X:%02X date=%02X/%02X/%02X\r\n",
           registers[3], registers[2], registers[1],
           registers[6], registers[4], registers[7]);
#endif
}
#endif

#if ENABLE_SWV_DEBUG_OUTPUT
static void CAN_SWV_Task(void)
{
  /*
   * Printing inside the CAN interrupt would make the interrupt too slow.
   * Instead, take an atomic copy here in the main loop and print the newest
   * frame.  The 100 ms limit caps SWV traffic at 10 lines/second; "skipped"
   * reports how many faster frames arrived between displayed snapshots.
   */
  static uint32_t last_print_ms;
  static uint32_t last_status_ms;
  static uint32_t last_print_count;
  uint32_t now = HAL_GetTick();

  if ((can_latest.count != last_print_count) &&
      ((now - last_print_ms) >= CAN_SWV_MIN_PERIOD_MS))
  {
    CAN_Snapshot_t snapshot;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    snapshot.tick_ms = can_latest.tick_ms;
    snapshot.count = can_latest.count;
    snapshot.id = can_latest.id;
    snapshot.extended = can_latest.extended;
    snapshot.dlc = can_latest.dlc;
    for (uint8_t i = 0U; i < 8U; i++) snapshot.data[i] = can_latest.data[i];
    __set_PRIMASK(primask);

    printf("[CAN RX] count=%lu skipped=%lu id=0x%lX type=%s dlc=%u data=",
           snapshot.count, snapshot.count - last_print_count - 1U,
           snapshot.id, snapshot.extended ? "EXT" : "STD", snapshot.dlc);
    for (uint8_t i = 0U; i < snapshot.dlc; i++)
      printf("%02X%s", snapshot.data[i], (i + 1U < snapshot.dlc) ? " " : "");
    printf(" age=%lu ms\r\n", now - snapshot.tick_ms);

    last_print_count = snapshot.count;
    last_print_ms = now;
  }

  /* A periodic line proves CAN is still being serviced even if no frames arrive. */
  if ((now - last_status_ms) >= CAN_SWV_STATUS_PERIOD_MS)
  {
    last_status_ms = now;
    printf("[CAN STATUS] rx=%lu state=%lu hal_error=0x%08lX ESR=0x%08lX\r\n",
           can_latest.count, (uint32_t)HAL_CAN_GetState(&hcan1),
           HAL_CAN_GetError(&hcan1), hcan1.Instance->ESR);
  }
}
#endif

static void CAN_Drain(void)
{
  /*
   * Empty FIFO0 completely from the CAN RX interrupt so bursts do not leave
   * old frames queued.  Keeping all HAL FIFO access in this interrupt path
   * avoids the main loop and ISR trying to remove a message at the same time.
   */
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      SetError(ERROR_CAN, 1U);
      break;
    }
    CAN_Store(&header, data);
  }
}

HAL_StatusTypeDef Telemetry_CAN_SendCommand(uint8_t command,
                                            const uint8_t *payload,
                                            uint8_t payload_length)
{
  /*
   * This function only queues a frame in a free bxCAN transmit mailbox; it
   * does not wait for the frame to leave the bus.  That keeps future command
   * sends from delaying GPS, RS232, SD-card, or ESP32 servicing.
   *
   * Wire format for standard ID TELEMETRY_CAN_COMMAND_ID (0x5C0):
   *   byte 0    command code
   *   byte 1..7 optional command payload (maximum seven bytes)
   * Multi-byte payload values are packed by the caller in little-endian order.
   */
  CAN_TxHeaderTypeDef header = {0};
  uint8_t data[8] = {0};
  uint32_t mailbox;

  if ((payload_length > 7U) || ((payload_length != 0U) && (payload == NULL)))
    return HAL_ERROR;

  if (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_LISTENING)
    return HAL_ERROR;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    return HAL_BUSY;

  header.StdId = TELEMETRY_CAN_COMMAND_ID;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = payload_length + 1U;
  header.TransmitGlobalTime = DISABLE;

  data[0] = command;
  for (uint8_t i = 0U; i < payload_length; i++) data[i + 1U] = payload[i];

  HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
  if (result == HAL_ERROR) SetError(ERROR_CAN, 1U);
  return result;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* HAL invokes this callback from CAN1_RX0_IRQHandler(). */
  if (hcan->Instance == CAN1) CAN_Drain();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1) SetError(ERROR_CAN, 1U);
}

static HAL_StatusTypeDef CAN_StartListenAll(void)
{
  /* ID=0 with mask=0 accepts every standard and extended CAN identifier. */
  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) return HAL_ERROR;
  if (HAL_CAN_Start(&hcan1) != HAL_OK) return HAL_ERROR;
  return HAL_CAN_ActivateNotification(&hcan1,
      CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR);
}

static void SD_LogSnapshot(void)
{
  /*
   * Open/append/sync/close once per row.  f_sync() and f_close() cost time,
   * but make a completed row much more likely to survive unexpected power
   * loss, which is important for vehicle telemetry.
   */
  /* New filename prevents the expanded IMU rows mixing with an older header. */
  static const char path[] = "0:/TELIMU2.CSV";
  static const char header[] =
      "tick_ms,stm32_rtc,rtc_valid,can_count,can_id,extended,dlc,data,gps_fix_valid,"
      "gps_lat_deg,gps_lon_deg,gps_speed_valid,gps_speed_mph,"
      "vehicle_speed_mph,vehicle_speed_source,"
      "imu_speed_mph,imu_forward_accel_mg,"
      "imu_ready,imu_calibrated,imu_valid,imu_sample_ms,imu_age_ms,"
      "imu_sample_count,imu_read_errors,"
      "imu_acc_x_raw,imu_acc_y_raw,imu_acc_z_raw,"
      "imu_gyro_x_raw,imu_gyro_y_raw,imu_gyro_z_raw,"
      "imu_acc_x_mg,imu_acc_y_mg,imu_acc_z_mg,"
      "imu_gyro_x_mdps,imu_gyro_y_mdps,imu_gyro_z_mdps,"
      "imu_linear_x_mg,imu_linear_y_mg,imu_linear_z_mg,"
      "imu_total_g_mg,imu_dynamic_g_mg,imu_peak_window_g_mg,"
      "imu_peak_boot_g_mg\r\n";
  char hex[17] = {0};
  char lat[20];
  char lon[20];
  char rtc_text[24];
  TelemetryDateTime_t date_time = {0};
  UINT written = 0U;
  FRESULT result = f_open(&USERFile, path, FA_OPEN_ALWAYS | FA_WRITE);
  if (result == FR_OK)
  {
    if (f_size(&USERFile) == 0U)
      /* A newly-created file gets column names before its first data row. */
      result = f_write(&USERFile, header, sizeof(header) - 1U, &written);
    if (result == FR_OK) result = f_lseek(&USERFile, f_size(&USERFile));
    for (uint8_t i = 0U; i < can_latest.dlc && i < 8U; i++)
      (void)snprintf(&hex[i * 2U], 3U, "%02X", can_latest.data[i]);
    FormatSignedFixed6(latest_gps_lat_deg, lat, sizeof(lat));
    FormatSignedFixed6(latest_gps_lon_deg, lon, sizeof(lon));
    (void)STM32_RTC_Read(&date_time);
    TelemetryDateTime_Format(&date_time, rtc_text, sizeof(rtc_text));
    VehicleSpeed_UpdateFromSources();
    uint32_t gps_hundredths = (uint32_t)(latest_gps_speed_mph * 100.0f + 0.5f);
    uint32_t vehicle_hundredths =
        (uint32_t)(latest_vehicle_speed_mph * 100.0f + 0.5f);
    uint32_t imu_hundredths =
        (uint32_t)(latest_imu_speed_mph * 100.0f + 0.5f);
    uint32_t imu_age_ms = imu_telemetry.valid ?
        (HAL_GetTick() - imu_telemetry.sample_ms) : 0U;
    uint32_t imu_peak_window_g_mg = imu_telemetry.peak_window_g_mg;
    if (result == FR_OK && f_printf(&USERFile,
        "%lu,%s,%u,%lu,0x%lX,%u,%u,%s,%u,%s,%s,%u,%lu.%02lu,"
        "%lu.%02lu,%s,%lu.%02lu,%ld,%u,%u,%u,%lu,%lu,%lu,%lu,"
        "%d,%d,%d,%d,%d,%d,"
        "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
        "%lu,%lu,%lu,%lu\r\n",
        HAL_GetTick(), rtc_text, date_time.valid,
        can_latest.count, can_latest.id, can_latest.extended,
        can_latest.dlc, hex, latest_gps_fix_valid, lat, lon,
        latest_gps_speed_valid, gps_hundredths / 100U,
        gps_hundredths % 100U, vehicle_hundredths / 100U,
        vehicle_hundredths % 100U,
        latest_vehicle_speed_source,
        imu_hundredths / 100U, imu_hundredths % 100U,
        (long)imu_telemetry.forward_accel_mg,
        imu_ready, imu_calibrated, imu_telemetry.valid,
        imu_telemetry.sample_ms, imu_age_ms, imu_telemetry.sample_count,
        imu_telemetry.read_error_count,
        imu_telemetry.acc_x_raw, imu_telemetry.acc_y_raw,
        imu_telemetry.acc_z_raw, imu_telemetry.gyro_x_raw,
        imu_telemetry.gyro_y_raw, imu_telemetry.gyro_z_raw,
        (long)imu_telemetry.acc_x_mg, (long)imu_telemetry.acc_y_mg,
        (long)imu_telemetry.acc_z_mg, (long)imu_telemetry.gyro_x_mdps,
        (long)imu_telemetry.gyro_y_mdps, (long)imu_telemetry.gyro_z_mdps,
        (long)imu_telemetry.linear_x_mg, (long)imu_telemetry.linear_y_mg,
        (long)imu_telemetry.linear_z_mg,
        imu_telemetry.total_g_mg, imu_telemetry.dynamic_g_mg,
        imu_peak_window_g_mg, imu_telemetry.peak_boot_g_mg) < 0)
      result = FR_DISK_ERR;
    if (result == FR_OK) result = f_sync(&USERFile);
    if (f_close(&USERFile) != FR_OK && result == FR_OK) result = FR_DISK_ERR;
  }
  if (result == FR_OK)
  {
    /* Begin a new one-second peak window without discarding the current sample. */
    imu_telemetry.peak_window_g_mg = imu_telemetry.dynamic_g_mg;
    sd_led_until = HAL_GetTick() + ACTIVITY_PULSE_MS;
    SetError(ERROR_SD, 0U);
  }
  else
  {
    sd_ready = 0U;
    SetError(ERROR_SD, 1U);
  }
}

static void SD_Task(void)
{
  /* Mount on insertion, retry failures, and log once per second when enabled. */
  static uint32_t last_log_ms;
  uint32_t now = HAL_GetTick();
  if (sd_ready == 0U)
  {
    if ((now - sd_last_attempt_ms) < SD_RETRY_PERIOD_MS) return;
    sd_last_attempt_ms = now;
    if (HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin) != GPIO_PIN_RESET) return;
    if ((retUSER == 0U) && (f_mount(&USERFatFS, USERPath, 1U) == FR_OK))
    {
      sd_ready = 1U;
      SetError(ERROR_SD, 0U);
      if (sd_logging_enabled != 0U) SD_LogSnapshot();
    }
    else SetError(ERROR_SD, 1U);
  }
  else if ((sd_logging_enabled != 0U) &&
           ((now - last_log_ms) >= TELEMETRY_PERIOD_MS))
  {
    last_log_ms = now;
    SD_LogSnapshot();
  }
}

static uint8_t GPS_ChecksumValid(const char *s)
{
  /* NMEA checksum is the XOR of all bytes between '$' and '*'. */
  uint8_t actual = 0U, expected = 0U;
  if (s[0] != '$') return 0U;
  s++;
  while (*s != '\0' && *s != '*') actual ^= (uint8_t)*s++;
  if (*s++ != '*') return 0U;
  for (uint8_t i = 0U; i < 2U; i++)
  {
    char c = *s++;
    expected <<= 4;
    if (c >= '0' && c <= '9') expected |= (uint8_t)(c - '0');
    else if (c >= 'A' && c <= 'F') expected |= (uint8_t)(c - 'A' + 10);
    else if (c >= 'a' && c <= 'f') expected |= (uint8_t)(c - 'a' + 10);
    else return 0U;
  }
  return (actual == expected) ? 1U : 0U;
}

static uint8_t GPS_GetField(const char *sentence, uint8_t field_index,
                            char *out, size_t out_len)
{
  /*
   * Extract one comma-delimited NMEA field without strtok().  The parser can
   * therefore examine the same read-only sentence several times safely.
   */
  uint8_t current_field = 0U;
  size_t out_pos = 0U;

  if ((sentence == NULL) || (out == NULL) || (out_len == 0U)) return 0U;
  out[0] = '\0';

  for (const char *p = sentence; *p != '\0'; p++)
  {
    char ch = *p;
    if ((ch == ',') || (ch == '*') || (ch == '\r') || (ch == '\n'))
    {
      if (current_field == field_index)
      {
        out[out_pos] = '\0';
        return 1U;
      }
      current_field++;
      if (ch != ',') break;
    }
    else if ((current_field == field_index) && (out_pos < (out_len - 1U)))
    {
      out[out_pos++] = ch;
    }
  }

  if (current_field == field_index)
  {
    out[out_pos] = '\0';
    return 1U;
  }
  return 0U;
}

static uint8_t GPS_IsNavSentence(const char *sentence)
{
  /*
   * The first two letters are a variable talker ID (GP/GN/GL/etc.).  The
   * message type is always at characters 3..5.  Only these reference types
   * contain the position or ground-speed values used by telemetry.
   */
  if ((sentence == NULL) || (strlen(sentence) < 6U) ||
      ((sentence[0] != '$') && (sentence[0] != '!'))) return 0U;

  return ((strncmp(&sentence[3], "RMC", 3U) == 0) ||
          (strncmp(&sentence[3], "GGA", 3U) == 0) ||
          (strncmp(&sentence[3], "VTG", 3U) == 0)) ? 1U : 0U;
}

static uint8_t GPS_ParseCoordDeg(const char *value, const char *hemisphere,
                                 float *coord_deg)
{
  /* Convert NMEA ddmm.mmmm/dddmm.mmmm into signed decimal degrees. */
  if ((value == NULL) || (hemisphere == NULL) || (coord_deg == NULL) ||
      (value[0] == '\0') || (hemisphere[0] == '\0')) return 0U;

  float raw = strtof(value, NULL);
  int degrees = (int)(raw / 100.0f);
  float minutes = raw - ((float)degrees * 100.0f);
  *coord_deg = (float)degrees + (minutes / 60.0f);
  if ((hemisphere[0] == 'S') || (hemisphere[0] == 'W')) *coord_deg = -*coord_deg;
  return 1U;
}

static uint8_t GPS_ParseSpeedMph(const char *sentence, float *speed_mph)
{
  /* RMC and VTG report knots; VTG can fall back to km/h. */
  char field[20];
  if ((sentence == NULL) || (speed_mph == NULL) || (strlen(sentence) < 6U)) return 0U;

  if ((strncmp(&sentence[3], "RMC", 3U) == 0) &&
      GPS_GetField(sentence, 2U, field, sizeof(field)) && (field[0] == 'A') &&
      GPS_GetField(sentence, 7U, field, sizeof(field)) && (field[0] != '\0'))
  {
    *speed_mph = strtof(field, NULL) * KNOTS_TO_MPH;
    return 1U;
  }

  if (strncmp(&sentence[3], "VTG", 3U) == 0)
  {
    if (GPS_GetField(sentence, 5U, field, sizeof(field)) && (field[0] != '\0'))
    {
      *speed_mph = strtof(field, NULL) * KNOTS_TO_MPH;
      return 1U;
    }
    if (GPS_GetField(sentence, 7U, field, sizeof(field)) && (field[0] != '\0'))
    {
      *speed_mph = strtof(field, NULL) * KMH_TO_MPH;
      return 1U;
    }
  }
  return 0U;
}

static uint8_t GPS_ParseDateTimeUtc(const char *sentence,
                                    TelemetryDateTime_t *date_time)
{
  /*
   * An active RMC sentence carries UTC as hhmmss.sss in field 1 and ddmmyy
   * in field 9.  Fractional seconds are intentionally ignored because the
   * NMEA sentence arrives after the exact GPS second boundary.  GPS PPS can
   * be used later if sub-second RTC alignment is required.
   */
  char time[16];
  char status[4];
  char date[10];

  if ((sentence == NULL) || (date_time == NULL) ||
      (strncmp(&sentence[3], "RMC", 3U) != 0) ||
      !GPS_GetField(sentence, 1U, time, sizeof(time)) ||
      !GPS_GetField(sentence, 2U, status, sizeof(status)) ||
      !GPS_GetField(sentence, 9U, date, sizeof(date)) ||
      (status[0] != 'A') || (strlen(time) < 6U) || (strlen(date) != 6U))
    return 0U;

  for (uint8_t i = 0U; i < 6U; i++)
  {
    if ((time[i] < '0') || (time[i] > '9') ||
        (date[i] < '0') || (date[i] > '9')) return 0U;
  }

  memset(date_time, 0, sizeof(*date_time));
  date_time->hour = (uint8_t)((time[0] - '0') * 10 + (time[1] - '0'));
  date_time->minute = (uint8_t)((time[2] - '0') * 10 + (time[3] - '0'));
  date_time->second = (uint8_t)((time[4] - '0') * 10 + (time[5] - '0'));
  date_time->day = (uint8_t)((date[0] - '0') * 10 + (date[1] - '0'));
  date_time->month = (uint8_t)((date[2] - '0') * 10 + (date[3] - '0'));
  date_time->year = (uint16_t)(2000U +
      (uint16_t)((date[4] - '0') * 10 + (date[5] - '0')));

  if (TelemetryDateTime_IsValid(date_time) == 0U) return 0U;
  date_time->weekday = TelemetryDateTime_CalcWeekday(date_time->year,
                                                     date_time->month,
                                                     date_time->day);
  date_time->valid = 1U;
  return 1U;
}

static void FormatSignedFixed6(float value, char *out, size_t out_len)
{
  /* Avoid requiring floating-point printf support just to log coordinates. */
  int32_t scaled = (int32_t)(value * 1000000.0f);
  uint32_t magnitude = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;
  (void)snprintf(out, out_len, "%s%lu.%06lu", (scaled < 0) ? "-" : "",
                 magnitude / 1000000U, magnitude % 1000000U);
}

static void FormatSignedMilli(int32_t milli_value, char *out, size_t out_len)
{
  /*
   * Newlib-nano does not reliably implement the long-long printf modifier.
   * Compute the magnitude safely in 64 bits, then print its 32-bit pieces with
   * the supported %lu modifier. All telemetry milli-values fit in uint32_t.
   */
  uint32_t magnitude = (milli_value < 0) ?
      (uint32_t)(-(int64_t)milli_value) : (uint32_t)milli_value;
  (void)snprintf(out, out_len, "%s%lu.%03lu",
                 (milli_value < 0) ? "-" : "",
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)(magnitude % 1000U));
}

static void GPS_UpdateNavFromSentence(const char *sentence)
{
  /*
   * RMC: active fix, latitude, longitude, and speed in knots.
   * GGA: latitude/longitude/elevation when fix quality is nonzero.
   * VTG: ground speed without a position fix.
   */
  char status[8], lat[20], ns[4], lon[20], ew[4];
  float lat_deg = 0.0f, lon_deg = 0.0f, speed_mph = 0.0f;
  uint8_t updated_fix = 0U;
  uint32_t now = HAL_GetTick();

  TelemetryDateTime_t gps_date_time = {0};
  if (GPS_ParseDateTimeUtc(sentence, &gps_date_time) &&
      ((gps_rtc_sync_count == 0U) ||
       ((now - gps_rtc_last_sync_ms) >= GPS_RTC_SYNC_PERIOD_MS)))
  {
    if (STM32_RTC_Set(&gps_date_time) != 0U)
    {
      char gps_time_text[24];
      gps_rtc_last_sync_ms = now;
      gps_rtc_sync_count++;
      stm32_rtc_time_source = "GPS_UTC";
      TelemetryDateTime_Format(&gps_date_time, gps_time_text,
                               sizeof(gps_time_text));
      printf("[GPS RTC] internal RTC synchronized to %s UTC count=%lu\r\n",
             gps_time_text, gps_rtc_sync_count);
    }
    else
    {
      printf("[GPS RTC] internal RTC synchronization failed\r\n");
    }
  }

  if (GPS_ParseSpeedMph(sentence, &speed_mph))
  {
    latest_gps_speed_mph = speed_mph;
    latest_gps_speed_valid = 1U;
    latest_gps_speed_ms = now;
  }

  if ((strncmp(&sentence[3], "RMC", 3U) == 0) &&
      GPS_GetField(sentence, 2U, status, sizeof(status)) && (status[0] == 'A') &&
      GPS_GetField(sentence, 3U, lat, sizeof(lat)) &&
      GPS_GetField(sentence, 4U, ns, sizeof(ns)) &&
      GPS_GetField(sentence, 5U, lon, sizeof(lon)) &&
      GPS_GetField(sentence, 6U, ew, sizeof(ew)) &&
      GPS_ParseCoordDeg(lat, ns, &lat_deg) && GPS_ParseCoordDeg(lon, ew, &lon_deg))
  {
    updated_fix = 1U;
  }
  else if ((strncmp(&sentence[3], "GGA", 3U) == 0) &&
           GPS_GetField(sentence, 6U, status, sizeof(status)) &&
           (status[0] != '\0') && (status[0] != '0') &&
           GPS_GetField(sentence, 2U, lat, sizeof(lat)) &&
           GPS_GetField(sentence, 3U, ns, sizeof(ns)) &&
           GPS_GetField(sentence, 4U, lon, sizeof(lon)) &&
           GPS_GetField(sentence, 5U, ew, sizeof(ew)) &&
           GPS_ParseCoordDeg(lat, ns, &lat_deg) && GPS_ParseCoordDeg(lon, ew, &lon_deg))
  {
    updated_fix = 1U;

    /*
     * GGA field 9 is orthometric elevation above mean sea level; field 10
     * identifies metres. RMC contains position and time but no elevation, so
     * elevation has its own validity flag and freshness timestamp.
     */
    char altitude[20], altitude_unit[4];
    if (GPS_GetField(sentence, 9U, altitude, sizeof(altitude)) &&
        GPS_GetField(sentence, 10U, altitude_unit, sizeof(altitude_unit)) &&
        (altitude[0] != '\0') && (altitude_unit[0] == 'M'))
    {
      char *end = NULL;
      float altitude_m = strtof(altitude, &end);
      if ((end != altitude) && (*end == '\0') && isfinite(altitude_m) &&
          (altitude_m >= -1000.0f) && (altitude_m <= 60000.0f))
      {
        latest_gps_altitude_mm = (int32_t)(altitude_m * 1000.0f +
            ((altitude_m >= 0.0f) ? 0.5f : -0.5f));
        latest_gps_altitude_valid = 1U;
        latest_gps_altitude_ms = now;
      }
    }
  }

  if (updated_fix != 0U)
  {
    latest_gps_lat_deg = lat_deg;
    latest_gps_lon_deg = lon_deg;
    latest_gps_fix_valid = 1U;
    latest_gps_fix_ms = now;
  }

  VehicleSpeed_UpdateFromSources();

  if ((now - gps_last_nav_print_ms) >= GPS_NAV_PRINT_PERIOD_MS)
  {
    char lat_text[20], lon_text[20], altitude_text[20];
    uint32_t speed_hundredths = (uint32_t)(latest_gps_speed_mph * 100.0f + 0.5f);
    gps_last_nav_print_ms = now;
    FormatSignedFixed6(latest_gps_lat_deg, lat_text, sizeof(lat_text));
    FormatSignedFixed6(latest_gps_lon_deg, lon_text, sizeof(lon_text));
    FormatSignedMilli(latest_gps_altitude_mm, altitude_text,
                      sizeof(altitude_text));
    printf("[GPS] nav=%lu filtered=%lu checksum_err=%lu fix=%u lat=%s lon=%s alt=%s m alt_valid=%u speed=%lu.%02lu mph\r\n",
           gps_valid_count, gps_filtered_count, gps_checksum_error_count,
           latest_gps_fix_valid, lat_text, lon_text, altitude_text,
           latest_gps_altitude_valid,
           speed_hundredths / 100U, speed_hundredths % 100U);
  }
}

static void GPS_ProcessByte(uint8_t byte)
{
  /*
   * SPI is a clocked interface, so the STM32 sends 0xFF bytes to make the GPS
   * shift pending output back.  A returned 0xFF means "nothing available".
   * Bytes before '$' are ignored, and a new '$' abandons any partial sentence.
   */
  if (byte == 0xFFU) return;
  if (byte == '$') gps_nmea_index = 0U;
  if ((gps_nmea_index == 0U) && (byte != '$')) return;
  if (gps_nmea_index < (GPS_NMEA_SIZE - 1U)) gps_nmea[gps_nmea_index++] = (char)byte;
  else gps_nmea_index = 0U;
  if (byte == '\n' && gps_nmea_index != 0U)
  {
    gps_nmea[gps_nmea_index] = '\0';
    if (GPS_ChecksumValid(gps_nmea) == 0U)
    {
      gps_checksum_error_count++;
    }
    else if (GPS_IsNavSentence(gps_nmea) != 0U)
    {
      /* Only checksum-valid RMC/GGA/VTG sentences reach telemetry. */
      gps_valid_count++;
      gps_last_valid_ms = HAL_GetTick();
      GPS_UpdateNavFromSentence(gps_nmea);
    }
    else gps_filtered_count++;
    gps_nmea_index = 0U;
  }
}

static void GPS_Task(void)
{
  /*
   * Run frequently enough to drain the GPS SPI stream without delaying the
   * one-second SD/RS232/ESP32 work.  Parsed data has a freshness timeout so a
   * disconnected receiver cannot leave an old position marked valid forever.
   */
  uint32_t now = HAL_GetTick();
  /* Expire stale sources and keep the CAN -> GPS -> IMU selection current. */
  VehicleSpeed_UpdateFromSources();

  if ((now - gps_last_poll_ms) < GPS_POLL_PERIOD_MS) return;
  gps_last_poll_ms = now;
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi2, gps_tx, gps_rx,
                                                     GPS_SPI_BLOCK_SIZE, 100U);
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_SET);
  if (result == HAL_OK)
    for (uint16_t i = 0U; i < GPS_SPI_BLOCK_SIZE; i++) GPS_ProcessByte(gps_rx[i]);
}

static int32_t ESP32_Exchange(uint8_t *tx, uint8_t *rx)
{
  /*
   * READY high says the ESP32 has armed its SPI slave transaction.  CS is then
   * held low for exactly one 768-byte ASCII frame, matching SPI_RX_FRAME_LEN in
   * ESP32_Telemetry_ADV.  After CS rises, READY must return low before another
   * frame is allowed; this prevents reading a stale reply.
   * Return values: 0=success, -1=not ready, -2=SPI error, -3=READY stuck high.
   */
  uint32_t start = HAL_GetTick();
  while (HAL_GPIO_ReadPin(ESP32_Ready_GPIO_Port, ESP32_Ready_Pin) == GPIO_PIN_RESET)
    if ((HAL_GetTick() - start) >= ESP32_READY_TIMEOUT_MS) return -1;
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi4, tx, rx,
      ESP32_ASCII_FRAME_SIZE, ESP32_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  if (result != HAL_OK) return -2;

  /* The tested ESP32 slave pulls READY low after processing each CS edge. */
  start = HAL_GetTick();
  while (HAL_GPIO_ReadPin(ESP32_Ready_GPIO_Port, ESP32_Ready_Pin) == GPIO_PIN_SET)
    if ((HAL_GetTick() - start) >= ESP32_READY_TIMEOUT_MS) return -3;
  return 0;
}

static void ESP32_QueueResponse(const char *id, const char *status,
                                const char *message)
{
  if (id == NULL) id = "0";
  if (status == NULL) status = "ERR";
  if (message == NULL) message = "NO_MESSAGE";
  (void)snprintf(esp32_response, sizeof(esp32_response),
                 "$RSP,%s,%s,%s\r\n", id, status, message);
  esp32_response_pending = 1U;
}

static void ESP32_ExecuteCommand(const char *id, const char *verb)
{
  if ((id == NULL) || (verb == NULL)) return;

  if (strcmp(verb, "START_LOG") == 0)
  {
    if (sd_ready != 0U)
    {
      sd_logging_enabled = 1U;
      ESP32_QueueResponse(id, "OK", "LOG_STARTED");
    }
    else ESP32_QueueResponse(id, "ERR", "SD_NOT_READY");
    return;
  }

  if (strcmp(verb, "STOP_LOG") == 0)
  {
    sd_logging_enabled = 0U;
    ESP32_QueueResponse(id, "OK", "LOG_STOPPED");
    return;
  }

  if (strcmp(verb, "CALIBRATE_IMU") == 0)
  {
#if ENABLE_BMI270_IMU
    if (imu_ready == 0U)
      ESP32_QueueResponse(id, "ERR", "IMU_NOT_READY");
    else
    {
      int8_t result = Future_BMI270_CalibrateAtRest();
      if (result == BMI2_OK)
      {
        SetError(ERROR_IMU, 0U);
        ESP32_QueueResponse(id, "OK", "IMU_CALIBRATED_AT_REST");
      }
      else
      {
        SetError(ERROR_IMU, 1U);
        ESP32_QueueResponse(id, "ERR",
            (result == BMI2_E_PRECON_ERROR) ?
              ((imu_calibration_wrong_mount != 0U) ?
               "IMU_WRONG_MOUNT_ORIENTATION" : "IMU_NOT_STATIONARY") :
            "IMU_CALIBRATION_FAILED");
      }
    }
#else
    ESP32_QueueResponse(id, "ERR", "IMU_DISABLED");
#endif
    return;
  }

  if (strcmp(verb, "CLEAR_ERRORS") == 0)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    error_flags = 0U;
    __set_PRIMASK(primask);
    gps_checksum_error_count = 0U;
    esp32_fail_count = 0U;
    ESP32_QueueResponse(id, "OK", "ERRORS_CLEARED");
    return;
  }

  if (strncmp(verb, "SET_RTC,", 8U) == 0)
  {
    unsigned int year, month, day, hour, minute, second;
    int consumed = 0;
    TelemetryDateTime_t requested = {0};
    TelemetryDateTime_t readback = {0};
    char readback_text[24];

    if ((sscanf(verb + 8U, "%u-%u-%u,%u:%u:%u%n", &year, &month, &day,
                &hour, &minute, &second, &consumed) != 6) ||
        ((verb + 8U)[consumed] != '\0'))
    {
      ESP32_QueueResponse(id, "ERR", "BAD_RTC_FORMAT");
      return;
    }

    if ((year < 2000U) || (year > 2099U) || (month < 1U) ||
        (month > 12U) || (day < 1U) || (day > 31U) || (hour > 23U) ||
        (minute > 59U) || (second > 59U))
    {
      ESP32_QueueResponse(id, "ERR", "RTC_RANGE_INVALID");
      return;
    }

    requested.year = (uint16_t)year;
    requested.month = (uint8_t)month;
    requested.day = (uint8_t)day;
    requested.hour = (uint8_t)hour;
    requested.minute = (uint8_t)minute;
    requested.second = (uint8_t)second;
    requested.valid = 1U;

    if (TelemetryDateTime_IsValid(&requested) == 0U)
    {
      ESP32_QueueResponse(id, "ERR", "RTC_RANGE_INVALID");
      return;
    }
    requested.weekday = TelemetryDateTime_CalcWeekday(requested.year,
                                                       requested.month,
                                                       requested.day);
    if (STM32_RTC_Set(&requested) == 0U)
    {
      ESP32_QueueResponse(id, "ERR", "STM32_RTC_SET_FAILED");
      return;
    }

    /*
     * App time takes effect immediately.  If GPS already established UTC,
     * delay its next drift correction for a full synchronization interval so
     * the app value is not overwritten by the next RMC sentence.
     */
    stm32_rtc_time_source = "APP";
    if (gps_rtc_sync_count != 0U) gps_rtc_last_sync_ms = HAL_GetTick();

#if ENABLE_EXTERNAL_PCF85263A_RTC
    if (Future_PCF85263A_Set(&requested) == 0U)
    {
      ESP32_QueueResponse(id, "ERR", "PCF85263A_SET_FAILED_INTERNAL_OK");
      return;
    }
#endif

    if (STM32_RTC_Read(&readback) != 0U)
    {
      TelemetryDateTime_Format(&readback, readback_text, sizeof(readback_text));
      printf("[ESP32 RTC] SET_RTC applied and read back: %s\r\n", readback_text);
    }
#if ENABLE_EXTERNAL_PCF85263A_RTC
    ESP32_QueueResponse(id, "OK", "RTC_SET_INTERNAL_AND_PCF85263A");
#else
    ESP32_QueueResponse(id, "OK", "RTC_SET_INTERNAL");
#endif
    return;
  }

  ESP32_QueueResponse(id, "ERR", "UNKNOWN_CMD");
}

static void ESP32_HandleCommand(char *command)
{
  char *p = command;
  if (p == NULL) return;
  if (*p == '$') p++;
  if (strncmp(p, "ESP_ACK", 7U) == 0) return;

  if (strncmp(p, "ESP_CMD", 7U) == 0)
  {
    char id[16] = "0";
    char verb[ESP32_COMMAND_SIZE] = {0};
    char *seq = strstr(p, "seq=");
    char *cmd = strstr(p, "cmd=\"");
    if (seq != NULL)
    {
      unsigned int value;
      if (sscanf(seq, "seq=%u", &value) == 1)
        (void)snprintf(id, sizeof(id), "%u", value);
    }
    if (cmd != NULL)
    {
      char *src = cmd + 5;
      size_t i = 0U;
      while ((src[i] != '\0') && (src[i] != '"') &&
             (i < (sizeof(verb) - 1U)))
      {
        verb[i] = src[i];
        i++;
      }
      verb[i] = '\0';
      /* ESP32 repeats queued commands for link reliability. Execute each ID once. */
      if ((strcmp(id, esp32_last_command_id) == 0) &&
          (strcmp(verb, esp32_last_command) == 0))
      {
        printf("[ESP32 RX] duplicate command ignored: id=%s\r\n", id);
        return;
      }
      (void)snprintf(esp32_last_command_id, sizeof(esp32_last_command_id),
                     "%s", id);
      (void)snprintf(esp32_last_command, sizeof(esp32_last_command), "%s", verb);
      ESP32_ExecuteCommand(id, verb);
      return;
    }
    ESP32_QueueResponse(id, "ERR", "ESP_CMD_MISSING_CMD");
    return;
  }

  if ((strncmp(p, "SET_RTC,", 8U) == 0) ||
      (strcmp(p, "START_LOG") == 0) || (strcmp(p, "STOP_LOG") == 0) ||
      (strcmp(p, "CALIBRATE_IMU") == 0) ||
      (strcmp(p, "CLEAR_ERRORS") == 0))
    ESP32_ExecuteCommand("0", p);
}

static void ESP32_ProcessRxFrame(const uint8_t *rx, size_t len)
{
  char command[ESP32_COMMAND_SIZE] = {0};
  size_t pos = 0U;
  if (rx == NULL) return;

  for (size_t i = 0U; (i < len) && (pos < (sizeof(command) - 1U)); i++)
  {
    uint8_t c = rx[i];
    if ((c == 0x00U) || (c == 0xFFU))
    {
      if (pos != 0U) break;
      continue;
    }
    if ((c == '\r') || (c == '\n')) break;
    if ((c >= 32U) && (c <= 126U)) command[pos++] = (char)c;
  }
  command[pos] = '\0';
  if (pos != 0U)
  {
    printf("[ESP32 RX] %s\r\n", command);
    ESP32_HandleCommand(command);
  }
}

static void ESP32_Task(void)
{
  /*
   * The ESP32 bridge queues one 768-byte slave transaction at a time.  Every
   * second the STM32 sends either a command response or a $TEL status frame.
   * Simultaneous MISO data can carry any supported ESP_CMD from the BLE app.
   */
  static uint32_t last_ms;
  uint32_t now = HAL_GetTick();
  uint8_t sending_response;
  TelemetryDateTime_t date_time = {0};
  char rtc_text[24];

  if ((now - last_ms) < TELEMETRY_PERIOD_MS) return;
  last_ms = now;
  memset(esp32_tx_frame, 0, sizeof(esp32_tx_frame));
  memset(esp32_rx_frame, 0, sizeof(esp32_rx_frame));
  sending_response = esp32_response_pending;

  if (sending_response != 0U)
  {
    (void)snprintf((char *)esp32_tx_frame, sizeof(esp32_tx_frame), "%s",
                   esp32_response);
  }
  else
  {
    uint8_t rtc_valid = STM32_RTC_Read(&date_time);
    VehicleSpeed_UpdateFromSources();
    uint32_t vehicle_hundredths =
        (uint32_t)(latest_vehicle_speed_mph * 100.0f + 0.5f);
    uint32_t imu_hundredths =
        (uint32_t)(latest_imu_speed_mph * 100.0f + 0.5f);
    TelemetryDateTime_Format(&date_time, rtc_text, sizeof(rtc_text));
    (void)snprintf((char *)esp32_tx_frame, sizeof(esp32_tx_frame),
        "$TEL,seq=%lu,ms=%lu,rtc=%s,rtc_valid=%u,rtc_source=%s,"
        "speed_mph=%lu.%02lu,speed_source=%s,can_rx=%lu,gps_valid=%u,"
        "imu_speed_mph=%lu.%02lu,imu_forward_accel_mg=%ld,"
        "imu_ready=%u,imu_calibrated=%u,imu_mount_valid=%u,imu_valid=%u,imu_dynamic_g_mg=%lu,"
        "imu_peak_g_mg=%lu,logging=%u,sd=%s\r\n",
        esp32_sequence++, now, rtc_text, rtc_valid, stm32_rtc_time_source,
        vehicle_hundredths / 100U, vehicle_hundredths % 100U,
        latest_vehicle_speed_source, can_latest.count, latest_gps_fix_valid,
        imu_hundredths / 100U, imu_hundredths % 100U,
        (long)imu_telemetry.forward_accel_mg,
        imu_ready, imu_calibrated, imu_mount_valid, imu_telemetry.valid,
        imu_telemetry.dynamic_g_mg, imu_telemetry.peak_boot_g_mg,
        sd_logging_enabled, sd_ready ? "OK" : "ERR");
  }

  int32_t result = ESP32_Exchange(esp32_tx_frame, esp32_rx_frame);
  if (result == 0)
  {
    if (sending_response != 0U) esp32_response_pending = 0U;
    ESP32_ProcessRxFrame(esp32_rx_frame, sizeof(esp32_rx_frame));
    esp32_last_result = 0;
    esp32_pass_count++;
    esp32_last_pass_ms = now;
    SetError(ERROR_ESP32, 0U);
  }
  else
  {
    esp32_last_result = result;
    esp32_fail_count++;
    SetError(ERROR_ESP32, 1U);
  }
}

static void RS232_Task(void)
{
  /*
   * Once per second, build one complete legacy block in persistent RAM and
   * start an interrupt-driven UART transfer.  The buffer is static because
   * HAL continues reading it after this function returns.  CAN priority is
   * higher than USART1, so CAN reception continues throughout RS232 output.
   * Unknown rows retain 0xHHHHHHHH instead of reporting a false zero value.
   */
  static uint32_t last_ms;
  static char block[4096];
  uint32_t now = HAL_GetTick();
  if ((now - last_ms) < TELEMETRY_PERIOD_MS) return;
  if (rs232_tx_busy != 0U) return;
  last_ms = now;
  size_t pos = (size_t)snprintf(block, sizeof(block), "raw_data\r\nABCDEF\r\n");
  for (uint32_t i = 0U; i < SUN_RAW_TABLE_COUNT && pos < sizeof(block); i++)
  {
    /* Copy all three interrupt-owned fields as one consistent snapshot. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t valid = sun_raw_table[i].valid;
    uint32_t high_word = sun_raw_table[i].high_word;
    uint32_t low_word = sun_raw_table[i].low_word;
    __set_PRIMASK(primask);

    /*
     * DataProcessor expects LOW then HIGH.  Swap only for presentation so the
     * ASCII hex spells the original little-endian byte sequence from CAN.
     */
    uint32_t low_wire_hex = ByteSwapU32(low_word);
    uint32_t high_wire_hex = ByteSwapU32(high_word);
    int n = valid ?
      snprintf(&block[pos], sizeof(block) - pos, "%s,0x%08lX,0x%08lX\r\n",
        sun_raw_table[i].name, low_wire_hex, high_wire_hex) :
      snprintf(&block[pos], sizeof(block) - pos, "%s,0xHHHHHHHH,0xHHHHHHHH\r\n",
        sun_raw_table[i].name);
    if (n < 0 || (size_t)n >= (sizeof(block) - pos)) break;
    pos += (size_t)n;
  }
  char lat[20], lon[20], altitude[20];
  char forward_g[20], linear_x_g[20], linear_y_g[20], linear_z_g[20];
  char total_g[20], dynamic_g[20], peak_boot_g[20];
  char rtc_text[24];
  char uptime_text[24];
  TelemetryDateTime_t date_time = {0};
  FormatSignedFixed6(latest_gps_lat_deg, lat, sizeof(lat));
  FormatSignedFixed6(latest_gps_lon_deg, lon, sizeof(lon));
  FormatSignedMilli(latest_gps_altitude_mm, altitude, sizeof(altitude));
  FormatSignedMilli(imu_telemetry.forward_accel_mg, forward_g,
                    sizeof(forward_g));
  FormatSignedMilli(imu_telemetry.linear_x_mg, linear_x_g,
                    sizeof(linear_x_g));
  FormatSignedMilli(imu_telemetry.linear_y_mg, linear_y_g,
                    sizeof(linear_y_g));
  FormatSignedMilli(imu_telemetry.linear_z_mg, linear_z_g,
                    sizeof(linear_z_g));
  FormatSignedMilli((int32_t)imu_telemetry.total_g_mg, total_g,
                    sizeof(total_g));
  FormatSignedMilli((int32_t)imu_telemetry.dynamic_g_mg, dynamic_g,
                    sizeof(dynamic_g));
  FormatSignedMilli((int32_t)imu_telemetry.peak_boot_g_mg, peak_boot_g,
                    sizeof(peak_boot_g));
  (void)STM32_RTC_Read(&date_time);
  TelemetryDateTime_Format(&date_time, rtc_text, sizeof(rtc_text));
  BoardUptime_Format(now, uptime_text, sizeof(uptime_text));
  VehicleSpeed_UpdateFromSources();
  uint32_t gps_speed_hundredths = latest_gps_speed_valid ?
      (uint32_t)(latest_gps_speed_mph * 100.0f + 0.5f) : 0U;
  uint32_t imu_speed_hundredths =
      (uint32_t)(latest_imu_speed_mph * 100.0f + 0.5f);
  uint32_t vehicle_speed_hundredths =
      (uint32_t)(latest_vehicle_speed_mph * 100.0f + 0.5f);
  int n = snprintf(&block[pos], sizeof(block) - pos,
      "BME,DISABLED\r\n"
      "NAV,IMU_MPH=%lu.%02lu,GPS_MPH=%lu.%02lu,GPS_VALID=%u,"
      "VEHICLE_MPH=%lu.%02lu,SOURCE=%s,LAT=%s,LON=%s,FIX=%u,AGE_MS=%lu,"
      "ELEV_M=%s,ELEV_VALID=%u,ELEV_AGE_MS=%lu\r\n"
      "IMU_G,VALID=%u,CALIBRATED=%u,MOUNT_VALID=%u,FORWARD_G=%s,LINEAR_X_G=%s,"
      "LINEAR_Y_G=%s,LINEAR_Z_G=%s,TOTAL_G=%s,DYNAMIC_G=%s,"
      "PEAK_BOOT_G=%s,AGE_MS=%lu\r\n"
      "TL_TIM,%s,RTC_SOURCE=%s,UPTIME_MS=%lu\r\n"
      "TL_UPT,%s\r\nVWXYZ\r\n",
      imu_speed_hundredths / 100U, imu_speed_hundredths % 100U,
      gps_speed_hundredths / 100U, gps_speed_hundredths % 100U,
      latest_gps_speed_valid, vehicle_speed_hundredths / 100U,
      vehicle_speed_hundredths % 100U, latest_vehicle_speed_source,
      lat, lon, latest_gps_fix_valid,
      latest_gps_fix_valid ? (now - latest_gps_fix_ms) : 0U,
      altitude, latest_gps_altitude_valid,
      latest_gps_altitude_valid ? (now - latest_gps_altitude_ms) : 0U,
      imu_telemetry.valid, imu_calibrated, imu_mount_valid, forward_g, linear_x_g,
      linear_y_g, linear_z_g, total_g, dynamic_g, peak_boot_g,
      imu_telemetry.valid ? (now - imu_telemetry.sample_ms) : 0U,
      rtc_text, stm32_rtc_time_source, now, uptime_text);
  if (n > 0 && (size_t)n < (sizeof(block) - pos)) pos += (size_t)n;
  rs232_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)block, (uint16_t)pos) == HAL_OK)
  {
#if RS232_SWV_ECHO_BLOCK
    /* SWV echo is debug-only; USART1 carries one asynchronous copy. */
    printf("[RS232 TX START] bytes=%u\r\n%s", (unsigned int)pos, block);
#else
    printf("[RS232 TX START] bytes=%u\r\n", (unsigned int)pos);
#endif
  }
  else
  {
    rs232_tx_busy = 0U;
    SetError(ERROR_RS232, 1U);
    printf("[RS232 TX] FAILED uart_state=%lu uart_error=0x%08lX\r\n",
           (uint32_t)HAL_UART_GetState(&huart1), HAL_UART_GetError(&huart1));
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* The complete raw_data...VWXYZ block has left the UART peripheral. */
    rs232_tx_busy = 0U;
    rs232_led_until = HAL_GetTick() + ACTIVITY_PULSE_MS;
    SetError(ERROR_RS232, 0U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    rs232_tx_busy = 0U;
    SetError(ERROR_RS232, 1U);
  }
}

#if ENABLE_SWV_DEBUG_OUTPUT
static void ErrorFlags_SWV_Task(void)
{
  /* Print only when the bitmask changes so the reason for red LED 10 is clear. */
  static uint32_t previous_flags = 0xFFFFFFFFUL;
  uint32_t flags = error_flags;
  if (flags == previous_flags) return;
  previous_flags = flags;

  printf("[ERROR FLAGS] mask=0x%02lX SD=%u CAN=%u RS232=%u ESP32=%u IMU=%u ",
         flags,
         (flags & ERROR_SD) ? 1U : 0U,
         (flags & ERROR_CAN) ? 1U : 0U,
         (flags & ERROR_RS232) ? 1U : 0U,
         (flags & ERROR_ESP32) ? 1U : 0U,
         (flags & ERROR_IMU) ? 1U : 0U);
  printf("esp_result=%ld esp_pass=%lu esp_fail=%lu sd_ready=%u\r\n",
         (long)esp32_last_result, esp32_pass_count, esp32_fail_count, sd_ready);
}
#endif

static void StatusLED_Task(void)
{
  /*
   * Y6=SD write, Y7=CAN RX, Y8=RS232 TX, Y9=recent ESP32 success,
   * Y5=fresh filtered GPS navigation, R10=one or more active subsystem errors,
   * and G11=one second on/one second off heartbeat.
   */
  uint32_t now = HAL_GetTick();
  LED_Write(Green_LED11_GPIO_Port, Green_LED11_Pin,
            ((now % HEARTBEAT_PERIOD_MS) < HEARTBEAT_ON_MS));
  LED_Write(Yellow_LED6_GPIO_Port, Yellow_LED6_Pin,
            sd_ready && ((int32_t)(sd_led_until - now) > 0));
  LED_Write(Yellow_LED7_GPIO_Port, Yellow_LED7_Pin,
            ((int32_t)(can_led_until - now) > 0));
  LED_Write(Yellow_LED8_GPIO_Port, Yellow_LED8_Pin,
            ((int32_t)(rs232_led_until - now) > 0));
  LED_Write(Yellow_LED9_GPIO_Port, Yellow_LED9_Pin,
            (esp32_last_pass_ms != 0U) && ((now - esp32_last_pass_ms) < 2000U));
  LED_Write(Yellow_LED5_GPIO_Port, Yellow_LED5_Pin,
            (gps_valid_count != 0U) && ((now - gps_last_valid_ms) < GPS_VALID_TIMEOUT_MS));
  LED_Write(Red_LED10_GPIO_Port, Red_LED10_Pin, (error_flags != 0U));
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

  /* Reset all peripherals, initialize the Flash interface, and start SysTick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
#if ENABLE_STM32_INTERNAL_RTC
  MX_RTC_Init();
#endif
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* Bosch's delay callback uses the Cortex-M4 cycle counter for microseconds. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /*
   * Application startup begins after CubeMX has configured the clocks and
   * enabled peripheral registers.  The STM32 internal RTC uses HSE/16 and is
   * synchronized by valid GPS UTC or SET_RTC from the app.  The enabled
   * BMI270 is initialized and calibrated at rest before normal sampling.
   * External RTC and BME280 initialization remains feature-gated.
   */
  HAL_GPIO_WritePin(GPIOD, Green_LED11_Pin | Red_LED10_Pin | Yellow_LED9_Pin |
                    Yellow_LED8_Pin | Yellow_LED7_Pin | Yellow_LED6_Pin |
                    Yellow_LED5_Pin, LED_OFF);
  /* Every SPI chip-select is active-low, so idle all devices high. */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
  /* 0xFF clocks bytes out of the u-blox GPS without sending a command. */
  memset(gps_tx, 0xFF, sizeof(gps_tx));
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n[BOOT] telemetry_adv_v1 started; RTC=HSE/16; GPS/app time sync; BMI270 boot calibration; PCF85263A/BME disabled\r\n");
  printf("[LED] Y6=SD Y7=CAN Y8=RS232 Y9=ESP32 R10=error G11=heartbeat Y5=GPS\r\n");
  if (CAN_StartListenAll() != HAL_OK)
  {
    SetError(ERROR_CAN, 1U);
    printf("[CAN] listen-all start failed, error=0x%08lX\r\n", HAL_CAN_GetError(&hcan1));
  }
  else
  {
    printf("[CAN] listen-all started at 250000 bit/s\r\n");
  }

#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR || ENABLE_EXTERNAL_PCF85263A_RTC
  /* Compiled only when at least one external sensor/RTC feature is enabled. */
  FutureSensors_Init();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * Cooperative scheduler: each task returns quickly unless its interval
     * has elapsed.  There is no RTOS, so keeping this loop moving allows CAN
     * reception and the heartbeat/status LEDs to remain responsive.
     */
    /* CAN FIFO0 is drained by CAN1_RX0_IRQHandler; do not race it here. */
#if ENABLE_SWV_DEBUG_OUTPUT
    /* These tasks exist only to produce human-readable debugger output. */
    CAN_SWV_Task();
#endif
    GPS_Task();
    SD_Task();
    RS232_Task();
    ESP32_Task();
#if ENABLE_SWV_DEBUG_OUTPUT
    ErrorFlags_SWV_Task();
#endif
    StatusLED_Task();
#if ENABLE_SWV_DEBUG_OUTPUT
    STM32_RTC_Task();
#endif
#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR || ENABLE_EXTERNAL_PCF85263A_RTC
    /* Service only the external devices whose feature switches are enabled. */
    FutureSensors_Task();
#endif
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

  /** Configure the main internal regulator output voltage.
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initialize the RCC oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize the CPU, AHB, and APB bus clocks.
  */
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);

  /** Enable the clock security system.
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog filter.
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the digital filter.
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  /* Permit access to RTC calendar and backup registers in the backup domain. */
  HAL_PWR_EnableBkUpAccess();
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize the STM32 internal RTC peripheral.
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  /* 16 MHz HSE / 16 / (124 + 1) / (7999 + 1) = exactly 1 Hz. */
  hrtc.Init.AsynchPrediv = 124;
  hrtc.Init.SynchPrediv = 7999;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  /*
   * Do not reset the calendar on every MCU reset.  Backup register DR0 and the
   * RTC calendar survive resets while the backup domain remains powered.  This
   * board has no VBAT source, so complete power removal loses both values.
   */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == STM32_RTC_BACKUP_MAGIC)
  {
    stm32_rtc_ready = 1U;
    stm32_rtc_time_source = "PRESERVED";
    return;
  }

  /* First boot (or backup-domain loss): start from a documented default. */
  sTime.Hours = 0U;
  sTime.Minutes = 0U;
  sTime.Seconds = 0U;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = STM32_RTC_DEFAULT_WEEKDAY;
  sDate.Month = STM32_RTC_DEFAULT_MONTH;
  sDate.Date = STM32_RTC_DEFAULT_DAY;
  sDate.Year = (uint8_t)(STM32_RTC_DEFAULT_YEAR - 2000U);
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, STM32_RTC_BACKUP_MAGIC);
  stm32_rtc_ready = 1U;
  return;
  /* USER CODE END Check_RTC_BKUP */

  /** CubeMX-generated fallback calendar initialization (bypassed above).
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration. */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration. */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  /* APB1 is 32 MHz; divide by 16 for the GPS module's 2 MHz SPI clock. */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration. */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration. */
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure the ESP32 chip-select startup level. */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);

  /* Idle both active-low BMI270/BME280 chip selects before configuring pins. */
  HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin|BME_CS_Pin, GPIO_PIN_SET);

  /* Configure the GPS control-pin startup levels. */
  HAL_GPIO_WritePin(GPIOC, GPS_RST_Pin|GPS_CS_Pin|GPS_INT_Pin|GPS_CSC8_Pin, GPIO_PIN_RESET);

  /* Configure the active-low LED startup levels. */
  HAL_GPIO_WritePin(GPIOD, Green_LED11_Pin|Red_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin, GPIO_PIN_RESET);

  /* Configure the ESP32 chip-select output. */
  GPIO_InitStruct.Pin = ESP32_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP32_CS_GPIO_Port, &GPIO_InitStruct);

  /* Configure the ESP32 READY handshake input. */
  GPIO_InitStruct.Pin = ESP32_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESP32_Ready_GPIO_Port, &GPIO_InitStruct);

  /* Configure the SD-card detect and GPS pulse-per-second inputs. */
  GPIO_InitStruct.Pin = SD_Detect_Pin|GPS_PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*
   * The BMI270 drives IMU_INT toward the STM32.  It is a GPIO input for now;
   * EXTI can be enabled after the sensor's interrupt polarity/routing is set.
   */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /* Configure the BMI270 and BME280 active-low chip-select outputs. */
  GPIO_InitStruct.Pin = IMU_CS_Pin|BME_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure the GPS reset, chip-select, and module-control outputs. */
  GPIO_InitStruct.Pin = GPS_RST_Pin|GPS_CS_Pin|GPS_INT_Pin|GPS_CSC8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure all active-low status LED outputs. */
  GPIO_InitStruct.Pin = Green_LED11_Pin|Red_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Configure PC9 as the MCO2 clock output. */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure PA8 as the MCO1 clock output. */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure the external RTC interrupt signal as an input. */
  GPIO_InitStruct.Pin = RTC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTC_INT_GPIO_Port, &GPIO_InitStruct);

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
  HAL_GPIO_WritePin(Red_LED10_GPIO_Port, Red_LED10_Pin, LED_ON);
  HAL_GPIO_WritePin(Green_LED11_GPIO_Port, Green_LED11_Pin, LED_OFF);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* Add project-specific reporting here if full assertions are enabled. */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
