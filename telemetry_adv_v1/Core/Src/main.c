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
#include "telemetry_protocol.h"

/*
 * Future sensor feature switches
 * ------------------------------
 * Keep these at 0 while the devices do not respond.  The preprocessor removes
 * every guarded include, function, initialization call, and polling call, so
 * disabled devices produce no I2C/SPI traffic and cannot raise an error LED.
 *
 * Before changing a switch to 1, first configure the matching bus and pins in
 * telemetry_adv_v1.ioc and add the Bosch .c files to the CubeIDE build:
 *   BMI270: SPI1 with IMU_CS (bmi270.h includes bmi2.h/bmi2_defs.h)
 *   BME280: SPI1 with BME_CS (bme280.h includes bme280_defs.h)
 *   RTC:    I2C1 for the external PCF8523
 *
 * SPI1 is shared by the BMI270 and BME280 at 4 MHz.  Transfers use the
 * already-enabled SPI1 IRQ; chip select guarantees only one device responds.
 */
#define ENABLE_BMI270_IMU              0U
#define ENABLE_BME280_SENSOR           0U
#define ENABLE_STM32_INTERNAL_RTC       1U
#define ENABLE_EXTERNAL_PCF8523_RTC    0U

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
 * - SPI3 and FatFS append CAN/GPS snapshots to the SD card.
 * - SPI4 runs the tested request/response link check with the ESP32.
 * - CAN1 receives all bus traffic and USART1 sends the legacy raw_data block.
 * - SWV/ITM carries printf diagnostics without mixing them into USART1 data.
 *
 * The STM32 internal RTC is active.  External PCF8523, BME280, and IMU
 * access remains disabled until that hardware is ready.
 *
 * Transfer/interrupt policy
 * -------------------------
 * - CAN RX is interrupt-driven at NVIC priority 0, with a main-loop drain as
 *   a backup.  It can preempt every SPI/I2C task.
 * - Future BMI270/BME280 SPI1 transfers use HAL interrupt mode at priority 1.
 * - GPS SPI2 reads 256 bytes in about 1.024 ms at 2 MHz.  Polling this short
 *   burst is cheaper and more deterministic than roughly 256 HAL byte IRQs
 *   every 20 ms when DMA is not configured; CAN can still preempt it.
 * - FatFS requires synchronous disk operations, so SD SPI3 stays polling.
 *   Interrupting every SD byte would add thousands of IRQs per sector.
 * - ESP32 SPI4 moves only 32 bytes per transaction (about 64 us at 4 MHz), so
 *   polling has lower overhead while priority-0 CAN remains responsive.
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

/* Normalized date/time used by SD, RS232, and SWV regardless of RTC source. */
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Status LEDs are active-low on this PCB. */
#define LED_ON                         GPIO_PIN_RESET
#define LED_OFF                        GPIO_PIN_SET
#define HEARTBEAT_PERIOD_MS            1000U
#define HEARTBEAT_ON_MS                100U
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
#define KNOTS_TO_MPH                   1.15077945f
#define KMH_TO_MPH                     0.62137119f
#define ESP32_READY_TIMEOUT_MS         100U
#define ESP32_SPI_TIMEOUT_MS           100U
#define CAN_SWV_MIN_PERIOD_MS          20U
#define CAN_SWV_STATUS_PERIOD_MS       5000U
#define RS232_SWV_ECHO_BLOCK           1U
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
static float latest_gps_lat_deg;
static float latest_gps_lon_deg;
static float latest_gps_speed_mph;
static uint32_t latest_gps_fix_ms;
static uint32_t latest_gps_speed_ms;
static uint8_t latest_gps_fix_valid;
static uint8_t latest_gps_speed_valid;

/* ESP32 counters are useful watch-window diagnostics during bench testing. */
static uint8_t esp32_sequence;
static uint32_t esp32_pass_count;
static uint32_t esp32_fail_count;
static uint32_t esp32_last_pass_ms;
static volatile int32_t esp32_last_result;

/* Active time source until the external PCF8523 is commissioned. */
static uint8_t stm32_rtc_ready;
static uint32_t stm32_rtc_last_print_ms;

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
static void CAN_SWV_Task(void);
static void SD_Task(void);
static void SD_LogSnapshot(void);
static void GPS_Task(void);
static uint8_t GPS_GetField(const char *sentence, uint8_t field_index,
                            char *out, size_t out_len);
static uint8_t GPS_IsNavSentence(const char *sentence);
static uint8_t GPS_ParseCoordDeg(const char *value, const char *hemisphere,
                                 float *coord_deg);
static uint8_t GPS_ParseSpeedMph(const char *sentence, float *speed_mph);
static void GPS_UpdateNavFromSentence(const char *sentence);
static void FormatSignedFixed6(float value, char *out, size_t out_len);
static void ESP32_Task(void);
static void RS232_Task(void);
static void StatusLED_Task(void);
static void ErrorFlags_SWV_Task(void);
static void SetError(uint32_t mask, uint8_t active);
static uint8_t STM32_RTC_Read(TelemetryDateTime_t *date_time);
static void TelemetryDateTime_Format(const TelemetryDateTime_t *date_time,
                                     char *out, size_t out_len);
static void BoardUptime_Format(uint32_t uptime_ms, char *out, size_t out_len);
static void STM32_RTC_Task(void);

#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR || ENABLE_EXTERNAL_PCF8523_RTC
static void FutureSensors_Init(void);
static void FutureSensors_Task(void);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  /*
   * Newlib printf() eventually calls this function.  ITM stimulus port 0 is
   * used only when SWV is enabled by the debugger; otherwise characters are
   * discarded so a disconnected debugger can never block the main loop.
   */
  if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U) &&
      ((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0U) && ((ITM->TER & 1UL) != 0U))
  {
    ITM_SendChar((uint32_t)(uint8_t)ch);
  }
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
    printf("[STM32 RTC] %s source=LSI uptime=%s\r\n", text, uptime);
  }
  else
  {
    printf("[STM32 RTC] read failed\r\n");
  }
#endif
}

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

static void CAN_Store(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
  /* Cache the newest frame for SD/GPS telemetry and update the legacy table. */
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
 * Disabled future-sensor integration
 * ==================================
 * This section records how the working Telemetry_Reference connected the
 * devices.  Because every ENABLE_* switch is currently 0, none of the code in
 * this section is part of the firmware binary.  The bus-handle/pin names below
 * intentionally document what CubeMX must provide before enabling a device.
 */

#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR
/*
 * SPI1 transfer completion is signaled by HAL_SPI_IRQHandler() through these
 * flags.  __WFI() sleeps the CPU between interrupts instead of busy-polling
 * the SPI status register.  SysTick still wakes the CPU to enforce a timeout.
 */
static volatile uint8_t future_spi1_done;
static volatile uint8_t future_spi1_error;

static HAL_StatusTypeDef Future_SPI1_Wait(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while ((future_spi1_done == 0U) && (future_spi1_error == 0U))
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      (void)HAL_SPI_Abort(&hspi1);
      return HAL_TIMEOUT;
    }
    __WFI();
  }
  return (future_spi1_error == 0U) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef Future_SPI1_Transmit(uint8_t *data, uint16_t len)
{
  future_spi1_done = 0U;
  future_spi1_error = 0U;
  HAL_StatusTypeDef result = HAL_SPI_Transmit_IT(&hspi1, data, len);
  return (result == HAL_OK) ? Future_SPI1_Wait(100U) : result;
}

static HAL_StatusTypeDef Future_SPI1_Receive(uint8_t *data, uint16_t len)
{
  future_spi1_done = 0U;
  future_spi1_error = 0U;
  HAL_StatusTypeDef result = HAL_SPI_Receive_IT(&hspi1, data, len);
  return (result == HAL_OK) ? Future_SPI1_Wait(100U) : result;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) future_spi1_done = 1U;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) future_spi1_done = 1U;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) future_spi1_error = 1U;
}
#endif

#if ENABLE_BMI270_IMU
/* Board connection: BMI270 on SPI1 with active-low IMU_CS. */
static struct bmi2_dev future_bmi270;

static int8_t Future_BMI270_SPI_Read(uint8_t reg, uint8_t *data,
                                    uint32_t len, void *intf_ptr)
{
  (void)intf_ptr;
  uint8_t address = reg | 0x80U;
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = Future_SPI1_Transmit(&address, 1U);
  if (result == HAL_OK) result = Future_SPI1_Receive(data, (uint16_t)len);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return (result == HAL_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static int8_t Future_BMI270_SPI_Write(uint8_t reg, const uint8_t *data,
                                     uint32_t len, void *intf_ptr)
{
  (void)intf_ptr;
  uint8_t address = reg & 0x7FU;
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = Future_SPI1_Transmit(&address, 1U);
  if (result == HAL_OK)
    result = Future_SPI1_Transmit((uint8_t *)data, (uint16_t)len);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return (result == HAL_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static void Future_BMI270_DelayUs(uint32_t period_us, void *intf_ptr)
{
  (void)intf_ptr;
  HAL_Delay((period_us + 999U) / 1000U);
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
  future_bmi270.intf_ptr = NULL;
  future_bmi270.read_write_len = 32U;

  int8_t result = bmi270_init(&future_bmi270);
  if (result != BMI2_OK) return result;

  config[0].type = BMI2_ACCEL;
  config[1].type = BMI2_GYRO;
  result = bmi2_get_sensor_config(config, 2U, &future_bmi270);
  if (result != BMI2_OK) return result;

  config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  result = bmi2_set_sensor_config(config, 2U, &future_bmi270);
  return (result == BMI2_OK) ?
         bmi2_sensor_enable(sensors, 2U, &future_bmi270) : result;
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

#if ENABLE_EXTERNAL_PCF8523_RTC
/* Board connection: PCF8523 RTC on interrupt-enabled I2C1. */
#define FUTURE_PCF8523_ADDRESS         (0x68U << 1)
#define FUTURE_PCF8523_CONTROL1        0x00U
#define FUTURE_PCF8523_SECONDS         0x03U
static volatile uint8_t future_i2c1_done;
static volatile uint8_t future_i2c1_error;

static HAL_StatusTypeDef Future_I2C1_Wait(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while ((future_i2c1_done == 0U) && (future_i2c1_error == 0U))
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      (void)HAL_I2C_Master_Abort_IT(&hi2c1, FUTURE_PCF8523_ADDRESS);
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

static uint8_t Future_PCF8523_Init(void)
{
  uint8_t control1 = 0U; /* Clearing STOP allows the oscillator to run. */
  future_i2c1_done = 0U;
  future_i2c1_error = 0U;
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write_IT(&hi2c1,
      FUTURE_PCF8523_ADDRESS, FUTURE_PCF8523_CONTROL1,
      I2C_MEMADD_SIZE_8BIT, &control1, 1U);
  if (result == HAL_OK) result = Future_I2C1_Wait(100U);
  return (result == HAL_OK) ? 1U : 0U;
}

static uint8_t Future_PCF8523_ReadRaw(uint8_t registers[7])
{
  /* Raw BCD order: seconds, minutes, hours, day, weekday, month, year. */
  future_i2c1_done = 0U;
  future_i2c1_error = 0U;
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read_IT(&hi2c1,
      FUTURE_PCF8523_ADDRESS, FUTURE_PCF8523_SECONDS,
      I2C_MEMADD_SIZE_8BIT, registers, 7U);
  if (result == HAL_OK) result = Future_I2C1_Wait(100U);
  return (result == HAL_OK) ? 1U : 0U;
}
#endif /* ENABLE_EXTERNAL_PCF8523_RTC */

#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR || ENABLE_EXTERNAL_PCF8523_RTC
static void FutureSensors_Init(void)
{
#if ENABLE_BMI270_IMU
  printf("[FUTURE IMU] init result=%d\r\n", Future_BMI270_Init());
#endif
#if ENABLE_BME280_SENSOR
  printf("[FUTURE BME] init result=%d\r\n", Future_BME280_Init());
#endif
#if ENABLE_EXTERNAL_PCF8523_RTC
  printf("[FUTURE RTC] init result=%u\r\n", Future_PCF8523_Init());
#endif
}

static void FutureSensors_Task(void)
{
  static uint32_t last_ms;
  if ((HAL_GetTick() - last_ms) < 1000U) return;
  last_ms = HAL_GetTick();

#if ENABLE_BMI270_IMU
  struct bmi2_sens_data imu_data = {0};
  if (bmi2_get_sensor_data(&imu_data, &future_bmi270) == BMI2_OK)
    printf("[FUTURE IMU] acc=%d,%d,%d gyro=%d,%d,%d\r\n",
           imu_data.acc.x, imu_data.acc.y, imu_data.acc.z,
           imu_data.gyr.x, imu_data.gyr.y, imu_data.gyr.z);
#endif
#if ENABLE_BME280_SENSOR
  struct bme280_data bme_data = {0};
  if (bme280_get_sensor_data(BME280_ALL, &bme_data, &future_bme280) == BME280_OK)
    printf("[FUTURE BME] sample received\r\n");
#endif
#if ENABLE_EXTERNAL_PCF8523_RTC
  uint8_t registers[7];
  if (Future_PCF8523_ReadRaw(registers))
    printf("[FUTURE RTC] raw=%02X:%02X:%02X date=%02X/%02X/%02X\r\n",
           registers[2], registers[1], registers[0],
           registers[5], registers[3], registers[6]);
#endif
}
#endif

static void CAN_SWV_Task(void)
{
  /*
   * Printing inside the CAN interrupt would make the interrupt too slow.
   * Instead, take an atomic copy here in the main loop and print the newest
   * frame.  The 20 ms limit caps SWV traffic at 50 lines/second; "skipped"
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
   * loss—important for vehicle telemetry.
   */
  /* New filename avoids mixing RTC-aware rows with an older TELEM.CSV header. */
  static const char path[] = "0:/TELEMRTC.CSV";
  static const char header[] =
      "tick_ms,stm32_rtc,rtc_valid,can_count,can_id,extended,dlc,data,gps_fix_valid,"
      "gps_lat_deg,gps_lon_deg,gps_speed_valid,gps_speed_mph\r\n";
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
    uint32_t speed_hundredths = (uint32_t)(latest_gps_speed_mph * 100.0f + 0.5f);
    if (result == FR_OK && f_printf(&USERFile,
        "%lu,%s,%u,%lu,0x%lX,%u,%u,%s,%u,%s,%s,%u,%lu.%02lu\r\n",
        HAL_GetTick(), rtc_text, date_time.valid,
        can_latest.count, can_latest.id, can_latest.extended,
        can_latest.dlc, hex, latest_gps_fix_valid, lat, lon,
        latest_gps_speed_valid, speed_hundredths / 100U,
        speed_hundredths % 100U) < 0) result = FR_DISK_ERR;
    if (result == FR_OK) result = f_sync(&USERFile);
    if (f_close(&USERFile) != FR_OK && result == FR_OK) result = FR_DISK_ERR;
  }
  if (result == FR_OK)
  {
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
  /* Mount on insertion, retry failed mounts, and append one row per second. */
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
      SD_LogSnapshot();
    }
    else SetError(ERROR_SD, 1U);
  }
  else if ((now - last_log_ms) >= TELEMETRY_PERIOD_MS)
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

static void FormatSignedFixed6(float value, char *out, size_t out_len)
{
  /* Avoid requiring floating-point printf support just to log coordinates. */
  int32_t scaled = (int32_t)(value * 1000000.0f);
  uint32_t magnitude = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;
  (void)snprintf(out, out_len, "%s%lu.%06lu", (scaled < 0) ? "-" : "",
                 magnitude / 1000000U, magnitude % 1000000U);
}

static void GPS_UpdateNavFromSentence(const char *sentence)
{
  /*
   * RMC: active fix, latitude, longitude, and speed in knots.
   * GGA: latitude/longitude when fix quality is nonzero.
   * VTG: ground speed without a position fix.
   */
  char status[8], lat[20], ns[4], lon[20], ew[4];
  float lat_deg = 0.0f, lon_deg = 0.0f, speed_mph = 0.0f;
  uint8_t updated_fix = 0U;
  uint32_t now = HAL_GetTick();

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
  }

  if (updated_fix != 0U)
  {
    latest_gps_lat_deg = lat_deg;
    latest_gps_lon_deg = lon_deg;
    latest_gps_fix_valid = 1U;
    latest_gps_fix_ms = now;
  }

  if ((now - gps_last_nav_print_ms) >= GPS_NAV_PRINT_PERIOD_MS)
  {
    char lat_text[20], lon_text[20];
    uint32_t speed_hundredths = (uint32_t)(latest_gps_speed_mph * 100.0f + 0.5f);
    gps_last_nav_print_ms = now;
    FormatSignedFixed6(latest_gps_lat_deg, lat_text, sizeof(lat_text));
    FormatSignedFixed6(latest_gps_lon_deg, lon_text, sizeof(lon_text));
    printf("[GPS] nav=%lu filtered=%lu checksum_err=%lu fix=%u lat=%s lon=%s speed=%lu.%02lu mph\r\n",
           gps_valid_count, gps_filtered_count, gps_checksum_error_count,
           latest_gps_fix_valid, lat_text, lon_text,
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
  /* Expire stale values so old fixes are never presented as current data. */
  if (latest_gps_fix_valid && ((now - latest_gps_fix_ms) > GPS_FIX_VALID_MS))
    latest_gps_fix_valid = 0U;
  if (latest_gps_speed_valid && ((now - latest_gps_speed_ms) > GPS_SPEED_VALID_MS))
    latest_gps_speed_valid = 0U;

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
   * held low for exactly one 32-byte frame.  After CS rises, READY must return
   * low before another frame is allowed; this prevents reading a stale reply.
   * Return values: 0=success, -1=not ready, -2=SPI error, -3=READY stuck high.
   */
  uint32_t start = HAL_GetTick();
  while (HAL_GPIO_ReadPin(ESP32_Ready_GPIO_Port, ESP32_Ready_Pin) == GPIO_PIN_RESET)
    if ((HAL_GetTick() - start) >= ESP32_READY_TIMEOUT_MS) return -1;
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi4, tx, rx,
      TELEMETRY_FRAME_SIZE, ESP32_SPI_TIMEOUT_MS);
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

static void ESP32_Task(void)
{
  /*
   * Send a changing full-payload PING once a second to exercise MOSI and MISO.
   * The ESP32 cannot return the response until it processes the request after
   * CS rises, so a second NOP transaction clocks that stored response out.
   */
  static uint32_t last_ms;
  uint8_t request[TELEMETRY_FRAME_SIZE], ignored[TELEMETRY_FRAME_SIZE];
  uint8_t nop[TELEMETRY_FRAME_SIZE], response[TELEMETRY_FRAME_SIZE];
  uint8_t payload[TELEMETRY_PAYLOAD_SIZE];
  uint32_t now = HAL_GetTick();
  if ((now - last_ms) < TELEMETRY_PERIOD_MS) return;
  last_ms = now;
  for (uint8_t i = 0U; i < sizeof(payload); i++) payload[i] = (uint8_t)(can_latest.count + i);
  uint8_t request_sequence = ++esp32_sequence;
  telemetry_frame_build(request, TELEMETRY_CMD_PING, request_sequence,
                        TELEMETRY_STATUS_OK, payload, sizeof(payload));
  telemetry_frame_build(nop, TELEMETRY_CMD_NOP, ++esp32_sequence,
                        TELEMETRY_STATUS_OK, NULL, 0U);
  int32_t result = ESP32_Exchange(request, ignored);
  if (result == 0) result = ESP32_Exchange(nop, response);
  if ((result == 0) && (telemetry_frame_validate(response) == TELEMETRY_STATUS_OK) &&
      (response[TELEMETRY_OFFSET_COMMAND] == (TELEMETRY_CMD_PING | TELEMETRY_RESPONSE_BIT)) &&
      (response[TELEMETRY_OFFSET_SEQUENCE] == request_sequence) &&
      (response[TELEMETRY_OFFSET_LENGTH] == sizeof(payload)) &&
      (memcmp(&response[TELEMETRY_OFFSET_PAYLOAD], payload, sizeof(payload)) == 0))
  {
    esp32_last_result = 0;
    esp32_pass_count++;
    esp32_last_pass_ms = now;
    SetError(ERROR_ESP32, 0U);
  }
  else
  {
    /* Preserve link-stage failures; use -4 for a malformed/incorrect reply. */
    esp32_last_result = (result != 0) ? result : -4;
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
  char lat[20], lon[20];
  char rtc_text[24];
  char uptime_text[24];
  TelemetryDateTime_t date_time = {0};
  FormatSignedFixed6(latest_gps_lat_deg, lat, sizeof(lat));
  FormatSignedFixed6(latest_gps_lon_deg, lon, sizeof(lon));
  (void)STM32_RTC_Read(&date_time);
  TelemetryDateTime_Format(&date_time, rtc_text, sizeof(rtc_text));
  BoardUptime_Format(now, uptime_text, sizeof(uptime_text));
  uint32_t speed_hundredths = latest_gps_speed_valid ?
      (uint32_t)(latest_gps_speed_mph * 100.0f + 0.5f) : 0U;
  int n = snprintf(&block[pos], sizeof(block) - pos,
      "BME,DISABLED\r\n"
      "NAV,IMU_MPH=0.00,GPS_MPH=%lu.%02lu,GPS_VALID=%u,"
      "VEHICLE_MPH=%lu.%02lu,SOURCE=%s,LAT=%s,LON=%s,FIX=%u,AGE_MS=%lu\r\n"
      "TL_TIM,%s,UPTIME_MS=%lu\r\n"
      "TL_UPT,%s\r\nVWXYZ\r\n",
      speed_hundredths / 100U, speed_hundredths % 100U,
      latest_gps_speed_valid, speed_hundredths / 100U,
      speed_hundredths % 100U, latest_gps_speed_valid ? "GPS" : "NONE",
      lat, lon, latest_gps_fix_valid,
      latest_gps_fix_valid ? (now - latest_gps_fix_ms) : 0U,
      rtc_text, now, uptime_text);
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

static void ErrorFlags_SWV_Task(void)
{
  /* Print only when the bitmask changes so the reason for red LED 10 is clear. */
  static uint32_t previous_flags = 0xFFFFFFFFUL;
  uint32_t flags = error_flags;
  if (flags == previous_flags) return;
  previous_flags = flags;

  printf("[ERROR FLAGS] mask=0x%02lX SD=%u CAN=%u RS232=%u ESP32=%u ",
         flags,
         (flags & ERROR_SD) ? 1U : 0U,
         (flags & ERROR_CAN) ? 1U : 0U,
         (flags & ERROR_RS232) ? 1U : 0U,
         (flags & ERROR_ESP32) ? 1U : 0U);
  printf("esp_result=%ld esp_pass=%lu esp_fail=%lu sd_ready=%u\r\n",
         (long)esp32_last_result, esp32_pass_count, esp32_fail_count, sd_ready);
}

static void StatusLED_Task(void)
{
  /*
   * Y6=SD write, Y7=CAN RX, Y8=RS232 TX, Y9=recent ESP32 success,
   * Y5=fresh filtered GPS navigation, R10=latched subsystem error,
   * and G11=100 ms heartbeat once per second.
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
  /*
   * Application startup begins here after CubeMX has configured the clocks
   * and enabled the selected peripheral registers.  RTC/I2C, BME280, and IMU
   * initialization is intentionally omitted until that hardware is working.
   */
  HAL_GPIO_WritePin(GPIOD, Green_LED11_Pin | Red_LED10_Pin | Yellow_LED9_Pin |
                    Yellow_LED8_Pin | Yellow_LED7_Pin | Yellow_LED6_Pin |
                    Yellow_LED5_Pin, LED_OFF);
  /* Every SPI chip-select is active-low, so idle all devices high. */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
  /* 0xFF clocks bytes out of the u-blox GPS without sending a command. */
  memset(gps_tx, 0xFF, sizeof(gps_tx));
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n[BOOT] telemetry_adv_v1 started; STM32 RTC active; PCF8523/BME/IMU disabled\r\n");
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

#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR || ENABLE_EXTERNAL_PCF8523_RTC
  /* This call does not exist in the current binary because all switches are 0. */
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
    CAN_SWV_Task();
    GPS_Task();
    SD_Task();
    RS232_Task();
    ESP32_Task();
    ErrorFlags_SWV_Task();
    StatusLED_Task();
    STM32_RTC_Task();
#if ENABLE_BMI270_IMU || ENABLE_BME280_SENSOR || ENABLE_EXTERNAL_PCF8523_RTC
    /* Likewise, no future sensor is polled while its switch remains 0. */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  /** Initializes the CPU, AHB and APB buses clocks
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

  /** Enables the Clock Security System
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
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

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
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
   * RTC domain survive normal resets when VBAT/backup power is available.
   */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == STM32_RTC_BACKUP_MAGIC)
  {
    stm32_rtc_ready = 1U;
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

  /** Initialize RTC and set the Time and Date
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
  /* SPI1 parameter configuration*/
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
  /* SPI2 parameter configuration*/
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
  /* SPI3 parameter configuration*/
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
  /* SPI4 parameter configuration*/
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IMU_INT_Pin|IMU_CS_Pin|BME_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPS_RST_Pin|GPS_CS_Pin|GPS_INT_Pin|GPS_CSC8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_LED11_Pin|Red_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ESP32_CS_Pin */
  GPIO_InitStruct.Pin = ESP32_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP32_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP32_Ready_Pin */
  GPIO_InitStruct.Pin = ESP32_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESP32_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_Detect_Pin GPS_PPS_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin|GPS_PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_INT_Pin IMU_CS_Pin BME_CS_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin|IMU_CS_Pin|BME_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_RST_Pin GPS_CS_Pin GPS_INT_Pin GPS_CSC8_Pin */
  GPIO_InitStruct.Pin = GPS_RST_Pin|GPS_CS_Pin|GPS_INT_Pin|GPS_CSC8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_LED11_Pin Red_LED10_Pin Yellow_LED9_Pin Yellow_LED8_Pin
                           Yellow_LED7_Pin Yellow_LED6_Pin Yellow_LED5_Pin */
  GPIO_InitStruct.Pin = Green_LED11_Pin|Red_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_INT_Pin */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
