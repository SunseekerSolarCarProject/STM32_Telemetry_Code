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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include "bmi270.h"

#define BME280_FLOAT_ENABLE
#include "bme280.h"

/*
 * File overview
 * -------------
 * main.c owns the application-level orchestration for the telemetry board:
 * - BMI270 IMU setup and motion estimation.
 * - GPS SPI byte draining, NMEA filtering, and lat/lon/speed extraction.
 * - SD/FatFS CSV logging.
 * - ESP32 SPI telemetry/command exchange for the Bluetooth/app path.
 * - CAN listen-all capture and legacy Sunseeker raw_data RS232 block output.
 * - BME280 environmental reads and status LED activity pulses.
 *
 * CubeMX-generated peripheral init functions remain near the bottom of the
 * file.  Most application behavior is inside USER CODE sections above them.
 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 * Date/time snapshot used for both the Adalogger RTC and CSV/RS232 output.
 * The PCF8523 RTC stores fields separately, so keeping this normalized struct
 * makes validation, printing, and SET_RTC handling easier.
 */
typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t weekday;
  uint8_t valid;
} DateTime_t;

/*
 * Local IMU status values.  These are higher-level than the Bosch API result
 * codes and are meant for startup/debug decisions in this application.
 */
typedef enum
{
  IMU_OK = 0,
  IMU_ERR_NO_ACK = 1,
  IMU_ERR_READ_FAIL = 2,
  IMU_ERR_BAD_ID = 3,
  IMU_ERR_CONFIG = 4
} IMU_Status_t;

/*
 * Raw BMI270 sample values are signed 16-bit counts from the sensor.
 * Conversion to mg/mdps happens separately so logs can include either form.
 */
typedef struct
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyr_x;
  int16_t gyr_y;
  int16_t gyr_z;
} BMI270_Raw_t;

/*
 * Converted BMI270 values use integer units that are easy to print and log:
 * accel in milli-g, gyro in milli-degrees/second.
 */
typedef struct
{
  int32_t acc_x_mg;
  int32_t acc_y_mg;
  int32_t acc_z_mg;
  int32_t gyr_x_mdps;
  int32_t gyr_y_mdps;
  int32_t gyr_z_mdps;
} BMI270_Conv_t;

/*
 * Latest CAN frame cache.  Interrupt/poll receive paths update this struct,
 * while the telemetry builders take snapshots from it for logging.
 */
typedef struct
{
  uint32_t id;
  uint8_t is_extended;
  uint8_t dlc;
  uint8_t data[8];
  uint32_t count;
  uint8_t new_msg;
} STM32_CAN_Rx_t;

/*
 * SunRawEntry_t drives the RS232 "raw_data" block.  Each expected CAN packet
 * gets one table row so the output always has the same order, even before a
 * particular CAN ID has arrived.
 */
typedef struct
{
  uint32_t can_id;
  const char *name;
  uint32_t high_word;
  uint32_t low_word;
  uint8_t valid;
  uint32_t rx_count;
  uint32_t last_ms;
} SunRawEntry_t;

/*
 * Logical LED activities.  The LED task turns these into short pulses and an
 * error blink without each subsystem needing to know LED pin details.
 */
typedef enum
{
  STATUS_RS232_TX = 0,
  STATUS_CAN_RX,
  STATUS_ESP32_TX,
  STATUS_SD_LOG,
  STATUS_GPS_RX,
  STATUS_BME_READ,
  STATUS_BOOT_OK,
  STATUS_ACTIVITY_COUNT
} StatusActivity_t;

/* Pair a status LED GPIO port and pin in one compact table entry. */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} StatusLedPin_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * BMI270 register/address constants.
 * The Bosch driver handles most register work, but these addresses are used
 * by local startup diagnostics and status reads.
 */
#define BMI270_ADDR_LOW      (0x68 << 1)
#define BMI270_ADDR_HIGH     (0x69 << 1)
#define BMI270_REG_CHIP_ID   0x00
#define BMI270_EXPECTED_ID   0x24
#define BMI270_REG_DATA_8    		0x0C
#define BMI270_REG_ERR_REG          0x02
#define BMI270_REG_STATUS           0x03
#define BMI270_REG_INTERNAL_STATUS  0x21
#define BMI270_REG_SENSORTIME_0     0x18
#define BMI270_REG_ACC_CONF         0x40
#define BMI270_REG_ACC_RANGE        0x41
#define BMI270_REG_GYR_CONF         0x42
#define BMI270_REG_GYR_RANGE        0x43
#define BMI270_REG_PWR_CONF         0x7C
#define BMI270_REG_PWR_CTRL         0x7D

/*
 * Adalogger RTC is a PCF8523-compatible device on I2C.
 * Register names match the PCF8523 datasheet to make SET_RTC/readback easier
 * to audit when the RTC is not holding time.
 */
#define ADALOGGER_RTC_ADDR          (0x68 << 1)
#define PCF8523_REG_CONTROL1        0x00
#define PCF8523_REG_CONTROL2        0x01
#define PCF8523_REG_CONTROL3        0x02
#define PCF8523_REG_SECONDS         0x03
#define PCF8523_REG_MINUTES         0x04
#define PCF8523_REG_HOURS           0x05
#define PCF8523_REG_DAYS            0x06
#define PCF8523_REG_WEEKDAYS        0x07
#define PCF8523_REG_MONTHS          0x08
#define PCF8523_REG_YEARS           0x09
#define SET_ADALOGGER_RTC_ON_BOOT  0

/*
 * IMU speed-estimate tuning.
 *
 * Important limitation: the BMI270 does not provide reliable vehicle speed by
 * itself for long periods.  The code below performs real acceleration
 * integration, but it still drifts and should be treated as a fallback estimate.
 * Actual vehicle speed should come from CAN first, then GPS.
 */
#define GRAVITY_MPS2        	  	 9.80665f
#define ACC_1G_MG                    1000
#define STILL_GYRO_LIMIT_MDPS        5000     // 5 dps
#define STILL_ACC_MAG_TOL_MG         120      // accel magnitude must be near 1g
#define MOVE_LIN_ACC_THRESHOLD_MG    180
#define IMU_ACCEL_DEADBAND_MG        35       // ignore tiny residual accel bias
#define IMU_MAX_DT_MS                2000     // reset integration after long gaps
#define IMU_MAX_SPEED_MPH            120.0f   // reject runaway integration
#define IMU_SAMPLE_PERIOD_MS         100      // 10 Hz integration update
#define TELEMETRY_PERIOD_MS          1000     // 1 Hz SD/ESP32/debug output
#define STILL_COUNT_LIMIT            8
#define VELOCITY_DAMPING             0.92f    // per-update drift damping

/*
 * GPS SPI/NMEA settings.
 * GPS is read as a stream of bytes.  Larger SPI chunks and a byte ring buffer
 * keep the STM32 from falling behind when the GPS emits many NMEA sentences.
 */
#define GPS_SPI_READ_LEN 	  256
#define GPS_NMEA_BUF_LEN      128
#define GPS_SPI_TIMEOUT_MS    100
#define GPS_POLL_PERIOD_MS    50
#define GPS_PARSE_BUDGET_BYTES 1024
#define GPS_BYTE_RING_SIZE    4096
#define GPS_NAV_PRINT_PERIOD_MS 1000
#define GPS_SPEED_VALID_MS    5000
#define GPS_FIX_VALID_MS      5000
#define KNOTS_TO_MPH          1.15077945f
#define KMH_TO_MPH            0.62137119f
#define MPS_TO_MPH            2.23693629f
#define CAN_SPEED_VALID_MS    1000
#define CAN_SPEED_MAX_ABS_MPH 120.0f

/*
 * ESP32 telemetry link.
 * STM32 sends either a one-time CSV header, a command response, or a $LOG CSV
 * row in each SPI transaction.  ESP32_READY gates each transfer.
 */
#define ESP32_TX_FRAME_LEN       768
#define TELEMETRY_LINE_LEN       640
#define TELEMETRY_CSV_HEADER     "time_ms," \
                                 "adalogger_rtc,adalogger_rtc_valid," \
                                 "acc_x_mg,acc_y_mg,acc_z_mg," \
                                 "gyr_x_mdps,gyr_y_mdps,gyr_z_mdps," \
                                 "imu_speed_mph,gps_speed_mph,gps_speed_valid," \
                                 "can_speed_mph,can_speed_valid,can_speed_source," \
                                 "gps_lat_deg,gps_lon_deg,gps_fix_valid,gps_fix_age_ms," \
                                 "vehicle_speed_mph,vehicle_speed_source," \
                                 "bme_temp_c,bme_pressure_pa,bme_humidity_pct," \
                                 "can_rx_count,can_id,can_ext,can_dlc,can_data"
#define ESP32_SPI_TIMEOUT_MS     100
#define ESP32_SEND_PERIOD_MS     1000
#define ESP32_READY_ACTIVE       GPIO_PIN_SET
#define ESP32_RX_FRAME_LEN       ESP32_TX_FRAME_LEN
#define ESP32_RESPONSE_LEN       160
#define ESP32_CMD_LEN            160
#define BME280_SPI_TIMEOUT_MS    100
#define ADALOGGER_RTC_I2C_TIMEOUT_MS  100
#define ADALOGGER_RTC_I2C_RETRIES     2

/*
 * RS232 raw_data block settings.
 * The block is sent once per second and keeps legacy CAN hex rows plus a
 * compact NAV line for display software at 115200 baud.
 */
#define SUN_RAW_LINE_LEN        64
#define SUN_RAW_BLOCK_LEN       4096
#define SUN_RAW_SEND_PERIOD_MS  1000

#define STATUS_TICK_MS          20
#define STATUS_PULSE_MS         120
#define STATUS_HEARTBEAT_MS     500
#define STATUS_ERROR_BLINK_MS   250

#define STATUS_LED_ACTIVE       GPIO_PIN_SET
#define STATUS_LED_INACTIVE     GPIO_PIN_RESET

// MPPT Controller Addresses
#define MPPT_CAN_BASE       0x600       // CAN Base Address to send RTR requests
#define MPPT_CAN_ONOFF      0x10        // CAN Base Address to send on/off messages to the MPPTs
#define MPPT_CAN_ADDRESS1       0x00        // Address to specify MPPT 1
#define MPPT_CAN_ADDRESS2       0x01        // Address to specify MPPT 2

// Motor controller CAN base address and packet offsets
#define MC_CAN_BASE1        0x400       // High = CAN1_SERIAL Number        Low = 0x00004003                    P=1s
#define MC_CAN_BASE2        0x420       // High = CAN1_SERIAL Number        Low = 0x00004003                    P=1s
#define MC_LIMITS           0x01        // High = CAN_Err,Active Motor      Low = Error & Limit flags           P=200ms
#define MC_BUS              0x02        // High = Bus Current               Low = Bus Voltage                   P=200ms
#define MC_VELOCITY         0x03        // High = Velocity (m/s)            Low = Velocity (rpm)                P=200ms
#define MC_PHASE            0x04        // High = Phase C Current           Low = Phase B Current               P=200ms
#define MC_V_VECTOR         0x05        // High = Vd vector                 Low = Vq vector                     P=200ms
#define MC_I_VECTOR         0x06        // High = Id vector                 Low = Iq vector                     P=200ms
#define MC_BEMF_VECTOR      0x07        // High = BEMFd vector              Low = BEMFq vector                  P=200ms
#define MC_RAIL1            0x08        // High = 15V                       Low = Reserved                      P=1s
#define MC_RAIL2            0x09        // High = 3.3V                      Low = 1.9V                          P=1s
//#define MC_FAN            0x0A        // High = Reserved                  Low = Reserved                      P=
#define MC_TEMP1            0x0B        // High = Heatsink Temp (case)      Low = Motor Temp (internal)         P=1s
#define MC_TEMP2            0x0C        // High = Reserved                  Low = DSP Temp                      P=1s
//#define MC_TEMP3          0x0D        // High = Outlet Temp               Low = Capacitor Temp                P=
#define MC_CUMULATIVE       0x0E        // High = DC Bus AmpHours (A-Hr)    Low = Odometer  (m)                 P=1s
#define MC_SLIPSPEED        0x17        // High = Slip Speed (Hz       )    Low = Reserved                      P=200ms

// Driver controls CAN base address and packet offsets
#define DC_CAN_BASE         0x500       // High = CAN1_SERIAL Number        Low = "TRIb" string                 P=1s
#define DC_DRIVE            0x01        // High = Motor Current Setpoint    Low = Motor Velocity Setpoint       P=100ms
#define DC_POWER            0x02        // High = Bus Current Setpoint      Low = Unused                        P=100ms
#define DC_RESET            0x03        // High = Unused                    Low = Unused                        P=
#define DC_SWITCH           0x04        // High = Switch position           Low = Switch state change           P=100ms

// Steering Wheel CAN base address and packet offsets
#define STW_CAN_BASE         0x540       // High = CAN1_SERIAL Number        Low = "TRIb" string                 P=1s
#define STW_SWITCH           0x01        // High = Switch position           Low = Switch state change           P=100ms
//
#define STW_HORN       0x0001
#define STW_IND_L      0x0002
#define STW_IND_R      0x0004
#define STW_REGEN      0x0008
#define STW_CRUISE     0x0010

// Steering Wheel CAN base address and packet offsets
#define STW_CAN_BASE         0x540       // High = CAN1_SERIAL Number        Low = "CTv1" string                 P=1s
#define STW_SWITCH           0x01        // High = Switch position           Low = Switch state change           P=100ms

//Battery Protection System base address and packet offsets
#define BP_CAN_BASE         0x580       // High = "BPV1" string or nulls    Low = CAN1_SERIAL Number            P=10s
#define BP_VMAX             0x01        // High = Max. Voltage Value        Low = Max. Voltage Cell Num.        P=10s
#define BP_VMIN             0x02        // High = Min. Voltage Value        Low = Min. Voltage Cell Num.        P=10s
#define BP_TMAX             0x03        // High = Max. Temperature          Low = Max. Temperature Cell         P=10s
#define BP_PCDONE           0x04        // High = "BPV2" or "0000" string   Low = CAN1_SERIAL Number            P=When Ready
#define BP_ISH              0x05        // High = Shunt Current             Low = Battery SOC               P=1s
#define BP_PVSS             0x06        // High = Pack Voltage              Low = Shunt Sum                     P=1s
#define BP_RESET            0x07        // High = Unused                    Low = Unused                        P=
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*
 * IMU gravity/motion estimator state.
 * Gravity is learned while the car is physically still, then subtracted from
 * accelerometer readings to estimate linear acceleration.
 */
static float gravity_x_mg = 0.0f;
static float gravity_y_mg = 0.0f;
static float gravity_z_mg = 1000.0f;

static float vel_x_mps = 0.0f;
static float vel_y_mps = 0.0f;
static float vel_z_mps = 0.0f;
static uint32_t last_velocity_tick = 0;

/*
 * GPS receive state.
 * SPI reads write into gps_spi_rx, then complete blocks are copied into the
 * ring buffer. The NMEA parser drains the ring buffer later so short blocking
 * work elsewhere does not immediately lose GPS bytes.
 */
static char gps_nmea_buf[GPS_NMEA_BUF_LEN];
static uint16_t gps_nmea_index = 0;
static uint8_t gps_sentence_active = 0;
static uint8_t gps_spi_tx[GPS_SPI_READ_LEN];
static uint8_t gps_spi_rx[GPS_SPI_READ_LEN];
static volatile uint8_t gps_spi_busy = 0;
static volatile uint8_t gps_spi_done = 0;
static volatile uint8_t gps_spi_error = 0;
static uint32_t gps_spi_start_ms = 0;
static uint8_t gps_byte_ring[GPS_BYTE_RING_SIZE];
static uint16_t gps_byte_head = 0;
static uint16_t gps_byte_tail = 0;
static uint16_t gps_byte_count = 0;
static uint32_t gps_byte_overflow_count = 0;
static uint32_t gps_last_nav_print_ms = 0;

/* FatFS objects for the SD log file.  sd_ready gates all runtime writes. */
static FATFS fs;
static FIL log_file;
static uint8_t sd_ready = 0;

/*
 * Latest processed navigation and speed values.
 * Source priority for vehicle speed is CAN -> GPS -> IMU integrated estimate.
 */
static char latest_gps_sentence[GPS_NMEA_BUF_LEN];
static float latest_imu_mph = 0.0f;
static float latest_gps_lat_deg = 0.0f;
static float latest_gps_lon_deg = 0.0f;
static uint8_t latest_gps_fix_valid = 0;
static uint32_t latest_gps_fix_ms = 0;
static float latest_gps_speed_mph = 0.0f;
static uint8_t latest_gps_speed_valid = 0;
static uint32_t latest_gps_speed_ms = 0;
static float latest_can_speed_mph = 0.0f;
static uint8_t latest_can_speed_valid = 0;
static uint32_t latest_can_speed_ms = 0;
static const char *latest_can_speed_source = "NONE";
static float latest_vehicle_speed_mph = 0.0f;
static const char *latest_vehicle_speed_source = "NONE";

/*
 * ESP32 SPI transaction state.
 * The ESP32 is full-duplex SPI: every STM32 transmit can also receive a command
 * frame from the ESP32, so TX and RX buffers are both frame-sized.
 */
static uint32_t esp32_seq = 0;
static uint8_t esp32_tx_frame[ESP32_TX_FRAME_LEN];
static uint8_t esp32_rx_frame[ESP32_RX_FRAME_LEN];
static volatile uint8_t esp32_spi_busy = 0;
static volatile uint8_t esp32_spi_done = 0;
static volatile uint8_t esp32_spi_error = 0;
static uint32_t esp32_spi_start_ms = 0;

static char esp32_response[ESP32_RESPONSE_LEN];
static uint8_t esp32_response_pending = 0;
static uint8_t esp32_header_pending = 1;

/* Runtime switch controlled by ESP32 commands; SD must also be mounted. */
static uint8_t sd_logging_enabled = 1;

/* Sensor/device health flags used by startup summary and runtime tasks. */
static uint8_t adalogger_rtc_ok = 0;

static struct bme280_dev bme280_dev;
static uint8_t bme280_ok = 0;
static volatile uint8_t bme280_spi_done = 0;
static volatile uint8_t bme280_spi_error = 0;

static float latest_bme_temp_c = 0.0f;
static float latest_bme_pressure_pa = 0.0f;
static float latest_bme_humidity_pct = 0.0f;

static STM32_CAN_Rx_t latest_can_rx = {0};

/* Activity LEDs are driven by StatusLed_Task(), not directly by subsystems. */
static const StatusLedPin_t status_leds[STATUS_ACTIVITY_COUNT] =
{
  { GPIOD, LED1_Pin  },  // RS232 TX
  { GPIOD, LED2_Pin  },  // CAN RX
  { GPIOD, LED3_Pin  },  // ESP32 TX
  { GPIOD, LED4_Pin  },  // SD log
  { GPIOD, LED5_Pin  },  // GPS RX
  { GPIOD, LED6_Pin },  // BME280 read
  { GPIOD, LED7_Pin }   // Boot OK / spare
};

static volatile uint8_t status_activity_request[STATUS_ACTIVITY_COUNT];
static uint32_t status_activity_until[STATUS_ACTIVITY_COUNT];

static uint8_t status_error_latched = 0;

/*
 * Fixed RS232 CAN table.  The output order does not depend on receive order,
 * which makes the downstream parser/display simpler.
 */
static SunRawEntry_t sun_raw_table[] =
{
  { MC_CAN_BASE1 + MC_BUS,        "MC1BUS", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE1 + MC_VELOCITY,   "MC1VEL", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE1 + MC_TEMP1,      "MC1TP1", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE1 + MC_TEMP2,      "MC1TP2", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE1 + MC_PHASE,      "MC1PHA", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE1 + MC_CUMULATIVE, "MC1CUM", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE1 + MC_V_VECTOR,   "MC1VVC", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE1 + MC_I_VECTOR,   "MC1IVC", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE1 + MC_BEMF_VECTOR,"MC1BEM", 0, 0, 0, 0, 0 },

  { MC_CAN_BASE2 + MC_BUS,        "MC2BUS", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_VELOCITY,   "MC2VEL", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_TEMP1,      "MC2TP1", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_TEMP2,      "MC2TP2", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_PHASE,      "MC2PHA", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_CUMULATIVE, "MC2CUM", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_V_VECTOR,   "MC2VVC", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_I_VECTOR,   "MC2IVC", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_BEMF_VECTOR,"MC2BEM", 0, 0, 0, 0, 0 },

  { DC_CAN_BASE + DC_DRIVE,       "DC_DRV", 0, 0, 0, 0, 0 },
  { DC_CAN_BASE + DC_SWITCH,      "DC_SWC", 0, 0, 0, 0, 0 },

  { BP_CAN_BASE + BP_VMAX,        "BP_VMX", 0, 0, 0, 0, 0 },
  { BP_CAN_BASE + BP_VMIN,        "BP_VMN", 0, 0, 0, 0, 0 },
  { BP_CAN_BASE + BP_TMAX,        "BP_TMX", 0, 0, 0, 0, 0 },
  { BP_CAN_BASE + BP_ISH,         "BP_ISH", 0, 0, 0, 0, 0 },
  { BP_CAN_BASE + BP_PVSS,        "BP_PVS", 0, 0, 0, 0, 0 },

  { MC_CAN_BASE1 + MC_LIMITS,     "MC1LIM", 0, 0, 0, 0, 0 },
  { MC_CAN_BASE2 + MC_LIMITS,     "MC2LIM", 0, 0, 0, 0, 0 }
};

#define SUN_RAW_TABLE_COUNT  (sizeof(sun_raw_table) / sizeof(sun_raw_table[0]))

static char sun_raw_block[SUN_RAW_BLOCK_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static int32_t raw_acc_to_mg(int16_t raw);
static int32_t raw_gyro_to_mdps(int16_t raw);
static int32_t abs_i32(int32_t value);
static void BMI270_InitGravityEstimate(void);
static void BMI270_UpdateVelocityAndPrint(struct bmi2_sens_data *sensor_data,
                                          uint8_t print_debug);

static void GPS_Select(void);
static void GPS_Deselect(void);
static void GPS_InitPins(void);
static void GPS_PollSPI(void);
static void GPS_ServiceSPI(void);
static void GPS_BufferPush(uint8_t byte);
static uint8_t GPS_BufferPop(uint8_t *byte);
static void GPS_BufferRxBlock(const uint8_t *data, uint16_t len);
static void GPS_ProcessBufferedBytes(uint16_t max_bytes);
static void GPS_ProcessByte(uint8_t byte);
static uint8_t GPS_GetField(const char *sentence,
                            uint8_t field_index,
                            char *out,
                            size_t out_len);
static uint8_t GPS_IsNavSentence(const char *sentence);
static uint8_t GPS_ParseCoordDeg(const char *value,
                                 const char *hemisphere,
                                 float *coord_deg);
static uint8_t GPS_ParseSpeedMph(const char *sentence, float *speed_mph);
static void GPS_UpdateNavFromSentence(const char *sentence);
static void VehicleSpeed_UpdateFromSources(void);

static void ESP32_Select(void);
static void ESP32_Deselect(void);
static uint8_t ESP32_IsReady(void);
static void ESP32_SendTelemetry(struct bmi2_sens_data *sensor_data);
static void ESP32_ServiceSPI(void);
static void ESP32_ProcessRxFrame(const uint8_t *rx, size_t len);
static void ESP32_HandleCommand(char *cmd);
static void ESP32_ExecuteCommand(const char *id, const char *verb);
static void ESP32_QueueResponse(const char *id,
                                const char *status,
                                const char *message);

static void SD_LogStart(void);
static void SD_LogStop(void);

static uint8_t DateTime_IsValid(const DateTime_t *dt);
static uint8_t DateTime_CalcWeekday(uint16_t year, uint8_t month, uint8_t day);

static int Telemetry_BuildCSVLine(struct bmi2_sens_data *sensor_data,
                                  char *line,
                                  size_t line_size);


static uint8_t ADALOGGER_RTC_Init(void);
static uint8_t ADALOGGER_RTC_Read(DateTime_t *dt);
static uint8_t ADALOGGER_RTC_Set(const DateTime_t *dt);
static void ADALOGGER_RTC_Print(void);
static void ADALOGGER_RTC_RecoverI2CBus(const char *reason);

static void DateTime_ToString(const DateTime_t *dt, char *out, size_t out_len);

static uint8_t BME280_InitSensor(void);
static uint8_t BME280_ReadAndPrint(void);

static uint8_t STM32_CAN_ListenAllInit(void);
static void STM32_CAN_PrintLatest(void);
static void CAN_DataToHex(const uint8_t *data, uint8_t len, char *out, size_t out_len);
static void STM32_CAN_StoreRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
static void STM32_CAN_PollRx(void);
static void STM32_CAN_DebugPrint(void);
static uint8_t STM32_CAN_SendTestFrame(uint32_t std_id, uint8_t *data, uint8_t dlc);

static void Print_StartupSummary(int8_t bmi_result);

static uint32_t CAN_MakeU32_LE(const uint8_t *data);
static float CAN_U32ToFloat(uint32_t raw);
static void VehicleSpeed_UpdateFromCANVelocity(uint32_t id, uint32_t velocity_word);
static void SunRaw_UpdateFromCAN(uint32_t id, uint8_t dlc, const uint8_t *data);
static int SunRaw_BuildBlock(char *out, size_t out_len);
static void RS232_SendString(const char *text);
static void RS232_SendSunRawBlock(void);

static void StatusLed_InitState(void);
static void StatusLed_Task(void);
static void StatusLed_RequestPulse(StatusActivity_t activity);
static void StatusLed_SetError(void);
static void StatusLed_BootSequence(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SWV_SendChar(char ch)
{
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
  {
    return;
  }

  if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U)
  {
    return;
  }

  if ((ITM->TER & 1UL) == 0U)
  {
    return;
  }

  uint32_t timeout = 10000U;

  while ((ITM->PORT[0].u32 == 0UL) && (timeout > 0U))
  {
    timeout--;
  }

  if (timeout > 0U)
  {
    ITM->PORT[0].u8 = (uint8_t)ch;
  }
}

int _write(int file, char *ptr, int len)
{
  (void)file;

  for (int i = 0; i < len; i++)
  {
    SWV_SendChar(ptr[i]);
  }

  return len;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    GPS_Deselect();
    gps_spi_error = 0;
    gps_spi_busy = 0;
    gps_spi_done = 1;
  }
  else if (hspi->Instance == SPI3)
  {
    ESP32_Deselect();
    esp32_spi_error = 0;
    esp32_spi_busy = 0;
    esp32_spi_done = 1;
  }
  else if (hspi->Instance == SPI5)
  {
    bme280_spi_error = 0;
    bme280_spi_done = 1;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI5)
  {
    bme280_spi_error = 0;
    bme280_spi_done = 1;
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI5)
  {
    bme280_spi_error = 0;
    bme280_spi_done = 1;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    GPS_Deselect();
    gps_spi_error = 1;
    gps_spi_busy = 0;
    gps_spi_done = 1;
  }
  else if (hspi->Instance == SPI3)
  {
    ESP32_Deselect();
    esp32_spi_error = 1;
    esp32_spi_busy = 0;
    esp32_spi_done = 1;
  }
  else if (hspi->Instance == SPI5)
  {
    bme280_spi_error = 1;
    bme280_spi_done = 1;
  }
}

static void Print_StartupSummary(int8_t bmi_result)
{
  printf("\r\n================ SYSTEM INIT SUMMARY ================\r\n");

  printf("ADALOGGER_RTC:     %s\r\n",
         adalogger_rtc_ok ? "OK" : "FAILED");

  printf("SD CARD:           %s\r\n",
         sd_ready ? "OK" : "FAILED");

  printf("CAN1 LISTEN-ALL:   started, check CAN RX prints for traffic\r\n");

  printf("BMI270 IMU:        %s, result=%d\r\n",
         (bmi_result == BMI2_OK) ? "OK" : "FAILED",
         bmi_result);

  printf("BME280 SENSOR:     %s\r\n",
         bme280_ok ? "OK" : "FAILED");

  printf("GPS SPI:           interrupt transfer mode, waiting for NMEA sentences\r\n");

  printf("ESP32 SPI3:        ready-check interrupt transmit/receive enabled\r\n");

  printf("=====================================================\r\n");
  printf("Starting repeated data loop...\r\n\r\n");
}

static const char *FatFs_ErrorString(FRESULT res)
{
  switch (res)
  {
    case FR_OK: return "FR_OK";
    case FR_DISK_ERR: return "FR_DISK_ERR";
    case FR_INT_ERR: return "FR_INT_ERR";
    case FR_NOT_READY: return "FR_NOT_READY";
    case FR_NO_FILE: return "FR_NO_FILE";
    case FR_NO_PATH: return "FR_NO_PATH";
    case FR_INVALID_NAME: return "FR_INVALID_NAME";
    case FR_DENIED: return "FR_DENIED";
    case FR_EXIST: return "FR_EXIST";
    case FR_INVALID_OBJECT: return "FR_INVALID_OBJECT";
    case FR_WRITE_PROTECTED: return "FR_WRITE_PROTECTED";
    case FR_INVALID_DRIVE: return "FR_INVALID_DRIVE";
    case FR_NOT_ENABLED: return "FR_NOT_ENABLED";
    case FR_NO_FILESYSTEM: return "FR_NO_FILESYSTEM";
    case FR_MKFS_ABORTED: return "FR_MKFS_ABORTED";
    case FR_TIMEOUT: return "FR_TIMEOUT";
    case FR_LOCKED: return "FR_LOCKED";
    case FR_NOT_ENOUGH_CORE: return "FR_NOT_ENOUGH_CORE";
    case FR_TOO_MANY_OPEN_FILES: return "FR_TOO_MANY_OPEN_FILES";
    case FR_INVALID_PARAMETER: return "FR_INVALID_PARAMETER";
    default: return "UNKNOWN";
  }
}

static struct bmi2_dev bmi;
static uint8_t bmi270_addr = 0x68;   // debug showed addr = 0x68

static int8_t stm32_bmi2_i2c_read(uint8_t reg_addr,
                                  uint8_t *reg_data,
                                  uint32_t len,
                                  void *intf_ptr)
{
  uint8_t addr_7bit = *((uint8_t *)intf_ptr);

  if (HAL_I2C_Mem_Read(&hi2c1,
                       addr_7bit << 1,
                       reg_addr,
                       I2C_MEMADD_SIZE_8BIT,
                       reg_data,
                       (uint16_t)len,
                       1000) == HAL_OK)
  {
    return BMI2_OK;
  }

  return BMI2_E_COM_FAIL;
}

static int8_t stm32_bmi2_i2c_write(uint8_t reg_addr,
                                   const uint8_t *reg_data,
                                   uint32_t len,
                                   void *intf_ptr)
{
  uint8_t addr_7bit = *((uint8_t *)intf_ptr);

  if (HAL_I2C_Mem_Write(&hi2c1,
                        addr_7bit << 1,
                        reg_addr,
                        I2C_MEMADD_SIZE_8BIT,
                        (uint8_t *)reg_data,
                        (uint16_t)len,
                        1000) == HAL_OK)
  {
    return BMI2_OK;
  }

  return BMI2_E_COM_FAIL;
}

static void stm32_bmi2_delay_us(uint32_t period_us, void *intf_ptr)
{
  (void)intf_ptr;

  uint32_t delay_ms = (period_us + 999U) / 1000U;

  if (delay_ms == 0U)
  {
    delay_ms = 1U;
  }

  HAL_Delay(delay_ms);
}

static int8_t BMI270_API_Init(void)
{
  int8_t rslt;
  uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  struct bmi2_sens_config config[2];

  memset(&bmi, 0, sizeof(bmi));

  bmi.intf = BMI2_I2C_INTF;
  bmi.read = stm32_bmi2_i2c_read;
  bmi.write = stm32_bmi2_i2c_write;
  bmi.delay_us = stm32_bmi2_delay_us;
  bmi.intf_ptr = &bmi270_addr;

  /*
   * This chunk size matters for the config-file upload.
   * 32 bytes is a safe I2C burst size for STM32 HAL testing.
   */
  bmi.read_write_len = 32;

  rslt = bmi270_init(&bmi);
  printf("bmi270_init result = %d\r\n", rslt);

  if (rslt != BMI2_OK)
  {
    return rslt;
  }

  config[0].type = BMI2_ACCEL;
  config[1].type = BMI2_GYRO;

  rslt = bmi2_get_sensor_config(config, 2, &bmi);
  printf("get config result = %d\r\n", rslt);

  if (rslt != BMI2_OK)
  {
    return rslt;
  }

  config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

  config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  rslt = bmi2_set_sensor_config(config, 2, &bmi);
  printf("set config result = %d\r\n", rslt);

  if (rslt != BMI2_OK)
  {
    return rslt;
  }

  rslt = bmi2_sensor_enable(sensor_list, 2, &bmi);
  printf("sensor enable result = %d\r\n", rslt);

  return rslt;
}

static void BMI270_InitGravityEstimate(void)
{
  struct bmi2_sens_data sensor_data = { 0 };

  printf("Keep IMU still for gravity estimate...\r\n");

  HAL_Delay(200);

  if (bmi2_get_sensor_data(&sensor_data, &bmi) == BMI2_OK)
  {
    gravity_x_mg = (float)raw_acc_to_mg(sensor_data.acc.x);
    gravity_y_mg = (float)raw_acc_to_mg(sensor_data.acc.y);
    gravity_z_mg = (float)raw_acc_to_mg(sensor_data.acc.z);

    vel_x_mps = 0.0f;
    vel_y_mps = 0.0f;
    vel_z_mps = 0.0f;
    last_velocity_tick = HAL_GetTick();

    printf("Gravity estimate mg: X=%ld Y=%ld Z=%ld\r\n",
           (long)gravity_x_mg,
           (long)gravity_y_mg,
           (long)gravity_z_mg);
  }
}

static void BMI270_PrintInternalStatus(void)
{
  uint8_t internal_status = 0;

  HAL_I2C_Mem_Read(&hi2c1,
                   bmi270_addr << 1,
                   0x21,
                   I2C_MEMADD_SIZE_8BIT,
                   &internal_status,
                   1,
                   100);

  printf("BMI270 INTERNAL_STATUS = 0x%02X\r\n", internal_status);
}

static void LED_AllOff(void)
{
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
}

static int32_t raw_acc_to_mg(int16_t raw)
{
  /*
   * Your BMI270 API setup used +/-2g.
   * For +/-2g, 16384 LSB = 1g = 1000 mg.
   */
  return ((int32_t)raw * 1000L) / 16384L;
}

static int32_t abs_i32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t raw_gyro_to_mdps(int16_t raw)
{
  /*
   * Gyro range is +/-2000 dps:
   * mdps = raw * 2000000 / 32768
   */
  return (int32_t)(((int64_t)raw * 2000000LL) / 32768LL);
}

static void BMI270_UpdateVelocityAndPrint(struct bmi2_sens_data *sensor_data,
                                          uint8_t print_debug)
{
	  /*
	   * Convert the raw Bosch API sample into engineering units first.  Keeping
	   * the conversion local makes the subsequent still/moving checks easier to
	   * read and keeps all thresholds in mg/mdps.
	   */
	  int32_t ax_mg = raw_acc_to_mg(sensor_data->acc.x);
	  int32_t ay_mg = raw_acc_to_mg(sensor_data->acc.y);
	  int32_t az_mg = raw_acc_to_mg(sensor_data->acc.z);

	  int32_t gx_mdps = raw_gyro_to_mdps(sensor_data->gyr.x);
	  int32_t gy_mdps = raw_gyro_to_mdps(sensor_data->gyr.y);
	  int32_t gz_mdps = raw_gyro_to_mdps(sensor_data->gyr.z);

	  float acc_mag_mg = sqrtf((float)(ax_mg * ax_mg) +
	                           (float)(ay_mg * ay_mg) +
	                           (float)(az_mg * az_mg));

	  int32_t gyro_sum_mdps =
	      abs_i32(gx_mdps) +
	      abs_i32(gy_mdps) +
	      abs_i32(gz_mdps);

	  uint8_t gyro_still = (gyro_sum_mdps < STILL_GYRO_LIMIT_MDPS);
	  uint8_t accel_near_1g =
	      (fabsf(acc_mag_mg - ACC_1G_MG) < STILL_ACC_MAG_TOL_MG);

	  /*
	   * "Physically still" means:
	   * - the gyro is quiet, so the sensor is not rotating, and
	   * - total acceleration is close to 1g, so it is mostly gravity.
	   */
	  uint8_t physically_still = gyro_still && accel_near_1g;

	  /*
	   * While still, update gravity estimate to whatever angle the IMU is sitting at.
	   * This prevents a tilted-but-still IMU from being treated as moving.
	   */
	  if (physically_still)
	  {
	    gravity_x_mg = (0.90f * gravity_x_mg) + (0.10f * (float)ax_mg);
	    gravity_y_mg = (0.90f * gravity_y_mg) + (0.10f * (float)ay_mg);
	    gravity_z_mg = (0.90f * gravity_z_mg) + (0.10f * (float)az_mg);
	  }

	  int32_t lin_ax_mg = (int32_t)((float)ax_mg - gravity_x_mg);
	  int32_t lin_ay_mg = (int32_t)((float)ay_mg - gravity_y_mg);
	  int32_t lin_az_mg = (int32_t)((float)az_mg - gravity_z_mg);

	  /*
	   * Linear acceleration is what remains after subtracting estimated gravity.
	   * This is useful for motion detection, but integrating it into speed drifts
	   * very quickly in a real vehicle.  CAN/GPS remain the real speed sources.
	   */
	  float lin_acc_mag_mg = sqrtf((float)(lin_ax_mg * lin_ax_mg) +
	                               (float)(lin_ay_mg * lin_ay_mg) +
	                               (float)(lin_az_mg * lin_az_mg));

	  uint8_t moving = 0;

	  if (!physically_still && lin_acc_mag_mg > MOVE_LIN_ACC_THRESHOLD_MG)
	  {
	    moving = 1;
	  }

	  uint32_t now_ms = HAL_GetTick();
	  uint32_t dt_ms = now_ms - last_velocity_tick;

	  if ((last_velocity_tick == 0U) || (dt_ms > IMU_MAX_DT_MS))
	  {
	    /*
	     * If the sample gap is too large, integration would create a bogus
	     * velocity jump.  Reset the integration clock and keep the previous
	     * velocity state bounded.
	     */
	    dt_ms = 0U;
	  }

	  last_velocity_tick = now_ms;

	  if (physically_still)
	  {
	    /*
	     * Zero-velocity update: when the IMU is actually still, force velocity
	     * to zero.  This is the main thing that prevents drift from growing
	     * without bound while the car is parked.
	     */
	    vel_x_mps = 0.0f;
	    vel_y_mps = 0.0f;
	    vel_z_mps = 0.0f;
	  }
	  else if (dt_ms > 0U)
	  {
	    float dt_s = (float)dt_ms / 1000.0f;
	    float lin_ax_mps2 = 0.0f;
	    float lin_ay_mps2 = 0.0f;
	    float lin_az_mps2 = 0.0f;

	    /*
	     * Convert linear acceleration from mg to m/s^2 after a small deadband.
	     * The deadband removes residual gravity/bias errors that otherwise
	     * integrate into speed even when the car is not truly accelerating.
	     */
	    if (abs_i32(lin_ax_mg) > IMU_ACCEL_DEADBAND_MG)
	    {
	      lin_ax_mps2 = ((float)lin_ax_mg / 1000.0f) * GRAVITY_MPS2;
	    }

	    if (abs_i32(lin_ay_mg) > IMU_ACCEL_DEADBAND_MG)
	    {
	      lin_ay_mps2 = ((float)lin_ay_mg / 1000.0f) * GRAVITY_MPS2;
	    }

	    if (abs_i32(lin_az_mg) > IMU_ACCEL_DEADBAND_MG)
	    {
	      lin_az_mps2 = ((float)lin_az_mg / 1000.0f) * GRAVITY_MPS2;
	    }

	    /*
	     * Integrate acceleration into velocity on each sensor axis.
	     * This is a real physics calculation: v = v + a * dt.
	     */
	    vel_x_mps += lin_ax_mps2 * dt_s;
	    vel_y_mps += lin_ay_mps2 * dt_s;
	    vel_z_mps += lin_az_mps2 * dt_s;

	    /*
	     * Small damping limits long-term drift from accelerometer bias.  Without
	     * GPS/CAN correction, IMU-only speed will still drift over time.
	     */
	    vel_x_mps *= VELOCITY_DAMPING;
	    vel_y_mps *= VELOCITY_DAMPING;
	    vel_z_mps *= VELOCITY_DAMPING;
	  }
	  else
	  {
	    /* No elapsed time means no integration this pass. */
	  }

	  float imu_speed_mps = sqrtf((vel_x_mps * vel_x_mps) +
	                              (vel_y_mps * vel_y_mps) +
	                              (vel_z_mps * vel_z_mps));

	  latest_imu_mph = imu_speed_mps * MPS_TO_MPH;

	  if (latest_imu_mph > IMU_MAX_SPEED_MPH)
	  {
	    vel_x_mps = 0.0f;
	    vel_y_mps = 0.0f;
	    vel_z_mps = 0.0f;
	    latest_imu_mph = 0.0f;
	  }

	  int32_t speed_cmph = (int32_t)(latest_imu_mph * 100.0f);
	  VehicleSpeed_UpdateFromSources();

	  if (print_debug)
	  {
	    printf("ACC mg: X=%6ld Y=%6ld Z=%6ld | "
	           "LIN mg: X=%6ld Y=%6ld Z=%6ld | "
	           "LIN_MAG=%6ld mg | "
	           "GYR mdps: X=%7ld Y=%7ld Z=%7ld | "
	           "IMU_SPEED_MPH=%ld.%02ld | VEHICLE_MPH=%.2f(%s) | %s\r\n",
	           (long)ax_mg, (long)ay_mg, (long)az_mg,
	           (long)lin_ax_mg, (long)lin_ay_mg, (long)lin_az_mg,
	           (long)lin_acc_mag_mg,
	           (long)gx_mdps, (long)gy_mdps, (long)gz_mdps,
	           (long)(speed_cmph / 100),
	           (long)abs_i32(speed_cmph % 100),
	           (double)latest_vehicle_speed_mph,
	           latest_vehicle_speed_source,
	           moving ? "MOVING" : "STILL");
	  }
}

static void GPS_Select(void)
{
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_RESET);
}

static void GPS_Deselect(void)
{
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_SET);
}

static void GPS_InitPins(void)
{
  GPS_Deselect();

  // Release GPS reset
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);

  // Give GPS time to boot
  HAL_Delay(1000);

  printf("GPS pins initialized\r\n");
}

static void GPS_BufferPush(uint8_t byte)
{
  /*
   * Push one raw GPS byte into the ring.  When the ring is full, discard the
   * oldest byte instead of rejecting the newest one.  For live navigation,
   * newest GPS fixes are more valuable than stale buffered sentences.
   */
  gps_byte_ring[gps_byte_head] = byte;
  gps_byte_head = (uint16_t)((gps_byte_head + 1U) % GPS_BYTE_RING_SIZE);

  if (gps_byte_count < GPS_BYTE_RING_SIZE)
  {
    gps_byte_count++;
  }
  else
  {
    gps_byte_tail = (uint16_t)((gps_byte_tail + 1U) % GPS_BYTE_RING_SIZE);
    gps_byte_overflow_count++;
  }
}

static uint8_t GPS_BufferPop(uint8_t *byte)
{
  /* Return 0 when the parser has caught up with all buffered GPS bytes. */
  if ((byte == NULL) || (gps_byte_count == 0U))
  {
    return 0;
  }

  *byte = gps_byte_ring[gps_byte_tail];
  gps_byte_tail = (uint16_t)((gps_byte_tail + 1U) % GPS_BYTE_RING_SIZE);
  gps_byte_count--;

  return 1;
}

static void GPS_BufferRxBlock(const uint8_t *data, uint16_t len)
{
  /*
   * Queue a completed SPI receive block quickly.  Parsing is intentionally done
   * later so the SPI service path stays short and can start the next transfer.
   */
  if (data == NULL)
  {
    return;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    GPS_BufferPush(data[i]);
  }
}

static void GPS_ProcessBufferedBytes(uint16_t max_bytes)
{
  uint8_t byte;

  /*
   * Bounded parser budget prevents GPS parsing from monopolizing the main loop
   * if a large burst arrives.  The task runs frequently, so it catches up over
   * several passes without starving CAN, LEDs, or ESP32 service.
   */
  while ((max_bytes > 0U) && GPS_BufferPop(&byte))
  {
    GPS_ProcessByte(byte);
    max_bytes--;
  }
}

static void GPS_ProcessByte(uint8_t byte)
{
  /*
   * u-blox SPI often returns 0xFF when no byte is available.
   * Ignore it.
   */
  if (byte == 0xFF)
  {
    return;
  }

  /*
   * Start of NMEA sentence.
   */
  if (byte == '$')
  {
    /* A new '$' always starts a fresh NMEA sentence. */
    gps_sentence_active = 1;
    gps_nmea_index = 0;
    gps_nmea_buf[gps_nmea_index++] = (char)byte;
    return;
  }

  /*
   * Ignore bytes until a sentence starts.
   */
  if (!gps_sentence_active)
  {
    return;
  }

  /*
   * Store byte if there is room.
   */
  if (gps_nmea_index < (GPS_NMEA_BUF_LEN - 1))
  {
    gps_nmea_buf[gps_nmea_index++] = (char)byte;
  }
  else
  {
    /*
     * Overflow protection: reset the sentence.
     */
    gps_sentence_active = 0;
    gps_nmea_index = 0;
    return;
  }

  /*
   * End of NMEA sentence is usually '\n'.
   */
  if (byte == '\n')
  {
	  gps_nmea_buf[gps_nmea_index] = '\0';

	  /* Remove \n and optional \r before saving to CSV */
	  while ((gps_nmea_index > 0) &&
	         ((gps_nmea_buf[gps_nmea_index - 1] == '\n') ||
	          (gps_nmea_buf[gps_nmea_index - 1] == '\r')))
	  {
	    gps_nmea_index--;
	    gps_nmea_buf[gps_nmea_index] = '\0';
	  }

	  if (GPS_IsNavSentence(gps_nmea_buf))
	  {
	    /*
	     * Keep only navigation sentences.  This avoids logging/parsing the GPS
	     * module's extra status/debug sentences and reduces downstream delay.
	     */
	    strncpy(latest_gps_sentence, gps_nmea_buf, GPS_NMEA_BUF_LEN - 1);
	    latest_gps_sentence[GPS_NMEA_BUF_LEN - 1] = '\0';
	    GPS_UpdateNavFromSentence(gps_nmea_buf);
	    StatusLed_RequestPulse(STATUS_GPS_RX);
	  }

	  gps_sentence_active = 0;
	  gps_nmea_index = 0;
  }
}

static uint8_t GPS_GetField(const char *sentence,
                            uint8_t field_index,
                            char *out,
                            size_t out_len)
{
  /*
   * Extract comma-delimited NMEA fields without using strtok().
   * strtok() modifies the source string and keeps global parser state, which is
   * not a good fit for repeatedly parsing the same sentence in embedded code.
   */
  uint8_t current_field = 0;
  size_t out_pos = 0;

  if ((sentence == NULL) || (out == NULL) || (out_len == 0))
  {
    return 0;
  }

  out[0] = '\0';

  for (const char *p = sentence; *p != '\0'; p++)
  {
    char ch = *p;

    if ((ch == ',') || (ch == '*') || (ch == '\r') || (ch == '\n'))
    {
      if (current_field == field_index)
      {
        out[out_pos] = '\0';
        return 1;
      }

      current_field++;

      if (ch != ',')
      {
        break;
      }

      continue;
    }

    if (current_field == field_index)
    {
      if (out_pos < (out_len - 1U))
      {
        out[out_pos++] = ch;
      }
    }
  }

  if (current_field == field_index)
  {
    out[out_pos] = '\0';
    return 1;
  }

  return 0;
}

static uint8_t GPS_IsNavSentence(const char *sentence)
{
  /*
   * NMEA talker IDs vary (GP, GN, GL, etc.), but the message type sits at
   * sentence[3..5].  RMC/GGA/VTG provide the navigation values this firmware
   * actually uses.
   */
  if ((sentence == NULL) ||
      (strlen(sentence) < 6U) ||
      ((sentence[0] != '$') && (sentence[0] != '!')))
  {
    return 0;
  }

  return ((strncmp(&sentence[3], "RMC", 3) == 0) ||
          (strncmp(&sentence[3], "GGA", 3) == 0) ||
          (strncmp(&sentence[3], "VTG", 3) == 0));
}

static uint8_t GPS_ParseCoordDeg(const char *value,
                                 const char *hemisphere,
                                 float *coord_deg)
{
  /*
   * NMEA coordinates are ddmm.mmmm for latitude and dddmm.mmmm for longitude.
   * Dividing the integer degree portion from the minutes portion converts them
   * to normal signed decimal degrees for logging/app display.
   */
  float raw;
  int degrees;
  float minutes;

  if ((value == NULL) ||
      (hemisphere == NULL) ||
      (coord_deg == NULL) ||
      (value[0] == '\0') ||
      (hemisphere[0] == '\0'))
  {
    return 0;
  }

  raw = strtof(value, NULL);
  degrees = (int)(raw / 100.0f);
  minutes = raw - ((float)degrees * 100.0f);

  *coord_deg = (float)degrees + (minutes / 60.0f);

  if ((hemisphere[0] == 'S') || (hemisphere[0] == 'W'))
  {
    *coord_deg = -*coord_deg;
  }

  return 1;
}

static uint8_t GPS_ParseSpeedMph(const char *sentence, float *speed_mph)
{
  /*
   * RMC speed is knots.  VTG may provide knots and/or km/h.
   * The parsed value is normalized to mph so it can be compared with CAN speed.
   */
  char field[20];

  if ((sentence == NULL) ||
      (speed_mph == NULL) ||
      (strlen(sentence) < 6U) ||
      ((sentence[0] != '$') && (sentence[0] != '!')))
  {
    return 0;
  }

  if ((strncmp(&sentence[3], "RMC", 3) == 0) &&
      GPS_GetField(sentence, 2, field, sizeof(field)) &&
      (field[0] == 'A') &&
      GPS_GetField(sentence, 7, field, sizeof(field)) &&
      (field[0] != '\0'))
  {
    *speed_mph = strtof(field, NULL) * KNOTS_TO_MPH;
    return 1;
  }

  if (strncmp(&sentence[3], "VTG", 3) == 0)
  {
    if (GPS_GetField(sentence, 5, field, sizeof(field)) &&
        (field[0] != '\0'))
    {
      *speed_mph = strtof(field, NULL) * KNOTS_TO_MPH;
      return 1;
    }

    if (GPS_GetField(sentence, 7, field, sizeof(field)) &&
        (field[0] != '\0'))
    {
      *speed_mph = strtof(field, NULL) * KMH_TO_MPH;
      return 1;
    }
  }

  return 0;
}

static void GPS_UpdateNavFromSentence(const char *sentence)
{
  /*
   * Update speed and position independently.  Some NMEA messages have speed
   * but no coordinates (VTG), while others have coordinates and may also carry
   * speed (RMC).  Each latest_* field has its own timeout.
   */
  float speed_mph;
  float lat_deg;
  float lon_deg;
  char status[8];
  char lat[20];
  char ns[4];
  char lon[20];
  char ew[4];
  uint8_t updated_fix = 0;

  if (GPS_ParseSpeedMph(sentence, &speed_mph))
  {
    latest_gps_speed_mph = speed_mph;
    latest_gps_speed_valid = 1;
    latest_gps_speed_ms = HAL_GetTick();
  }

  if ((strncmp(&sentence[3], "RMC", 3) == 0) &&
      GPS_GetField(sentence, 2, status, sizeof(status)) &&
      (status[0] == 'A') &&
      GPS_GetField(sentence, 3, lat, sizeof(lat)) &&
      GPS_GetField(sentence, 4, ns, sizeof(ns)) &&
      GPS_GetField(sentence, 5, lon, sizeof(lon)) &&
      GPS_GetField(sentence, 6, ew, sizeof(ew)) &&
      GPS_ParseCoordDeg(lat, ns, &lat_deg) &&
      GPS_ParseCoordDeg(lon, ew, &lon_deg))
  {
    /* RMC status 'A' means active/valid navigation data. */
    updated_fix = 1;
  }
  else if ((strncmp(&sentence[3], "GGA", 3) == 0) &&
           GPS_GetField(sentence, 6, status, sizeof(status)) &&
           (status[0] != '\0') &&
           (status[0] != '0') &&
           GPS_GetField(sentence, 2, lat, sizeof(lat)) &&
           GPS_GetField(sentence, 3, ns, sizeof(ns)) &&
           GPS_GetField(sentence, 4, lon, sizeof(lon)) &&
           GPS_GetField(sentence, 5, ew, sizeof(ew)) &&
           GPS_ParseCoordDeg(lat, ns, &lat_deg) &&
           GPS_ParseCoordDeg(lon, ew, &lon_deg))
  {
    /* GGA fix quality 0 is invalid; any nonzero value is usable here. */
    updated_fix = 1;
  }

  if (updated_fix)
  {
    latest_gps_lat_deg = lat_deg;
    latest_gps_lon_deg = lon_deg;
    latest_gps_fix_valid = 1;
    latest_gps_fix_ms = HAL_GetTick();
  }

  VehicleSpeed_UpdateFromSources();

  if ((updated_fix || GPS_ParseSpeedMph(sentence, &speed_mph)) &&
      ((HAL_GetTick() - gps_last_nav_print_ms) >= GPS_NAV_PRINT_PERIOD_MS))
  {
    gps_last_nav_print_ms = HAL_GetTick();

    printf("GPS nav: fix=%u lat=%.6f lon=%.6f speed=%.2f mph buf=%u ovf=%lu\r\n",
           latest_gps_fix_valid,
           (double)latest_gps_lat_deg,
           (double)latest_gps_lon_deg,
           (double)latest_gps_speed_mph,
           gps_byte_count,
           (unsigned long)gps_byte_overflow_count);
  }
}

static void VehicleSpeed_UpdateFromSources(void)
{
  /*
   * Central vehicle-speed arbitration.
   *
   * 1. Fresh CAN motor-controller velocity wins.
   * 2. Fresh GPS speed is the next-best source.
   * 3. IMU integrated speed is only a fallback estimate because it drifts.
   */
  if (latest_can_speed_valid &&
      ((HAL_GetTick() - latest_can_speed_ms) > CAN_SPEED_VALID_MS))
  {
    latest_can_speed_valid = 0;
    latest_can_speed_source = "NONE";
  }

  if (latest_gps_speed_valid &&
      ((HAL_GetTick() - latest_gps_speed_ms) > GPS_SPEED_VALID_MS))
  {
    latest_gps_speed_valid = 0;
  }

  if (latest_gps_fix_valid &&
      ((HAL_GetTick() - latest_gps_fix_ms) > GPS_FIX_VALID_MS))
  {
    latest_gps_fix_valid = 0;
  }

  if (latest_can_speed_valid &&
      ((HAL_GetTick() - latest_can_speed_ms) <= CAN_SPEED_VALID_MS))
  {
    latest_vehicle_speed_mph = latest_can_speed_mph;
    latest_vehicle_speed_source = latest_can_speed_source;
  }
  else if (latest_gps_speed_valid &&
      ((HAL_GetTick() - latest_gps_speed_ms) <= GPS_SPEED_VALID_MS))
  {
    latest_vehicle_speed_mph = latest_gps_speed_mph;
    latest_vehicle_speed_source = "GPS";
  }
  else if (latest_imu_mph > 0.05f)
  {
    latest_vehicle_speed_mph = latest_imu_mph;
    latest_vehicle_speed_source = "IMU_INTEGRATED";
  }
  else
  {
    latest_vehicle_speed_mph = 0.0f;
    latest_vehicle_speed_source = "NONE";
  }
}

static void GPS_PollSPI(void)
{
  GPS_ServiceSPI();

  if (gps_spi_busy)
  {
    return;
  }

  memset(gps_spi_tx, 0xFF, sizeof(gps_spi_tx));
  memset(gps_spi_rx, 0xFF, sizeof(gps_spi_rx));

  GPS_Select();
  gps_spi_done = 0;
  gps_spi_error = 0;
  gps_spi_busy = 1;
  gps_spi_start_ms = HAL_GetTick();

  if (HAL_SPI_TransmitReceive_IT(&hspi1,
                                 gps_spi_tx,
                                 gps_spi_rx,
                                 GPS_SPI_READ_LEN) != HAL_OK)
  {
    GPS_Deselect();
    gps_spi_busy = 0;
    gps_spi_done = 0;
    printf("GPS SPI interrupt read start failed, HAL err=0x%08lX\r\n",
           (unsigned long)HAL_SPI_GetError(&hspi1));
    StatusLed_SetError();
  }
}

static void GPS_ServiceSPI(void)
{
  if (gps_spi_done)
  {
    uint8_t had_error = gps_spi_error;

    gps_spi_done = 0;
    gps_spi_error = 0;

    if (had_error)
    {
      printf("GPS SPI interrupt read failed, HAL err=0x%08lX\r\n",
             (unsigned long)HAL_SPI_GetError(&hspi1));
      StatusLed_SetError();
      return;
    }

    GPS_BufferRxBlock(gps_spi_rx, GPS_SPI_READ_LEN);
  }

  GPS_ProcessBufferedBytes(GPS_PARSE_BUDGET_BYTES);

  if (gps_spi_busy &&
      ((HAL_GetTick() - gps_spi_start_ms) > GPS_SPI_TIMEOUT_MS))
  {
    HAL_SPI_Abort(&hspi1);
    GPS_Deselect();
    gps_spi_busy = 0;
    gps_spi_done = 0;
    gps_spi_error = 0;
    printf("GPS SPI interrupt read timeout, HAL err=0x%08lX\r\n",
           (unsigned long)HAL_SPI_GetError(&hspi1));
    StatusLed_SetError();
  }
}

static void SD_LogInit(void)
{
	 FRESULT res;
	  char filename[32];

	  /*
	   * SD card mount is intentionally retried.  With jumper wires or marginal
	   * contacts, the first FatFS mount can fail even after the low-level driver
	   * later succeeds.
	   */
	  HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);

	  printf("SD mount starting...\r\n");

	  /*
	   * Clear any previous mount state first.
	   */
	  f_mount(NULL, USERPath, 0);

	  for (uint8_t attempt = 1; attempt <= 5; attempt++)
	  {
	    printf("SD mount attempt %u...\r\n", attempt);

	    res = f_mount(&fs, USERPath, 1);

	    if (res == FR_OK)
	    {
	      break;
	    }

	    printf("SD mount attempt %u failed: %d (%s)\r\n",
	           attempt,
	           res,
	           FatFs_ErrorString(res));

	    HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_SET);
	    HAL_Delay(300);
	  }

	  if (res != FR_OK)
	  {
	    printf("SD mount failed final: %d (%s)\r\n",
	           res,
	           FatFs_ErrorString(res));

	    sd_ready = 0;
	    return;
	  }

	  snprintf(filename, sizeof(filename), "%s/imu_gps.csv", USERPath);

	  printf("Opening SD file: %s\r\n", filename);

	  res = f_open(&log_file, filename, FA_OPEN_APPEND | FA_WRITE);
	  if (res != FR_OK)
	  {
	    printf("SD file open failed: %d (%s)\r\n",
	           res,
	           FatFs_ErrorString(res));

	    sd_ready = 0;
	    return;
	  }

	  if (f_size(&log_file) == 0)
	  {
	    /*
	     * Only write the CSV header for a brand-new/empty file.  Existing logs
	     * are appended so a power cycle does not overwrite prior telemetry.
	     */
	    const char *header = TELEMETRY_CSV_HEADER "\r\n";

	    UINT bw = 0;

	    res = f_write(&log_file, header, strlen(header), &bw);
	    if ((res != FR_OK) || (bw != strlen(header)))
	    {
	      printf("SD header write failed: %d (%s), bw=%u\r\n",
	             res,
	             FatFs_ErrorString(res),
	             bw);

	      sd_ready = 0;
	      return;
	    }

	    res = f_sync(&log_file);
	    if (res != FR_OK)
	    {
	      printf("SD header sync failed: %d (%s)\r\n",
	             res,
	             FatFs_ErrorString(res));

	      sd_ready = 0;
	      return;
	    }
	  }

	  sd_ready = 1;
	  printf("SD log ready\r\n");
}

static void SD_LogIMUAndGPS(struct bmi2_sens_data *sensor_data)
{
	if (!sd_ready || !sd_logging_enabled)
	{
	  /* SD is optional at runtime; telemetry continues over ESP32/RS232. */
	  return;
	}

	  char line[TELEMETRY_LINE_LEN];

	  int len = Telemetry_BuildCSVLine(sensor_data, line, sizeof(line));

	  if (len > 0)
	  {
	    UINT bw;
	    FRESULT wr = f_write(&log_file, line, (UINT)len, &bw);
	    FRESULT sy = f_sync(&log_file);

	    /*
	     * f_sync() trades write speed for survivability.  It makes each row much
	     * more likely to survive if power is removed during a run.
	     */
	    if ((wr == FR_OK) && (sy == FR_OK) && (bw == (UINT)len))
	    {
	      StatusLed_RequestPulse(STATUS_SD_LOG);
	    }
	    else
	    {
	      printf("SD log write/sync failed: wr=%d sy=%d bw=%u len=%d\r\n",
	             wr, sy, bw, len);

	      StatusLed_SetError();
	    }
	  }
}

static void SD_LogStart(void)
{
  if (!sd_ready)
  {
    SD_LogInit();
  }

  if (sd_ready)
  {
    sd_logging_enabled = 1;
    printf("SD logging started\r\n");
  }
  else
  {
    printf("SD logging start failed: SD not ready\r\n");
  }
}

static void SD_LogStop(void)
{
  sd_logging_enabled = 0;

  if (sd_ready)
  {
    f_sync(&log_file);
  }

  printf("SD logging stopped\r\n");
}

static int Telemetry_BuildCSVLine(struct bmi2_sens_data *sensor_data,
                                  char *line,
                                  size_t line_size)
{
  /*
   * Single source of truth for SD and ESP32 telemetry rows.
   * If a column is added here, TELEMETRY_CSV_HEADER must be updated too.
   */
  if ((sensor_data == NULL) || (line == NULL) || (line_size == 0))
  {
    return -1;
  }

  int32_t ax_mg = raw_acc_to_mg(sensor_data->acc.x);
  int32_t ay_mg = raw_acc_to_mg(sensor_data->acc.y);
  int32_t az_mg = raw_acc_to_mg(sensor_data->acc.z);

  int32_t gx_mdps = raw_gyro_to_mdps(sensor_data->gyr.x);
  int32_t gy_mdps = raw_gyro_to_mdps(sensor_data->gyr.y);
  int32_t gz_mdps = raw_gyro_to_mdps(sensor_data->gyr.z);

  DateTime_t adalogger_dt = {0};

  char adalogger_rtc_text[40] = "ADALOGGER_RTC_READ_FAIL";
  char can_data_text[32] = "NO_CAN";

  uint8_t adalogger_valid = 0;
  uint32_t gps_fix_age_ms = 0xFFFFFFFFUL;

  STM32_CAN_Rx_t can_snapshot;

  /*
   * Take a quick snapshot because latest_can_rx can be updated
   * from the CAN interrupt callback.
   */
  __disable_irq();
  can_snapshot = latest_can_rx;
  __enable_irq();

  CAN_DataToHex(can_snapshot.data,
                can_snapshot.dlc,
                can_data_text,
                sizeof(can_data_text));

  if (adalogger_rtc_ok && ADALOGGER_RTC_Read(&adalogger_dt))
  {
    adalogger_valid = adalogger_dt.valid;
    DateTime_ToString(&adalogger_dt, adalogger_rtc_text, sizeof(adalogger_rtc_text));
  }

  VehicleSpeed_UpdateFromSources();

  /*
   * A valid flag plus age lets the receiver decide how stale location data can
   * be before it stops using it.  Invalid fix age is intentionally 0xFFFFFFFF
   * in CSV so it is obvious in offline analysis.
   */
  if (latest_gps_fix_valid)
  {
    gps_fix_age_ms = HAL_GetTick() - latest_gps_fix_ms;
  }

  int len = snprintf(line,
                     line_size,
                     "%lu,\"%s\",%u,"
                     "%ld,%ld,%ld,%ld,%ld,%ld,"
                     "%.2f,%.2f,%u,%.2f,%u,\"%s\","
                     "%.6f,%.6f,%u,%lu,%.2f,\"%s\","
                     "%.2f,%.2f,%.2f,"
                     "%lu,0x%03lX,%u,%u,\"%s\"\r\n",
                     (unsigned long)HAL_GetTick(),

                     adalogger_rtc_text,
                     (unsigned int)adalogger_valid,

                     (long)ax_mg,
                     (long)ay_mg,
                     (long)az_mg,
                     (long)gx_mdps,
                     (long)gy_mdps,
                     (long)gz_mdps,
                     (double)latest_imu_mph,
                     (double)latest_gps_speed_mph,
                     (unsigned int)latest_gps_speed_valid,
                     (double)latest_can_speed_mph,
                     (unsigned int)latest_can_speed_valid,
                     latest_can_speed_source,
                     (double)latest_gps_lat_deg,
                     (double)latest_gps_lon_deg,
                     (unsigned int)latest_gps_fix_valid,
                     (unsigned long)gps_fix_age_ms,
                     (double)latest_vehicle_speed_mph,
                     latest_vehicle_speed_source,

                     (double)latest_bme_temp_c,
                     (double)latest_bme_pressure_pa,
                     (double)latest_bme_humidity_pct,

                     (unsigned long)can_snapshot.count,
                     (unsigned long)can_snapshot.id,
                     (unsigned int)can_snapshot.is_extended,
                     (unsigned int)can_snapshot.dlc,
                     can_data_text);

  if ((len < 0) || (len >= (int)line_size))
  {
    printf("Telemetry CSV line truncated or failed\r\n");
  }

  return len;
}

static void ESP32_Select(void)
{
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);
}

static void ESP32_Deselect(void)
{
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);
}

static uint8_t ESP32_IsReady(void)
{
  return (HAL_GPIO_ReadPin(ESP32_Ready_GPIO_Port, ESP32_Ready_Pin) == ESP32_READY_ACTIVE);
}

static void ESP32_SendTelemetry(struct bmi2_sens_data *sensor_data)
{
  /*
   * ESP32 transaction priority:
   * 1. Response to a command from the ESP32/app.
   * 2. One-time CSV header so the app can map fields by name.
   * 3. Normal $LOG CSV telemetry row.
   */
  ESP32_ServiceSPI();

  if (esp32_spi_busy)
  {
    return;
  }

  if (!ESP32_IsReady())
  {
    return;
  }

  char line[TELEMETRY_LINE_LEN];
  uint8_t sending_response = 0;

  memset(esp32_tx_frame, 0, sizeof(esp32_tx_frame));
  memset(esp32_rx_frame, 0, sizeof(esp32_rx_frame));

  /*
   * If STM32 owes the ESP32/Desktop a command response,
   * send that first. Otherwise send normal telemetry.
   */
  if (esp32_response_pending)
  {
    snprintf((char *)esp32_tx_frame,
             sizeof(esp32_tx_frame),
             "%s",
             esp32_response);

    sending_response = 1;
  }
  else if (esp32_header_pending)
  {
    /* $HDR is not logged to SD; it is only for the app/parser on ESP32. */
    snprintf((char *)esp32_tx_frame,
             sizeof(esp32_tx_frame),
             "$HDR,%s\r\n",
             TELEMETRY_CSV_HEADER);
  }
  else
  {
    /* Normal app telemetry: "$LOG,<sequence>,<csv row>". */
    int len = Telemetry_BuildCSVLine(sensor_data, line, sizeof(line));

    if (len <= 0)
    {
      return;
    }

    snprintf((char *)esp32_tx_frame,
             sizeof(esp32_tx_frame),
             "$LOG,%lu,%s",
             (unsigned long)esp32_seq++,
             line);
  }

  ESP32_Select();

  esp32_spi_done = 0;
  esp32_spi_error = 0;
  esp32_spi_busy = 1;
  esp32_spi_start_ms = HAL_GetTick();

  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(&hspi3,
                                                        esp32_tx_frame,
                                                        esp32_rx_frame,
                                                        ESP32_TX_FRAME_LEN);

  if (status == HAL_OK)
  {
    if (sending_response)
    {
      esp32_response_pending = 0;
    }
    else if (esp32_header_pending)
    {
      esp32_header_pending = 0;
    }
  }
  else
  {
    ESP32_Deselect();
    esp32_spi_busy = 0;
    esp32_spi_done = 0;
    printf("ESP32 SPI interrupt transmit/receive start failed, HAL err=0x%08lX\r\n",
           (unsigned long)HAL_SPI_GetError(&hspi3));
    StatusLed_SetError();
  }
}

static void ESP32_ServiceSPI(void)
{
  if (esp32_spi_done)
  {
    uint8_t had_error = esp32_spi_error;

    esp32_spi_done = 0;
    esp32_spi_error = 0;

    if (had_error)
    {
      printf("ESP32 SPI interrupt transmit/receive failed, HAL err=0x%08lX\r\n",
             (unsigned long)HAL_SPI_GetError(&hspi3));
      StatusLed_SetError();
      return;
    }

    StatusLed_RequestPulse(STATUS_ESP32_TX);

    /*
     * Process any command the ESP32 returned on MISO.
     */
    ESP32_ProcessRxFrame(esp32_rx_frame, sizeof(esp32_rx_frame));
  }

  if (esp32_spi_busy &&
      ((HAL_GetTick() - esp32_spi_start_ms) > ESP32_SPI_TIMEOUT_MS))
  {
    HAL_SPI_Abort(&hspi3);
    ESP32_Deselect();
    esp32_spi_busy = 0;
    esp32_spi_done = 0;
    esp32_spi_error = 0;
    printf("ESP32 SPI interrupt transmit/receive timeout, HAL err=0x%08lX\r\n",
           (unsigned long)HAL_SPI_GetError(&hspi3));
    StatusLed_SetError();
  }
}

static void ESP32_QueueResponse(const char *id,
                                const char *status,
                                const char *message)
{
  if (id == NULL)
  {
    id = "0";
  }

  if (status == NULL)
  {
    status = "ERR";
  }

  if (message == NULL)
  {
    message = "NO_MESSAGE";
  }

  snprintf(esp32_response,
           sizeof(esp32_response),
           "$RSP,%s,%s,%s\r\n",
           id,
           status,
           message);

  esp32_response_pending = 1;
}

static uint8_t DateTime_IsValid(const DateTime_t *dt)
{
  if (dt == NULL)
  {
    return 0;
  }

  if ((dt->year < 2000U) || (dt->year > 2099U)) return 0;
  if ((dt->month < 1U) || (dt->month > 12U)) return 0;
  if ((dt->day < 1U) || (dt->day > 31U)) return 0;
  if (dt->hour > 23U) return 0;
  if (dt->minute > 59U) return 0;
  if (dt->second > 59U) return 0;

  return 1;
}

/*
 * Returns weekday using 0 = Sunday, 1 = Monday, ..., 6 = Saturday.
 */
static uint8_t DateTime_CalcWeekday(uint16_t year, uint8_t month, uint8_t day)
{
  static const uint8_t table[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

  if (month < 3U)
  {
    year--;
  }

  return (uint8_t)((year + year / 4U - year / 100U + year / 400U +
                    table[month - 1U] + day) % 7U);
}

static void ESP32_ProcessRxFrame(const uint8_t *rx, size_t len)
{
  /*
   * SPI frames are fixed-size and padded with 0x00/0xFF.  This extracts the
   * first printable command string from the returned MISO bytes.
   */
  char cmd[ESP32_CMD_LEN];
  size_t pos = 0;

  if (rx == NULL)
  {
    return;
  }

  memset(cmd, 0, sizeof(cmd));

  /*
   * Extract first printable ASCII command from the RX frame.
   * Ignores 0x00 and 0xFF padding.
   */
  for (size_t i = 0; i < len && pos < (sizeof(cmd) - 1U); i++)
  {
    uint8_t c = rx[i];

    if ((c == 0x00U) || (c == 0xFFU))
    {
      if (pos > 0U)
      {
        break;
      }

      continue;
    }

    if ((c == '\r') || (c == '\n'))
    {
      break;
    }

    if ((c >= 32U) && (c <= 126U))
    {
      cmd[pos++] = (char)c;
    }
  }

  cmd[pos] = '\0';

  if (pos == 0U)
  {
    return;
  }

  printf("ESP32 CMD RX: %s\r\n", cmd);

  ESP32_HandleCommand(cmd);
}

static void ESP32_HandleCommand(char *cmd)
{
  /*
   * Accept a few command formats so the ESP32 firmware/app can evolve without
   * requiring the STM32 parser to be rewritten every time.
   */
  char *p;

  if (cmd == NULL)
  {
    return;
  }

  p = cmd;

  if (*p == '$')
  {
    p++;
  }

  /*
   * ESP_ACK means the SPI return path is alive,
   * but it is not a command for STM32 to execute.
   */
  if (strncmp(p, "ESP_ACK", 7) == 0)
  {
    printf("ESP32 ACK ignored: %s\r\n", p);
    return;
  }

  /*
   * Format:
   * CMD,42,SET_RTC,2026-05-31,16:02:18
   */
  if (strncmp(p, "CMD,", 4) == 0)
  {
    char *id;
    char *verb;
    char *comma;

    p += 4;

    id = p;
    comma = strchr(id, ',');

    if (comma == NULL)
    {
      ESP32_QueueResponse("0", "ERR", "BAD_CMD_FORMAT");
      return;
    }

    *comma = '\0';
    verb = comma + 1;

    ESP32_ExecuteCommand(id, verb);
    return;
  }

  /*
   * Format:
   * ESP_CMD,seq=44,cmd="SET_RTC,2026-05-31,16:02:18",...
   */
  if (strncmp(p, "ESP_CMD", 7) == 0)
  {
    char id_buf[16] = "0";
    char verb_buf[ESP32_CMD_LEN];
    char *seq_field;
    char *cmd_field;
    char *src;
    size_t i = 0;

    memset(verb_buf, 0, sizeof(verb_buf));

    seq_field = strstr(p, "seq=");
    if (seq_field != NULL)
    {
      unsigned int seq = 0;

      if (sscanf(seq_field, "seq=%u", &seq) == 1)
      {
        snprintf(id_buf, sizeof(id_buf), "%u", seq);
      }
    }

    cmd_field = strstr(p, "cmd=\"");
    if (cmd_field != NULL)
    {
      src = cmd_field + 5;

      while (src[i] != '\0' &&
             src[i] != '"' &&
             i < sizeof(verb_buf) - 1U)
      {
        verb_buf[i] = src[i];
        i++;
      }

      verb_buf[i] = '\0';

      printf("ESP32 ESP_CMD parsed: id=%s cmd=%s\r\n", id_buf, verb_buf);
      ESP32_ExecuteCommand(id_buf, verb_buf);
      return;
    }

    cmd_field = strstr(p, "cmd=");
    if (cmd_field != NULL)
    {
      src = cmd_field + 4;

      while (src[i] != '\0' &&
             src[i] != '\r' &&
             src[i] != '\n' &&
             i < sizeof(verb_buf) - 1U)
      {
        verb_buf[i] = src[i];
        i++;
      }

      verb_buf[i] = '\0';

      printf("ESP32 ESP_CMD parsed: id=%s cmd=%s\r\n", id_buf, verb_buf);
      ESP32_ExecuteCommand(id_buf, verb_buf);
      return;
    }

    printf("ESP_CMD missing cmd field: %s\r\n", p);
    ESP32_QueueResponse(id_buf, "ERR", "ESP_CMD_MISSING_CMD");
    return;
  }

  /*
   * Direct command format:
   * SET_RTC,2026-05-31,16:02:18
   * START_LOG
   * STOP_LOG
   */
  if ((strncmp(p, "SET_RTC,", 8) == 0) ||
      (strncmp(p, "START_LOG", 9) == 0) ||
      (strncmp(p, "STOP_LOG", 8) == 0))
  {
    ESP32_ExecuteCommand("0", p);
    return;
  }

  printf("ESP32 RX ignored unknown frame: %s\r\n", p);
}

static void ESP32_ExecuteCommand(const char *id, const char *verb)
{
  /*
   * Commands are intentionally small and synchronous:
   * - START_LOG / STOP_LOG control SD logging
   * - SET_RTC updates the Adalogger RTC
   */
  if ((id == NULL) || (verb == NULL))
  {
    return;
  }

  if (strncmp(verb, "START_LOG", 9) == 0)
  {
    SD_LogStart();

    if (sd_ready && sd_logging_enabled)
    {
      ESP32_QueueResponse(id, "OK", "LOG_STARTED");
    }
    else
    {
      ESP32_QueueResponse(id, "ERR", "SD_NOT_READY");
    }

    return;
  }

  if (strncmp(verb, "STOP_LOG", 8) == 0)
  {
    SD_LogStop();
    ESP32_QueueResponse(id, "OK", "LOG_STOPPED");
    return;
  }

  if (strncmp(verb, "SET_RTC,", 8) == 0)
  {
    unsigned int year;
    unsigned int month;
    unsigned int day;
    unsigned int hour;
    unsigned int minute;
    unsigned int second;
    DateTime_t dt = {0};

    if (sscanf(verb + 8,
               "%u-%u-%u,%u:%u:%u",
               &year,
               &month,
               &day,
               &hour,
               &minute,
               &second) != 6)
    {
      printf("SET_RTC parse failed: %s\r\n", verb);
      ESP32_QueueResponse(id, "ERR", "BAD_RTC_FORMAT");
      return;
    }

    dt.year = (uint16_t)year;
    dt.month = (uint8_t)month;
    dt.day = (uint8_t)day;
    dt.hour = (uint8_t)hour;
    dt.minute = (uint8_t)minute;
    dt.second = (uint8_t)second;
    dt.weekday = DateTime_CalcWeekday(dt.year, dt.month, dt.day);
    dt.valid = 1;

    printf("SET_RTC requested: %04u-%02u-%02u %02u:%02u:%02u\r\n",
           dt.year,
           dt.month,
           dt.day,
           dt.hour,
           dt.minute,
           dt.second);

    if (!DateTime_IsValid(&dt))
    {
      ESP32_QueueResponse(id, "ERR", "RTC_RANGE_INVALID");
      return;
    }

    if (!adalogger_rtc_ok)
    {
      printf("SET_RTC failed: RTC not ready\r\n");
      ESP32_QueueResponse(id, "ERR", "RTC_NOT_READY");
      return;
    }

    if (ADALOGGER_RTC_Set(&dt))
    {
      ADALOGGER_RTC_Print();
      ESP32_QueueResponse(id, "OK", "RTC_SET");
    }
    else
    {
      ESP32_QueueResponse(id, "ERR", "RTC_SET_FAILED");
    }

    return;
  }

  printf("Unknown ESP32 command verb: %s\r\n", verb);
  ESP32_QueueResponse(id, "ERR", "UNKNOWN_CMD");
}

static uint8_t bcd_to_bin(uint8_t bcd)
{
  return (uint8_t)(((bcd >> 4) * 10U) + (bcd & 0x0FU));
}

static uint8_t bin_to_bcd(uint8_t bin)
{
  return (uint8_t)(((bin / 10U) << 4) | (bin % 10U));
}

static void DateTime_ToString(const DateTime_t *dt, char *out, size_t out_len)
{
  if ((dt == NULL) || (out == NULL) || (out_len == 0))
  {
    return;
  }

  if (dt->valid)
  {
    snprintf(out, out_len,
             "%04u-%02u-%02u %02u:%02u:%02u",
             dt->year,
             dt->month,
             dt->day,
             dt->hour,
             dt->minute,
             dt->second);
  }
  else
  {
    snprintf(out, out_len,
             "%04u-%02u-%02u %02u:%02u:%02u_INVALID",
             dt->year,
             dt->month,
             dt->day,
             dt->hour,
             dt->minute,
             dt->second);
  }
}

/*
 * ADALOGGER_RTC / PCF8523
 * This reads the RTC chip on the Adalogger FeatherWing through I2C2.
 */
static void ADALOGGER_RTC_RecoverI2CBus(const char *reason)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  printf("ADALOGGER_RTC: recovering I2C2 bus after %s, HAL err=0x%08lX\r\n",
         (reason != NULL) ? reason : "I2C error",
         (unsigned long)HAL_I2C_GetError(&hi2c2));

  HAL_I2C_DeInit(&hi2c2);

  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(1);

  for (uint8_t i = 0; i < 9U; i++)
  {
    if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == GPIO_PIN_SET)
    {
      break;
    }

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  /*
   * STOP condition: SDA low while SCL high, then SDA high.
   */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(1);

  MX_I2C2_Init();
}

static HAL_StatusTypeDef ADALOGGER_RTC_ReadRegs(uint8_t reg,
                                                uint8_t *data,
                                                uint16_t len)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  for (uint8_t attempt = 0; attempt <= ADALOGGER_RTC_I2C_RETRIES; attempt++)
  {
    status = HAL_I2C_Mem_Read(&hi2c2,
                              ADALOGGER_RTC_ADDR,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              len,
                              ADALOGGER_RTC_I2C_TIMEOUT_MS);

    if (status == HAL_OK)
    {
      return HAL_OK;
    }

    printf("ADALOGGER_RTC: read reg 0x%02X failed, status=%d err=0x%08lX attempt=%u\r\n",
           reg,
           status,
           (unsigned long)HAL_I2C_GetError(&hi2c2),
           (unsigned int)(attempt + 1U));

    ADALOGGER_RTC_RecoverI2CBus("read");
  }

  return status;
}

static HAL_StatusTypeDef ADALOGGER_RTC_WriteRegs(uint8_t reg,
                                                 uint8_t *data,
                                                 uint16_t len)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  for (uint8_t attempt = 0; attempt <= ADALOGGER_RTC_I2C_RETRIES; attempt++)
  {
    status = HAL_I2C_Mem_Write(&hi2c2,
                               ADALOGGER_RTC_ADDR,
                               reg,
                               I2C_MEMADD_SIZE_8BIT,
                               data,
                               len,
                               ADALOGGER_RTC_I2C_TIMEOUT_MS);

    if (status == HAL_OK)
    {
      return HAL_OK;
    }

    printf("ADALOGGER_RTC: write reg 0x%02X failed, status=%d err=0x%08lX attempt=%u\r\n",
           reg,
           status,
           (unsigned long)HAL_I2C_GetError(&hi2c2),
           (unsigned int)(attempt + 1U));

    ADALOGGER_RTC_RecoverI2CBus("write");
  }

  return status;
}

static HAL_StatusTypeDef ADALOGGER_RTC_WriteReg(uint8_t reg, uint8_t value)
{
  return ADALOGGER_RTC_WriteRegs(reg, &value, 1);
}

static uint8_t ADALOGGER_RTC_Init(void)
{
  HAL_StatusTypeDef ready = HAL_ERROR;

  for (uint8_t attempt = 0; attempt <= ADALOGGER_RTC_I2C_RETRIES; attempt++)
  {
    ready = HAL_I2C_IsDeviceReady(&hi2c2,
                                  ADALOGGER_RTC_ADDR,
                                  3,
                                  ADALOGGER_RTC_I2C_TIMEOUT_MS);

    if (ready == HAL_OK)
    {
      break;
    }

    printf("ADALOGGER_RTC: ready check failed, status=%d err=0x%08lX attempt=%u\r\n",
           ready,
           (unsigned long)HAL_I2C_GetError(&hi2c2),
           (unsigned int)(attempt + 1U));

    ADALOGGER_RTC_RecoverI2CBus("ready check");
  }

  if (ready != HAL_OK)
  {
    printf("ADALOGGER_RTC: PCF8523 not found on I2C2\r\n");
    return 0;
  }

  /*
   * Control_1 = 0x00:
   * normal running, 24-hour mode, stop bit cleared.
   */
  if (ADALOGGER_RTC_WriteReg(PCF8523_REG_CONTROL1, 0x00) != HAL_OK)
  {
    printf("ADALOGGER_RTC: control write failed\r\n");
    return 0;
  }

  printf("ADALOGGER_RTC: PCF8523 detected on I2C2\r\n");
  return 1;
}

static uint8_t ADALOGGER_RTC_Read(DateTime_t *dt)
{
  uint8_t data[7];

  if (dt == NULL)
  {
    return 0;
  }

  if (ADALOGGER_RTC_ReadRegs(PCF8523_REG_SECONDS, data, 7) != HAL_OK)
  {
    dt->valid = 0;
    return 0;
  }

  /*
   * Seconds bit 7 is the oscillator stop flag.
   * If it is set, the external RTC time may not be valid.
   */
  uint8_t osc_stopped = data[0] & 0x80U;

  dt->second  = bcd_to_bin(data[0] & 0x7FU);
  dt->minute  = bcd_to_bin(data[1] & 0x7FU);
  dt->hour    = bcd_to_bin(data[2] & 0x3FU);
  dt->day     = bcd_to_bin(data[3] & 0x3FU);
  dt->weekday = bcd_to_bin(data[4] & 0x07U);
  dt->month   = bcd_to_bin(data[5] & 0x1FU);
  dt->year    = 2000U + bcd_to_bin(data[6]);

  dt->valid = (osc_stopped == 0U);

  return 1;
}

static uint8_t ADALOGGER_RTC_Set(const DateTime_t *dt)
{
  uint8_t data[7];

  if (dt == NULL)
  {
    return 0;
  }

  data[0] = bin_to_bcd(dt->second) & 0x7FU;
  data[1] = bin_to_bcd(dt->minute);
  data[2] = bin_to_bcd(dt->hour);
  data[3] = bin_to_bcd(dt->day);
  data[4] = bin_to_bcd(dt->weekday);
  data[5] = bin_to_bcd(dt->month);
  data[6] = bin_to_bcd((uint8_t)(dt->year - 2000U));

  if (ADALOGGER_RTC_WriteRegs(PCF8523_REG_SECONDS, data, 7) != HAL_OK)
  {
    printf("ADALOGGER_RTC: set time failed\r\n");
    return 0;
  }

  printf("ADALOGGER_RTC: time set\r\n");
  return 1;
}

static void ADALOGGER_RTC_Print(void)
{
  DateTime_t dt;
  char text[40];

  if (ADALOGGER_RTC_Read(&dt))
  {
    DateTime_ToString(&dt, text, sizeof(text));
    printf("ADALOGGER_RTC: %s\r\n", text);
  }
  else
  {
    printf("ADALOGGER_RTC: read failed\r\n");
  }
}

static HAL_StatusTypeDef BME280_WaitForSPI5IT(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (!bme280_spi_done)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      HAL_SPI_Abort(&hspi5);
      bme280_spi_error = 1;
      return HAL_TIMEOUT;
    }
  }

  if (bme280_spi_error)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef BME280_TransmitIT(uint8_t *data, uint16_t len)
{
  bme280_spi_done = 0;
  bme280_spi_error = 0;

  HAL_StatusTypeDef status = HAL_SPI_Transmit_IT(&hspi5, data, len);

  if (status != HAL_OK)
  {
    return status;
  }

  return BME280_WaitForSPI5IT(BME280_SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef BME280_ReceiveIT(uint8_t *data, uint16_t len)
{
  bme280_spi_done = 0;
  bme280_spi_error = 0;

  HAL_StatusTypeDef status = HAL_SPI_Receive_IT(&hspi5, data, len);

  if (status != HAL_OK)
  {
    return status;
  }

  return BME280_WaitForSPI5IT(BME280_SPI_TIMEOUT_MS);
}

static void BME280_Select(void)
{
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_RESET);
}

static void BME280_Deselect(void)
{
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
}

static int8_t BME280_SPI_Read(uint8_t reg_addr,
                              uint8_t *reg_data,
                              uint32_t len,
                              void *intf_ptr)
{
  (void)intf_ptr;

  uint8_t addr = reg_addr | 0x80U;

  BME280_Select();

  if (BME280_TransmitIT(&addr, 1) != HAL_OK)
  {
    BME280_Deselect();
    return BME280_E_COMM_FAIL;
  }

  if (BME280_ReceiveIT(reg_data, (uint16_t)len) != HAL_OK)
  {
    BME280_Deselect();
    return BME280_E_COMM_FAIL;
  }

  BME280_Deselect();
  return BME280_OK;
}

static int8_t BME280_SPI_Write(uint8_t reg_addr,
                               const uint8_t *reg_data,
                               uint32_t len,
                               void *intf_ptr)
{
  (void)intf_ptr;

  uint8_t addr = reg_addr & 0x7FU;

  BME280_Select();

  if (BME280_TransmitIT(&addr, 1) != HAL_OK)
  {
    BME280_Deselect();
    return BME280_E_COMM_FAIL;
  }

  if (BME280_TransmitIT((uint8_t *)reg_data, (uint16_t)len) != HAL_OK)
  {
    BME280_Deselect();
    return BME280_E_COMM_FAIL;
  }

  BME280_Deselect();
  return BME280_OK;
}

static void BME280_DelayUs(uint32_t period_us, void *intf_ptr)
{
  (void)intf_ptr;

  uint32_t delay_ms = (period_us + 999U) / 1000U;
  if (delay_ms == 0U)
  {
    delay_ms = 1U;
  }

  HAL_Delay(delay_ms);
}

static uint8_t BME280_InitSensor(void)
{
  int8_t rslt;
  struct bme280_settings settings;

  BME280_Deselect();
  HAL_Delay(10);

  memset(&bme280_dev, 0, sizeof(bme280_dev));

  bme280_dev.intf = BME280_SPI_INTF;
  bme280_dev.read = BME280_SPI_Read;
  bme280_dev.write = BME280_SPI_Write;
  bme280_dev.delay_us = BME280_DelayUs;
  bme280_dev.intf_ptr = NULL;

  rslt = bme280_init(&bme280_dev);
  printf("BME280 init result = %d, chip_id = 0x%02X\r\n",
         rslt,
         bme280_dev.chip_id);

  if (rslt != BME280_OK)
  {
    return 0;
  }

  settings.osr_h = BME280_OVERSAMPLING_1X;
  settings.osr_p = BME280_OVERSAMPLING_1X;
  settings.osr_t = BME280_OVERSAMPLING_1X;
  settings.filter = BME280_FILTER_COEFF_OFF;
  settings.standby_time = BME280_STANDBY_TIME_1000_MS;

  rslt = bme280_set_sensor_settings(BME280_SEL_OSR_TEMP |
                                    BME280_SEL_OSR_PRESS |
                                    BME280_SEL_OSR_HUM |
                                    BME280_SEL_FILTER |
                                    BME280_SEL_STANDBY,
                                    &settings,
                                    &bme280_dev);

  if (rslt != BME280_OK)
  {
    printf("BME280 settings failed: %d\r\n", rslt);
    return 0;
  }

  rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_dev);

  if (rslt != BME280_OK)
  {
    printf("BME280 normal mode failed: %d\r\n", rslt);
    return 0;
  }

  printf("BME280 initialized successfully.\r\n");
  return 1;
}

static uint8_t BME280_ReadAndPrint(void)
{
  struct bme280_data data;
  int8_t rslt;

  if (!bme280_ok)
  {
    return 0;
  }

  rslt = bme280_get_sensor_data(BME280_ALL, &data, &bme280_dev);

  if (rslt != BME280_OK)
  {
    printf("BME280 read failed: %d\r\n", rslt);
    return 0;
  }

  latest_bme_temp_c = (float)data.temperature;
  latest_bme_pressure_pa = (float)data.pressure;
  latest_bme_humidity_pct = (float)data.humidity;

  printf("BME280: temp=%.2f C | pressure=%.2f Pa | humidity=%.2f %%\r\n",
         latest_bme_temp_c,
         latest_bme_pressure_pa,
         latest_bme_humidity_pct);
  StatusLed_RequestPulse(STATUS_BME_READ);
  return 1;
}

static uint8_t STM32_CAN_ListenAllInit(void)
{
  CAN_FilterTypeDef filter;

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  /*
   * Accept all IDs:
   * ID = 0, MASK = 0 means no ID bits are filtered out.
   */
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;

  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    printf("CAN filter config failed\r\n");
    return 0;
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    printf("CAN start failed\r\n");
    return 0;
  }

  if (HAL_CAN_ActivateNotification(&hcan1,
                                   CAN_IT_RX_FIFO0_MSG_PENDING |
                                   CAN_IT_ERROR |
                                   CAN_IT_BUSOFF) != HAL_OK)
  {
    printf("CAN notification failed\r\n");
    return 0;
  }

  printf("CAN1 listen-all initialized.\r\n");
  return 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1)
  {
    return;
  }

  STM32_CAN_PollRx();
}

static void CAN_DataToHex(const uint8_t *data, uint8_t len, char *out, size_t out_len)
{
  size_t pos = 0;

  if ((data == NULL) || (out == NULL) || (out_len == 0))
  {
    return;
  }

  out[0] = '\0';

  for (uint8_t i = 0; i < len && i < 8; i++)
  {
    int written = snprintf(&out[pos], out_len - pos, "%02X", data[i]);

    if (written <= 0)
    {
      break;
    }

    pos += (size_t)written;

    if ((i + 1U) < len && pos < (out_len - 1U))
    {
      out[pos++] = ' ';
      out[pos] = '\0';
    }
  }
}

static void STM32_CAN_PrintLatest(void)
{
  char data_text[32];

  if (!latest_can_rx.new_msg)
  {
    return;
  }

  latest_can_rx.new_msg = 0;

  CAN_DataToHex(latest_can_rx.data,
                latest_can_rx.dlc,
                data_text,
                sizeof(data_text));

  printf("CAN RX #%lu | ID=0x%03lX | %s | DLC=%u | DATA=%s\r\n",
         (unsigned long)latest_can_rx.count,
         (unsigned long)latest_can_rx.id,
         latest_can_rx.is_extended ? "EXT" : "STD",
         latest_can_rx.dlc,
         data_text);
}

static uint32_t CAN_MakeU32_LE(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static float CAN_U32ToFloat(uint32_t raw)
{
  /*
   * Motor-controller velocity is currently assumed to be an IEEE-754 float
   * packed into one CAN word.  If the controller uses scaled integers instead,
   * change this conversion and VehicleSpeed_UpdateFromCANVelocity().
   */
  float value;

  memcpy(&value, &raw, sizeof(value));
  return value;
}

static void VehicleSpeed_UpdateFromCANVelocity(uint32_t id, uint32_t velocity_word)
{
  /*
   * Decode MC1VEL/MC2VEL high word as meters/second, normalize to mph, then
   * mark it fresh for CAN_SPEED_VALID_MS.  Absolute value is used because
   * display speed is usually non-directional here.
   */
  float velocity_mps = CAN_U32ToFloat(velocity_word);
  float speed_mph;

  if (!isfinite(velocity_mps))
  {
    return;
  }

  speed_mph = fabsf(velocity_mps) * MPS_TO_MPH;

  if (speed_mph > CAN_SPEED_MAX_ABS_MPH)
  {
    return;
  }

  latest_can_speed_mph = speed_mph;
  latest_can_speed_valid = 1;
  latest_can_speed_ms = HAL_GetTick();
  latest_can_speed_source =
      (id == (MC_CAN_BASE1 + MC_VELOCITY)) ? "CAN_MC1" : "CAN_MC2";

  VehicleSpeed_UpdateFromSources();
}

static void SunRaw_UpdateFromCAN(uint32_t id, uint8_t dlc, const uint8_t *data)
{
  /*
   * Store raw CAN payloads for RS232 legacy display.  The table keeps high/low
   * words exactly as hexadecimal so external display code can decode whatever
   * packet-specific units it needs.
   */
  if ((data == NULL) || (dlc < 8))
  {
    return;
  }

  for (uint32_t i = 0; i < SUN_RAW_TABLE_COUNT; i++)
  {
    if (sun_raw_table[i].can_id == id)
    {
      /*
       * Old telemetry format is name, high 32-bit word, low 32-bit word.
       * Data bytes 0..3 are low word. Data bytes 4..7 are high word.
       */
      sun_raw_table[i].low_word  = CAN_MakeU32_LE(&data[0]);
      sun_raw_table[i].high_word = CAN_MakeU32_LE(&data[4]);
      sun_raw_table[i].valid = 1;
      sun_raw_table[i].rx_count++;
      sun_raw_table[i].last_ms = HAL_GetTick();

      if ((id == (MC_CAN_BASE1 + MC_VELOCITY)) ||
          (id == (MC_CAN_BASE2 + MC_VELOCITY)))
      {
        VehicleSpeed_UpdateFromCANVelocity(id, sun_raw_table[i].high_word);
      }

      return;
    }
  }
}

static int SunRaw_BuildBlock(char *out, size_t out_len)
{
  /*
   * Build the complete RS232 block in memory first, then transmit it once.
   * This avoids interleaving partial lines with other UART/debug output and
   * lets us bail out if the block would exceed SUN_RAW_BLOCK_LEN.
   */
  size_t pos = 0;
  DateTime_t dt = {0};
  char time_text[40] = "RTC_READ_FAIL";

  if ((out == NULL) || (out_len == 0))
  {
    return -1;
  }

  if (adalogger_rtc_ok && ADALOGGER_RTC_Read(&dt))
  {
    if (dt.valid)
    {
      snprintf(time_text, sizeof(time_text),
               "%04u-%02u-%02uT%02u:%02u:%02u",
               dt.year,
               dt.month,
               dt.day,
               dt.hour,
               dt.minute,
               dt.second);
    }
    else
    {
      snprintf(time_text, sizeof(time_text),
               "%04u-%02u-%02uT%02u:%02u:%02u_INVALID",
               dt.year,
               dt.month,
               dt.day,
               dt.hour,
               dt.minute,
               dt.second);
    }
  }

  int written = snprintf(&out[pos], out_len - pos,
                         "raw_data\r\n"
                         "ABCDEF\r\n");

  if (written < 0)
  {
    return -1;
  }

  pos += (size_t)written;

  for (uint32_t i = 0; i < SUN_RAW_TABLE_COUNT; i++)
  {
    uint32_t high_word;
    uint32_t low_word;
    uint8_t valid;

    /*
     * Snapshot each row because CAN interrupt can update it.
     */
    __disable_irq();
    high_word = sun_raw_table[i].high_word;
    low_word  = sun_raw_table[i].low_word;
    valid     = sun_raw_table[i].valid;
    __enable_irq();

    if (valid)
    {
      written = snprintf(&out[pos], out_len - pos,
                         "%s,0x%08lX,0x%08lX\r\n",
                         sun_raw_table[i].name,
                         (unsigned long)high_word,
                         (unsigned long)low_word);
    }
    else
    {
      written = snprintf(&out[pos], out_len - pos,
                         "%s,0xHHHHHHHH,0xHHHHHHHH\r\n",
                         sun_raw_table[i].name);
    }

    if (written < 0)
    {
      return -1;
    }

    pos += (size_t)written;

    if (pos >= out_len)
    {
      return -1;
    }
  }

  written = snprintf(&out[pos], out_len - pos,
                     "BME,T=%.2f,P=%.2f,H=%.2f\r\n",
                     latest_bme_temp_c,
                     latest_bme_pressure_pa,
                     latest_bme_humidity_pct);
  if (written < 0) return -1;
  pos += (size_t)written;

  VehicleSpeed_UpdateFromSources();

  /*
   * NAV is the compact display line for RS232.  CAN speed is not repeated here
   * because MC1VEL/MC2VEL already carry the raw hexadecimal velocity packet
   * above; VEHICLE_MPH reflects whichever source currently wins arbitration.
   */
  uint32_t gps_fix_age_ms = latest_gps_fix_valid ?
                            (HAL_GetTick() - latest_gps_fix_ms) :
                            0UL;

  written = snprintf(&out[pos], out_len - pos,
                     "NAV,IMU_MPH=%.2f,GPS_MPH=%.2f,GPS_VALID=%u,VEHICLE_MPH=%.2f,SOURCE=%s,LAT=%.6f,LON=%.6f,FIX=%u,AGE_MS=%lu\r\n",
                     latest_imu_mph,
                     latest_gps_speed_mph,
                     latest_gps_speed_valid,
                     latest_vehicle_speed_mph,
                     latest_vehicle_speed_source,
                     latest_gps_lat_deg,
                     latest_gps_lon_deg,
                     latest_gps_fix_valid,
                     (unsigned long)gps_fix_age_ms);
  if (written < 0) return -1;
  pos += (size_t)written;

  written = snprintf(&out[pos], out_len - pos,
                     "TL_TIM,%s\r\n"
                     "VWXYZ\r\n",
                     time_text);

  if (written < 0)
  {
    return -1;
  }

  pos += (size_t)written;

  return (int)pos;
}

static void RS232_SendString(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart1,
                        (uint8_t *)text,
                        (uint16_t)strlen(text),
                        1000);

  if (status == HAL_OK)
  {
    StatusLed_RequestPulse(STATUS_RS232_TX);
  }
  else
  {
    StatusLed_SetError();
  }
}

static void RS232_SendSunRawBlock(void)
{
  int len = SunRaw_BuildBlock(sun_raw_block, sizeof(sun_raw_block));

  if (len > 0)
  {
    RS232_SendString(sun_raw_block);
  }
}

static void StatusLed_InitState(void)
{
  for (uint8_t i = 0; i < STATUS_ACTIVITY_COUNT; i++)
  {
    HAL_GPIO_WritePin(status_leds[i].port,
                      status_leds[i].pin,
                      STATUS_LED_INACTIVE);

    status_activity_request[i] = 0;
    status_activity_until[i] = 0;
  }

  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);

  status_error_latched = 0;
}

static void StatusLed_RequestPulse(StatusActivity_t activity)
{
  if (activity < STATUS_ACTIVITY_COUNT)
  {
    /*
     * Safe to call from normal code or from CAN callback.
     * The main status task handles the actual GPIO update.
     */
    status_activity_request[activity] = 1;
  }
}

static void StatusLed_SetError(void)
{
  status_error_latched = 1;
}

static void StatusLed_Task(void)
{
  static uint32_t last_heartbeat = 0;
  static uint8_t heartbeat_state = 0;

  uint32_t now = HAL_GetTick();

  /*
   * Heartbeat:
   * green blink = firmware alive
   * red blink = latched error
   */
  if (status_error_latched)
  {
    if ((now - last_heartbeat) >= STATUS_ERROR_BLINK_MS)
    {
      last_heartbeat = now;
      heartbeat_state ^= 1U;

      HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_Red_GPIO_Port,
                        LED_Red_Pin,
                        heartbeat_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
  }
  else
  {
    if ((now - last_heartbeat) >= STATUS_HEARTBEAT_MS)
    {
      last_heartbeat = now;
      heartbeat_state ^= 1U;

      HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_Green_GPIO_Port,
                        LED_Green_Pin,
                        heartbeat_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
  }

  /*
   * Activity LEDs:
   * Each request turns the LED on for STATUS_PULSE_MS.
   */
  for (uint8_t i = 0; i < STATUS_ACTIVITY_COUNT; i++)
  {
    if (status_activity_request[i])
    {
      status_activity_request[i] = 0;
      status_activity_until[i] = now + STATUS_PULSE_MS;
    }

    HAL_GPIO_WritePin(status_leds[i].port,
                      status_leds[i].pin,
                      (now < status_activity_until[i]) ?
                      STATUS_LED_ACTIVE :
                      STATUS_LED_INACTIVE);
  }
}

static void StatusLed_BootSequence(void)
{
  StatusLed_InitState();

  for (uint8_t i = 0; i < STATUS_ACTIVITY_COUNT; i++)
  {
    HAL_GPIO_WritePin(status_leds[i].port,
                      status_leds[i].pin,
                      STATUS_LED_ACTIVE);

    HAL_Delay(80);

    HAL_GPIO_WritePin(status_leds[i].port,
                      status_leds[i].pin,
                      STATUS_LED_INACTIVE);
  }

  StatusLed_RequestPulse(STATUS_BOOT_OK);
}

static void STM32_CAN_StoreRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
  if (rx_header->IDE == CAN_ID_STD)
  {
    latest_can_rx.id = rx_header->StdId;
    latest_can_rx.is_extended = 0;
  }
  else
  {
    latest_can_rx.id = rx_header->ExtId;
    latest_can_rx.is_extended = 1;
  }

  latest_can_rx.dlc = rx_header->DLC;

  for (uint8_t i = 0; i < 8; i++)
  {
    latest_can_rx.data[i] = rx_data[i];
  }

  latest_can_rx.count++;
  latest_can_rx.new_msg = 1;

  SunRaw_UpdateFromCAN(latest_can_rx.id,
                       latest_can_rx.dlc,
                       rx_data);

  StatusLed_RequestPulse(STATUS_CAN_RX);
}

static void STM32_CAN_PollRx(void)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
      STM32_CAN_StoreRxFrame(&rx_header, rx_data);
    }
  }
}

static void STM32_CAN_DebugPrint(void)
{
  uint32_t esr = CAN1->ESR;

  printf("CAN DBG: state=%d hal_err=0x%08lX ESR=0x%08lX MSR=0x%08lX RF0R=0x%08lX fill=%lu TEC=%lu REC=%lu LEC=%lu\r\n",
         HAL_CAN_GetState(&hcan1),
         (unsigned long)HAL_CAN_GetError(&hcan1),
         (unsigned long)CAN1->ESR,
         (unsigned long)CAN1->MSR,
         (unsigned long)CAN1->RF0R,
         (unsigned long)HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0),
         (unsigned long)((esr >> 16) & 0xFF),
         (unsigned long)((esr >> 24) & 0xFF),
         (unsigned long)((esr >> 4) & 0x7));
}

static uint8_t STM32_CAN_SendTestFrame(uint32_t std_id, uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint32_t start;

    tx_header.StdId = std_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        printf("CAN TX queue failed, HAL err=0x%08lX ESR=0x%08lX\r\n",
               (unsigned long)HAL_CAN_GetError(&hcan1),
               (unsigned long)CAN1->ESR);
        return 0;
    }

    start = HAL_GetTick();

    while (HAL_CAN_IsTxMessagePending(&hcan1, tx_mailbox))
    {
        if ((HAL_GetTick() - start) > 20)
        {
            printf("CAN TX pending timeout. ESR=0x%08lX TEC=%lu REC=%lu LEC=%lu\r\n",
                   (unsigned long)CAN1->ESR,
                   (unsigned long)((CAN1->ESR >> 16) & 0xFF),
                   (unsigned long)((CAN1->ESR >> 24) & 0xFF),
                   (unsigned long)((CAN1->ESR >> 4) & 0x7));
            return 0;
        }
    }

    printf("CAN TX complete ID=0x%03lX\r\n", (unsigned long)std_id);
    return 1;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  StatusLed_BootSequence();
  printf("Testing RTC sources...\r\n");

  adalogger_rtc_ok = ADALOGGER_RTC_Init();

  if (adalogger_rtc_ok)
  {
    #if SET_ADALOGGER_RTC_ON_BOOT
      DateTime_t set_adalogger_time =
      {
        .year = 2026,
        .month = 5,
        .day = 16,
        .hour = 17,
        .minute = 30,
        .second = 0,
        .weekday = 6,   // use your preferred weekday convention
        .valid = 1
      };

      if (ADALOGGER_RTC_Set(&set_adalogger_time))
      {
        printf("ADALOGGER_RTC manually set.\r\n");
      }
    #endif

      ADALOGGER_RTC_Print();
    }

    GPS_InitPins();
    SD_LogInit();

    uint8_t can_ok = STM32_CAN_ListenAllInit();

    if (can_ok)
    {
      printf("CAN1: listen-all mode ready.\r\n");
    }
    else
    {
      printf("CAN1: listen-all mode failed.\r\n");
    }
    printf("Starting BMI270 SensorAPI init...\r\n");

    int8_t bmi_result = BMI270_API_Init();

    printf("BMI270 API init final result = %d\r\n", bmi_result);

    BMI270_PrintInternalStatus();

    printf("Starting BME280 SensorAPI init...\r\n");
    bme280_ok = BME280_InitSensor();
    printf("BME280 API init final result = %d\r\n", bme280_ok);

    if (bmi_result == BMI2_OK)
    {
      BMI270_InitGravityEstimate();
    }

    if (bmi_result == BMI2_OK)
    {
      printf("BMI270 initialized successfully.\r\n");
      HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
    }
    else
    {
      printf("BMI270 initialization failed. result = %d\r\n", bmi_result);
      HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
    }

    Print_StartupSummary(bmi_result);
    HAL_Delay(1000);

    LED_AllOff();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
  	    /*
  	     * Service nonblocking SPI completions first on every loop pass.
  	     * GPS and ESP32 use interrupt-driven transfers; these calls process
  	     * completed transfers, handle timeouts, and keep buffers moving.
  	     */
  	    GPS_ServiceSPI();
  	    ESP32_ServiceSPI();

  	    /*
  	     * IMU task
  	     * Samples at 10 Hz for velocity integration, but only emits the larger
  	     * SD/ESP32/debug telemetry once per second.
  	     */
  	    static uint32_t last_imu_sample = 0;
  	    static uint32_t last_telemetry_log = 0;

  	    if ((HAL_GetTick() - last_imu_sample) >= IMU_SAMPLE_PERIOD_MS)
  	    {
  	      uint32_t imu_now_ms = HAL_GetTick();
  	      uint8_t telemetry_due =
  	          ((imu_now_ms - last_telemetry_log) >= TELEMETRY_PERIOD_MS);

  	      last_imu_sample = imu_now_ms;

  	      if (telemetry_due)
  	      {
  	        last_telemetry_log = imu_now_ms;
  	      }

  	      if (bmi_result == BMI2_OK)
  	      {
  	        /*
  	         * Read one accelerometer+gyro sample.  Every sample updates IMU
  	         * integrated speed; only telemetry_due samples are printed/logged.
  	         */
  	        struct bmi2_sens_data sensor_data = { 0 };

  	        int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi);

  	        if (rslt == BMI2_OK)
  	        {
  	          if (telemetry_due)
  	          {
  	            int16_t ax = sensor_data.acc.x;
  	            int16_t ay = sensor_data.acc.y;
  	            int16_t az = sensor_data.acc.z;

  	            int16_t gx = sensor_data.gyr.x;
  	            int16_t gy = sensor_data.gyr.y;
  	            int16_t gz = sensor_data.gyr.z;

  	            int32_t ax_mg = raw_acc_to_mg(ax);
  	            int32_t ay_mg = raw_acc_to_mg(ay);
  	            int32_t az_mg = raw_acc_to_mg(az);

  	            int32_t gx_mdps = raw_gyro_to_mdps(gx);
  	            int32_t gy_mdps = raw_gyro_to_mdps(gy);
  	            int32_t gz_mdps = raw_gyro_to_mdps(gz);

  	            printf("ACC raw: X=%6d Y=%6d Z=%6d | ACC mg: X=%6ld Y=%6ld Z=%6ld | "
  	                   "GYR raw: X=%6d Y=%6d Z=%6d | GYR mdps: X=%7ld Y=%7ld Z=%7ld\r\n",
  	                   ax, ay, az,
  	                   (long)ax_mg, (long)ay_mg, (long)az_mg,
  	                   gx, gy, gz,
  	                   (long)gx_mdps, (long)gy_mdps, (long)gz_mdps);
  	          }

  	          BMI270_UpdateVelocityAndPrint(&sensor_data, telemetry_due);

  	          if (telemetry_due)
  	          {
  	            /*
  	             * Logs one telemetry row to SD.
  	             * This row includes the latest CAN snapshot, BME values, parsed
  	             * GPS fix/speed, RTC values, and IMU values.
  	            */
  	            SD_LogIMUAndGPS(&sensor_data);

  	            /*
  	             * Sends the same telemetry row to the ESP32 if READY is active.
  	             */
  	            ESP32_SendTelemetry(&sensor_data);
  	          }
  	        }
  	        else
  	        {
  	          if (telemetry_due)
  	          {
  	            printf("bmi2_get_sensor_data failed: %d\r\n", rslt);
  	          }
  	          StatusLed_SetError();
  	        }
  	      }
  	      else
  	      {
  	        if (telemetry_due)
  	        {
  	          printf("BMI270 not initialized. result = %d\r\n", bmi_result);
  	        }
  	        StatusLed_SetError();
  	      }
  	    }

  	    /*
  	     * GPS task
  	     * Starts frequent SPI reads to keep the GPS output buffer drained.
  	     * GPS_ServiceSPI() above parses completed blocks from the byte ring.
  	     */
  	    static uint32_t last_gps_read = 0;

  	    if ((HAL_GetTick() - last_gps_read) >= GPS_POLL_PERIOD_MS)
  	    {
  	      last_gps_read = HAL_GetTick();
  	      GPS_PollSPI();
  	    }

  	    /*
  	     * BME280 task
  	     * Runs every 1000 ms.
  	     */
  	    static uint32_t last_bme_read = 0;

  	    if ((HAL_GetTick() - last_bme_read) >= 1000)
  	    {
  	      last_bme_read = HAL_GetTick();
  	      BME280_ReadAndPrint();
  	    }

  	    static uint32_t last_can_test = 0;

  	    /*
  	     * Local CAN test frame.  This is useful for bus bring-up, but on a real
  	     * vehicle it can be disabled if it creates unwanted traffic.
  	     */
  	    if ((HAL_GetTick() - last_can_test) >= 1000)
  	    {
  	      last_can_test = HAL_GetTick();

  	      uint8_t test_data[8] = {0x11, 0x22, 0x33, 0x44,
  	                              0x55, 0x66, 0x77, 0x88};

  	      STM32_CAN_SendTestFrame(0x402, test_data, 8);
  	    }

  	    /*
  	     * RS232 raw_data output task
  	     * Sends the legacy CAN hex table plus BME/NAV/time summary at 115200.
  	     */
  	    static uint32_t last_sun_raw_send = 0;

  	    if ((HAL_GetTick() - last_sun_raw_send) >= SUN_RAW_SEND_PERIOD_MS)
  	    {
  	      last_sun_raw_send = HAL_GetTick();

  	      RS232_SendSunRawBlock();

  	      /*
  	       * Optional SWV copy.
  	       */
  	      //printf("%s", sun_raw_block);
  	    }

  	    /*
  	     * CAN print task
  	     * Poll all pending frames so the CAN FIFO does not back up, then print
  	     * the newest frame for SWV/debug visibility.
  	     */
  	    STM32_CAN_PollRx();
  	    STM32_CAN_PrintLatest();

  	    static uint32_t last_can_debug = 0;

  	    if ((HAL_GetTick() - last_can_debug) >= 1000)
  	    {
  	      last_can_debug = HAL_GetTick();
  	      STM32_CAN_DebugPrint();
  	    }

  	    /*
  	     * LED heartbeat/activity task
  	     * Must run often so short status pulses are visible and error blink
  	     * timing remains stable.
  	     */
  	    StatusLed_Task();

  	    HAL_Delay(STATUS_TICK_MS);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin|LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_Green_Pin|LED_Red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BME_CS_Pin */
  GPIO_InitStruct.Pin = BME_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BME_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDC_CS_Pin */
  GPIO_InitStruct.Pin = SDC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SDC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_RST_Pin GPS_CS_Pin */
  GPIO_InitStruct.Pin = GPS_RST_Pin|GPS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED5_Pin LED6_Pin LED7_Pin ESP32_CS_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin|LED7_Pin|ESP32_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP32_Ready_Pin */
  GPIO_InitStruct.Pin = ESP32_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESP32_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Green_Pin LED_Red_Pin */
  GPIO_InitStruct.Pin = LED_Green_Pin|LED_Red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
