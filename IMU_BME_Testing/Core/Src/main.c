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
#include "bmi270.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMI270_CHIP_ID_REG                 0x00U
#define BMI270_ERROR_REG                   0x02U
#define BMI270_ACC_RANGE_REG               0x41U
#define BMI270_CHIP_ID_VALUE               0x24U
#define BMI270_SPI_READ_BIT                0x80U
#define BMI270_SPI_TIMEOUT_MS              100U
#define BMI270_STARTUP_DELAY_MS            10U
#define BMI270_SAMPLE_INTERVAL_MS          250U
#define BMI270_HEALTH_INTERVAL_MS          5000U
#define BMI270_RETRY_INTERVAL_MS           5000U
#define BMI270_CONFIG_BURST_LENGTH         64U
#define BMI270_RAW_PREFLIGHT_READS          5U
#define BMI270_DIAG_E_VERIFY               INT8_C(-100)
#define HEARTBEAT_TOGGLE_INTERVAL_MS       500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Watch these variables in the debugger when SWV text is unavailable. */
volatile uint8_t bmi270_chip_id = 0U;
volatile uint8_t bmi270_communication_ok = 0U;
volatile HAL_StatusTypeDef bmi270_last_hal_status = HAL_ERROR;
volatile uint32_t bmi270_success_count = 0U;
volatile uint32_t bmi270_error_count = 0U;
volatile uint8_t bmi270_ready = 0U;
volatile int8_t bmi270_last_result = BMI2_E_COM_FAIL;
volatile uint8_t bmi270_internal_status = 0U;
volatile uint8_t bmi270_status = 0U;
volatile int16_t bmi270_accel_raw[3] = {0};
volatile int16_t bmi270_gyro_raw[3] = {0};
volatile uint32_t bmi270_sensor_time = 0U;

/* Exact low-level transaction information. phase: 1=address, 2=data. */
volatile uint8_t bmi270_spi_last_op = 0U;
volatile uint8_t bmi270_spi_last_reg = 0U;
volatile uint32_t bmi270_spi_last_len = 0U;
volatile uint8_t bmi270_spi_last_phase = 0U;
volatile uint32_t bmi270_spi_last_hal_error = 0U;
volatile uint32_t bmi270_spi_last_sr = 0U;
volatile uint32_t bmi270_spi_transaction_count = 0U;
volatile uint32_t bmi270_spi_failure_count = 0U;

static struct bmi2_dev bmi270_dev;
static const char *bmi270_stage = "not-started";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch);
static void BMI270_DelayUs(uint32_t period_us, void *intf_ptr);
static BMI2_INTF_RETURN_TYPE BMI270_BusRead(uint8_t reg_addr,
                                            uint8_t *reg_data,
                                            uint32_t len,
                                            void *intf_ptr);
static BMI2_INTF_RETURN_TYPE BMI270_BusWrite(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t len,
                                             void *intf_ptr);
static HAL_StatusTypeDef BMI270_RawRead(uint8_t reg_addr,
                                        uint8_t *reg_data,
                                        uint16_t len);
static void BMI270_PrintBusState(void);
static void BMI270_PrintRegisters(const char *label);
static uint8_t BMI270_RawPreflight(void);
static int8_t BMI270_WriteReadbackTest(void);
static int8_t BMI270_StartActive(void);
static void BMI270_ReadSample(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Route printf output to SWV ITM stimulus port 0.
  */
int __io_putchar(int ch)
{
  ITM_SendChar((uint32_t)ch);
  return ch;
}

static void BMI270_DelayUs(uint32_t period_us, void *intf_ptr)
{
  uint32_t cycles_per_us;
  uint32_t start;

  (void)intf_ptr;
  cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  start = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - start) < (period_us * cycles_per_us))
  {
  }
}

static void BMI270_SaveBusState(uint8_t op,
                                uint8_t reg_addr,
                                uint32_t len,
                                uint8_t phase,
                                HAL_StatusTypeDef status)
{
  bmi270_spi_last_op = op;
  bmi270_spi_last_reg = (uint8_t)(reg_addr & 0x7FU);
  bmi270_spi_last_len = len;
  bmi270_spi_last_phase = phase;
  bmi270_last_hal_status = status;
  bmi270_spi_last_hal_error = HAL_SPI_GetError(&hspi1);
  bmi270_spi_last_sr = hspi1.Instance->SR;
  bmi270_spi_transaction_count++;

  if (status != HAL_OK)
  {
    bmi270_spi_failure_count++;
  }
}

static BMI2_INTF_RETURN_TYPE BMI270_BusRead(uint8_t reg_addr,
                                            uint8_t *reg_data,
                                            uint32_t len,
                                            void *intf_ptr)
{
  SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)intf_ptr;
  HAL_StatusTypeDef status;
  uint8_t phase = 1U;

  if ((spi == NULL) || (reg_data == NULL) || (len == 0U) || (len > UINT16_MAX))
  {
    return (BMI2_INTF_RETURN_TYPE)-1;
  }

  /* The BME280 shares SPI1 and must never be selected during an IMU transfer. */
  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(spi, &reg_addr, 1U, BMI270_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    phase = 2U;
    status = HAL_SPI_Receive(spi, reg_data, (uint16_t)len,
                             BMI270_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  BMI270_SaveBusState((uint8_t)'R', reg_addr, len, phase, status);
  return (status == HAL_OK) ? BMI2_INTF_RET_SUCCESS : (BMI2_INTF_RETURN_TYPE)-1;
}

static BMI2_INTF_RETURN_TYPE BMI270_BusWrite(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t len,
                                             void *intf_ptr)
{
  SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)intf_ptr;
  HAL_StatusTypeDef status;
  uint8_t phase = 1U;

  if ((spi == NULL) || (reg_data == NULL) || (len == 0U) || (len > UINT16_MAX))
  {
    return (BMI2_INTF_RETURN_TYPE)-1;
  }

  HAL_GPIO_WritePin(BME_CS_GPIO_Port, BME_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(spi, &reg_addr, 1U, BMI270_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    phase = 2U;
    status = HAL_SPI_Transmit(spi, (uint8_t *)reg_data, (uint16_t)len,
                              BMI270_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  BMI270_SaveBusState((uint8_t)'W', reg_addr, len, phase, status);
  return (status == HAL_OK) ? BMI2_INTF_RET_SUCCESS : (BMI2_INTF_RETURN_TYPE)-1;
}

static HAL_StatusTypeDef BMI270_RawRead(uint8_t reg_addr,
                                        uint8_t *reg_data,
                                        uint16_t len)
{
  uint8_t rx_data[17] = {0U};
  BMI2_INTF_RETURN_TYPE result;

  if ((reg_data == NULL) || (len == 0U) || (len > 16U))
  {
    return HAL_ERROR;
  }

  /* BMI270 SPI reads return one dummy byte before the requested register. */
  result = BMI270_BusRead((uint8_t)(reg_addr | BMI270_SPI_READ_BIT),
                          rx_data, (uint32_t)len + 1U, &hspi1);
  if (result == BMI2_INTF_RET_SUCCESS)
  {
    memcpy(reg_data, &rx_data[1], len);
    return HAL_OK;
  }

  return bmi270_last_hal_status;
}

static void BMI270_PrintBusState(void)
{
  printf("[BMI270 BUS] stage=%s op=%c reg=0x%02X len=%lu phase=%u "
         "HAL=%u HAL_ERR=0x%08lX HAL_STATE=%u SPI_SR=0x%08lX txns=%lu failures=%lu\r\n",
         bmi270_stage,
         (bmi270_spi_last_op != 0U) ? (char)bmi270_spi_last_op : '-',
         (unsigned int)bmi270_spi_last_reg,
         (unsigned long)bmi270_spi_last_len,
         (unsigned int)bmi270_spi_last_phase,
         (unsigned int)bmi270_last_hal_status,
         (unsigned long)bmi270_spi_last_hal_error,
         (unsigned int)HAL_SPI_GetState(&hspi1),
         (unsigned long)bmi270_spi_last_sr,
         (unsigned long)bmi270_spi_transaction_count,
         (unsigned long)bmi270_spi_failure_count);
}

static void BMI270_PrintRegisters(const char *label)
{
  uint8_t chip_id = 0U;
  uint8_t error = 0U;
  uint8_t status = 0U;
  uint8_t internal_status = 0U;
  uint8_t acc_config[2] = {0U};
  uint8_t gyr_config[2] = {0U};
  uint8_t power[2] = {0U};
  int8_t result = BMI2_OK;

  if (bmi270_ready != 0U)
  {
    result |= bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chip_id, 1U, &bmi270_dev);
    result |= bmi2_get_regs(BMI270_ERROR_REG, &error, 1U, &bmi270_dev);
    result |= bmi2_get_regs(BMI2_STATUS_ADDR, &status, 1U, &bmi270_dev);
    result |= bmi2_get_regs(BMI2_INTERNAL_STATUS_ADDR, &internal_status, 1U,
                            &bmi270_dev);
    result |= bmi2_get_regs(BMI2_ACC_CONF_ADDR, acc_config, 2U, &bmi270_dev);
    result |= bmi2_get_regs(BMI2_GYR_CONF_ADDR, gyr_config, 2U, &bmi270_dev);
    result |= bmi2_get_regs(BMI2_PWR_CONF_ADDR, power, 2U, &bmi270_dev);
  }
  else
  {
    if ((BMI270_RawRead(BMI2_CHIP_ID_ADDR, &chip_id, 1U) != HAL_OK) ||
        (BMI270_RawRead(BMI270_ERROR_REG, &error, 1U) != HAL_OK) ||
        (BMI270_RawRead(BMI2_STATUS_ADDR, &status, 1U) != HAL_OK) ||
        (BMI270_RawRead(BMI2_INTERNAL_STATUS_ADDR, &internal_status, 1U) != HAL_OK) ||
        (BMI270_RawRead(BMI2_ACC_CONF_ADDR, acc_config, 2U) != HAL_OK) ||
        (BMI270_RawRead(BMI2_GYR_CONF_ADDR, gyr_config, 2U) != HAL_OK) ||
        (BMI270_RawRead(BMI2_PWR_CONF_ADDR, power, 2U) != HAL_OK))
    {
      result = BMI2_E_COM_FAIL;
    }
  }

  bmi270_internal_status = internal_status;
  printf("[BMI270 REGS %s] result=%d CHIP_ID[00]=%02X ERR[02]=%02X "
         "STATUS[03]=%02X INTERNAL[21]=%02X ACC[40:41]=%02X,%02X "
         "GYR[42:43]=%02X,%02X PWR[7C:7D]=%02X,%02X\r\n",
         label, (int)result, (unsigned int)chip_id, (unsigned int)error,
         (unsigned int)status, (unsigned int)internal_status,
         (unsigned int)acc_config[0], (unsigned int)acc_config[1],
         (unsigned int)gyr_config[0], (unsigned int)gyr_config[1],
         (unsigned int)power[0], (unsigned int)power[1]);
}

static uint8_t BMI270_RawPreflight(void)
{
  uint8_t chip_id = 0U;
  uint8_t valid_reads = 0U;
  uint8_t index;

  /* The first access after power-up selects SPI and is intentionally ignored. */
  bmi270_stage = "raw-spi-wakeup";
  (void)BMI270_RawRead(BMI270_CHIP_ID_REG, &chip_id, 1U);

  bmi270_stage = "raw-chip-id";
  for (index = 0U; index < BMI270_RAW_PREFLIGHT_READS; index++)
  {
    bmi270_last_hal_status = BMI270_RawRead(BMI270_CHIP_ID_REG, &chip_id, 1U);
    bmi270_chip_id = chip_id;
    if ((bmi270_last_hal_status == HAL_OK) &&
        (chip_id == BMI270_CHIP_ID_VALUE))
    {
      valid_reads++;
      bmi270_success_count++;
    }
    else
    {
      bmi270_error_count++;
    }
  }

  bmi270_communication_ok = (valid_reads == BMI270_RAW_PREFLIGHT_READS) ? 1U : 0U;
  printf("[BMI270 PREFLIGHT] valid=%u/%u CHIP_ID=0x%02X expected=0x%02X HAL=%u\r\n",
         (unsigned int)valid_reads,
         (unsigned int)BMI270_RAW_PREFLIGHT_READS,
         (unsigned int)bmi270_chip_id,
         (unsigned int)BMI270_CHIP_ID_VALUE,
         (unsigned int)bmi270_last_hal_status);
  BMI270_PrintRegisters("RESET");

  if (bmi270_communication_ok == 0U)
  {
    BMI270_PrintBusState();
  }

  return bmi270_communication_ok;
}

static int8_t BMI270_WriteReadbackTest(void)
{
  uint8_t saved = 0U;
  uint8_t changed = 0U;
  uint8_t readback = 0U;
  uint8_t restored = 0U;
  uint8_t have_saved = 0U;
  int8_t result;

  bmi270_stage = "write-readback";
  result = bmi2_get_regs(BMI270_ACC_RANGE_REG, &saved, 1U, &bmi270_dev);
  if (result == BMI2_OK)
  {
    have_saved = 1U;
    changed = (uint8_t)((saved & (uint8_t)~BMI2_ACC_RANGE_MASK) |
                        (((saved & BMI2_ACC_RANGE_MASK) + 1U) & BMI2_ACC_RANGE_MASK));
    result = bmi2_set_regs(BMI270_ACC_RANGE_REG, &changed, 1U, &bmi270_dev);
  }
  if (result == BMI2_OK)
  {
    result = bmi2_get_regs(BMI270_ACC_RANGE_REG, &readback, 1U, &bmi270_dev);
  }
  if ((result == BMI2_OK) && (readback != changed))
  {
    result = BMI270_DIAG_E_VERIFY;
  }

  /* If the original value was read, restore the live range immediately. */
  if ((have_saved != 0U) &&
      (bmi2_set_regs(BMI270_ACC_RANGE_REG, &saved, 1U, &bmi270_dev) == BMI2_OK))
  {
    if (bmi2_get_regs(BMI270_ACC_RANGE_REG, &restored, 1U, &bmi270_dev) != BMI2_OK)
    {
      result = BMI2_E_COM_FAIL;
    }
  }
  else if (have_saved != 0U)
  {
    result = BMI2_E_COM_FAIL;
  }
  if ((result == BMI2_OK) && (restored != saved))
  {
    result = BMI270_DIAG_E_VERIFY;
  }

  printf("[BMI270 WRITE TEST] result=%d reg=0x%02X saved=0x%02X "
         "wrote=0x%02X read=0x%02X restored=0x%02X %s\r\n",
         (int)result, (unsigned int)BMI270_ACC_RANGE_REG,
         (unsigned int)saved, (unsigned int)changed,
         (unsigned int)readback, (unsigned int)restored,
         (result == BMI2_OK) ? "PASS" : "FAIL");
  return result;
}

static int8_t BMI270_StartActive(void)
{
  struct bmi2_sens_config config[2] = {0};
  uint8_t sensors[2] = {BMI2_ACCEL, BMI2_GYRO};
  int8_t result;

  bmi270_ready = 0U;
  memset(&bmi270_dev, 0, sizeof(bmi270_dev));
  bmi270_dev.intf = BMI2_SPI_INTF;
  bmi270_dev.intf_ptr = &hspi1;
  bmi270_dev.read = BMI270_BusRead;
  bmi270_dev.write = BMI270_BusWrite;
  bmi270_dev.delay_us = BMI270_DelayUs;
  bmi270_dev.read_write_len = BMI270_CONFIG_BURST_LENGTH;

  bmi270_stage = "bmi270-init";
  result = bmi270_init(&bmi270_dev);
  printf("[BMI270 INIT] stage=%s result=%d chip=0x%02X load_status=0x%02X\r\n",
         bmi270_stage, (int)result, (unsigned int)bmi270_dev.chip_id,
         (unsigned int)bmi270_dev.load_status);
  if (result != BMI2_OK)
  {
    BMI270_PrintBusState();
    printf("[BMI270 BOOT] ready=0 result=%d failed_stage=%s\r\n",
           (int)result, bmi270_stage);
    return result;
  }

  config[0].type = BMI2_ACCEL;
  config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

  config[1].type = BMI2_GYRO;
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  config[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

  bmi270_stage = "set-config";
  result = bmi2_set_sensor_config(config, 2U, &bmi270_dev);
  printf("[BMI270 INIT] stage=%s result=%d\r\n", bmi270_stage, (int)result);
  if (result != BMI2_OK)
  {
    BMI270_PrintBusState();
    printf("[BMI270 BOOT] ready=0 result=%d failed_stage=%s\r\n",
           (int)result, bmi270_stage);
    return result;
  }

  bmi270_stage = "enable-accel-gyro";
  result = bmi270_sensor_enable(sensors, 2U, &bmi270_dev);
  printf("[BMI270 INIT] stage=%s result=%d\r\n", bmi270_stage, (int)result);
  if (result != BMI2_OK)
  {
    BMI270_PrintBusState();
    printf("[BMI270 BOOT] ready=0 result=%d failed_stage=%s\r\n",
           (int)result, bmi270_stage);
    return result;
  }

  HAL_Delay(50U);
  bmi270_ready = 1U;
  BMI270_PrintRegisters("ACTIVE");

  result = BMI270_WriteReadbackTest();
  if (result != BMI2_OK)
  {
    bmi270_ready = 0U;
    BMI270_PrintBusState();
    printf("[BMI270 BOOT] ready=0 result=%d failed_stage=%s\r\n",
           (int)result, bmi270_stage);
    return result;
  }

  bmi270_stage = "active";
  bmi270_communication_ok = 1U;
  printf("[BMI270 BOOT] ready=1 result=0; accel=100Hz +/-4g gyro=100Hz +/-2000dps\r\n");
  return BMI2_OK;
}

static void BMI270_ReadSample(void)
{
  static uint32_t sequence = 0U;
  static uint32_t previous_sensor_time = 0U;
  static uint32_t unchanged_time_count = 0U;
  struct bmi2_sens_data data = {0};
  int32_t ax_mg;
  int32_t ay_mg;
  int32_t az_mg;
  int32_t gx_mdps;
  int32_t gy_mdps;
  int32_t gz_mdps;
  int8_t result;

  bmi270_stage = "sample";
  result = bmi2_get_sensor_data(&data, &bmi270_dev);
  bmi270_last_result = result;
  if (result != BMI2_OK)
  {
    bmi270_communication_ok = 0U;
    bmi270_ready = 0U;
    bmi270_error_count++;
    printf("[BMI270 SAMPLE] result=%d FAIL\r\n", (int)result);
    BMI270_PrintBusState();
    return;
  }

  bmi270_status = data.status;
  bmi270_accel_raw[0] = data.acc.x;
  bmi270_accel_raw[1] = data.acc.y;
  bmi270_accel_raw[2] = data.acc.z;
  bmi270_gyro_raw[0] = data.gyr.x;
  bmi270_gyro_raw[1] = data.gyr.y;
  bmi270_gyro_raw[2] = data.gyr.z;
  bmi270_sensor_time = data.sens_time;

  if (data.sens_time == previous_sensor_time)
  {
    unchanged_time_count++;
  }
  else
  {
    unchanged_time_count = 0U;
  }
  previous_sensor_time = data.sens_time;

  ax_mg = ((int32_t)data.acc.x * 4000) / 32768;
  ay_mg = ((int32_t)data.acc.y * 4000) / 32768;
  az_mg = ((int32_t)data.acc.z * 4000) / 32768;
  gx_mdps = (int32_t)(((int64_t)data.gyr.x * 2000000LL) / 32768LL);
  gy_mdps = (int32_t)(((int64_t)data.gyr.y * 2000000LL) / 32768LL);
  gz_mdps = (int32_t)(((int64_t)data.gyr.z * 2000000LL) / 32768LL);

  sequence++;
  bmi270_success_count++;
  printf("[BMI270 SAMPLE] n=%lu result=0 STATUS=0x%02X DRDY_A=%u DRDY_G=%u "
         "TIME=%lu time_stuck=%lu Araw=%d,%d,%d Amg=%ld,%ld,%ld "
         "Graw=%d,%d,%d Gmdps=%ld,%ld,%ld\r\n",
         (unsigned long)sequence, (unsigned int)data.status,
         ((data.status & BMI2_DRDY_ACC) != 0U) ? 1U : 0U,
         ((data.status & BMI2_DRDY_GYR) != 0U) ? 1U : 0U,
         (unsigned long)data.sens_time, (unsigned long)unchanged_time_count,
         (int)data.acc.x, (int)data.acc.y, (int)data.acc.z,
         (long)ax_mg, (long)ay_mg, (long)az_mg,
         (int)data.gyr.x, (int)data.gyr.y, (int)data.gyr.z,
         (long)gx_mdps, (long)gy_mdps, (long)gz_mdps);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t heartbeat_tick = HAL_GetTick();
  uint32_t imu_sample_tick;
  uint32_t imu_health_tick;
  uint32_t imu_retry_tick;

  /* Enable the Cortex-M4 cycle counter for Bosch's microsecond delays. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  printf("\r\n========================================\r\n");
  printf("STM32 BMI270 SPI register and live-data diagnostic\r\n");
  printf("SWV ITM port 0, core clock %lu Hz\r\n",
         (unsigned long)HAL_RCC_GetHCLKFreq());
  printf("SPI1 clock=%lu Hz mode=0 MSB-first; polling transfers (no SPI IRQ required)\r\n",
         (unsigned long)(HAL_RCC_GetPCLK2Freq() / 16U));
  printf("Pins: PA5=SCK PA6=MISO PA7=MOSI PA3=IMU_CS; BME_CS held high\r\n");
  printf("Expected BMI270 CHIP_ID: 0x%02X\r\n",
         (unsigned int)BMI270_CHIP_ID_VALUE);
  printf("========================================\r\n");

  /* Allow the BMI270 to finish power-on before its first SPI transaction. */
  HAL_Delay(BMI270_STARTUP_DELAY_MS);
  if (BMI270_RawPreflight() != 0U)
  {
    bmi270_last_result = BMI270_StartActive();
  }
  else
  {
    bmi270_last_result = BMI2_E_DEV_NOT_FOUND;
    printf("[BMI270 BOOT] ready=0 result=%d; initialization skipped until CHIP_ID is 0x24\r\n",
           (int)bmi270_last_result);
  }
  imu_sample_tick = HAL_GetTick();
  imu_health_tick = HAL_GetTick();
  imu_retry_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* A toggle every 500 ms produces one complete LED blink each second. */
    if ((uint32_t)(now - heartbeat_tick) >= HEARTBEAT_TOGGLE_INTERVAL_MS)
    {
      heartbeat_tick = now;
      HAL_GPIO_TogglePin(Green_LED11_GPIO_Port, Green_LED11_Pin);
    }

    if ((bmi270_ready != 0U) &&
        ((uint32_t)(now - imu_sample_tick) >= BMI270_SAMPLE_INTERVAL_MS))
    {
      imu_sample_tick = now;
      BMI270_ReadSample();
    }

    /* Periodically expose configuration and error registers while active. */
    if ((bmi270_ready != 0U) &&
        ((uint32_t)(now - imu_health_tick) >= BMI270_HEALTH_INTERVAL_MS))
    {
      imu_health_tick = now;
      bmi270_stage = "health-registers";
      BMI270_PrintRegisters("HEALTH");
    }

    /* A disconnected or failed device is retried without requiring a reset. */
    if ((bmi270_ready == 0U) &&
        ((uint32_t)(now - imu_retry_tick) >= BMI270_RETRY_INTERVAL_MS))
    {
      imu_retry_tick = now;
      if (BMI270_RawPreflight() != 0U)
      {
        bmi270_last_result = BMI270_StartActive();
      }
      else
      {
        bmi270_last_result = BMI2_E_DEV_NOT_FOUND;
      }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin|BME_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_LED11_GPIO_Port, Green_LED11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU_CS_Pin BME_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin|BME_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED11_Pin */
  GPIO_InitStruct.Pin = Green_LED11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED11_GPIO_Port, &GPIO_InitStruct);

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
