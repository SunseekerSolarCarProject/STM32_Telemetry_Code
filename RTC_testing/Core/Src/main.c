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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint8_t connected;
  uint8_t rtc_mode;
  uint8_t oscillator_stop_flag;
  uint8_t counter_stopped;
  uint8_t hundredths;
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint8_t day;
  uint8_t weekday;
  uint8_t month;
  uint16_t year;
  uint8_t raw_time[8];
  uint8_t raw_oscillator;
  uint8_t raw_function;
  uint8_t raw_stop_enable;
  uint8_t scl_level;
  uint8_t sda_level;
  uint16_t i2c_sr1;
  uint16_t i2c_sr2;
  uint32_t sample_count;
  uint32_t advance_count;
  uint32_t consecutive_failures;
  uint32_t recovery_count;
  uint32_t i2c_error;
  HAL_StatusTypeDef last_hal_status;
} RTC_TestResult;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PCF85263A_I2C_ADDRESS       (0x51U << 1)
#define PCF85263A_REG_TIME          0x00U
#define PCF85263A_REG_OSCILLATOR    0x25U
#define PCF85263A_REG_FUNCTION      0x28U
#define PCF85263A_REG_STOP_ENABLE   0x2EU
#define PCF85263A_I2C_TIMEOUT_MS    100U
#define RTC_SWV_STATUS_PERIOD_MS    1000U
#define RTC_RECOVERY_FAILURE_COUNT  20U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* Add rtc_test to STM32CubeIDE Live Expressions while debugging. */
volatile RTC_TestResult rtc_test;
static uint32_t rtc_swv_last_print_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

static uint8_t RTC_BcdToBinary(uint8_t value);
static void RTC_RecordI2CState(HAL_StatusTypeDef status);
static void RTC_I2CRecoverBus(void);
static void RTC_TestUpdate(void);
static void RTC_SWVPrintStatus(void);
static void SWV_SendChar(char ch);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void SWV_SendChar(char ch)
{
  uint32_t timeout;

  /* Never block the firmware when SWV/ITM is not enabled by the debugger. */
  if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) ||
      ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U) ||
      ((ITM->TER & 1UL) == 0U))
  {
    return;
  }

  timeout = 10000U;
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
  int index;

  (void)file;
  for (index = 0; index < len; index++)
  {
    SWV_SendChar(ptr[index]);
  }

  return len;
}

static uint8_t RTC_BcdToBinary(uint8_t value)
{
  return (uint8_t)(((value >> 4) * 10U) + (value & 0x0FU));
}

static void RTC_RecordI2CState(HAL_StatusTypeDef status)
{
  rtc_test.last_hal_status = status;
  rtc_test.i2c_error = HAL_I2C_GetError(&hi2c1);
  rtc_test.i2c_sr1 = (uint16_t)hi2c1.Instance->SR1;
  rtc_test.i2c_sr2 = (uint16_t)hi2c1.Instance->SR2;
  rtc_test.scl_level = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) ? 1U : 0U;
  rtc_test.sda_level = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
}

static void RTC_I2CRecoverBus(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t pulse;

  rtc_test.recovery_count++;
  (void)HAL_I2C_DeInit(&hi2c1);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_Delay(1U);

  /* Release a target that stopped part-way through a byte. */
  for (pulse = 0U; pulse < 9U; pulse++)
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
    {
      break;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1U);
  }

  /* Generate a manual STOP: SDA low, SCL high, then SDA high. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1U);

  MX_I2C1_Init();
  rtc_test.consecutive_failures = 0U;
  RTC_RecordI2CState(HAL_OK);
}

static void RTC_TestUpdate(void)
{
  uint8_t time_raw[8];
  uint8_t oscillator;
  uint8_t function;
  uint8_t stop_enable;
  uint16_t previous_time;
  uint16_t current_time;
  HAL_StatusTypeDef status;

  rtc_test.sample_count++;

  status = HAL_I2C_IsDeviceReady(&hi2c1, PCF85263A_I2C_ADDRESS, 2U,
                                 PCF85263A_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    rtc_test.connected = 0U;
    rtc_test.consecutive_failures++;
    RTC_RecordI2CState(status);
    return;
  }

  status = HAL_I2C_Mem_Read(&hi2c1, PCF85263A_I2C_ADDRESS,
                            PCF85263A_REG_TIME, I2C_MEMADD_SIZE_8BIT,
                            time_raw, sizeof(time_raw),
                            PCF85263A_I2C_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, PCF85263A_I2C_ADDRESS,
                              PCF85263A_REG_OSCILLATOR,
                              I2C_MEMADD_SIZE_8BIT, &oscillator, 1U,
                              PCF85263A_I2C_TIMEOUT_MS);
  }
  if (status == HAL_OK)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, PCF85263A_I2C_ADDRESS,
                              PCF85263A_REG_FUNCTION,
                              I2C_MEMADD_SIZE_8BIT, &function, 1U,
                              PCF85263A_I2C_TIMEOUT_MS);
  }
  if (status == HAL_OK)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, PCF85263A_I2C_ADDRESS,
                              PCF85263A_REG_STOP_ENABLE,
                              I2C_MEMADD_SIZE_8BIT, &stop_enable, 1U,
                              PCF85263A_I2C_TIMEOUT_MS);
  }

  RTC_RecordI2CState(status);
  if (status != HAL_OK)
  {
    rtc_test.connected = 0U;
    rtc_test.consecutive_failures++;
    return;
  }

  previous_time = (uint16_t)(((uint16_t)rtc_test.raw_time[1] << 8) |
                             rtc_test.raw_time[0]);
  current_time = (uint16_t)(((uint16_t)time_raw[1] << 8) | time_raw[0]);
  if ((rtc_test.connected != 0U) && (current_time != previous_time))
  {
    rtc_test.advance_count++;
  }

  rtc_test.connected = 1U;
  rtc_test.consecutive_failures = 0U;
  rtc_test.raw_oscillator = oscillator;
  rtc_test.raw_function = function;
  rtc_test.raw_stop_enable = stop_enable;
  rtc_test.rtc_mode = ((function & 0x10U) == 0U) ? 1U : 0U;
  rtc_test.oscillator_stop_flag = ((time_raw[1] & 0x80U) != 0U) ? 1U : 0U;
  rtc_test.counter_stopped = stop_enable & 0x01U;

  rtc_test.hundredths = RTC_BcdToBinary(time_raw[0]);
  rtc_test.seconds = RTC_BcdToBinary(time_raw[1] & 0x7FU);
  rtc_test.minutes = RTC_BcdToBinary(time_raw[2] & 0x7FU);
  rtc_test.hours = RTC_BcdToBinary(time_raw[3] & 0x3FU);
  rtc_test.day = RTC_BcdToBinary(time_raw[4] & 0x3FU);
  rtc_test.weekday = time_raw[5] & 0x07U;
  rtc_test.month = RTC_BcdToBinary(time_raw[6] & 0x1FU);
  rtc_test.year = (uint16_t)(2000U + RTC_BcdToBinary(time_raw[7]));

  for (uint32_t index = 0U; index < sizeof(time_raw); index++)
  {
    rtc_test.raw_time[index] = time_raw[index];
  }
}

static void RTC_SWVPrintStatus(void)
{
  if (rtc_test.connected == 0U)
  {
    printf("[RTC] NO RESPONSE address=0x51 HAL=%u error=0x%08lX "
           "SCL=%u SDA=%u SR1=0x%04X SR2=0x%04X failures=%lu recoveries=%lu\r\n",
           (unsigned int)rtc_test.last_hal_status,
           (unsigned long)rtc_test.i2c_error,
           (unsigned int)rtc_test.scl_level,
           (unsigned int)rtc_test.sda_level,
           (unsigned int)rtc_test.i2c_sr1,
           (unsigned int)rtc_test.i2c_sr2,
           (unsigned long)rtc_test.consecutive_failures,
           (unsigned long)rtc_test.recovery_count);
    return;
  }

  printf("[RTC] OK %04u-%02u-%02u %02u:%02u:%02u.%02u "
         "weekday=%u advancing=%lu rtc_mode=%u OS_flag=%u stopped=%u\r\n",
         (unsigned int)rtc_test.year,
         (unsigned int)rtc_test.month,
         (unsigned int)rtc_test.day,
         (unsigned int)rtc_test.hours,
         (unsigned int)rtc_test.minutes,
         (unsigned int)rtc_test.seconds,
         (unsigned int)rtc_test.hundredths,
         (unsigned int)rtc_test.weekday,
         (unsigned long)rtc_test.advance_count,
         (unsigned int)rtc_test.rtc_mode,
         (unsigned int)rtc_test.oscillator_stop_flag,
         (unsigned int)rtc_test.counter_stopped);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  RTC_TestUpdate();
  printf("\r\n[BOOT] PCF85263A I2C/SWV test; address=0x51; "
         "core clock=%lu Hz\r\n", (unsigned long)SystemCoreClock);
  RTC_SWVPrintStatus();
  rtc_swv_last_print_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    RTC_TestUpdate();
    if ((rtc_test.connected == 0U) &&
        (rtc_test.consecutive_failures >= RTC_RECOVERY_FAILURE_COUNT))
    {
      printf("[RTC] Attempting I2C bus recovery...\r\n");
      RTC_I2CRecoverBus();
      RTC_TestUpdate();
    }
    if ((HAL_GetTick() - rtc_swv_last_print_ms) >= RTC_SWV_STATUS_PERIOD_MS)
    {
      rtc_swv_last_print_ms = HAL_GetTick();
      RTC_SWVPrintStatus();
    }
    HAL_Delay(250U);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
