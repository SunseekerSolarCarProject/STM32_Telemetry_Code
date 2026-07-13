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

#include "telemetry_protocol.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESP32_READY_TIMEOUT_MS  500U
#define ESP32_SPI_TIMEOUT_MS    100U
#define ESP32_PASS_INTERVAL_MS  250U
#define ESP32_FAIL_BLINK_MS     100U

/* The board LEDs are wired active-low. */
#define LED_ON_STATE            GPIO_PIN_RESET
#define LED_OFF_STATE           GPIO_PIN_SET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi4;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* These values can be watched live in the STM32CubeIDE debugger. */
volatile int32_t esp32_last_result = -1;
volatile uint32_t esp32_ping_pass_count = 0;
volatile uint32_t esp32_ping_fail_count = 0;
volatile uint32_t esp32_consecutive_passes = 0;
volatile uint32_t esp32_last_hal_spi_error = 0;
volatile uint8_t esp32_last_slave_status = 0xFFU;
volatile uint8_t esp32_last_request[TELEMETRY_FRAME_SIZE];
volatile uint8_t esp32_last_response[TELEMETRY_FRAME_SIZE];

static uint8_t esp32_sequence = 0;
static uint32_t esp32_ping_number = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
static int32_t ESP32_PingTest(void);
static void ITM_PrintTestResult(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum esp32_test_result {
  ESP32_TEST_OK = 0,
  ESP32_TEST_REQUEST_READY_TIMEOUT = -1,
  ESP32_TEST_REQUEST_SPI_ERROR = -2,
  ESP32_TEST_RESPONSE_READY_TIMEOUT = -3,
  ESP32_TEST_RESPONSE_SPI_ERROR = -4,
  ESP32_TEST_BAD_RESPONSE_FRAME = -5,
  ESP32_TEST_BAD_RESPONSE_COMMAND = -6,
  ESP32_TEST_BAD_RESPONSE_SEQUENCE = -7,
  ESP32_TEST_SLAVE_REPORTED_ERROR = -8,
  ESP32_TEST_BAD_ECHO_LENGTH = -9,
  ESP32_TEST_BAD_ECHO_DATA = -10,
  ESP32_TEST_REQUEST_READY_STUCK_HIGH = -11,
  ESP32_TEST_RESPONSE_READY_STUCK_HIGH = -12,
};

enum esp32_exchange_result {
  ESP32_EXCHANGE_OK = 0,
  ESP32_EXCHANGE_READY_TIMEOUT = -1,
  ESP32_EXCHANGE_SPI_ERROR = -2,
  ESP32_EXCHANGE_READY_STUCK_HIGH = -3,
};

/* Newlib's _write() calls this function. CMSIS drops the character without
   blocking when ITM stimulus port 0 is not enabled by the debugger. */
int __io_putchar(int ch)
{
  (void)ITM_SendChar((uint32_t)(uint8_t)ch);
  return ch;
}

static void ITM_PrintFrame(const char *label,
                           const volatile uint8_t frame[TELEMETRY_FRAME_SIZE])
{
  printf("%s=", label);
  for (uint32_t i = 0; i < TELEMETRY_FRAME_SIZE; ++i) {
    printf("%02X", (unsigned int)frame[i]);
    if ((i + 1U) != TELEMETRY_FRAME_SIZE) {
      putchar(' ');
    }
  }
  printf("\r\n");
}

static void ITM_PrintTestResult(void)
{
  printf("[SPI] test=%lu result=%ld pass=%lu fail=%lu ready=%u "
         "tx_seq=%u rx_seq=%u status=%u\r\n",
         (unsigned long)esp32_ping_number,
         (long)esp32_last_result,
         (unsigned long)esp32_ping_pass_count,
         (unsigned long)esp32_ping_fail_count,
         (unsigned int)HAL_GPIO_ReadPin(ESP32_Ready_GPIO_Port,
                                        ESP32_Ready_Pin),
         (unsigned int)esp32_last_request[TELEMETRY_OFFSET_SEQUENCE],
         (unsigned int)esp32_last_response[TELEMETRY_OFFSET_SEQUENCE],
         (unsigned int)esp32_last_slave_status);
  ITM_PrintFrame("TX", esp32_last_request);
  ITM_PrintFrame("RX", esp32_last_response);
}

static int32_t ESP32_WaitReady(uint32_t timeout_ms)
{
  const uint32_t start = HAL_GetTick();

  while (HAL_GPIO_ReadPin(ESP32_Ready_GPIO_Port, ESP32_Ready_Pin) ==
         GPIO_PIN_RESET) {
    if ((HAL_GetTick() - start) >= timeout_ms) {
      return ESP32_EXCHANGE_READY_TIMEOUT;
    }
  }

  return ESP32_EXCHANGE_OK;
}

static int32_t ESP32_WaitReadyLow(uint32_t timeout_ms)
{
  const uint32_t start = HAL_GetTick();

  /* The ESP32 post-transaction callback pulls READY low. Observing this edge
     prevents a stale READY-high level from starting the next transfer early. */
  while (HAL_GPIO_ReadPin(ESP32_Ready_GPIO_Port, ESP32_Ready_Pin) ==
         GPIO_PIN_SET) {
    if ((HAL_GetTick() - start) >= timeout_ms) {
      return ESP32_EXCHANGE_READY_STUCK_HIGH;
    }
  }

  return ESP32_EXCHANGE_OK;
}

static int32_t ESP32_ExchangeFrame(uint8_t tx[TELEMETRY_FRAME_SIZE],
                                   uint8_t rx[TELEMETRY_FRAME_SIZE])
{
  if (ESP32_WaitReady(ESP32_READY_TIMEOUT_MS) != ESP32_EXCHANGE_OK) {
    return ESP32_EXCHANGE_READY_TIMEOUT;
  }

  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);
  const HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(&hspi4, tx, rx, TELEMETRY_FRAME_SIZE,
                              ESP32_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

  if (status != HAL_OK) {
    esp32_last_hal_spi_error = HAL_SPI_GetError(&hspi4);
    return ESP32_EXCHANGE_SPI_ERROR;
  }
  if (ESP32_WaitReadyLow(ESP32_READY_TIMEOUT_MS) != ESP32_EXCHANGE_OK) {
    return ESP32_EXCHANGE_READY_STUCK_HIGH;
  }

  return ESP32_EXCHANGE_OK;
}

static int32_t ESP32_RPC(uint8_t command, const uint8_t *payload,
                         uint8_t payload_length,
                         uint8_t response[TELEMETRY_FRAME_SIZE])
{
  uint8_t request[TELEMETRY_FRAME_SIZE];
  uint8_t ignored[TELEMETRY_FRAME_SIZE];
  uint8_t nop[TELEMETRY_FRAME_SIZE];
  const uint8_t request_sequence = ++esp32_sequence;

  telemetry_frame_build(request, command, request_sequence,
                        TELEMETRY_STATUS_OK, payload, payload_length);

  for (uint32_t i = 0; i < TELEMETRY_FRAME_SIZE; ++i) {
    esp32_last_request[i] = request[i];
    esp32_last_response[i] = 0U;
  }
  esp32_last_slave_status = 0xFFU;

  int32_t result = ESP32_ExchangeFrame(request, ignored);
  if (result == ESP32_EXCHANGE_READY_TIMEOUT) {
    return ESP32_TEST_REQUEST_READY_TIMEOUT;
  }
  if (result == ESP32_EXCHANGE_SPI_ERROR) {
    return ESP32_TEST_REQUEST_SPI_ERROR;
  }
  if (result == ESP32_EXCHANGE_READY_STUCK_HIGH) {
    return ESP32_TEST_REQUEST_READY_STUCK_HIGH;
  }

  /* The slave processes a request after CS rises, so clock the response out
     with a second, valid NOP transaction. */
  telemetry_frame_build(nop, TELEMETRY_CMD_NOP, ++esp32_sequence,
                        TELEMETRY_STATUS_OK, NULL, 0);
  result = ESP32_ExchangeFrame(nop, response);
  if (result == ESP32_EXCHANGE_READY_TIMEOUT) {
    return ESP32_TEST_RESPONSE_READY_TIMEOUT;
  }
  if (result == ESP32_EXCHANGE_SPI_ERROR) {
    return ESP32_TEST_RESPONSE_SPI_ERROR;
  }
  if (result == ESP32_EXCHANGE_READY_STUCK_HIGH) {
    return ESP32_TEST_RESPONSE_READY_STUCK_HIGH;
  }

  for (uint32_t i = 0; i < TELEMETRY_FRAME_SIZE; ++i) {
    esp32_last_response[i] = response[i];
  }

  if (telemetry_frame_validate(response) != TELEMETRY_STATUS_OK) {
    return ESP32_TEST_BAD_RESPONSE_FRAME;
  }
  if (response[TELEMETRY_OFFSET_COMMAND] !=
      (uint8_t)(command | TELEMETRY_RESPONSE_BIT)) {
    return ESP32_TEST_BAD_RESPONSE_COMMAND;
  }
  if (response[TELEMETRY_OFFSET_SEQUENCE] != request_sequence) {
    return ESP32_TEST_BAD_RESPONSE_SEQUENCE;
  }

  esp32_last_slave_status = response[TELEMETRY_OFFSET_STATUS];
  if (esp32_last_slave_status != TELEMETRY_STATUS_OK) {
    return ESP32_TEST_SLAVE_REPORTED_ERROR;
  }

  return ESP32_TEST_OK;
}

static int32_t ESP32_PingTest(void)
{
  uint8_t pattern[TELEMETRY_PAYLOAD_SIZE];
  uint8_t response[TELEMETRY_FRAME_SIZE];

  /* A changing, full-size pattern exercises every MOSI and MISO data byte. */
  ++esp32_ping_number;
  for (uint32_t i = 0; i < TELEMETRY_PAYLOAD_SIZE; ++i) {
    pattern[i] = (uint8_t)(esp32_ping_number + (i * 37U));
  }

  int32_t result = ESP32_RPC(TELEMETRY_CMD_PING, pattern,
                             (uint8_t)sizeof(pattern), response);
  if (result != ESP32_TEST_OK) {
    return result;
  }
  if (response[TELEMETRY_OFFSET_LENGTH] != sizeof(pattern)) {
    return ESP32_TEST_BAD_ECHO_LENGTH;
  }
  if (memcmp(&response[TELEMETRY_OFFSET_PAYLOAD], pattern,
             sizeof(pattern)) != 0) {
    return ESP32_TEST_BAD_ECHO_DATA;
  }

  return ESP32_TEST_OK;
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
  MX_SPI4_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  /* CS is active-low and must be high whenever no transaction is running. */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

  /* CubeMX initializes these active-low LEDs low, so turn all of them off. */
  HAL_GPIO_WritePin(GPIOD,
                    Green_LED11_Pin | Red_LED10_Pin | Yellow_LED9_Pin |
                    Yellow_LED8_Pin | Yellow_LED7_Pin | Yellow_LED6_Pin |
                    Yellow_LED5_Pin,
                    LED_OFF_STATE);

  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n[BOOT] ESP32 SPI test; SWV ITM port 0; core clock=%lu Hz\r\n",
         (unsigned long)HAL_RCC_GetHCLKFreq());

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    esp32_last_result = ESP32_PingTest();

    if (esp32_last_result == ESP32_TEST_OK) {
      ++esp32_ping_pass_count;
      ++esp32_consecutive_passes;
      HAL_GPIO_WritePin(Yellow_LED9_GPIO_Port, Yellow_LED9_Pin, LED_ON_STATE);
      ITM_PrintTestResult();
      HAL_Delay(ESP32_PASS_INTERVAL_MS);
    } else {
      ++esp32_ping_fail_count;
      esp32_consecutive_passes = 0;
      HAL_GPIO_TogglePin(Yellow_LED9_GPIO_Port, Yellow_LED9_Pin);
      ITM_PrintTestResult();
      HAL_Delay(ESP32_FAIL_BLINK_MS);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_4);

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

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
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ESP32_Ready_GPIO_Port, &GPIO_InitStruct);

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
