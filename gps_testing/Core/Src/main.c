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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  GPS_FRAME_WAIT = 0,
  GPS_FRAME_NMEA,
  GPS_FRAME_UBX_SYNC2,
  GPS_FRAME_UBX_CLASS,
  GPS_FRAME_UBX_ID,
  GPS_FRAME_UBX_LEN_L,
  GPS_FRAME_UBX_LEN_H,
  GPS_FRAME_UBX_PAYLOAD,
  GPS_FRAME_UBX_CK_A,
  GPS_FRAME_UBX_CK_B
} GPS_FrameState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_SPI_READ_LEN          256U
#define GPS_POLL_PERIOD_MS         20U
#define GPS_COMM_TIMEOUT_MS      3000U
#define GPS_STATUS_PERIOD_MS     1000U
#define HEARTBEAT_TOGGLE_MS       500U
#define GPS_NMEA_MAX_LEN          160U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
static uint8_t gps_spi_tx[GPS_SPI_READ_LEN];
static uint8_t gps_spi_rx[GPS_SPI_READ_LEN];
static char gps_nmea[GPS_NMEA_MAX_LEN];
static GPS_FrameState_t gps_frame_state = GPS_FRAME_WAIT;
static uint16_t gps_nmea_index = 0U;
static uint8_t gps_ubx_class = 0U;
static uint8_t gps_ubx_id = 0U;
static uint16_t gps_ubx_length = 0U;
static uint16_t gps_ubx_payload_index = 0U;
static uint8_t gps_ubx_ck_a = 0U;
static uint8_t gps_ubx_ck_b = 0U;
static uint8_t gps_ubx_received_ck_a = 0U;
static uint32_t gps_raw_byte_count = 0U;
static uint32_t gps_nmea_count = 0U;
static uint32_t gps_ubx_count = 0U;
static uint32_t gps_frame_error_count = 0U;
static uint32_t gps_last_valid_frame_ms = 0U;
static uint32_t gps_monitor_start_ms = 0U;
static uint8_t gps_spi_error = 0U;
static uint8_t gps_frame_error = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static void GPS_Init(void);
static void GPS_Task(void);
static void GPS_ProcessByte(uint8_t byte);
static void GPS_RecordValidFrame(void);
static uint8_t GPS_NMEA_ChecksumValid(const char *sentence);
static void GPS_UBX_ChecksumAdd(uint8_t byte);
static void StatusLED_Task(void);
static void SWV_SendChar(char ch);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SWV_SendChar(char ch)
{
  /* Do not block the application when SWV/ITM is not enabled by the debugger. */
  if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) ||
      ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U) ||
      ((ITM->TER & 1UL) == 0U))
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

static void GPS_UBX_ChecksumAdd(uint8_t byte)
{
  gps_ubx_ck_a = (uint8_t)(gps_ubx_ck_a + byte);
  gps_ubx_ck_b = (uint8_t)(gps_ubx_ck_b + gps_ubx_ck_a);
}

static uint8_t GPS_NMEA_ChecksumValid(const char *sentence)
{
  uint8_t checksum = 0U;
  uint8_t expected = 0U;
  const char *p = sentence;

  if ((p == NULL) || (*p != '$'))
  {
    return 0U;
  }

  p++;
  while ((*p != '\0') && (*p != '*') && (*p != '\r') && (*p != '\n'))
  {
    checksum ^= (uint8_t)*p;
    p++;
  }

  if ((*p != '*') || (p[1] == '\0') || (p[2] == '\0'))
  {
    return 0U;
  }

  for (uint8_t i = 1U; i <= 2U; i++)
  {
    char digit = p[i];
    expected <<= 4;

    if ((digit >= '0') && (digit <= '9'))
    {
      expected |= (uint8_t)(digit - '0');
    }
    else if ((digit >= 'A') && (digit <= 'F'))
    {
      expected |= (uint8_t)(digit - 'A' + 10);
    }
    else if ((digit >= 'a') && (digit <= 'f'))
    {
      expected |= (uint8_t)(digit - 'a' + 10);
    }
    else
    {
      return 0U;
    }
  }

  return (checksum == expected) ? 1U : 0U;
}

static void GPS_RecordValidFrame(void)
{
  gps_last_valid_frame_ms = HAL_GetTick();
  gps_spi_error = 0U;
  gps_frame_error = 0U;
}

static void GPS_ProcessByte(uint8_t byte)
{
  /* 0xFF means "no data" only while waiting for the start of a frame. */
  if ((gps_frame_state == GPS_FRAME_WAIT) && (byte == 0xFFU))
  {
    return;
  }

  gps_raw_byte_count++;

  switch (gps_frame_state)
  {
    case GPS_FRAME_WAIT:
      if (byte == (uint8_t)'$')
      {
        gps_nmea_index = 0U;
        gps_nmea[gps_nmea_index++] = (char)byte;
        gps_frame_state = GPS_FRAME_NMEA;
      }
      else if (byte == 0xB5U)
      {
        gps_frame_state = GPS_FRAME_UBX_SYNC2;
      }
      break;

    case GPS_FRAME_NMEA:
      if (byte == (uint8_t)'$')
      {
        /* Recover cleanly if a partial sentence was present at startup. */
        gps_nmea_index = 0U;
        gps_nmea[gps_nmea_index++] = (char)byte;
      }
      else if (gps_nmea_index < (GPS_NMEA_MAX_LEN - 1U))
      {
        gps_nmea[gps_nmea_index++] = (char)byte;

        if (byte == (uint8_t)'\n')
        {
          gps_nmea[gps_nmea_index] = '\0';

          if (GPS_NMEA_ChecksumValid(gps_nmea))
          {
            gps_nmea_count++;
            GPS_RecordValidFrame();
            printf("GPS NMEA: %s", gps_nmea);
          }
          else
          {
            gps_frame_error_count++;
            gps_frame_error = 1U;
          }

          gps_nmea_index = 0U;
          gps_frame_state = GPS_FRAME_WAIT;
        }
      }
      else
      {
        gps_frame_error_count++;
        gps_frame_error = 1U;
        gps_nmea_index = 0U;
        gps_frame_state = GPS_FRAME_WAIT;
      }
      break;

    case GPS_FRAME_UBX_SYNC2:
      if (byte == 0x62U)
      {
        gps_ubx_ck_a = 0U;
        gps_ubx_ck_b = 0U;
        gps_frame_state = GPS_FRAME_UBX_CLASS;
      }
      else
      {
        gps_frame_state = (byte == 0xB5U) ? GPS_FRAME_UBX_SYNC2 : GPS_FRAME_WAIT;
      }
      break;

    case GPS_FRAME_UBX_CLASS:
      gps_ubx_class = byte;
      GPS_UBX_ChecksumAdd(byte);
      gps_frame_state = GPS_FRAME_UBX_ID;
      break;

    case GPS_FRAME_UBX_ID:
      gps_ubx_id = byte;
      GPS_UBX_ChecksumAdd(byte);
      gps_frame_state = GPS_FRAME_UBX_LEN_L;
      break;

    case GPS_FRAME_UBX_LEN_L:
      gps_ubx_length = byte;
      GPS_UBX_ChecksumAdd(byte);
      gps_frame_state = GPS_FRAME_UBX_LEN_H;
      break;

    case GPS_FRAME_UBX_LEN_H:
      gps_ubx_length |= (uint16_t)((uint16_t)byte << 8);
      GPS_UBX_ChecksumAdd(byte);
      gps_ubx_payload_index = 0U;
      gps_frame_state = (gps_ubx_length == 0U) ? GPS_FRAME_UBX_CK_A : GPS_FRAME_UBX_PAYLOAD;
      break;

    case GPS_FRAME_UBX_PAYLOAD:
      GPS_UBX_ChecksumAdd(byte);
      gps_ubx_payload_index++;
      if (gps_ubx_payload_index >= gps_ubx_length)
      {
        gps_frame_state = GPS_FRAME_UBX_CK_A;
      }
      break;

    case GPS_FRAME_UBX_CK_A:
      gps_ubx_received_ck_a = byte;
      gps_frame_state = GPS_FRAME_UBX_CK_B;
      break;

    case GPS_FRAME_UBX_CK_B:
      if ((gps_ubx_received_ck_a == gps_ubx_ck_a) && (byte == gps_ubx_ck_b))
      {
        gps_ubx_count++;
        GPS_RecordValidFrame();
        printf("GPS UBX: class=0x%02X id=0x%02X payload=%u bytes checksum=OK\r\n",
               gps_ubx_class, gps_ubx_id, gps_ubx_length);
      }
      else
      {
        gps_frame_error_count++;
        gps_frame_error = 1U;
      }
      gps_frame_state = GPS_FRAME_WAIT;
      break;

    default:
      gps_frame_state = GPS_FRAME_WAIT;
      break;
  }
}

static void GPS_Init(void)
{
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
  memset(gps_spi_tx, 0xFF, sizeof(gps_spi_tx));

  /* The reset pin starts low in MX_GPIO_Init; allow the receiver to boot. */
  HAL_Delay(1000U);
  gps_monitor_start_ms = HAL_GetTick();
  gps_last_valid_frame_ms = gps_monitor_start_ms;

  printf("\r\nM9 GPS SPI/SWV test started\r\n");
  printf("Waiting up to %lu ms for a checksum-valid NMEA or UBX message...\r\n",
         (unsigned long)GPS_COMM_TIMEOUT_MS);
}

static void GPS_Task(void)
{
  static uint32_t last_poll_ms = 0U;
  uint32_t now = HAL_GetTick();

  if ((now - last_poll_ms) < GPS_POLL_PERIOD_MS)
  {
    return;
  }
  last_poll_ms = now;

  memset(gps_spi_rx, 0xFF, sizeof(gps_spi_rx));
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2,
                                                     gps_spi_tx,
                                                     gps_spi_rx,
                                                     GPS_SPI_READ_LEN,
                                                     100U);
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_SET);

  if (status != HAL_OK)
  {
    if (gps_spi_error == 0U)
    {
      printf("GPS SPI ERROR: HAL status=%d error=0x%08lX\r\n",
             (int)status, (unsigned long)HAL_SPI_GetError(&hspi2));
    }
    gps_spi_error = 1U;
    return;
  }

  for (uint16_t i = 0U; i < GPS_SPI_READ_LEN; i++)
  {
    GPS_ProcessByte(gps_spi_rx[i]);
  }
}

static void StatusLED_Task(void)
{
  static uint32_t last_heartbeat_ms = 0U;
  static uint32_t last_status_ms = 0U;
  uint32_t now = HAL_GetTick();
  uint8_t communication_ok =
      ((gps_nmea_count + gps_ubx_count) > 0U) &&
      ((now - gps_last_valid_frame_ms) <= GPS_COMM_TIMEOUT_MS);
  uint8_t communication_timed_out =
      ((now - gps_monitor_start_ms) > GPS_COMM_TIMEOUT_MS) &&
      ((now - gps_last_valid_frame_ms) > GPS_COMM_TIMEOUT_MS);

  /* A 500 ms toggle gives one complete on/off heartbeat each second. */
  if ((now - last_heartbeat_ms) >= HEARTBEAT_TOGGLE_MS)
  {
    last_heartbeat_ms = now;
    HAL_GPIO_TogglePin(Green_LED11_GPIO_Port, Green_LED11_Pin);
  }

  HAL_GPIO_WritePin(Yellow_LED8_GPIO_Port, Yellow_LED8_Pin,
                    communication_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RED_LED10_GPIO_Port, RED_LED10_Pin,
                    (gps_spi_error || gps_frame_error || communication_timed_out) ?
                    GPIO_PIN_SET : GPIO_PIN_RESET);

  if ((now - last_status_ms) >= GPS_STATUS_PERIOD_MS)
  {
    last_status_ms = now;
    printf("GPS status: NMEA=%lu UBX=%lu bytes=%lu frame_errors=%lu PPS=%s comm=%s\r\n",
           (unsigned long)gps_nmea_count,
           (unsigned long)gps_ubx_count,
           (unsigned long)gps_raw_byte_count,
           (unsigned long)gps_frame_error_count,
           (HAL_GPIO_ReadPin(GPS_PPS_GPIO_Port, GPS_PPS_Pin) == GPIO_PIN_SET) ? "HIGH" : "LOW",
           communication_ok ? "OK" : (communication_timed_out ? "TIMEOUT" : "WAITING"));
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  GPS_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    GPS_Task();
    StatusLED_Task();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_CS_GPIO_Port, GPS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_LED11_Pin|RED_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPS_CS_Pin GPS_RST_Pin */
  GPIO_InitStruct.Pin = GPS_CS_Pin|GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_INT_Pin */
  GPIO_InitStruct.Pin = GPS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_LED11_Pin RED_LED10_Pin Yellow_LED9_Pin Yellow_LED8_Pin
                           Yellow_LED7_Pin Yellow_LED6_Pin Yellow_LED5_Pin */
  GPIO_InitStruct.Pin = Green_LED11_Pin|RED_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_PPS_Pin */
  GPIO_InitStruct.Pin = GPS_PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_PPS_GPIO_Port, &GPIO_InitStruct);

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
  /* Make fatal initialization failures visible even if SWV is unavailable. */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = RED_LED10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED10_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(RED_LED10_GPIO_Port, RED_LED10_Pin, GPIO_PIN_SET);
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
