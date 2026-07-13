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
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  SD_TEST_PENDING = 0,
  SD_TEST_ERROR,
  SD_TEST_OK
} SD_TestStateTypeDef;

typedef struct
{
  uint32_t tick_ms;
  uint32_t rx_count;
  uint32_t id;
  uint8_t is_extended;
  uint8_t dlc;
  uint8_t data[8];
} CAN_RxSnapshotTypeDef;

typedef enum
{
  CAN_TX_EVENT_NONE = 0,
  CAN_TX_EVENT_COMPLETE,
  CAN_TX_EVENT_ABORT,
  CAN_TX_EVENT_ERROR
} CAN_TxEventTypeDef;

typedef struct
{
  volatile uint8_t pending;
  volatile uint32_t event_count;
  volatile CAN_TxEventTypeDef event;
  volatile uint32_t tick_ms;
  volatile uint32_t mailbox_index;
  volatile uint32_t tx_queued;
  volatile uint32_t tx_complete;
  volatile uint32_t tx_abort;
  volatile uint32_t error;
  volatile uint32_t esr;
  volatile uint32_t tsr;
  volatile uint32_t msr;
} CAN_TxEventSnapshotTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_PERIOD_MS      1000U
#define HEARTBEAT_ON_TIME_MS     100U
#define SD_TEST_RETRY_PERIOD_MS  2000U
#define ITM_PUTCHAR_TIMEOUT     1000000U
#define APP_DEBUG_PRINT_PERIOD_MS 1000U
#define SD_DETECT_ACTIVE_STATE   GPIO_PIN_RESET
#define CAN_SD_LOG_PERIOD_MS     1000U
#define CAN_DEBUG_PRINT_PERIOD_MS 1000U
#define CAN_RX_POLL_MAX_PER_LOOP 8U
#define CAN_RX_ISR_DRAIN_MAX     3U
#define SD_ACTIVITY_LED_ON_TIME_MS 100U
#define CAN_RX_LED_ON_TIME_MS    100U
#define CAN_LOOPBACK_MODE        0U
#define CAN_TX_ENABLE            1U
#define CAN_TX_PERIOD_MS         ((CAN_LOOPBACK_MODE != 0U) ? 1000U : 1000U)
#define CAN_TX_STD_ID            0x540U
#define CAN_TX_DLC               8U
#define CAN_DEVICE_SERIAL        0x20240786UL
#define CAN_TX_REQUIRE_RECENT_RX 0U
#define CAN_TX_RECENT_RX_MS      500U
#define CAN_TX_MAX_TEC           255U
#define CAN_TX_MAX_REC           127U
#define CAN_LISTEN_ONLY_MODE     0U
#define CAN_APB1_CLOCK_HZ        32000000UL
#define CAN_TIME_QUANTA          16UL
#define CAN_TARGET_BITRATE_HZ    250000UL
#define CAN_TARGET_PRESCALER     (CAN_APB1_CLOCK_HZ / (CAN_TIME_QUANTA * CAN_TARGET_BITRATE_HZ))
#define CAN_SYNC_JUMP_WIDTH      CAN_SJW_3TQ
#define CAN_TIME_SEG1            CAN_BS1_12TQ
#define CAN_TIME_SEG2            CAN_BS2_3TQ
#define CAN_SAMPLE_POINT_PERMIL  812UL
#define CAN_RX_SENSE_GPIO_Port   GPIOD
#define CAN_RX_SENSE_Pin         GPIO_PIN_0
#define LED_ACTIVE_STATE         GPIO_PIN_RESET
#define LED_INACTIVE_STATE       GPIO_PIN_SET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
static SD_TestStateTypeDef SdTestState = SD_TEST_PENDING;
static uint32_t SdLastTestTick;
static uint8_t SdTestAttempted;
static FRESULT SdLastFatFsResult = FR_OK;
static uint32_t SdLastActivityTick;
static uint32_t CanLastSdLogTick;
static uint32_t CanLastLoggedRxCount;
static uint32_t CanLastDebugPrintTick;
static uint32_t CanLastDebugPrintRxCount;
static uint8_t CanDebugPrintedOnce;
static uint32_t AppLastDebugPrintTick;
static uint32_t AppLoopCount;
static uint32_t AppLastDebugRxCount;
static uint8_t AppDebugPrintedOnce;
static uint32_t CanLastTxTick;
static uint32_t CanTxSequence;
static uint32_t CanTxQueuedCount;
static volatile uint32_t CanTxCompleteCount;
static volatile uint32_t CanTxAbortCount;
static volatile uint32_t CanTxLastCompleteTick;
static uint8_t CanReady;
static volatile uint8_t CanRxIrqPending;
static volatile uint32_t CanLastRxIrqTick;
static volatile uint32_t CanRxIrqDrainCount;
static volatile uint32_t CanRxFifoOverrunCount;
static volatile uint32_t CanRxReadFailCount;
static volatile uint32_t CanRxEmptyRaceCount;
static CAN_RxSnapshotTypeDef CanLatestRx;
static CAN_TxEventSnapshotTypeDef CanTxEvent;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void Heartbeat_Task(void);
static void App_DebugPrint_Task(void);
static HAL_StatusTypeDef CAN1_ListenAllStart(void);
static void CAN_PollRx_Task(void);
static uint32_t CAN_DrainRxFifo(uint8_t print_frames, uint32_t max_frames);
static void CAN_RecordRxFrame(const CAN_RxHeaderTypeDef *rx_header, const uint8_t *rx_data);
static void CAN_TxSimple_Task(void);
static void CAN_TxEventLog_Task(void);
static void CAN_RecordTxEvent(CAN_TxEventTypeDef event, uint32_t mailbox_index);
static void CAN_DebugPrint_Task(void);
static void CAN_PrintErrorFlags(uint32_t error_flags);
static const char *CAN_TxEventName(CAN_TxEventTypeDef event);
static const char *CAN_ModeName(uint32_t mode);
static const char *CAN_LecName(uint32_t lec);
static void CanStatusLed_Task(void);
static void CAN_SDLog_Task(void);
static HAL_StatusTypeDef CAN_QueueSimpleFrame(uint32_t sequence, uint32_t *tx_mailbox);
static void SdCardStatus_Task(void);
static FRESULT SdCard_RunCsvWriteTest(void);
static FRESULT SdCard_AppendCsvRow(const char *status);
static void FormatCanDataHex(char *output, size_t output_size, const uint8_t *data, uint8_t dlc);
static const char *SdTestStateName(SD_TestStateTypeDef state);
static const char *FatFsResultName(FRESULT result);
static uint8_t IsSdCardDetected(void);
static const char *GpioPinStateName(GPIO_PinState state);
static void SetLed(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_PinState state);
static void SetErrorLed(GPIO_PinState state);
static void SetSdStatusLed(GPIO_PinState state);
static void SetCanStatusLed(GPIO_PinState state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0UL) &&
      ((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&
      ((ITM->TER & 1UL) != 0UL))
  {
    uint32_t timeout = ITM_PUTCHAR_TIMEOUT;

    while (ITM->PORT[0U].u32 == 0UL)
    {
      if (--timeout == 0UL)
      {
        return ch;
      }
    }

    ITM->PORT[0U].u8 = (uint8_t)ch;
  }

  return ch;
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
  MX_FATFS_Init();
  MX_SPI3_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  __HAL_DBGMCU_FREEZE_CAN1();
  SetErrorLed(GPIO_PIN_RESET);
  SetSdStatusLed(GPIO_PIN_RESET);
  SetCanStatusLed(GPIO_PIN_RESET);
  printf("\r\n[BOOT] SD card test firmware started at %lu ms\r\n", HAL_GetTick());
  printf("[BOOT] HCLK=%lu Hz, PCLK1=%lu Hz, SPI3 prescaler enum=%lu, SD_DETECT=%s (%lu), inserted=%s\r\n",
         HAL_RCC_GetHCLKFreq(),
         HAL_RCC_GetPCLK1Freq(),
         hspi3.Init.BaudRatePrescaler,
         GpioPinStateName(HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin)),
         (uint32_t)HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin),
         IsSdCardDetected() ? "yes" : "no");
  printf("[BOOT] FatFs link retUSER=%u, USERPath=%s\r\n", retUSER, USERPath);
  if (CAN1_ListenAllStart() == HAL_OK)
  {
    CanReady = 1U;
    printf("[CAN] CAN1 listen-all ready, pins=PD0/RX PD1/TX, mode=%s, prescaler=%lu, tq=%lu, sample=%lu.%lu%%, bitrate=%lu bit/s, CAN_RX_PD0=%lu\r\n",
           CAN_ModeName(hcan1.Init.Mode),
           hcan1.Init.Prescaler,
           CAN_TIME_QUANTA,
           CAN_SAMPLE_POINT_PERMIL / 10UL,
           CAN_SAMPLE_POINT_PERMIL % 10UL,
           CAN_APB1_CLOCK_HZ / (hcan1.Init.Prescaler * CAN_TIME_QUANTA),
           (uint32_t)HAL_GPIO_ReadPin(CAN_RX_SENSE_GPIO_Port, CAN_RX_SENSE_Pin));
  }
  else
  {
    CanReady = 0U;
    SetErrorLed(GPIO_PIN_SET);
    printf("[CAN] CAN1 listen-all start failed, HAL error=0x%08lX\r\n",
           HAL_CAN_GetError(&hcan1));
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    AppLoopCount++;
    Heartbeat_Task();
    CAN_PollRx_Task();
    CAN_DebugPrint_Task();
    App_DebugPrint_Task();
    CAN_TxSimple_Task();
    CAN_TxEventLog_Task();
    SdCardStatus_Task();
    CanStatusLed_Task();
    CAN_SDLog_Task();
    HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_2);

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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_LED11_Pin|Red_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_Detect_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_LED11_Pin Red_LED10_Pin Yellow_LED9_Pin Yellow_LED8_Pin
                           Yellow_LED7_Pin Yellow_LED6_Pin Yellow_LED5_Pin */
  GPIO_InitStruct.Pin = Green_LED11_Pin|Red_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Heartbeat_Task(void)
{
  uint32_t phase = HAL_GetTick() % HEARTBEAT_PERIOD_MS;
  SetLed(Green_LED11_GPIO_Port,
         Green_LED11_Pin,
         (phase < HEARTBEAT_ON_TIME_MS) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_DebugPrint_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t rx_delta;
  uint32_t esr;
  uint32_t tec;
  uint32_t rec;
  uint32_t lec;

  if (AppDebugPrintedOnce != 0U &&
      (now - AppLastDebugPrintTick) < APP_DEBUG_PRINT_PERIOD_MS)
  {
    return;
  }

  AppDebugPrintedOnce = 1U;
  AppLastDebugPrintTick = now;
  rx_delta = CanLatestRx.rx_count - AppLastDebugRxCount;
  esr = hcan1.Instance->ESR;
  tec = (esr >> 16) & 0xFFU;
  rec = (esr >> 24) & 0xFFU;
  lec = (esr >> 4) & 0x07U;

  printf("[APP] DBG t=%lu loop=%lu sd=%s fatfs=%s can_ready=%u can_state=%lu mode=%s CAN_RX_PD0=%lu rx=%lu rx_delta=%lu irq=%u irq_drain=%lu fifo_ovr=%lu read_fail=%lu empty_race=%lu RF0R=0x%08lX TEC=%lu REC=%lu LEC=%lu err=0x%08lX\r\n",
         now,
         AppLoopCount,
         SdTestStateName(SdTestState),
         FatFsResultName(SdLastFatFsResult),
         CanReady,
         (uint32_t)HAL_CAN_GetState(&hcan1),
         CAN_ModeName(hcan1.Init.Mode),
         (uint32_t)HAL_GPIO_ReadPin(CAN_RX_SENSE_GPIO_Port, CAN_RX_SENSE_Pin),
         CanLatestRx.rx_count,
         rx_delta,
         CanRxIrqPending,
         CanRxIrqDrainCount,
         CanRxFifoOverrunCount,
         CanRxReadFailCount,
         CanRxEmptyRaceCount,
         hcan1.Instance->RF0R,
         tec,
         rec,
         lec,
         HAL_CAN_GetError(&hcan1));

  AppLastDebugRxCount = CanLatestRx.rx_count;
}

static HAL_StatusTypeDef CAN1_ListenAllStart(void)
{
  CAN_FilterTypeDef filter_config = {0};
  HAL_StatusTypeDef status;

  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterIdHigh = 0x0000U;
  filter_config.FilterIdLow = 0x0000U;
  filter_config.FilterMaskIdHigh = 0x0000U;
  filter_config.FilterMaskIdLow = 0x0000U;
  filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter_config.FilterActivation = ENABLE;
  filter_config.SlaveStartFilterBank = 14;

  status = HAL_CAN_ConfigFilter(&hcan1, &filter_config);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_CAN_Start(&hcan1);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_CAN_ActivateNotification(&hcan1,
                                      CAN_IT_TX_MAILBOX_EMPTY |
                                      CAN_IT_RX_FIFO0_MSG_PENDING |
                                      CAN_IT_RX_FIFO0_FULL |
                                      CAN_IT_RX_FIFO0_OVERRUN |
                                      CAN_IT_ERROR_WARNING |
                                      CAN_IT_ERROR_PASSIVE |
                                      CAN_IT_BUSOFF |
                                      CAN_IT_LAST_ERROR_CODE |
                                      CAN_IT_ERROR);
}

static void CAN_PollRx_Task(void)
{
  if (CanReady == 0U)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  (void)CAN_DrainRxFifo(1U, CAN_RX_POLL_MAX_PER_LOOP);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

static uint32_t CAN_DrainRxFifo(uint8_t print_frames, uint32_t max_frames)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint32_t frames_read = 0U;

  if ((__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV0) != RESET))
  {
    CanRxFifoOverrunCount++;
    __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
  }

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U &&
         frames_read < max_frames)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      uint32_t can_error = HAL_CAN_GetError(&hcan1);

      if ((can_error & HAL_CAN_ERROR_PARAM) != 0U &&
          HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U)
      {
        CanRxEmptyRaceCount++;
        if ((can_error & ~HAL_CAN_ERROR_PARAM) == 0U)
        {
          (void)HAL_CAN_ResetError(&hcan1);
        }
        break;
      }

      CanRxReadFailCount++;
      if (print_frames != 0U)
      {
        printf("[CAN] RX read failed, HAL error=0x%08lX, ESR=0x%08lX\r\n",
               can_error,
               hcan1.Instance->ESR);
      }
      break;
    }

    frames_read++;
    CAN_RecordRxFrame(&rx_header, rx_data);

    if (print_frames != 0U &&
        (CanLatestRx.rx_count <= 20U || (CanLatestRx.rx_count % 50U) == 0U))
    {
      char data_hex[17];
      FormatCanDataHex(data_hex, sizeof(data_hex), CanLatestRx.data, CanLatestRx.dlc);
      printf("[CAN] RX #%lu ID=0x%lX %s DLC=%u DATA=%s\r\n",
             CanLatestRx.rx_count,
             CanLatestRx.id,
             (CanLatestRx.is_extended != 0U) ? "EXT" : "STD",
             CanLatestRx.dlc,
             data_hex);
    }
  }

  if ((__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV0) != RESET))
  {
    CanRxFifoOverrunCount++;
    __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
  }

  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U)
  {
    CanRxIrqPending = 0U;
  }

  return frames_read;
}

static void CAN_RecordRxFrame(const CAN_RxHeaderTypeDef *rx_header, const uint8_t *rx_data)
{
  CanLatestRx.tick_ms = HAL_GetTick();
  CanLatestRx.rx_count++;
  CanLatestRx.is_extended = (rx_header->IDE == CAN_ID_EXT) ? 1U : 0U;
  CanLatestRx.id = (CanLatestRx.is_extended != 0U) ? rx_header->ExtId : rx_header->StdId;
  CanLatestRx.dlc = (rx_header->DLC <= 8U) ? (uint8_t)rx_header->DLC : 8U;
  memset(CanLatestRx.data, 0, sizeof(CanLatestRx.data));
  memcpy(CanLatestRx.data, rx_data, CanLatestRx.dlc);
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CanRxIrqPending = 1U;
    CanLastRxIrqTick = HAL_GetTick();
    CanRxIrqDrainCount += CAN_DrainRxFifo(0U, CAN_RX_ISR_DRAIN_MAX);
  }
}

static void CAN_TxSimple_Task(void)
{
#if CAN_TX_ENABLE
  uint32_t now = HAL_GetTick();
  uint32_t tx_mailbox;
  uint32_t esr;
  uint32_t tec;
  uint32_t rec;
  HAL_StatusTypeDef status;

  if (CanReady == 0U)
  {
    return;
  }

  if ((now - CanLastTxTick) < CAN_TX_PERIOD_MS)
  {
    return;
  }

  CanLastTxTick = now;
  esr = hcan1.Instance->ESR;
  tec = (esr >> 16) & 0xFFU;
  rec = (esr >> 24) & 0xFFU;

  if (tec > CAN_TX_MAX_TEC || rec > CAN_TX_MAX_REC)
  {
    printf("[CAN] TX 0x%03lX held: bus counters TEC=%lu REC=%lu err=0x%08lX ESR=0x%08lX\r\n",
           (uint32_t)CAN_TX_STD_ID,
           tec,
           rec,
           HAL_CAN_GetError(&hcan1),
           esr);
    return;
  }

#if CAN_TX_REQUIRE_RECENT_RX && (CAN_LOOPBACK_MODE == 0U)
  if (CanLatestRx.rx_count == 0U || (now - CanLatestRx.tick_ms) > CAN_TX_RECENT_RX_MS)
  {
    printf("[CAN] TX 0x%03lX held: no recent RX traffic rx=%lu age=%lu ms\r\n",
           (uint32_t)CAN_TX_STD_ID,
           CanLatestRx.rx_count,
           (CanLatestRx.rx_count == 0U) ? 0UL : (now - CanLatestRx.tick_ms));
    return;
  }
#endif

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
  {
    printf("[CAN] TX 0x%03lX skipped: no free mailbox TSR=0x%08lX err=0x%08lX\r\n",
           (uint32_t)CAN_TX_STD_ID,
           hcan1.Instance->TSR,
           HAL_CAN_GetError(&hcan1));
    return;
  }

  status = CAN_QueueSimpleFrame(CanTxSequence, &tx_mailbox);
  if (status == HAL_OK)
  {
    CanTxQueuedCount++;
    printf("[CAN] TX QUEUE t=%lu q=%lu ID=0x%03lX STD DATA DLC=%lu payload=8607242032764D54 serial=0x%08lX seq=%lu mailbox=0x%08lX free=%lu ESR=0x%08lX TSR=0x%08lX\r\n",
           now,
           CanTxQueuedCount,
           (uint32_t)CAN_TX_STD_ID,
           (uint32_t)CAN_TX_DLC,
           CAN_DEVICE_SERIAL,
           CanTxSequence,
           tx_mailbox,
           HAL_CAN_GetTxMailboxesFreeLevel(&hcan1),
           hcan1.Instance->ESR,
           hcan1.Instance->TSR);
    CanTxSequence++;
  }
  else
  {
    printf("[CAN] TX queue failed ID=0x%03lX status=%d err=0x%08lX ESR=0x%08lX TSR=0x%08lX\r\n",
           (uint32_t)CAN_TX_STD_ID,
           (int)status,
           HAL_CAN_GetError(&hcan1),
           hcan1.Instance->ESR,
           hcan1.Instance->TSR);
  }
#endif
}

static void CAN_TxEventLog_Task(void)
{
  CAN_TxEventTypeDef event;
  uint32_t event_count;
  uint32_t tick_ms;
  uint32_t mailbox_index;
  uint32_t tx_queued;
  uint32_t tx_complete;
  uint32_t tx_abort;
  uint32_t error;
  uint32_t esr;
  uint32_t tsr;
  uint32_t msr;
  uint32_t tec;
  uint32_t rec;
  uint32_t lec;

  if (CanTxEvent.pending == 0U)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
  HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  event = CanTxEvent.event;
  event_count = CanTxEvent.event_count;
  tick_ms = CanTxEvent.tick_ms;
  mailbox_index = CanTxEvent.mailbox_index;
  tx_queued = CanTxEvent.tx_queued;
  tx_complete = CanTxEvent.tx_complete;
  tx_abort = CanTxEvent.tx_abort;
  error = CanTxEvent.error;
  esr = CanTxEvent.esr;
  tsr = CanTxEvent.tsr;
  msr = CanTxEvent.msr;
  CanTxEvent.pending = 0U;
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);

  tec = (esr >> 16U) & 0xFFU;
  rec = (esr >> 24U) & 0xFFU;
  lec = (esr >> 4U) & 0x07U;

  printf("[CAN] TX EVENT #%lu t=%lu event=%s mb=%lu ID=0x%03lX q=%lu done=%lu abort=%lu err=0x%08lX ESR=0x%08lX TEC=%lu REC=%lu LEC=%lu(%s) TSR=0x%08lX MSR=0x%08lX\r\n",
         event_count,
         tick_ms,
         CAN_TxEventName(event),
         mailbox_index,
         (uint32_t)CAN_TX_STD_ID,
         tx_queued,
         tx_complete,
         tx_abort,
         error,
         esr,
         tec,
         rec,
         lec,
         CAN_LecName(lec),
         tsr,
         msr);

  if (error != HAL_CAN_ERROR_NONE)
  {
    CAN_PrintErrorFlags(error);
  }
}

static void CAN_RecordTxEvent(CAN_TxEventTypeDef event, uint32_t mailbox_index)
{
  CanTxEvent.event = event;
  CanTxEvent.tick_ms = HAL_GetTick();
  CanTxEvent.mailbox_index = mailbox_index;
  CanTxEvent.tx_queued = CanTxQueuedCount;
  CanTxEvent.tx_complete = CanTxCompleteCount;
  CanTxEvent.tx_abort = CanTxAbortCount;
  CanTxEvent.error = HAL_CAN_GetError(&hcan1);
  CanTxEvent.esr = hcan1.Instance->ESR;
  CanTxEvent.tsr = hcan1.Instance->TSR;
  CanTxEvent.msr = hcan1.Instance->MSR;
  CanTxEvent.event_count++;
  CanTxEvent.pending = 1U;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CanRxIrqPending = 1U;
    CanLastRxIrqTick = HAL_GetTick();
    CanRxIrqDrainCount += CAN_DrainRxFifo(0U, CAN_RX_ISR_DRAIN_MAX);
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CanTxCompleteCount++;
    CanTxLastCompleteTick = HAL_GetTick();
    CAN_RecordTxEvent(CAN_TX_EVENT_COMPLETE, 0U);
  }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CanTxCompleteCount++;
    CanTxLastCompleteTick = HAL_GetTick();
    CAN_RecordTxEvent(CAN_TX_EVENT_COMPLETE, 1U);
  }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CanTxCompleteCount++;
    CanTxLastCompleteTick = HAL_GetTick();
    CAN_RecordTxEvent(CAN_TX_EVENT_COMPLETE, 2U);
  }
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CanTxAbortCount++;
    CAN_RecordTxEvent(CAN_TX_EVENT_ABORT, 0U);
  }
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CanTxAbortCount++;
    CAN_RecordTxEvent(CAN_TX_EVENT_ABORT, 1U);
  }
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CanTxAbortCount++;
    CAN_RecordTxEvent(CAN_TX_EVENT_ABORT, 2U);
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN_RecordTxEvent(CAN_TX_EVENT_ERROR, 0xFFFFFFFFUL);
    CanRxIrqPending = 0U;
  }
}

static void CAN_DebugPrint_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t esr;
  uint32_t can_error;
  uint32_t rx_delta;
  uint32_t fifo0_level;
  uint32_t tec;
  uint32_t rec;
  uint32_t lec;
  uint32_t rx_pin;
  uint32_t tx_free;

  if (CanDebugPrintedOnce != 0U &&
      (now - CanLastDebugPrintTick) < CAN_DEBUG_PRINT_PERIOD_MS)
  {
    return;
  }

  CanDebugPrintedOnce = 1U;
  CanLastDebugPrintTick = now;
  esr = hcan1.Instance->ESR;
  can_error = HAL_CAN_GetError(&hcan1);
  rx_delta = CanLatestRx.rx_count - CanLastDebugPrintRxCount;
  fifo0_level = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
  tec = (esr >> 16) & 0xFFU;
  rec = (esr >> 24) & 0xFFU;
  lec = (esr >> 4) & 0x07U;
  rx_pin = (uint32_t)HAL_GPIO_ReadPin(CAN_RX_SENSE_GPIO_Port, CAN_RX_SENSE_Pin);
  tx_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

  printf("[CAN] DBG t=%lu ready=%u state=%lu mode=%s bitrate=%lu rx=%lu delta=%lu fifo0=%lu irq=%u txq=%lu txdone=%lu txabort=%lu txfree=%lu txdone_tick=%lu CAN_RX_PD0=%lu err=0x%08lX ESR=0x%08lX TEC=%lu REC=%lu LEC=%lu MSR=0x%08lX TSR=0x%08lX RF0R=0x%08lX IER=0x%08lX BTR=0x%08lX\r\n",
         now,
         CanReady,
         (uint32_t)HAL_CAN_GetState(&hcan1),
         CAN_ModeName(hcan1.Init.Mode),
         CAN_APB1_CLOCK_HZ / (hcan1.Init.Prescaler * CAN_TIME_QUANTA),
         CanLatestRx.rx_count,
         rx_delta,
         fifo0_level,
         CanRxIrqPending,
         CanTxQueuedCount,
         CanTxCompleteCount,
         CanTxAbortCount,
         tx_free,
         CanTxLastCompleteTick,
         rx_pin,
         can_error,
         esr,
         tec,
         rec,
         lec,
         hcan1.Instance->MSR,
         hcan1.Instance->TSR,
         hcan1.Instance->RF0R,
         hcan1.Instance->IER,
         hcan1.Instance->BTR);

  if (rx_delta != 0U)
  {
    char data_hex[17];
    FormatCanDataHex(data_hex, sizeof(data_hex), CanLatestRx.data, CanLatestRx.dlc);
    printf("[CAN] DBG latest tick=%lu ID=0x%lX %s DLC=%u DATA=%s\r\n",
           CanLatestRx.tick_ms,
           CanLatestRx.id,
           (CanLatestRx.is_extended != 0U) ? "EXT" : "STD",
           CanLatestRx.dlc,
           data_hex);
  }

  if (can_error != HAL_CAN_ERROR_NONE)
  {
    CAN_PrintErrorFlags(can_error);
  }

  CanLastDebugPrintRxCount = CanLatestRx.rx_count;
}

static void CAN_PrintErrorFlags(uint32_t error_flags)
{
  uint32_t known_flags = HAL_CAN_ERROR_EWG |
                         HAL_CAN_ERROR_EPV |
                         HAL_CAN_ERROR_BOF |
                         HAL_CAN_ERROR_STF |
                         HAL_CAN_ERROR_FOR |
                         HAL_CAN_ERROR_ACK |
                         HAL_CAN_ERROR_BR |
                         HAL_CAN_ERROR_BD |
                         HAL_CAN_ERROR_CRC |
                         HAL_CAN_ERROR_RX_FOV0 |
                         HAL_CAN_ERROR_RX_FOV1 |
                         HAL_CAN_ERROR_TX_ALST0 |
                         HAL_CAN_ERROR_TX_TERR0 |
                         HAL_CAN_ERROR_TX_ALST1 |
                         HAL_CAN_ERROR_TX_TERR1 |
                         HAL_CAN_ERROR_TX_ALST2 |
                         HAL_CAN_ERROR_TX_TERR2 |
                         HAL_CAN_ERROR_TIMEOUT |
                         HAL_CAN_ERROR_NOT_INITIALIZED |
                         HAL_CAN_ERROR_NOT_READY |
                         HAL_CAN_ERROR_NOT_STARTED |
                         HAL_CAN_ERROR_PARAM |
                         HAL_CAN_ERROR_INTERNAL;
#ifdef HAL_CAN_ERROR_INVALID_CALLBACK
  known_flags |= HAL_CAN_ERROR_INVALID_CALLBACK;
#endif

  printf("[CAN] ERR flags:%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s UNKNOWN=0x%08lX\r\n",
         (error_flags & HAL_CAN_ERROR_EWG) ? " EWG" : "",
         (error_flags & HAL_CAN_ERROR_EPV) ? " EPV" : "",
         (error_flags & HAL_CAN_ERROR_BOF) ? " BOF" : "",
         (error_flags & HAL_CAN_ERROR_STF) ? " STF" : "",
         (error_flags & HAL_CAN_ERROR_FOR) ? " FOR" : "",
         (error_flags & HAL_CAN_ERROR_ACK) ? " ACK" : "",
         (error_flags & HAL_CAN_ERROR_BR) ? " BIT_RECESSIVE" : "",
         (error_flags & HAL_CAN_ERROR_BD) ? " BIT_DOMINANT" : "",
         (error_flags & HAL_CAN_ERROR_CRC) ? " CRC" : "",
         (error_flags & HAL_CAN_ERROR_RX_FOV0) ? " RX_FOV0" : "",
         (error_flags & HAL_CAN_ERROR_RX_FOV1) ? " RX_FOV1" : "",
         (error_flags & HAL_CAN_ERROR_TX_ALST0) ? " TX_ALST0" : "",
         (error_flags & HAL_CAN_ERROR_TX_TERR0) ? " TX_TERR0" : "",
         (error_flags & HAL_CAN_ERROR_TX_ALST1) ? " TX_ALST1" : "",
         (error_flags & HAL_CAN_ERROR_TX_TERR1) ? " TX_TERR1" : "",
         (error_flags & HAL_CAN_ERROR_TX_ALST2) ? " TX_ALST2" : "",
         (error_flags & HAL_CAN_ERROR_TX_TERR2) ? " TX_TERR2" : "",
         (error_flags & HAL_CAN_ERROR_TIMEOUT) ? " TIMEOUT" : "",
         (error_flags & HAL_CAN_ERROR_NOT_INITIALIZED) ? " NOT_INITIALIZED" : "",
         (error_flags & HAL_CAN_ERROR_NOT_READY) ? " NOT_READY" : "",
         (error_flags & HAL_CAN_ERROR_NOT_STARTED) ? " NOT_STARTED" : "",
         (error_flags & HAL_CAN_ERROR_PARAM) ? " PARAM" : "",
#ifdef HAL_CAN_ERROR_INVALID_CALLBACK
         (error_flags & HAL_CAN_ERROR_INVALID_CALLBACK) ? " INVALID_CALLBACK" : "",
#else
         "",
#endif
         (error_flags & HAL_CAN_ERROR_INTERNAL) ? " INTERNAL" : "",
         (error_flags == HAL_CAN_ERROR_NONE) ? " NONE" : "",
          error_flags & ~known_flags);
}

static const char *CAN_TxEventName(CAN_TxEventTypeDef event)
{
  switch (event)
  {
    case CAN_TX_EVENT_COMPLETE:
      return "complete";
    case CAN_TX_EVENT_ABORT:
      return "abort";
    case CAN_TX_EVENT_ERROR:
      return "error";
    case CAN_TX_EVENT_NONE:
    default:
      return "none";
  }
}

static const char *CAN_ModeName(uint32_t mode)
{
  switch (mode)
  {
    case CAN_MODE_NORMAL:
      return "normal";
    case CAN_MODE_LOOPBACK:
      return "loopback";
    case CAN_MODE_SILENT:
      return "silent";
    case CAN_MODE_SILENT_LOOPBACK:
      return "silent_loopback";
    default:
      return "unknown";
  }
}

static const char *CAN_LecName(uint32_t lec)
{
  switch (lec)
  {
    case 0U:
      return "none";
    case 1U:
      return "stuff";
    case 2U:
      return "form";
    case 3U:
      return "ack";
    case 4U:
      return "bit_recessive";
    case 5U:
      return "bit_dominant";
    case 6U:
      return "crc";
    case 7U:
      return "software_set";
    default:
      return "unknown";
  }
}

static void CanStatusLed_Task(void)
{
  uint32_t now = HAL_GetTick();

  if (CanReady == 0U)
  {
    SetCanStatusLed(((now / 250U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return;
  }

  if (CanLatestRx.rx_count == 0U)
  {
    SetCanStatusLed((now % 1000U) < 100U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return;
  }

  if ((now - CanLatestRx.tick_ms) < CAN_RX_LED_ON_TIME_MS ||
      (now - CanLastRxIrqTick) < CAN_RX_LED_ON_TIME_MS ||
      CanRxIrqPending != 0U)
  {
    SetCanStatusLed(GPIO_PIN_SET);
  }
  else
  {
    SetCanStatusLed(GPIO_PIN_RESET);
  }
}

static HAL_StatusTypeDef CAN_QueueSimpleFrame(uint32_t sequence, uint32_t *tx_mailbox)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8];

  (void)sequence;

  tx_data[0] = (uint8_t)CAN_DEVICE_SERIAL;
  tx_data[1] = (uint8_t)(CAN_DEVICE_SERIAL >> 8);
  tx_data[2] = (uint8_t)(CAN_DEVICE_SERIAL >> 16);
  tx_data[3] = (uint8_t)(CAN_DEVICE_SERIAL >> 24);
  tx_data[4] = '2';
  tx_data[5] = 'v';
  tx_data[6] = 'M';
  tx_data[7] = 'T';

  tx_header.StdId = CAN_TX_STD_ID;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = CAN_TX_DLC;
  tx_header.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, tx_mailbox);
}

static void SdCardStatus_Task(void)
{
  uint32_t now = HAL_GetTick();

  if (SdTestState != SD_TEST_OK &&
      (SdTestAttempted == 0U || (now - SdLastTestTick) >= SD_TEST_RETRY_PERIOD_MS))
  {
    SdTestAttempted = 1U;
    SdLastTestTick = now;

    if (IsSdCardDetected() == 0U)
    {
      SdTestState = SD_TEST_PENDING;
      SetErrorLed(GPIO_PIN_RESET);
      SetSdStatusLed(((now / 250U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      printf("[APP] Waiting for SD card, SD_DETECT=%s (%lu)\r\n",
             GpioPinStateName(HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin)),
             (uint32_t)HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin));
      return;
    }

    printf("[APP] SD CSV test attempt at %lu ms\r\n", now);
    SdLastFatFsResult = SdCard_RunCsvWriteTest();

    if (SdLastFatFsResult == FR_OK)
    {
      SdTestState = SD_TEST_OK;
      SetErrorLed(GPIO_PIN_RESET);
      printf("[APP] SD CSV test passed\r\n");
    }
    else
    {
      SdTestState = SD_TEST_ERROR;
      SetErrorLed(GPIO_PIN_SET);
      printf("[APP] SD CSV test failed: %s (%d)\r\n",
             FatFsResultName(SdLastFatFsResult),
             (int)SdLastFatFsResult);
    }
  }

  if (SdTestState == SD_TEST_PENDING)
  {
    SetSdStatusLed(((now / 250U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if (SdTestState == SD_TEST_ERROR)
  {
    SetSdStatusLed(((now / 500U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if (SdTestState == SD_TEST_OK)
  {
    SetSdStatusLed((now - SdLastActivityTick) < SD_ACTIVITY_LED_ON_TIME_MS ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

static void CAN_SDLog_Task(void)
{
  uint32_t now = HAL_GetTick();
  FRESULT result;

  if (SdTestState != SD_TEST_OK)
  {
    return;
  }

  if ((now - CanLastSdLogTick) < CAN_SD_LOG_PERIOD_MS)
  {
    return;
  }

  CanLastSdLogTick = now;
  result = SdCard_AppendCsvRow("can_snapshot");
  if (result == FR_OK)
  {
    CanLastLoggedRxCount = CanLatestRx.rx_count;
  }
  else
  {
    SdTestState = SD_TEST_ERROR;
    SdLastFatFsResult = result;
    SetErrorLed(GPIO_PIN_SET);
    printf("[APP] CAN SD log append failed: %s (%d)\r\n",
           FatFsResultName(result),
           (int)result);
  }
}

static FRESULT SdCard_RunCsvWriteTest(void)
{
  FRESULT result = SdCard_AppendCsvRow("sd_write_ok");

  if (result == FR_OK)
  {
    CanLastLoggedRxCount = CanLatestRx.rx_count;
  }

  return result;
}

static FRESULT SdCard_AppendCsvRow(const char *status)
{
  static const char log_path[] = "0:/CANLOG.CSV";
  static const char csv_header[] =
      "tick_ms,status,can_rx_count,can_rx_delta,can_id,can_ext,can_dlc,can_data,can_last_rx_tick_ms,can_hal_state,can_hal_error,can_esr\r\n";

  char data_hex[17];
  FRESULT result;
  UINT bytes_written;
  int chars_written;
  uint32_t can_delta = CanLatestRx.rx_count - CanLastLoggedRxCount;

  if (retUSER != 0U)
  {
    printf("[FATFS] Driver link failed, retUSER=%u\r\n", retUSER);
    return FR_NOT_READY;
  }

  printf("[FATFS] Mounting %s, SD_DETECT=%s (%lu)\r\n",
         USERPath,
         GpioPinStateName(HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin)),
         (uint32_t)HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin));
  result = f_mount(&USERFatFS, USERPath, 1);
  if (result != FR_OK)
  {
    printf("[FATFS] f_mount failed: %s (%d)\r\n",
           FatFsResultName(result),
           (int)result);
    return result;
  }
  printf("[FATFS] f_mount OK, fs_type=%u, csize=%lu sectors/cluster\r\n",
         USERFatFS.fs_type,
         (uint32_t)USERFatFS.csize);

  printf("[FATFS] Opening %s\r\n", log_path);
  result = f_open(&USERFile, log_path, FA_OPEN_ALWAYS | FA_WRITE);
  if (result != FR_OK)
  {
    printf("[FATFS] f_open failed: %s (%d)\r\n",
           FatFsResultName(result),
           (int)result);
    return result;
  }
  printf("[FATFS] f_open OK, existing size=%lu bytes\r\n", (uint32_t)f_size(&USERFile));

  if (f_size(&USERFile) == 0U)
  {
    printf("[FATFS] Writing CSV header\r\n");
    result = f_write(&USERFile, csv_header, sizeof(csv_header) - 1U, &bytes_written);
    if (result != FR_OK || bytes_written != (sizeof(csv_header) - 1U))
    {
      (void)f_close(&USERFile);
      printf("[FATFS] Header write failed: %s (%d), bytes=%u\r\n",
             FatFsResultName(result),
             (int)result,
             bytes_written);
      return (result == FR_OK) ? FR_DISK_ERR : result;
    }
    printf("[FATFS] Header write OK, bytes=%u\r\n", bytes_written);
  }

  printf("[FATFS] Seeking to end\r\n");
  result = f_lseek(&USERFile, f_size(&USERFile));
  if (result == FR_OK)
  {
    FormatCanDataHex(data_hex, sizeof(data_hex), CanLatestRx.data, CanLatestRx.dlc);
    printf("[FATFS] Appending CSV row\r\n");
    chars_written = f_printf(&USERFile,
                             "%lu,%s,%lu,%lu,0x%lX,%u,%u,%s,%lu,%lu,0x%08lX,0x%08lX\r\n",
                             HAL_GetTick(),
                             status,
                             CanLatestRx.rx_count,
                             can_delta,
                             CanLatestRx.id,
                             CanLatestRx.is_extended,
                             CanLatestRx.dlc,
                             data_hex,
                             CanLatestRx.tick_ms,
                             (uint32_t)HAL_CAN_GetState(&hcan1),
                             HAL_CAN_GetError(&hcan1),
                             hcan1.Instance->ESR);
    if (chars_written == EOF)
    {
      result = FR_DISK_ERR;
    }
    printf("[FATFS] CSV row write result chars=%d\r\n", chars_written);
  }
  else
  {
    printf("[FATFS] f_lseek failed: %s (%d)\r\n",
           FatFsResultName(result),
           (int)result);
  }

  if (result == FR_OK)
  {
    printf("[FATFS] Syncing file\r\n");
    result = f_sync(&USERFile);
    if (result != FR_OK)
    {
      printf("[FATFS] f_sync failed: %s (%d)\r\n",
             FatFsResultName(result),
             (int)result);
    }
  }

  if (f_close(&USERFile) != FR_OK && result == FR_OK)
  {
    result = FR_DISK_ERR;
    printf("[FATFS] f_close failed\r\n");
  }

  if (result == FR_OK)
  {
    SdLastActivityTick = HAL_GetTick();
    printf("[FATFS] CSV write complete\r\n");
  }

  return result;
}

static void FormatCanDataHex(char *output, size_t output_size, const uint8_t *data, uint8_t dlc)
{
  static const char hex_digits[] = "0123456789ABCDEF";
  size_t out_index = 0U;

  if (output_size == 0U)
  {
    return;
  }

  if (dlc > 8U)
  {
    dlc = 8U;
  }

  for (uint8_t i = 0U; i < dlc && (out_index + 2U) < output_size; i++)
  {
    output[out_index++] = hex_digits[(data[i] >> 4) & 0x0FU];
    output[out_index++] = hex_digits[data[i] & 0x0FU];
  }

  output[out_index] = '\0';
}

static const char *SdTestStateName(SD_TestStateTypeDef state)
{
  switch (state)
  {
    case SD_TEST_PENDING:
      return "pending";
    case SD_TEST_ERROR:
      return "error";
    case SD_TEST_OK:
      return "ok";
    default:
      return "unknown";
  }
}

static const char *FatFsResultName(FRESULT result)
{
  switch (result)
  {
    case FR_OK:
      return "FR_OK";
    case FR_DISK_ERR:
      return "FR_DISK_ERR";
    case FR_INT_ERR:
      return "FR_INT_ERR";
    case FR_NOT_READY:
      return "FR_NOT_READY";
    case FR_NO_FILE:
      return "FR_NO_FILE";
    case FR_NO_PATH:
      return "FR_NO_PATH";
    case FR_INVALID_NAME:
      return "FR_INVALID_NAME";
    case FR_DENIED:
      return "FR_DENIED";
    case FR_EXIST:
      return "FR_EXIST";
    case FR_INVALID_OBJECT:
      return "FR_INVALID_OBJECT";
    case FR_WRITE_PROTECTED:
      return "FR_WRITE_PROTECTED";
    case FR_INVALID_DRIVE:
      return "FR_INVALID_DRIVE";
    case FR_NOT_ENABLED:
      return "FR_NOT_ENABLED";
    case FR_NO_FILESYSTEM:
      return "FR_NO_FILESYSTEM";
    case FR_MKFS_ABORTED:
      return "FR_MKFS_ABORTED";
    case FR_TIMEOUT:
      return "FR_TIMEOUT";
    case FR_LOCKED:
      return "FR_LOCKED";
    case FR_NOT_ENOUGH_CORE:
      return "FR_NOT_ENOUGH_CORE";
    case FR_TOO_MANY_OPEN_FILES:
      return "FR_TOO_MANY_OPEN_FILES";
    case FR_INVALID_PARAMETER:
      return "FR_INVALID_PARAMETER";
    default:
      return "FR_UNKNOWN";
  }
}

static uint8_t IsSdCardDetected(void)
{
  return (HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin) == SD_DETECT_ACTIVE_STATE) ? 1U : 0U;
}

static const char *GpioPinStateName(GPIO_PinState state)
{
  return (state == GPIO_PIN_RESET) ? "LOW" : "HIGH";
}

static void SetLed(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_PinState state)
{
  HAL_GPIO_WritePin(gpio_port,
                    gpio_pin,
                    (state == GPIO_PIN_SET) ? LED_ACTIVE_STATE : LED_INACTIVE_STATE);
}

static void SetErrorLed(GPIO_PinState state)
{
  SetLed(Red_LED10_GPIO_Port, Red_LED10_Pin, state);
}

static void SetSdStatusLed(GPIO_PinState state)
{
  SetLed(Yellow_LED5_GPIO_Port, Yellow_LED5_Pin, state);
}

static void SetCanStatusLed(GPIO_PinState state)
{
  SetLed(Yellow_LED6_GPIO_Port, Yellow_LED6_Pin, state);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  SetErrorLed(GPIO_PIN_SET);
  SetLed(Green_LED11_GPIO_Port, Green_LED11_Pin, GPIO_PIN_RESET);
  SetSdStatusLed(GPIO_PIN_RESET);
  SetCanStatusLed(GPIO_PIN_RESET);
  printf("[ERROR] Error_Handler entered at %lu ms\r\n", HAL_GetTick());
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
