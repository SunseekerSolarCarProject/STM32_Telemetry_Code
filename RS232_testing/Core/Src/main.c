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
typedef struct
{
  uint32_t can_id;
  const char *name;
  volatile uint32_t high_word;
  volatile uint32_t low_word;
  volatile uint8_t valid;
} SunRawEntry_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SUN_RAW_SEND_PERIOD_MS       1000U
#define SUN_RAW_BLOCK_LEN            4096U
#define RS232_TX_TIMEOUT_MS          1000U
#define STATUS_LED_PULSE_MS          100U
#define HEARTBEAT_PERIOD_MS          1000U
#define HEARTBEAT_ON_TIME_MS         100U
#define SWV_PUTCHAR_TIMEOUT          10000U
#define SWV_ECHO_RS232_BLOCK         1U

/* The LEDs on this PCB are active-low. */
#define LED_ACTIVE_STATE             GPIO_PIN_RESET
#define LED_INACTIVE_STATE           GPIO_PIN_SET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/*
 * Fixed legacy Sunseeker row order copied from Telemetry_Reference.
 * A row remains as 0xHHHHHHHH until an 8-byte frame with its CAN ID arrives.
 */
static SunRawEntry_t sun_raw_table[] =
{
  { 0x402U, "MC1BUS", 0U, 0U, 0U },
  { 0x403U, "MC1VEL", 0U, 0U, 0U },
  { 0x40BU, "MC1TP1", 0U, 0U, 0U },
  { 0x40CU, "MC1TP2", 0U, 0U, 0U },
  { 0x404U, "MC1PHA", 0U, 0U, 0U },
  { 0x40EU, "MC1CUM", 0U, 0U, 0U },
  { 0x405U, "MC1VVC", 0U, 0U, 0U },
  { 0x406U, "MC1IVC", 0U, 0U, 0U },
  { 0x407U, "MC1BEM", 0U, 0U, 0U },

  { 0x422U, "MC2BUS", 0U, 0U, 0U },
  { 0x423U, "MC2VEL", 0U, 0U, 0U },
  { 0x42BU, "MC2TP1", 0U, 0U, 0U },
  { 0x42CU, "MC2TP2", 0U, 0U, 0U },
  { 0x424U, "MC2PHA", 0U, 0U, 0U },
  { 0x42EU, "MC2CUM", 0U, 0U, 0U },
  { 0x425U, "MC2VVC", 0U, 0U, 0U },
  { 0x426U, "MC2IVC", 0U, 0U, 0U },
  { 0x427U, "MC2BEM", 0U, 0U, 0U },

  { 0x501U, "DC_DRV",  0U, 0U, 0U },
  { 0x504U, "DC_SWC",  0U, 0U, 0U },

  { 0x581U, "BP_VMX",  0U, 0U, 0U },
  { 0x582U, "BP_VMN",  0U, 0U, 0U },
  { 0x583U, "BP_TMX",  0U, 0U, 0U },
  { 0x585U, "BP_ISH",  0U, 0U, 0U },
  { 0x586U, "BP_PVS",  0U, 0U, 0U },

  { 0x401U, "MC1LIM", 0U, 0U, 0U },
  { 0x421U, "MC2LIM", 0U, 0U, 0U }
};

#define SUN_RAW_TABLE_COUNT  (sizeof(sun_raw_table) / sizeof(sun_raw_table[0]))

static char sun_raw_block[SUN_RAW_BLOCK_LEN];
static volatile uint32_t can_rx_count;
static volatile uint32_t can_rx_pulse_tick;
static volatile uint32_t can_last_rx_id;
static volatile uint8_t can_last_rx_dlc;
static uint32_t rs232_tx_pulse_tick;
static uint32_t rs232_tx_attempt_count;
static uint32_t rs232_tx_success_count;
static uint8_t rs232_tx_led_active;
static uint8_t can_rx_led_active;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef CAN1_ListenAllStart(void);
static void CAN1_DrainRxFifo(void);
static uint32_t CAN_MakeU32LE(const uint8_t *data);
static void SunRaw_UpdateFromCAN(uint32_t id, uint8_t dlc, const uint8_t *data);
static int SunRaw_BuildBlock(char *out, size_t out_len);
static void RS232_SendSunRawBlock(void);
static void StatusLeds_Task(void);
static void SetLed(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Retarget printf() to SWV ITM stimulus port 0.  This remains completely
 * separate from USART1, so diagnostics cannot alter the RS232 byte stream.
 */
int __io_putchar(int ch)
{
  uint32_t timeout;

  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
  {
    return ch;
  }

  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U) ||
      ((ITM->TER & 1UL) == 0U))
  {
    return ch;
  }

  timeout = SWV_PUTCHAR_TIMEOUT;
  while ((ITM->PORT[0U].u32 == 0UL) && (timeout > 0U))
  {
    timeout--;
  }

  if (timeout > 0U)
  {
    ITM->PORT[0U].u8 = (uint8_t)ch;
  }

  return ch;
}

static void SetLed(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  HAL_GPIO_WritePin(port,
                    pin,
                    (state == GPIO_PIN_SET) ? LED_ACTIVE_STATE : LED_INACTIVE_STATE);
}

static HAL_StatusTypeDef CAN1_ListenAllStart(void)
{
  CAN_FilterTypeDef filter = {0};

  /* ID=0 and mask=0 accepts every standard and extended CAN identifier. */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

static uint32_t CAN_MakeU32LE(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static void SunRaw_UpdateFromCAN(uint32_t id, uint8_t dlc, const uint8_t *data)
{
  uint32_t i;

  if ((data == NULL) || (dlc < 8U))
  {
    return;
  }

  for (i = 0U; i < SUN_RAW_TABLE_COUNT; i++)
  {
    if (sun_raw_table[i].can_id == id)
    {
      /* Bytes 0..3 are the low word; bytes 4..7 are the high word. */
      sun_raw_table[i].low_word = CAN_MakeU32LE(&data[0]);
      sun_raw_table[i].high_word = CAN_MakeU32LE(&data[4]);
      sun_raw_table[i].valid = 1U;
      return;
    }
  }
}

static void CAN1_DrainRxFifo(void)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      break;
    }

    can_rx_count++;
    can_rx_pulse_tick = HAL_GetTick();
    can_rx_led_active = 1U;

    if (rx_header.IDE == CAN_ID_STD)
    {
      can_last_rx_id = rx_header.StdId;
      can_last_rx_dlc = (uint8_t)rx_header.DLC;
      SunRaw_UpdateFromCAN(rx_header.StdId, (uint8_t)rx_header.DLC, rx_data);
    }
    else
    {
      can_last_rx_id = rx_header.ExtId;
      can_last_rx_dlc = (uint8_t)rx_header.DLC;
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN1_DrainRxFifo();
  }
}

static int SunRaw_BuildBlock(char *out, size_t out_len)
{
  size_t pos = 0U;
  uint32_t i;
  int written;

  if ((out == NULL) || (out_len == 0U))
  {
    return -1;
  }

  written = snprintf(out, out_len, "raw_data\r\nABCDEF\r\n");
  if ((written < 0) || ((size_t)written >= out_len))
  {
    return -1;
  }
  pos = (size_t)written;

  for (i = 0U; i < SUN_RAW_TABLE_COUNT; i++)
  {
    uint32_t high_word;
    uint32_t low_word;
    uint8_t valid;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    high_word = sun_raw_table[i].high_word;
    low_word = sun_raw_table[i].low_word;
    valid = sun_raw_table[i].valid;
    __set_PRIMASK(primask);

    if (valid != 0U)
    {
      written = snprintf(&out[pos],
                         out_len - pos,
                         "%s,0x%08lX,0x%08lX\r\n",
                         sun_raw_table[i].name,
                         (unsigned long)high_word,
                         (unsigned long)low_word);
    }
    else
    {
      written = snprintf(&out[pos],
                         out_len - pos,
                         "%s,0xHHHHHHHH,0xHHHHHHHH\r\n",
                         sun_raw_table[i].name);
    }

    if ((written < 0) || ((size_t)written >= (out_len - pos)))
    {
      return -1;
    }
    pos += (size_t)written;
  }

  /*
   * This board does not configure the reference project's BME, GPS, IMU, or
   * RTC peripherals.  Keep the same RS232 schema and use its unavailable-data
   * fallbacks so the Sunseeker display parser sees an identical block shape.
   */
  written = snprintf(&out[pos],
                     out_len - pos,
                     "BME,T=0.00,P=0.00,H=0.00\r\n"
                     "NAV,IMU_MPH=0.00,GPS_MPH=0.00,GPS_VALID=0,VEHICLE_MPH=0.00,SOURCE=NONE,LAT=0.000000,LON=0.000000,FIX=0,AGE_MS=0\r\n"
                     "TL_TIM,RTC_READ_FAIL\r\n"
                     "VWXYZ\r\n");
  if ((written < 0) || ((size_t)written >= (out_len - pos)))
  {
    return -1;
  }

  pos += (size_t)written;
  return (int)pos;
}

static void RS232_SendSunRawBlock(void)
{
  int length = SunRaw_BuildBlock(sun_raw_block, sizeof(sun_raw_block));
  HAL_StatusTypeDef status;
  uint32_t start_tick;

  rs232_tx_attempt_count++;

  if (length <= 0)
  {
    SetLed(Red_LED10_GPIO_Port, Red_LED10_Pin, GPIO_PIN_SET);
    printf("[RS232] TX #%lu block build failed\r\n",
           (unsigned long)rs232_tx_attempt_count);
    return;
  }

  start_tick = HAL_GetTick();
  printf("[RS232] TX #%lu start t=%lu bytes=%d state=%lu err=0x%08lX SR=0x%08lX\r\n",
         (unsigned long)rs232_tx_attempt_count,
         (unsigned long)start_tick,
         length,
         (unsigned long)HAL_UART_GetState(&huart1),
         (unsigned long)HAL_UART_GetError(&huart1),
         (unsigned long)USART1->SR);

#if SWV_ECHO_RS232_BLOCK
  printf("[RS232] Bytes sent to USART1 will be:\r\n%s", sun_raw_block);
#endif

  status = HAL_UART_Transmit(&huart1,
                             (uint8_t *)sun_raw_block,
                             (uint16_t)length,
                             RS232_TX_TIMEOUT_MS);

  if (status == HAL_OK)
  {
    rs232_tx_success_count++;
    rs232_tx_pulse_tick = HAL_GetTick();
    rs232_tx_led_active = 1U;
    printf("[RS232] TX #%lu HAL_OK done=%lu elapsed=%lu ms success=%lu SR=0x%08lX\r\n",
           (unsigned long)rs232_tx_attempt_count,
           (unsigned long)HAL_GetTick(),
           (unsigned long)(HAL_GetTick() - start_tick),
           (unsigned long)rs232_tx_success_count,
           (unsigned long)USART1->SR);
  }
  else
  {
    SetLed(Red_LED10_GPIO_Port, Red_LED10_Pin, GPIO_PIN_SET);
    printf("[RS232] TX #%lu FAILED status=%d state=%lu err=0x%08lX SR=0x%08lX\r\n",
           (unsigned long)rs232_tx_attempt_count,
           (int)status,
           (unsigned long)HAL_UART_GetState(&huart1),
           (unsigned long)HAL_UART_GetError(&huart1),
           (unsigned long)USART1->SR);
  }
}

static void StatusLeds_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t heartbeat_phase = now % HEARTBEAT_PERIOD_MS;

  SetLed(Green_LED11_GPIO_Port,
         Green_LED11_Pin,
         (heartbeat_phase < HEARTBEAT_ON_TIME_MS) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if ((rs232_tx_led_active != 0U) &&
      ((now - rs232_tx_pulse_tick) >= STATUS_LED_PULSE_MS))
  {
    rs232_tx_led_active = 0U;
  }
  SetLed(Yellow_LED7_GPIO_Port,
         Yellow_LED7_Pin,
         (rs232_tx_led_active != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if ((can_rx_led_active != 0U) &&
      ((now - can_rx_pulse_tick) >= STATUS_LED_PULSE_MS))
  {
    can_rx_led_active = 0U;
  }
  SetLed(Yellow_LED6_GPIO_Port,
         Yellow_LED6_Pin,
         (can_rx_led_active != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  MX_USB_OTG_FS_PCD_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);

  /* CubeMX initializes these active-low LEDs low, so explicitly turn them off. */
  HAL_GPIO_WritePin(GPIOD,
                    Green_LED11_Pin | Red_LED10_Pin | Yellow_LED9_Pin |
                    Yellow_LED8_Pin | Yellow_LED7_Pin | Yellow_LED6_Pin |
                    Yellow_LED5_Pin,
                    LED_INACTIVE_STATE);

  printf("\r\n[BOOT] RS232/CAN telemetry firmware started t=%lu ms\r\n",
         (unsigned long)HAL_GetTick());
  printf("[BOOT] SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu Hz\r\n",
         (unsigned long)HAL_RCC_GetSysClockFreq(),
         (unsigned long)HAL_RCC_GetHCLKFreq(),
         (unsigned long)HAL_RCC_GetPCLK1Freq(),
         (unsigned long)HAL_RCC_GetPCLK2Freq());
  printf("[UART] USART1 TX=PA9 RX=PA10 baud=%lu 8-N-1 state=%lu BRR=0x%04lX CR1=0x%04lX\r\n",
         (unsigned long)huart1.Init.BaudRate,
         (unsigned long)HAL_UART_GetState(&huart1),
         (unsigned long)USART1->BRR,
         (unsigned long)USART1->CR1);
  printf("[RS232] Activity LED=Yellow_LED7/PD12 (active-low)\r\n");
  printf("[CAN] CAN1 RX=PD0 TX=PD1 bitrate=250000 bit/s, starting listen-all\r\n");

  if (CAN1_ListenAllStart() != HAL_OK)
  {
    SetLed(Red_LED10_GPIO_Port, Red_LED10_Pin, GPIO_PIN_SET);
    printf("[CAN] listen-all START FAILED state=%lu err=0x%08lX ESR=0x%08lX\r\n",
           (unsigned long)HAL_CAN_GetState(&hcan1),
           (unsigned long)HAL_CAN_GetError(&hcan1),
           (unsigned long)CAN1->ESR);
  }
  else
  {
    printf("[CAN] listen-all ready state=%lu BTR=0x%08lX\r\n",
           (unsigned long)HAL_CAN_GetState(&hcan1),
           (unsigned long)CAN1->BTR);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_rs232_send_tick;
    static uint32_t last_swv_status_tick;
    uint32_t now = HAL_GetTick();

    if ((now - last_rs232_send_tick) >= SUN_RAW_SEND_PERIOD_MS)
    {
      last_rs232_send_tick = now;
      RS232_SendSunRawBlock();
    }

    if ((now - last_swv_status_tick) >= 5000U)
    {
      uint32_t primask = __get_PRIMASK();
      uint32_t rx_count;
      uint32_t last_id;
      uint8_t last_dlc;

      last_swv_status_tick = now;
      __disable_irq();
      rx_count = can_rx_count;
      last_id = can_last_rx_id;
      last_dlc = can_last_rx_dlc;
      __set_PRIMASK(primask);

      printf("[STATUS] t=%lu uart_tx=%lu/%lu uart_state=%lu uart_err=0x%08lX can_rx=%lu last_id=0x%lX dlc=%u can_state=%lu can_err=0x%08lX\r\n",
             (unsigned long)now,
             (unsigned long)rs232_tx_success_count,
             (unsigned long)rs232_tx_attempt_count,
             (unsigned long)HAL_UART_GetState(&huart1),
             (unsigned long)HAL_UART_GetError(&huart1),
             (unsigned long)rx_count,
             (unsigned long)last_id,
             last_dlc,
             (unsigned long)HAL_CAN_GetState(&hcan1),
             (unsigned long)HAL_CAN_GetError(&hcan1));
    }

    StatusLeds_Task();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_LED11_Pin|Red_LED10_Pin|Yellow_LED9_Pin|Yellow_LED8_Pin
                          |Yellow_LED7_Pin|Yellow_LED6_Pin|Yellow_LED5_Pin, GPIO_PIN_RESET);

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
