/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
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

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SD_BLOCK_SIZE              512U
#define SD_SPI_TIMEOUT_MS          100U
#define SD_INIT_TIMEOUT_MS         1000U
#define SD_READY_TIMEOUT_MS        500U
#define SD_DATA_TIMEOUT_MS         200U
#define SD_SPI_INIT_PRESCALER      SPI_BAUDRATEPRESCALER_256
#define SD_SPI_RUN_PRESCALER       SPI_BAUDRATEPRESCALER_8

/* Set to 1 for sector-by-sector debugging.  Normal operation keeps only
   initialization and error messages so SWV CAN/RS232 output stays readable. */
#define SDIO_TRACE_OPERATIONS       0U

#define CMD0                       (0x40U + 0U)
#define CMD1                       (0x40U + 1U)
#define CMD8                       (0x40U + 8U)
#define CMD9                       (0x40U + 9U)
#define CMD12                      (0x40U + 12U)
#define CMD16                      (0x40U + 16U)
#define CMD17                      (0x40U + 17U)
#define CMD18                      (0x40U + 18U)
#define CMD24                      (0x40U + 24U)
#define CMD25                      (0x40U + 25U)
#define CMD55                      (0x40U + 55U)
#define CMD58                      (0x40U + 58U)
#define ACMD23                     (0x80U | (0x40U + 23U))
#define ACMD41                     (0x80U | (0x40U + 41U))

#define SD_CT_MMC                  0x01U
#define SD_CT_SD1                  0x02U
#define SD_CT_SD2                  0x04U
#define SD_CT_BLOCK                0x08U

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;

extern SPI_HandleTypeDef hspi3;

static void SD_Select(void);
static void SD_Deselect(void);
static uint8_t SD_SPI_Transfer(uint8_t data);
static void SD_SPI_SetSpeed(uint32_t baud_prescaler);
static uint8_t SD_WaitReady(uint32_t timeout_ms);
static uint8_t SD_SendCommandRaw(uint8_t cmd, uint32_t arg, uint8_t deselect_first);
static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg);
static uint8_t SD_ReceiveDataBlock(uint8_t *buff, UINT btr);
static uint8_t SD_TransmitDataBlock(const uint8_t *buff, uint8_t token);
static DRESULT SD_ReadCsd(uint8_t *csd);
static DRESULT SD_GetSectorCount(DWORD *sector_count);
static const char *SD_CardTypeName(uint8_t card_type);

static void SD_Select(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
  (void)SD_SPI_Transfer(0xFFU);
}

static void SD_Deselect(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  (void)SD_SPI_Transfer(0xFFU);
}

static uint8_t SD_SPI_Transfer(uint8_t data)
{
  uint8_t received = 0xFFU;
  static uint8_t spi_error_reported;

  if (HAL_SPI_TransmitReceive(&hspi3, &data, &received, 1U, SD_SPI_TIMEOUT_MS) != HAL_OK)
  {
    if (spi_error_reported == 0U)
    {
      spi_error_reported = 1U;
      printf("[SDIO] HAL_SPI_TransmitReceive failed, HAL error=0x%08lX\r\n",
             HAL_SPI_GetError(&hspi3));
    }
    return 0xFFU;
  }

  return received;
}

static void SD_SPI_SetSpeed(uint32_t baud_prescaler)
{
  __HAL_SPI_DISABLE(&hspi3);
  MODIFY_REG(hspi3.Instance->CR1, SPI_CR1_BR, baud_prescaler);
  __HAL_SPI_ENABLE(&hspi3);
}

static uint8_t SD_WaitReady(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  uint8_t response;

  do
  {
    response = SD_SPI_Transfer(0xFFU);
    if (response == 0xFFU)
    {
      return 1U;
    }
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  printf("[SDIO] Wait ready timeout after %lu ms\r\n", timeout_ms);
  return 0U;
}

static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg)
{
  uint8_t response;
  uint8_t original_cmd = cmd;

  if (cmd & 0x80U)
  {
    cmd &= 0x7FU;
    response = SD_SendCommandRaw(CMD55, 0U, 1U);
    if (response > 1U)
    {
      printf("[SDIO] CMD55 before ACMD%u failed, R1=0x%02X\r\n",
             (unsigned int)(original_cmd & 0x3FU),
             response);
      return response;
    }

    return SD_SendCommandRaw(cmd, arg, 0U);
  }

  return SD_SendCommandRaw(cmd, arg, 1U);
}

static uint8_t SD_SendCommandRaw(uint8_t cmd, uint32_t arg, uint8_t deselect_first)
{
  uint8_t response;
  uint8_t crc = 0x01U;

  if (deselect_first != 0U)
  {
    SD_Deselect();
    SD_Select();
  }

  if (SD_WaitReady(SD_READY_TIMEOUT_MS) == 0U)
  {
    printf("[SDIO] CMD%u wait-ready failed\r\n", (unsigned int)(cmd & 0x3FU));
    return 0xFFU;
  }

  if (cmd == CMD0)
  {
    crc = 0x95U;
  }
  else if (cmd == CMD8)
  {
    crc = 0x87U;
  }

  (void)SD_SPI_Transfer(cmd);
  (void)SD_SPI_Transfer((uint8_t)(arg >> 24));
  (void)SD_SPI_Transfer((uint8_t)(arg >> 16));
  (void)SD_SPI_Transfer((uint8_t)(arg >> 8));
  (void)SD_SPI_Transfer((uint8_t)arg);
  (void)SD_SPI_Transfer(crc);

  if (cmd == CMD12)
  {
    (void)SD_SPI_Transfer(0xFFU);
  }

  for (uint8_t n = 10U; n; n--)
  {
    response = SD_SPI_Transfer(0xFFU);
    if ((response & 0x80U) == 0U)
    {
      return response;
    }
  }

  printf("[SDIO] CMD%u no valid response\r\n", (unsigned int)(cmd & 0x3FU));
  return 0xFFU;
}

static uint8_t SD_ReceiveDataBlock(uint8_t *buff, UINT btr)
{
  uint8_t token;
  uint32_t start_tick = HAL_GetTick();

  do
  {
    token = SD_SPI_Transfer(0xFFU);
    if (token == 0xFEU)
    {
      break;
    }
  } while ((HAL_GetTick() - start_tick) < SD_DATA_TIMEOUT_MS);

  if (token != 0xFEU)
  {
    printf("[SDIO] Data token timeout, last token=0x%02X\r\n", token);
    return 0U;
  }

  do
  {
    *buff++ = SD_SPI_Transfer(0xFFU);
  } while (--btr);

  (void)SD_SPI_Transfer(0xFFU);
  (void)SD_SPI_Transfer(0xFFU);

  return 1U;
}

static uint8_t SD_TransmitDataBlock(const uint8_t *buff, uint8_t token)
{
  uint8_t response;

  if (SD_WaitReady(SD_READY_TIMEOUT_MS) == 0U)
  {
    printf("[SDIO] Write block wait-ready failed\r\n");
    return 0U;
  }

  (void)SD_SPI_Transfer(token);
  if (token == 0xFDU)
  {
    return 1U;
  }

  for (UINT i = 0U; i < SD_BLOCK_SIZE; i++)
  {
    (void)SD_SPI_Transfer(buff[i]);
  }

  (void)SD_SPI_Transfer(0xFFU);
  (void)SD_SPI_Transfer(0xFFU);

  response = SD_SPI_Transfer(0xFFU);
  if ((response & 0x1FU) != 0x05U)
  {
    printf("[SDIO] Write block rejected, response=0x%02X\r\n", response);
    return 0U;
  }

  return 1U;
}

static DRESULT SD_ReadCsd(uint8_t *csd)
{
  DRESULT result = RES_ERROR;

  if (SD_SendCommand(CMD9, 0U) == 0U &&
      SD_ReceiveDataBlock(csd, 16U) != 0U)
  {
    result = RES_OK;
  }

  SD_Deselect();
  return result;
}

static DRESULT SD_GetSectorCount(DWORD *sector_count)
{
  uint8_t csd[16];
  DWORD csize;
  uint8_t n;

  if (sector_count == NULL)
  {
    return RES_PARERR;
  }

  if (SD_ReadCsd(csd) != RES_OK)
  {
    printf("[SDIO] Read CSD failed while getting sector count\r\n");
    return RES_ERROR;
  }

  if ((csd[0] >> 6) == 1U)
  {
    csize = (DWORD)(csd[9]) + ((DWORD)csd[8] << 8) + ((DWORD)(csd[7] & 0x3FU) << 16) + 1U;
    *sector_count = csize << 10;
  }
  else
  {
    n = (uint8_t)((csd[5] & 0x0FU) + ((csd[10] & 0x80U) >> 7) + ((csd[9] & 0x03U) << 1) + 2U);
    csize = (DWORD)((csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 0x03U) << 10) + 1U);
    *sector_count = csize << (n - 9U);
  }

  return RES_OK;
}

static const char *SD_CardTypeName(uint8_t card_type)
{
  if (card_type == 0U)
  {
    return "UNKNOWN";
  }

  if (card_type & SD_CT_MMC)
  {
    return "MMC";
  }

  if (card_type & SD_CT_SD2)
  {
    return (card_type & SD_CT_BLOCK) ? "SDv2 SDHC/SDXC" : "SDv2 SDSC";
  }

  if (card_type & SD_CT_SD1)
  {
    return "SDv1";
  }

  return "UNKNOWN";
}

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
  uint8_t cmd;
  uint8_t ocr[4];
  uint8_t response;
  uint8_t ty = 0U;
  uint32_t start_tick;
  uint32_t attempts;

  printf("[SDIO] disk_initialize pdrv=%u at %lu ms\r\n", pdrv, HAL_GetTick());

  if (pdrv != 0U)
  {
    printf("[SDIO] Invalid physical drive %u\r\n", pdrv);
    return STA_NOINIT;
  }

  SD_Deselect();
  SD_SPI_SetSpeed(SD_SPI_INIT_PRESCALER);
  printf("[SDIO] SPI set to init speed (~125 kbit/s), sending 80 idle clocks\r\n");

  for (uint8_t i = 0U; i < 10U; i++)
  {
    SD_SPI_Transfer(0xFFU);
  }

  response = SD_SendCommand(CMD0, 0U);
  printf("[SDIO] CMD0 GO_IDLE R1=0x%02X\r\n", response);

  if (response == 1U)
  {
    response = SD_SendCommand(CMD8, 0x1AAU);
    printf("[SDIO] CMD8 SEND_IF_COND R1=0x%02X\r\n", response);

    if (response == 1U)
    {
      for (uint8_t i = 0U; i < 4U; i++)
      {
        ocr[i] = SD_SPI_Transfer(0xFFU);
      }
      printf("[SDIO] CMD8 echo OCR bytes: %02X %02X %02X %02X\r\n",
             ocr[0],
             ocr[1],
             ocr[2],
             ocr[3]);

      if (ocr[2] == 0x01U && ocr[3] == 0xAAU)
      {
        start_tick = HAL_GetTick();
        attempts = 0U;
        do
        {
          attempts++;
          response = SD_SendCommand(ACMD41, 1UL << 30);
          if (response == 0U)
          {
            break;
          }
        } while ((HAL_GetTick() - start_tick) < SD_INIT_TIMEOUT_MS);
        printf("[SDIO] ACMD41 HCS final R1=0x%02X after %lu attempts\r\n",
               response,
               attempts);

        if ((HAL_GetTick() - start_tick) < SD_INIT_TIMEOUT_MS &&
            SD_SendCommand(CMD58, 0U) == 0U)
        {
          for (uint8_t i = 0U; i < 4U; i++)
          {
            ocr[i] = SD_SPI_Transfer(0xFFU);
          }
          printf("[SDIO] CMD58 OCR: %02X %02X %02X %02X\r\n",
                 ocr[0],
                 ocr[1],
                 ocr[2],
                 ocr[3]);
          ty = (ocr[0] & 0x40U) ? (SD_CT_SD2 | SD_CT_BLOCK) : SD_CT_SD2;
        }
        else
        {
          printf("[SDIO] SDv2 init failed or timed out\r\n");
        }
      }
      else
      {
        printf("[SDIO] CMD8 voltage/check pattern mismatch\r\n");
      }
    }
    else
    {
      if (SD_SendCommand(ACMD41, 0U) <= 1U)
      {
        ty = SD_CT_SD1;
        cmd = ACMD41;
      }
      else
      {
        ty = SD_CT_MMC;
        cmd = CMD1;
      }

      start_tick = HAL_GetTick();
      attempts = 0U;
      do
      {
        attempts++;
        response = SD_SendCommand(cmd, 0U);
        if (response == 0U)
        {
          break;
        }
      } while ((HAL_GetTick() - start_tick) < SD_INIT_TIMEOUT_MS);
      printf("[SDIO] Legacy init with %s final R1=0x%02X after %lu attempts\r\n",
             (cmd == ACMD41) ? "ACMD41" : "CMD1",
             response,
             attempts);

      if ((HAL_GetTick() - start_tick) >= SD_INIT_TIMEOUT_MS ||
          SD_SendCommand(CMD16, SD_BLOCK_SIZE) != 0U)
      {
        ty = 0U;
        printf("[SDIO] Legacy init timed out or CMD16 set block size failed\r\n");
      }
    }
  }
  else
  {
    printf("[SDIO] Card did not enter idle state; check CS/SCK/MOSI/MISO/power\r\n");
  }

  CardType = ty;
  SD_Deselect();

  if (ty != 0U)
  {
    Stat &= (DSTATUS)~STA_NOINIT;
    SD_SPI_SetSpeed(SD_SPI_RUN_PRESCALER);
    printf("[SDIO] Card initialized: type=%s, Stat=0x%02X, SPI run speed ~4 Mbit/s\r\n",
           SD_CardTypeName(CardType),
           Stat);
  }
  else
  {
    Stat = STA_NOINIT;
    printf("[SDIO] Card initialization failed, Stat=0x%02X\r\n", Stat);
  }

  return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
  if (pdrv != 0U)
  {
    return STA_NOINIT;
  }

  return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  if (pdrv != 0U || buff == NULL || count == 0U)
  {
    printf("[SDIO] USER_read invalid args pdrv=%u buff=%p count=%u\r\n",
           pdrv,
           (void *)buff,
           count);
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT)
  {
    printf("[SDIO] USER_read requested while not initialized, Stat=0x%02X\r\n", Stat);
    return RES_NOTRDY;
  }

#if SDIO_TRACE_OPERATIONS
  printf("[SDIO] READ sector=%lu count=%u\r\n", sector, count);
#endif

  if ((CardType & SD_CT_BLOCK) == 0U)
  {
    sector *= SD_BLOCK_SIZE;
  }

  if (count == 1U)
  {
    if (SD_SendCommand(CMD17, sector) == 0U &&
        SD_ReceiveDataBlock(buff, SD_BLOCK_SIZE) != 0U)
    {
      count = 0U;
    }
  }
  else
  {
    if (SD_SendCommand(CMD18, sector) == 0U)
    {
      do
      {
        if (SD_ReceiveDataBlock(buff, SD_BLOCK_SIZE) == 0U)
        {
          break;
        }
        buff += SD_BLOCK_SIZE;
      } while (--count);
      (void)SD_SendCommand(CMD12, 0U);
    }
  }

  SD_Deselect();
  if (count != 0U)
  {
    printf("[SDIO] READ failed, remaining=%u\r\n", count);
  }
  else
  {
#if SDIO_TRACE_OPERATIONS
    printf("[SDIO] READ OK\r\n");
#endif
  }
  return count ? RES_ERROR : RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  if (pdrv != 0U || buff == NULL || count == 0U)
  {
    printf("[SDIO] USER_write invalid args pdrv=%u buff=%p count=%u\r\n",
           pdrv,
           (const void *)buff,
           count);
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT)
  {
    printf("[SDIO] USER_write requested while not initialized, Stat=0x%02X\r\n", Stat);
    return RES_NOTRDY;
  }

  if (Stat & STA_PROTECT)
  {
    printf("[SDIO] USER_write rejected: write protected\r\n");
    return RES_WRPRT;
  }

#if SDIO_TRACE_OPERATIONS
  printf("[SDIO] WRITE sector=%lu count=%u\r\n", sector, count);
#endif

  if ((CardType & SD_CT_BLOCK) == 0U)
  {
    sector *= SD_BLOCK_SIZE;
  }

  if (count == 1U)
  {
    if (SD_SendCommand(CMD24, sector) == 0U &&
        SD_TransmitDataBlock(buff, 0xFEU) != 0U)
    {
      count = 0U;
    }
  }
  else
  {
    if ((CardType & (SD_CT_SD1 | SD_CT_SD2)) != 0U)
    {
      (void)SD_SendCommand(ACMD23, count);
    }

    if (SD_SendCommand(CMD25, sector) == 0U)
    {
      do
      {
        if (SD_TransmitDataBlock(buff, 0xFCU) == 0U)
        {
          break;
        }
        buff += SD_BLOCK_SIZE;
      } while (--count);

      if (SD_TransmitDataBlock(NULL, 0xFDU) == 0U)
      {
        count = 1U;
      }
    }
  }

  SD_Deselect();
  if (count != 0U)
  {
    printf("[SDIO] WRITE failed, remaining=%u\r\n", count);
  }
  else
  {
#if SDIO_TRACE_OPERATIONS
    printf("[SDIO] WRITE OK\r\n");
#endif
  }
  return count ? RES_ERROR : RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
  DRESULT res = RES_ERROR;

  if (pdrv != 0U)
  {
    printf("[SDIO] USER_ioctl invalid pdrv=%u cmd=%u\r\n", pdrv, cmd);
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT)
  {
    printf("[SDIO] USER_ioctl cmd=%u while not initialized, Stat=0x%02X\r\n", cmd, Stat);
    return RES_NOTRDY;
  }

#if SDIO_TRACE_OPERATIONS
  printf("[SDIO] IOCTL cmd=%u\r\n", cmd);
#endif

  switch (cmd)
  {
    case CTRL_SYNC:
      SD_Select();
      res = SD_WaitReady(SD_READY_TIMEOUT_MS) ? RES_OK : RES_ERROR;
      SD_Deselect();
      break;

    case GET_SECTOR_COUNT:
      res = SD_GetSectorCount((DWORD *)buff);
      if (res == RES_OK)
      {
#if SDIO_TRACE_OPERATIONS
        printf("[SDIO] Sector count=%lu\r\n", *(DWORD *)buff);
#endif
      }
      break;

    case GET_SECTOR_SIZE:
      *(WORD *)buff = SD_BLOCK_SIZE;
      res = RES_OK;
#if SDIO_TRACE_OPERATIONS
      printf("[SDIO] Sector size=%u\r\n", SD_BLOCK_SIZE);
#endif
      break;

    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 128U;
      res = RES_OK;
#if SDIO_TRACE_OPERATIONS
      printf("[SDIO] Erase block size=%lu sectors\r\n", *(DWORD *)buff);
#endif
      break;

    default:
      res = RES_PARERR;
      break;
  }

#if SDIO_TRACE_OPERATIONS
  printf("[SDIO] IOCTL cmd=%u result=%u\r\n", cmd, res);
#endif
  return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

