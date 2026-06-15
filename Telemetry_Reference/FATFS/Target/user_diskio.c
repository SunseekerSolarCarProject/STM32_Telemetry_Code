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
#include "main.h"
#include "ff_gen_drv.h"

/* SD card is on SPI2 in your main.c */
extern SPI_HandleTypeDef hspi2;

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Card type flags */
#define CT_MMC      0x01U
#define CT_SD1      0x02U
#define CT_SD2      0x04U
#define CT_SDC      (CT_SD1 | CT_SD2)
#define CT_BLOCK    0x08U

/* SD commands */
#define CMD0        (0U)
#define CMD1        (1U)
#define ACMD41      (0x80U + 41U)
#define CMD8        (8U)
#define CMD9        (9U)
#define CMD10       (10U)
#define CMD12       (12U)
#define CMD16       (16U)
#define CMD17       (17U)
#define CMD18       (18U)
#define CMD23       (23U)
#define ACMD23      (0x80U + 23U)
#define CMD24       (24U)
#define CMD25       (25U)
#define CMD55       (55U)
#define CMD58       (58U)

/* Data tokens */
#define TOKEN_SINGLE_READ_WRITE   0xFEU
#define TOKEN_MULTI_WRITE         0xFCU
#define TOKEN_STOP_TRAN           0xFDU

/* Timeouts */
#define SD_SPI_TIMEOUT_MS         100U
#define SD_READY_TIMEOUT_MS       500U
#define SD_INIT_TIMEOUT_MS        3000U
#define SD_READ_TIMEOUT_MS        200U
#define SD_WRITE_TIMEOUT_MS       500U
#define SD_CMD0_RETRIES           20U

#define SD_DUMMY_BYTE             0xFFU
#define SD_R1_IDLE                0x01U
#define SD_R1_ILLEGAL_COMMAND     0x04U
#define SD_R1_NO_RESPONSE         0xFFU

#define SD_SPI_INIT_PRESCALER     SPI_BAUDRATEPRESCALER_128
#define SD_SPI_DATA_PRESCALER     SPI_BAUDRATEPRESCALER_8

static BYTE CardType = 0U;
static uint8_t LastDataToken = 0xFFU;

static void SD_SPI_SetPrescaler(uint32_t prescaler)
{
  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
  {
    /* Wait for any previous blocking transfer to finish. */
  }

  hspi2.Init.BaudRatePrescaler = prescaler;

  __HAL_SPI_DISABLE(&hspi2);
  MODIFY_REG(hspi2.Instance->CR1, SPI_CR1_BR, prescaler);
}

static void SD_Select(void)
{
  HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_RESET);
}

static void SD_Deselect(void)
{
  HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_SET);

  /* One extra clock after CS high */
  uint8_t tx = SD_DUMMY_BYTE;
  uint8_t rx = 0U;
  HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1U, SD_SPI_TIMEOUT_MS);
}

static uint8_t SD_SPI_TxRx(uint8_t data)
{
  uint8_t rx = SD_DUMMY_BYTE;

  if (HAL_SPI_TransmitReceive(&hspi2,
                              &data,
                              &rx,
                              1U,
                              SD_SPI_TIMEOUT_MS) != HAL_OK)
  {
    return 0x00U;
  }

  return rx;
}

static void SD_SPI_Send(const uint8_t *buff, UINT len)
{
  if ((buff == NULL) || (len == 0U))
  {
    return;
  }

  HAL_SPI_Transmit(&hspi2,
                   (uint8_t *)buff,
                   (uint16_t)len,
                   SD_SPI_TIMEOUT_MS);
}

static void SD_SPI_Recv(uint8_t *buff, UINT len)
{
  uint8_t tx_dummy[32];

  if ((buff == NULL) || (len == 0U))
  {
    return;
  }

  memset(tx_dummy, SD_DUMMY_BYTE, sizeof(tx_dummy));

  while (len > 0U)
  {
    uint16_t chunk = (len > sizeof(tx_dummy)) ?
                     (uint16_t)sizeof(tx_dummy) :
                     (uint16_t)len;

    if (HAL_SPI_TransmitReceive(&hspi2,
                                tx_dummy,
                                buff,
                                chunk,
                                SD_SPI_TIMEOUT_MS) != HAL_OK)
    {
      return;
    }

    buff += chunk;
    len -= chunk;
  }
}

static uint8_t SD_WaitReady(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  do
  {
    if (SD_SPI_TxRx(SD_DUMMY_BYTE) == 0xFFU)
    {
      return 1U;
    }
  } while ((HAL_GetTick() - start) < timeout_ms);

  return 0U;
}

static uint8_t SD_SelectWaitReady(void)
{
  SD_Select();

  if (SD_WaitReady(SD_READY_TIMEOUT_MS))
  {
    return 1U;
  }

  SD_Deselect();
  return 0U;
}

static void SD_SendInitialClockTrain(void)
{
  SD_Deselect();

  /*
   * Send at least 74 clocks with CS high.
   * 10 bytes = 80 clocks.
   */
  for (uint8_t i = 0U; i < 10U; i++)
  {
    SD_SPI_TxRx(SD_DUMMY_BYTE);
  }
}

static uint8_t SD_SendCmd(uint8_t cmd, DWORD arg)
{
  uint8_t res;
  uint8_t crc = 0x01U;

  /*
   * ACMD<n> is sent as CMD55 followed by CMD<n>.
   */
  if (cmd & 0x80U)
  {
    cmd &= 0x7FU;
    res = SD_SendCmd(CMD55, 0U);

    if (res > 1U)
    {
      return res;
    }
  }

  SD_Deselect();

  if (!SD_SelectWaitReady())
  {
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

  SD_SPI_TxRx((uint8_t)(0x40U | cmd));
  SD_SPI_TxRx((uint8_t)(arg >> 24));
  SD_SPI_TxRx((uint8_t)(arg >> 16));
  SD_SPI_TxRx((uint8_t)(arg >> 8));
  SD_SPI_TxRx((uint8_t)arg);
  SD_SPI_TxRx(crc);

  if (cmd == CMD12)
  {
    SD_SPI_TxRx(SD_DUMMY_BYTE);
  }

  for (uint8_t i = 0U; i < 10U; i++)
  {
    res = SD_SPI_TxRx(SD_DUMMY_BYTE);

    if ((res & 0x80U) == 0U)
    {
      return res;
    }
  }

  return res;
}

static uint8_t SD_RxDataBlock(uint8_t *buff, UINT btr)
{
  uint8_t token;
  uint32_t start = HAL_GetTick();

  if (buff == NULL)
  {
    return 0U;
  }

  do
  {
    token = SD_SPI_TxRx(SD_DUMMY_BYTE);
    LastDataToken = token;

    if (token == TOKEN_SINGLE_READ_WRITE)
    {
      SD_SPI_Recv(buff, btr);

      /* Discard CRC */
      SD_SPI_TxRx(SD_DUMMY_BYTE);
      SD_SPI_TxRx(SD_DUMMY_BYTE);

      return 1U;
    }
  } while ((HAL_GetTick() - start) < SD_READ_TIMEOUT_MS);

  return 0U;
}

static uint8_t SD_TxDataBlock(const uint8_t *buff, uint8_t token)
{
  uint8_t resp;

  if (!SD_WaitReady(SD_WRITE_TIMEOUT_MS))
  {
    return 0U;
  }

  SD_SPI_TxRx(token);

  if (token == TOKEN_STOP_TRAN)
  {
    return 1U;
  }

  if (buff == NULL)
  {
    return 0U;
  }

  SD_SPI_Send(buff, 512U);

  /* Dummy CRC */
  SD_SPI_TxRx(SD_DUMMY_BYTE);
  SD_SPI_TxRx(SD_DUMMY_BYTE);

  resp = SD_SPI_TxRx(SD_DUMMY_BYTE);

  if ((resp & 0x1FU) != 0x05U)
  {
    return 0U;
  }

  return 1U;
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
	 uint8_t n;
	  uint8_t ty = 0U;
	  uint8_t ocr[4];
	  uint32_t start;
	  uint8_t cmd0_res = 0xFFU;
	  uint8_t cmd8_res = 0xFFU;
	  uint8_t init_res = 0xFFU;

	  if (pdrv != 0U)
	  {
	    return STA_NOINIT;
	  }

	  Stat = STA_NOINIT;
	  CardType = 0U;

	  SD_SPI_SetPrescaler(SD_SPI_INIT_PRESCALER);

	  HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_SET);
	  HAL_Delay(50U);

	  SD_SendInitialClockTrain();

	  for (n = 0U; n < SD_CMD0_RETRIES; n++)
	  {
	    cmd0_res = SD_SendCmd(CMD0, 0U);

	    if (cmd0_res == 1U)
	    {
	      break;
	    }

	    SD_Deselect();
	    HAL_Delay(10U);
	    SD_SendInitialClockTrain();
	  }

	  if (cmd0_res == 1U)
	  {
	    /*
	     * SD version 2 card check.
	     */
	    cmd8_res = SD_SendCmd(CMD8, 0x1AAU);

	    if (cmd8_res == SD_R1_IDLE)
	    {
	      for (n = 0U; n < 4U; n++)
	      {
	        ocr[n] = SD_SPI_TxRx(SD_DUMMY_BYTE);
	      }

	      if ((ocr[2] == 0x01U) && (ocr[3] == 0xAAU))
	      {
	        start = HAL_GetTick();

	        do
	        {
	          init_res = SD_SendCmd(ACMD41, 1UL << 30);

	          if (init_res == 0U)
	          {
	            break;
	          }

	          HAL_Delay(10U);
	        } while ((HAL_GetTick() - start) < SD_INIT_TIMEOUT_MS);

	        if (((HAL_GetTick() - start) < SD_INIT_TIMEOUT_MS) &&
	            (SD_SendCmd(CMD58, 0U) == 0U))
	        {
	          for (n = 0U; n < 4U; n++)
	          {
	            ocr[n] = SD_SPI_TxRx(SD_DUMMY_BYTE);
	          }

	          ty = (ocr[0] & 0x40U) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
	        }
	        else if (init_res != 0U)
	        {
	          printf("SD init failed: ACMD41 timeout, last R1=0x%02X\r\n", init_res);
	        }
	        else
	        {
	          printf("SD init failed: CMD58 did not return ready\r\n");
	        }
	      }
	      else
	      {
	        printf("SD init failed: CMD8 OCR mismatch: %02X %02X %02X %02X\r\n",
	               ocr[0], ocr[1], ocr[2], ocr[3]);
	      }
	    }
	    else if ((cmd8_res & SD_R1_ILLEGAL_COMMAND) != 0U)
	    {
	      /*
	       * SD version 1 or MMC card.
	       */
	      uint8_t cmd;

	      if (SD_SendCmd(ACMD41, 0U) <= 1U)
	      {
	        ty = CT_SD1;
	        cmd = ACMD41;
	      }
	      else
	      {
	        ty = CT_MMC;
	        cmd = CMD1;
	      }

	      start = HAL_GetTick();

	      do
	      {
	        init_res = SD_SendCmd(cmd, 0U);

	        if (init_res == 0U)
	        {
	          break;
	        }

	        HAL_Delay(10U);
	      } while ((HAL_GetTick() - start) < SD_INIT_TIMEOUT_MS);

	      if (((HAL_GetTick() - start) >= SD_INIT_TIMEOUT_MS) ||
	          (SD_SendCmd(CMD16, 512U) != 0U))
	      {
	        printf("SD init failed: legacy init/CMD16 failed, CMD8 R1=0x%02X, last R1=0x%02X\r\n",
	               cmd8_res,
	               init_res);
	        ty = 0U;
	      }
	    }
	    else
	    {
	      printf("SD init failed: CMD8 no valid response after CMD0, R1=0x%02X\r\n",
	             cmd8_res);
	    }
	  }
	  else
	  {
	    printf("SD init failed: CMD0 no idle response, last R1=0x%02X\r\n", cmd0_res);
	  }

	  CardType = ty;
	  SD_Deselect();

	  if (ty != 0U)
	  {
	    Stat &= (DSTATUS)~STA_NOINIT;
	    SD_SPI_SetPrescaler(SD_SPI_DATA_PRESCALER);
	    printf("SD init OK: type=0x%02X (%s)\r\n",
	           CardType,
	           (CardType & CT_BLOCK) ? "block" : "byte");
	    printf("SD SPI switched to data prescaler /8\r\n");
	  }
	  else
	  {
	    Stat = STA_NOINIT;
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
	  DWORD lba = sector;
	  UINT requested = count;
	  uint8_t r1;

	  if ((pdrv != 0U) || (buff == NULL) || (count == 0U))
	  {
	    return RES_PARERR;
	  }

	  if (Stat & STA_NOINIT)
	  {
	    return RES_NOTRDY;
	  }

	  /*
	   * SDSC cards use byte addressing.
	   * SDHC/SDXC cards use block addressing.
	   */
	  if (!(CardType & CT_BLOCK))
	  {
	    sector *= 512U;
	  }

	  if (count == 1U)
	  {
	    r1 = SD_SendCmd(CMD17, sector);

	    if ((r1 == 0U) && SD_RxDataBlock(buff, 512U))
	    {
	      count = 0U;
	    }
	    else
	    {
	      printf("SD read failed: CMD17 lba=%lu addr=%lu R1=0x%02X token=0x%02X\r\n",
	             (unsigned long)lba,
	             (unsigned long)sector,
	             r1,
	             LastDataToken);
	    }
	  }
	  else
	  {
	    r1 = SD_SendCmd(CMD18, sector);

	    if (r1 == 0U)
	    {
	      do
	      {
	        if (!SD_RxDataBlock(buff, 512U))
	        {
	          printf("SD read failed: CMD18 lba=%lu addr=%lu remaining=%u token=0x%02X\r\n",
	                 (unsigned long)lba,
	                 (unsigned long)sector,
	                 count,
	                 LastDataToken);
	          break;
	        }

	        buff += 512U;
	      } while (--count);

	      SD_SendCmd(CMD12, 0U);
	    }
	    else
	    {
	      printf("SD read failed: CMD18 lba=%lu addr=%lu count=%u R1=0x%02X\r\n",
	             (unsigned long)lba,
	             (unsigned long)sector,
	             requested,
	             r1);
	    }
	  }

	  SD_Deselect();

	  return (count == 0U) ? RES_OK : RES_ERROR;
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
  /* USER CODE HERE */
	if ((pdrv != 0U) || (buff == NULL) || (count == 0U))
	  {
	    return RES_PARERR;
	  }

	  if (Stat & STA_NOINIT)
	  {
	    return RES_NOTRDY;
	  }

	  if (Stat & STA_PROTECT)
	  {
	    return RES_WRPRT;
	  }

	  /*
	   * SDSC cards use byte addressing.
	   * SDHC/SDXC cards use block addressing.
	   */
	  if (!(CardType & CT_BLOCK))
	  {
	    sector *= 512U;
	  }

	  if (count == 1U)
	  {
	    if ((SD_SendCmd(CMD24, sector) == 0U) &&
	        SD_TxDataBlock(buff, TOKEN_SINGLE_READ_WRITE))
	    {
	      count = 0U;
	    }
	  }
	  else
	  {
	    if (CardType & CT_SDC)
	    {
	      SD_SendCmd(ACMD23, count);
	    }
	    else
	    {
	      SD_SendCmd(CMD23, count);
	    }

	    if (SD_SendCmd(CMD25, sector) == 0U)
	    {
	      do
	      {
	        if (!SD_TxDataBlock(buff, TOKEN_MULTI_WRITE))
	        {
	          break;
	        }

	        buff += 512U;
	      } while (--count);

	      if (!SD_TxDataBlock(NULL, TOKEN_STOP_TRAN))
	      {
	        count = 1U;
	      }
	    }
	  }

	  SD_Deselect();

	  return (count == 0U) ? RES_OK : RES_ERROR;
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
	  BYTE csd[16];
	  DWORD csize;
	  uint8_t r1;

	  if (pdrv != 0U)
	  {
	    return RES_PARERR;
	  }

	  if (Stat & STA_NOINIT)
	  {
	    return RES_NOTRDY;
	  }

	  switch (cmd)
	  {
	    case CTRL_SYNC:
	      if (SD_SelectWaitReady())
	      {
	        SD_Deselect();
	        res = RES_OK;
	      }
	      break;

	    case GET_SECTOR_COUNT:
	      if (buff == NULL)
	      {
	        res = RES_PARERR;
	        break;
	      }

	      r1 = SD_SendCmd(CMD9, 0U);

	      if ((r1 == 0U) && SD_RxDataBlock(csd, 16U))
	      {
	        if ((csd[0] >> 6) == 1U)
	        {
	          /*
	           * SDHC / SDXC CSD version 2.0.
	           */
	          csize = ((DWORD)(csd[7] & 0x3FU) << 16) |
	                  ((DWORD)csd[8] << 8) |
	                  csd[9];

	          *(DWORD *)buff = (csize + 1U) << 10;
	        }
	        else
	        {
	          /*
	           * SDSC CSD version 1.0.
	           */
	          BYTE n;

	          n = (BYTE)((csd[5] & 0x0FU) +
	                     ((csd[10] & 0x80U) >> 7) +
	                     ((csd[9] & 0x03U) << 1) + 2U);

	          csize = ((DWORD)(csd[8] >> 6) |
	                   ((DWORD)csd[7] << 2) |
	                   ((DWORD)(csd[6] & 0x03U) << 10)) + 1U;

	          *(DWORD *)buff = csize << (n - 9U);
	        }

	        res = RES_OK;
	      }
	      else
	      {
	        printf("SD ioctl failed: CMD9 GET_SECTOR_COUNT R1=0x%02X token=0x%02X\r\n",
	               r1,
	               LastDataToken);
	      }

	      SD_Deselect();
	      break;

	    case GET_SECTOR_SIZE:
	      if (buff == NULL)
	      {
	        res = RES_PARERR;
	        break;
	      }

	      *(WORD *)buff = 512U;
	      res = RES_OK;
	      break;

	    case GET_BLOCK_SIZE:
	      if (buff == NULL)
	      {
	        res = RES_PARERR;
	        break;
	      }

	      /*
	       * Erase block size in units of sectors.
	       * Returning 1 is safe for FatFs.
	       */
	      *(DWORD *)buff = 1U;
	      res = RES_OK;
	      break;

	    default:
	      res = RES_PARERR;
	      break;
	  }

	  return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

