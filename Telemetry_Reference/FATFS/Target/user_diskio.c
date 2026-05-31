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
#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

#define SD_TIMEOUT_MS 1000

#define CMD0   (0)
#define CMD1   (1)
#define CMD8   (8)
#define CMD9   (9)
#define CMD12  (12)
#define CMD16  (16)
#define CMD17  (17)
#define CMD18  (18)
#define CMD24  (24)
#define CMD25  (25)
#define CMD55  (55)
#define CMD58  (58)
#define ACMD41 (0x80 + 41)

#define CT_MMC    0x01
#define CT_SD1    0x02
#define CT_SD2    0x04
#define CT_BLOCK  0x08

static uint8_t CardType = 0;

static void SD_Select(void)
{
  HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_RESET);
}

static void SD_Deselect(void)
{
  HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_SET);

  uint8_t dummy = 0xFF;
  HAL_SPI_Transmit(&hspi2, &dummy, 1, SD_TIMEOUT_MS);
}

static uint8_t SD_SPI_TxRx(uint8_t data)
{
  uint8_t rx = 0xFF;
  HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, SD_TIMEOUT_MS);
  return rx;
}

static void SD_SPI_RxBuf(uint8_t *buff, uint32_t len)
{
  while (len--)
  {
    *buff++ = SD_SPI_TxRx(0xFF);
  }
}

static void SD_SPI_TxBuf(const uint8_t *buff, uint32_t len)
{
  while (len--)
  {
    SD_SPI_TxRx(*buff++);
  }
}

static uint8_t SD_WaitReady(void)
{
  uint8_t res;
  uint32_t start = HAL_GetTick();

  do
  {
    res = SD_SPI_TxRx(0xFF);
    if (res == 0xFF)
    {
      return 1;
    }
  } while ((HAL_GetTick() - start) < SD_TIMEOUT_MS);

  return 0;
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
  uint8_t n;
  uint8_t res;

  if (cmd & 0x80)
  {
    cmd &= 0x7F;
    res = SD_SendCmd(CMD55, 0);

    if (res > 1)
    {
      return res;
    }
  }

  SD_Deselect();
  SD_Select();

  if (!SD_WaitReady())
  {
    return 0xFF;
  }

  SD_SPI_TxRx(0x40 | cmd);
  SD_SPI_TxRx((uint8_t)(arg >> 24));
  SD_SPI_TxRx((uint8_t)(arg >> 16));
  SD_SPI_TxRx((uint8_t)(arg >> 8));
  SD_SPI_TxRx((uint8_t)arg);

  if (cmd == CMD0)
  {
    SD_SPI_TxRx(0x95);
  }
  else if (cmd == CMD8)
  {
    SD_SPI_TxRx(0x87);
  }
  else
  {
    SD_SPI_TxRx(0x01);
  }

  if (cmd == CMD12)
  {
    SD_SPI_TxRx(0xFF);
  }

  n = 10;
  do
  {
    res = SD_SPI_TxRx(0xFF);
  } while ((res & 0x80) && --n);

  return res;
}

static uint8_t SD_RxDataBlock(uint8_t *buff, uint32_t len)
{
  uint8_t token;
  uint32_t start = HAL_GetTick();

  do
  {
    token = SD_SPI_TxRx(0xFF);
    if (token == 0xFE)
    {
      break;
    }
  } while ((HAL_GetTick() - start) < SD_TIMEOUT_MS);

  if (token != 0xFE)
  {
    return 0;
  }

  SD_SPI_RxBuf(buff, len);

  SD_SPI_TxRx(0xFF);
  SD_SPI_TxRx(0xFF);

  return 1;
}

static uint8_t SD_TxDataBlock(const uint8_t *buff, uint8_t token)
{
  uint8_t resp;

  if (!SD_WaitReady())
  {
    return 0;
  }

  SD_SPI_TxRx(token);

  if (token != 0xFD)
  {
    SD_SPI_TxBuf(buff, 512);

    SD_SPI_TxRx(0xFF);
    SD_SPI_TxRx(0xFF);

    resp = SD_SPI_TxRx(0xFF);

    if ((resp & 0x1F) != 0x05)
    {
      return 0;
    }
  }

  return 1;
}

static DWORD SD_GetSectorCount(void)
{
  uint8_t csd[16];
  DWORD csize;
  DWORD sector_count = 0;

  if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
  {
    if ((csd[0] >> 6) == 1)
    {
      csize = ((DWORD)(csd[7] & 0x3F) << 16) |
              ((DWORD)csd[8] << 8) |
              csd[9];

      sector_count = (csize + 1) * 1024;
    }
    else
    {
      uint8_t n;
      csize = ((DWORD)(csd[6] & 0x03) << 10) |
              ((DWORD)csd[7] << 2) |
              ((csd[8] & 0xC0) >> 6);

      n = ((csd[5] & 0x0F) +
           ((csd[10] & 0x80) >> 7) +
           ((csd[9] & 0x03) << 1) + 2);

      sector_count = (csize + 1) << (n - 9);
    }
  }

  SD_Deselect();
  return sector_count;
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
	  uint8_t cmd;
	  uint8_t ty = 0;
	  uint8_t ocr[4];
	  uint32_t start;

	  if (pdrv != 0)
	  {
	    return STA_NOINIT;
	  }

	  HAL_GPIO_WritePin(SDC_CS_GPIO_Port, SDC_CS_Pin, GPIO_PIN_SET);

	  for (n = 0; n < 10; n++)
	  {
	    SD_SPI_TxRx(0xFF);
	  }

	  if (SD_SendCmd(CMD0, 0) == 1)
	  {
	    if (SD_SendCmd(CMD8, 0x1AA) == 1)
	    {
	      for (n = 0; n < 4; n++)
	      {
	        ocr[n] = SD_SPI_TxRx(0xFF);
	      }

	      if ((ocr[2] == 0x01) && (ocr[3] == 0xAA))
	      {
	        start = HAL_GetTick();

	        while ((HAL_GetTick() - start) < SD_TIMEOUT_MS)
	        {
	          if (SD_SendCmd(ACMD41, 0x40000000) == 0)
	          {
	            break;
	          }
	        }

	        if ((HAL_GetTick() - start) < SD_TIMEOUT_MS)
	        {
	          if (SD_SendCmd(CMD58, 0) == 0)
	          {
	            for (n = 0; n < 4; n++)
	            {
	              ocr[n] = SD_SPI_TxRx(0xFF);
	            }

	            ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
	          }
	        }
	      }
	    }
	    else
	    {
	      if (SD_SendCmd(ACMD41, 0) <= 1)
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

	      while ((HAL_GetTick() - start) < SD_TIMEOUT_MS)
	      {
	        if (SD_SendCmd(cmd, 0) == 0)
	        {
	          break;
	        }
	      }

	      if (((HAL_GetTick() - start) >= SD_TIMEOUT_MS) ||
	          (SD_SendCmd(CMD16, 512) != 0))
	      {
	        ty = 0;
	      }
	    }
	  }

	  CardType = ty;
	  SD_Deselect();

	  if (ty)
	  {
	    Stat &= ~STA_NOINIT;
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
	 if (pdrv != 0)
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
	  if (pdrv || !count)
	  {
	    return RES_PARERR;
	  }

	  if (Stat & STA_NOINIT)
	  {
	    return RES_NOTRDY;
	  }

	  if (!(CardType & CT_BLOCK))
	  {
	    sector *= 512;
	  }

	  if (count == 1)
	  {
	    if ((SD_SendCmd(CMD17, sector) == 0) &&
	        SD_RxDataBlock(buff, 512))
	    {
	      count = 0;
	    }
	  }
	  else
	  {
	    if (SD_SendCmd(CMD18, sector) == 0)
	    {
	      do
	      {
	        if (!SD_RxDataBlock(buff, 512))
	        {
	          break;
	        }

	        buff += 512;
	      } while (--count);

	      SD_SendCmd(CMD12, 0);
	    }
	  }

	  SD_Deselect();

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
  /* USER CODE HERE */
	if (pdrv || !count)
	  {
	    return RES_PARERR;
	  }

	  if (Stat & STA_NOINIT)
	  {
	    return RES_NOTRDY;
	  }

	  if (!(CardType & CT_BLOCK))
	  {
	    sector *= 512;
	  }

	  if (count == 1)
	  {
	    if ((SD_SendCmd(CMD24, sector) == 0) &&
	        SD_TxDataBlock(buff, 0xFE))
	    {
	      count = 0;
	    }
	  }
	  else
	  {
	    if (CardType & CT_SD1 || CardType & CT_SD2)
	    {
	      SD_SendCmd(ACMD41, count);
	    }

	    if (SD_SendCmd(CMD25, sector) == 0)
	    {
	      do
	      {
	        if (!SD_TxDataBlock(buff, 0xFC))
	        {
	          break;
	        }

	        buff += 512;
	      } while (--count);

	      if (!SD_TxDataBlock(0, 0xFD))
	      {
	        count = 1;
	      }
	    }
	  }

	  SD_Deselect();

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

	  if (pdrv)
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
	      SD_Select();

	      if (SD_WaitReady())
	      {
	        res = RES_OK;
	      }

	      SD_Deselect();
	      break;

	    case GET_SECTOR_COUNT:
	      *(DWORD *)buff = SD_GetSectorCount();
	      res = RES_OK;
	      break;

	    case GET_SECTOR_SIZE:
	      *(WORD *)buff = 512;
	      res = RES_OK;
	      break;

	    case GET_BLOCK_SIZE:
	      *(DWORD *)buff = 128;
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

