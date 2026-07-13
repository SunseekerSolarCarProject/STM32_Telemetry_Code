/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESP32_CS_Pin GPIO_PIN_3
#define ESP32_CS_GPIO_Port GPIOE
#define ESP32_Ready_Pin GPIO_PIN_4
#define ESP32_Ready_GPIO_Port GPIOE
#define SD_Detect_Pin GPIO_PIN_13
#define SD_Detect_GPIO_Port GPIOC
#define IMU_INT_Pin GPIO_PIN_2
#define IMU_INT_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_3
#define IMU_CS_GPIO_Port GPIOA
#define BME_CS_Pin GPIO_PIN_4
#define BME_CS_GPIO_Port GPIOA
#define GPS_RST_Pin GPIO_PIN_4
#define GPS_RST_GPIO_Port GPIOC
#define GPS_CS_Pin GPIO_PIN_5
#define GPS_CS_GPIO_Port GPIOC
#define Green_LED11_Pin GPIO_PIN_8
#define Green_LED11_GPIO_Port GPIOD
#define Red_LED10_Pin GPIO_PIN_9
#define Red_LED10_GPIO_Port GPIOD
#define Yellow_LED9_Pin GPIO_PIN_10
#define Yellow_LED9_GPIO_Port GPIOD
#define Yellow_LED8_Pin GPIO_PIN_11
#define Yellow_LED8_GPIO_Port GPIOD
#define Yellow_LED7_Pin GPIO_PIN_12
#define Yellow_LED7_GPIO_Port GPIOD
#define Yellow_LED6_Pin GPIO_PIN_13
#define Yellow_LED6_GPIO_Port GPIOD
#define Yellow_LED5_Pin GPIO_PIN_14
#define Yellow_LED5_GPIO_Port GPIOD
#define GPS_INT_Pin GPIO_PIN_6
#define GPS_INT_GPIO_Port GPIOC
#define GPS_PPS_Pin GPIO_PIN_7
#define GPS_PPS_GPIO_Port GPIOC
#define GPS_CSC8_Pin GPIO_PIN_8
#define GPS_CSC8_GPIO_Port GPIOC
#define RTC_INT_Pin GPIO_PIN_9
#define RTC_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/*
 * SD card chip-select is physically wired to PC8.  Keep this guarded alias in
 * the user section so the FatFS driver still compiles if CubeMX is regenerated
 * from an older project view that gives PC8 a different label.
 */
#ifndef SD_CS_Pin
#define SD_CS_Pin GPIO_PIN_8
#define SD_CS_GPIO_Port GPIOC
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
