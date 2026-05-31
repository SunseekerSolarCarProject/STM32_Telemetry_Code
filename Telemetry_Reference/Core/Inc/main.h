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
#define BME_CS_Pin GPIO_PIN_6
#define BME_CS_GPIO_Port GPIOF
#define SDC_CS_Pin GPIO_PIN_1
#define SDC_CS_GPIO_Port GPIOC
#define GPS_RST_Pin GPIO_PIN_3
#define GPS_RST_GPIO_Port GPIOA
#define GPS_CS_Pin GPIO_PIN_4
#define GPS_CS_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOD
#define LED5_Pin GPIO_PIN_13
#define LED5_GPIO_Port GPIOD
#define LED6_Pin GPIO_PIN_14
#define LED6_GPIO_Port GPIOD
#define LED7_Pin GPIO_PIN_15
#define LED7_GPIO_Port GPIOD
#define ESP32_CS_Pin GPIO_PIN_0
#define ESP32_CS_GPIO_Port GPIOD
#define ESP32_Ready_Pin GPIO_PIN_1
#define ESP32_Ready_GPIO_Port GPIOD
#define LED_Green_Pin GPIO_PIN_13
#define LED_Green_GPIO_Port GPIOG
#define LED_Red_Pin GPIO_PIN_14
#define LED_Red_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
