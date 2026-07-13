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
#define SD_Detect_Pin GPIO_PIN_13
#define SD_Detect_GPIO_Port GPIOC
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
#define SD_CS_Pin GPIO_PIN_8
#define SD_CS_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
