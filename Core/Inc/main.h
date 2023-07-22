/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
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
#define TDC7200_EN_Pin GPIO_PIN_0
#define TDC7200_EN_GPIO_Port GPIOF
#define BUTTON1_Pin GPIO_PIN_7
#define BUTTON1_GPIO_Port GPIOA
#define PULSE_SIG_Pin GPIO_PIN_0
#define PULSE_SIG_GPIO_Port GPIOB
#define TDC7200_INT_Pin GPIO_PIN_8
#define TDC7200_INT_GPIO_Port GPIOA
#define ADXL345_Pin GPIO_PIN_9
#define ADXL345_GPIO_Port GPIOA
#define TDC7200_CS_Pin GPIO_PIN_11
#define TDC7200_CS_GPIO_Port GPIOA
#define BUTTON2_Pin GPIO_PIN_12
#define BUTTON2_GPIO_Port GPIOA
#define TDC7200_TRIG_Pin GPIO_PIN_6
#define TDC7200_TRIG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
