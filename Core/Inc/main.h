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
#define Pin_PullDown_HallEncoderB2_Pin GPIO_PIN_5
#define Pin_PullDown_HallEncoderB2_GPIO_Port GPIOE
#define Pin_PullDown_HallEncoderB3_Pin GPIO_PIN_6
#define Pin_PullDown_HallEncoderB3_GPIO_Port GPIOA
#define Pin_PullDown_HallEncoderB4_Pin GPIO_PIN_7
#define Pin_PullDown_HallEncoderB4_GPIO_Port GPIOA
#define Pin_Pushpull_MotorDirectionA1_Pin GPIO_PIN_0
#define Pin_Pushpull_MotorDirectionA1_GPIO_Port GPIOD
#define Pin_Pushpull_MotorDirectionB1_Pin GPIO_PIN_1
#define Pin_Pushpull_MotorDirectionB1_GPIO_Port GPIOD
#define Pin_Pushpull_MotorDirectionA2_Pin GPIO_PIN_2
#define Pin_Pushpull_MotorDirectionA2_GPIO_Port GPIOD
#define Pin_Pushpull_MotorDirectionB2_Pin GPIO_PIN_3
#define Pin_Pushpull_MotorDirectionB2_GPIO_Port GPIOD
#define Pin_Pushpull_MotorDirectionA3_Pin GPIO_PIN_4
#define Pin_Pushpull_MotorDirectionA3_GPIO_Port GPIOD
#define Pin_Pushpull_MotorDirectionB3_Pin GPIO_PIN_5
#define Pin_Pushpull_MotorDirectionB3_GPIO_Port GPIOD
#define Pin_Pushpull_MotorDirectionA4_Pin GPIO_PIN_6
#define Pin_Pushpull_MotorDirectionA4_GPIO_Port GPIOD
#define Pin_Pushpull_MotorDirectionB4_Pin GPIO_PIN_7
#define Pin_Pushpull_MotorDirectionB4_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
