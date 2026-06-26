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
#include "stm32g4xx_hal.h"

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
#define TEST_IO1_Pin GPIO_PIN_13
#define TEST_IO1_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOC
#define TEST_IO_Pin GPIO_PIN_10
#define TEST_IO_GPIO_Port GPIOG
#define ADC1_IN1_BUS_Pin GPIO_PIN_0
#define ADC1_IN1_BUS_GPIO_Port GPIOA
#define MOTOR_U_Pin GPIO_PIN_1
#define MOTOR_U_GPIO_Port GPIOA
#define MOTOR_V_Pin GPIO_PIN_2
#define MOTOR_V_GPIO_Port GPIOA
#define MOTOR_W_Pin GPIO_PIN_3
#define MOTOR_W_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define ADC1_IN15_NTC1_Pin GPIO_PIN_0
#define ADC1_IN15_NTC1_GPIO_Port GPIOB
#define ADC1_IN12_NTC2_Pin GPIO_PIN_1
#define ADC1_IN12_NTC2_GPIO_Port GPIOB
#define SPI1_CAL_Pin GPIO_PIN_2
#define SPI1_CAL_GPIO_Port GPIOB
#define MOTOR_FAULT_Pin GPIO_PIN_10
#define MOTOR_FAULT_GPIO_Port GPIOB
#define MOTOR_EN_Pin GPIO_PIN_12
#define MOTOR_EN_GPIO_Port GPIOB
#define SPI3_NSS_Pin GPIO_PIN_5
#define SPI3_NSS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
