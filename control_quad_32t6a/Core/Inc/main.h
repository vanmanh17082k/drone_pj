/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "nRF24L01.h"
#include "nrf24l01mbal.h"
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
#define CSN_pin_Pin GPIO_PIN_4
#define CSN_pin_GPIO_Port GPIOA
#define CE_pin_Pin GPIO_PIN_8
#define CE_pin_GPIO_Port GPIOA
#define up_pin_Pin GPIO_PIN_9
#define up_pin_GPIO_Port GPIOA
#define up_pin_EXTI_IRQn EXTI9_5_IRQn
#define down_pin_Pin GPIO_PIN_10
#define down_pin_GPIO_Port GPIOA
#define down_pin_EXTI_IRQn EXTI15_10_IRQn
#define setting_pin_0_Pin GPIO_PIN_11
#define setting_pin_0_GPIO_Port GPIOA
#define setting_pin_0_EXTI_IRQn EXTI15_10_IRQn
#define setting_pin_1_Pin GPIO_PIN_12
#define setting_pin_1_GPIO_Port GPIOA
#define setting_pin_1_EXTI_IRQn EXTI15_10_IRQn
#define setting_pin_2_Pin GPIO_PIN_15
#define setting_pin_2_GPIO_Port GPIOA
#define setting_pin_2_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */
typedef struct {
	uint8_t tx_quad_data[1];
	uint8_t old_tx_quad_data[1];
	uint16_t dataread_sw[2];
} data_send;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
