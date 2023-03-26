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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01mbal.h"
#include "mpu6050.h"
#include "PID control.h"
#include "math.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_ADDR 0xD0
#define Run_LRFB 5
#define Run_When_Start_Value 1300
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define CSN_pin_Pin GPIO_PIN_1
#define CSN_pin_GPIO_Port GPIOA
#define CE_pin_Pin GPIO_PIN_4
#define CE_pin_GPIO_Port GPIOA
#define irq_nrf_Pin GPIO_PIN_3
#define irq_nrf_GPIO_Port GPIOB
void set_val_for_nfr24(NRF24L01_config_TypeDef* nfr24_dummy);
void quad_up();
void quad_down();
void quad_left();
void quad_front();
void quad_behind();
void quad_stop();
void quad_start();
void quad_reset();
void calibrate_gyro();
void correct_data_and_calibrate_3truc();
void calculate_agl_roll_pitch();
void calculate_setpoint_pid();
void check_looptime();
void hacanh_quad();
void read_hc05();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CSN_pin_Pin GPIO_PIN_1
#define CSN_pin_GPIO_Port GPIOA
#define CE_pin_Pin GPIO_PIN_4
#define CE_pin_GPIO_Port GPIOA
#define hc_trigger_pin_Pin GPIO_PIN_0
#define hc_trigger_pin_GPIO_Port GPIOB
#define irq_nrf_Pin GPIO_PIN_3
#define irq_nrf_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
