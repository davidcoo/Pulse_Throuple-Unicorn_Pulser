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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	uint8_t brake;
	uint8_t throttle;
}pi_motor_command;

typedef struct {
	uint16_t pot_reading;
	uint16_t current_reading;
}adc_msg;

typedef enum {
	ERROR_BUTTON = 1,
	ERROR_BUTTON_RELEASED = 2,
	ERROR_HB = 3,
	NORMAL = 4,
	NORMAL_PUSHED = 5,
} zone_state_e;



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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define MCU_IND_Pin GPIO_PIN_2
#define MCU_IND_GPIO_Port GPIOA
#define I_SENSE_Pin GPIO_PIN_4
#define I_SENSE_GPIO_Port GPIOA
#define POTENTIOMETER_Pin GPIO_PIN_5
#define POTENTIOMETER_GPIO_Port GPIOA
#define LEFT_BLINKER_Pin GPIO_PIN_7
#define LEFT_BLINKER_GPIO_Port GPIOA
#define RIGHT_BLINKER_Pin GPIO_PIN_1
#define RIGHT_BLINKER_GPIO_Port GPIOB
#define MOTOR_IN2_Pin GPIO_PIN_2
#define MOTOR_IN2_GPIO_Port GPIOB
#define MOTOR_IN1_Pin GPIO_PIN_10
#define MOTOR_IN1_GPIO_Port GPIOB
#define MOTOR_EN_PWM_Pin GPIO_PIN_11
#define MOTOR_EN_PWM_GPIO_Port GPIOB
#define GPIO_1_Pin GPIO_PIN_12
#define GPIO_1_GPIO_Port GPIOB
#define GPIO_2_Pin GPIO_PIN_13
#define GPIO_2_GPIO_Port GPIOB
#define GPIO_3_Pin GPIO_PIN_14
#define GPIO_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
