/*
 * motor_control.c
 *
 *  Created on: Oct 17, 2022
 *      Author: DavidC
 */

#include <motor_control.h>
#include "stm32f1xx_hal.h"

#define MOTOR_IN1_Pin GPIO_PIN_2
#define MOTOR_IN1_GPIO_Port GPIOB
#define MOTOR_IN2_Pin GPIO_PIN_10
#define MOTOR_IN2_GPIO_Port GPIOB


static TIM_HandleTypeDef htim2;
static void MX_TIM2_Init(void);
static void Error_Handler(void);

void motor_init(){
	MX_TIM2_Init();
}



// Sets the direction of the motor
void set_motor_direction(motor_direction direction){
	// Mosfet logic means output is filled (set => low output, reset => high output)
	switch(direction){
		case FORWARD:
			HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
			break;
		case BACKWARD:
			HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
			break;
		case BRAKE:
			HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
			break;
		default:
			(void)direction;
	}
}

// Set Speed of the motor
void set_motor_speed(uint32_t percent_speed){
	// Motor speed PWM 50 Hz 100 Ticks at 5000 Hz
	TIM2->CCR4 = 100-percent_speed;
      	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

// Set Speed of motor in forward direction
void set_drive_speed(uint32_t percent_speed){
	set_motor_direction(FORWARD);
	set_motor_speed(percent_speed);
}

// Set Speed of motor in reverse direction
void set_reverse_speed(uint32_t percent_speed){
	set_motor_direction(BACKWARD);
	set_motor_speed(percent_speed);
}

// Set Speed of motor in brake direction
void set_brake_speed(uint32_t percent_speed){
	set_motor_direction(BRAKE);
	set_motor_speed(percent_speed);
}





/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

static void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


