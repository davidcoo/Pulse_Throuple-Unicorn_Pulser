/*
 * blinkers.c
 *
 *  Created on: Oct 22, 2022
 *      Author: DavidC
 */

#include <blinkers.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


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

TIM_HandleTypeDef htim3;

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

uint8_t hazard_state = 0;
uint8_t left_state  = 0;
uint8_t right_state = 0;



void blinkers_init(){
	MX_TIM3_Init();
}

void set_blinkers(uint8_t hazard, uint8_t right, uint8_t left){
	if (hazard == 1 && hazard_state == 0){
		hazards_on();
	}
	else if (hazard == 0 && hazard_state == 1){
		hazards_off();
	}
	if (hazard == 0){
		if (left == 1 && left_state == 0){
			left_blinker_on();
		}
		else if (left == 0 && left_state == 1){
			left_blinker_off();
		}
		if (right == 1 && right_state == 0){
			right_blinker_on();
		}
		else if (right == 0 && right_state == 1){
			right_blinker_off();
		}
	}
}


void left_blinker_on(){
	TIM3->CCR2 = 500;
	TIM3->CNT = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	left_state = 1;
}

void left_blinker_off(){
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	left_state = 0;
}

void right_blinker_on(){
	TIM3->CCR4 = 500;
	TIM3->CNT = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	right_state = 1;
}

void right_blinker_off(){
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
	right_state = 0;
}


void hazards_on(){
	left_blinker_on();
	right_blinker_on();
	hazard_state = 1;
}

void hazards_off(){
	left_blinker_off();
	right_blinker_off();
	hazard_state = 0;
}

