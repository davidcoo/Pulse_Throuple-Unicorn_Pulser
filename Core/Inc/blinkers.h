/*
 * blinkers.h
 *
 *  Created on: Oct 22, 2022
 *      Author: DavidC
 */

#ifndef INC_BLINKERS_H_
#define INC_BLINKERS_H_


#include "stm32f1xx_hal.h"
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void set_blinkers(uint8_t hazard, uint8_t right, uint8_t left);
void blinkers_init();
void left_blinker_on();
void left_blinker_off();
void right_blinker_on();
void right_blinker_off();
void hazards_on();
void hazards_off();


#endif /* INC_BLINKERS_H_ */
