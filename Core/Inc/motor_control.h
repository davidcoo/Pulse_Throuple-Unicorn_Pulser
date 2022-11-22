/*
 * motor_control.h
 *
 *  Created on: Oct 17, 2022
 *      Author: DavidC
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

typedef enum {
	FORWARD = 1,
	BACKWARD = 2,
	STOP = 3,
	BRAKE = 4
} motor_direction;


void motor_init();
void set_motor_direction(motor_direction direction);
void set_drive_speed(uint32_t percent_speed);
void set_reverse_speed(uint32_t percent_speed);
void set_brake_speed(uint32_t percent_speed);


#endif /* INC_MOTOR_CONTROL_H_ */
