/*
 * servo_control.h
 *
 *  Created on: Oct 17, 2022
 *      Author: DavidC
 */

#ifndef INC_SERVO_CONTROL_H_
#define INC_SERVO_CONTROL_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void servo_init();
void set_servo_pos(uint32_t pos);


#endif /* INC_MOTOR_CONTROL_H_ */
