/*
 * throuple_can.h
 *
 *  Created on: Nov 22, 2022
 *      Author: David Cooper
 */

#ifndef INC_THROUPLE_CAN_H_
#define INC_THROUPLE_CAN_H_


#include "stm32f1xx_hal.h"

void can_init();


typedef struct {
	uint16_t id;
	uint8_t msg[8];
} can_msg_t;

extern QueueHandle_t xQueueCANRx;





#endif /* INC_THROUPLE_CAN_H_ */
