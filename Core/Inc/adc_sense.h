/*
 * adc_sense.h
 */

#include <stdint.h>
#include "stm32f1xx_hal.h"

void current_sense_init();
uint16_t current_sense_read();

void pot_sense_init();
uint16_t pot_sense_read();
