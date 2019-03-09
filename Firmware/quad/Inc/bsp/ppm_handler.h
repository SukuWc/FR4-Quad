/*
 * ppm_handler.h
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */

#ifndef BSP_PPM_HANDLER_H_
#define BSP_PPM_HANDLER_H_

#include "main.h"

void timerCallbackFromISR(uint16_t timeMs);

extern volatile uint16_t ppm_values[8];
extern volatile uint8_t ppm_valid;

#endif /* BSP_PPM_HANDLER_H_ */
