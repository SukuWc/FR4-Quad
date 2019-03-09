/*
 * ppm_handler.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "main.h"

volatile uint16_t ppm_values[8] = {0};
volatile uint8_t ppm_valid = 0;

static volatile uint8_t index = 0;
static volatile uint16_t rawValues[8] = {0};

void timerCallbackFromISR(uint16_t timeMs){
	if (timeMs > 5000){
		if (index != 8){
			ppm_valid = 0;
		} else {
			for (uint8_t i = 0; i < 8; i++){
				uint16_t value = rawValues[i];
				if (value < 1000){
					value = 1000;
				}
				if (value > 2000){
					value = 2000;
				}
				ppm_values[i] = value;
			}
			ppm_valid = 1;
		}
		index = 0;
	} else {
		if (index < 8) {
			rawValues[index++] = timeMs;
		} else {
			ppm_valid = 0;
		}
	}
}
