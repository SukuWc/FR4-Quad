/*
 * tim_handler.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "bsp/ppm_handler.h"
#include "main.h"

volatile uint8_t inProcess = 0;
volatile uint8_t errorState = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM1) //check if the interrupt comes from TIM1
	{
    	if (inProcess){
    		errorState = 1;
    	}
    	__HAL_TIM_SET_COUNTER(htim, 0);
    	uint16_t value = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
    	timerCallbackFromISR(value);
	}
}
