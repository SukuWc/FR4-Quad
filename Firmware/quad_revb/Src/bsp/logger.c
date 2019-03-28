/*
 * logger.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "main.h"
#include "core/system_logger.h"

extern UART_HandleTypeDef huart1;

void sendMessage(uint8_t *buffer, uint16_t bufferLength){
	HAL_UART_Transmit_DMA(&huart1, buffer, bufferLength);
}

void messageSentFromISR(){
	messageFinishedFromISR();
}
