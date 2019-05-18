/*
 * ibus_handler.c
 *
 *  Created on: 2019. ápr. 5.
 *      Author: danim
 */

#include "usart.h"
#include "core/system_core.h"

enum IbusHandlerState{
	IBUS_INITIALIZING, IBUS_RECEIVING, IBUS_ERROR
};

volatile enum IbusHandlerState state;
volatile static uint8_t buffer[32] = {0};

volatile uint16_t channel_values[14] = {0};

#define IBUS_MSG_LENGTH 32

void initIbusHandler(){
	state = IBUS_INITIALIZING;
	HAL_UART_Receive_DMA(&huart2, buffer, IBUS_MSG_LENGTH);
}

void onUartReceiveFinished(){
	if (buffer[0] == 0x20 && buffer[1] == 0x40){
		for (int i = 0; i < 14; i++){
			channel_values[i] = (buffer[i * 2 + 3] << 8) | buffer[i * 2 + 2];
		}
		state = IBUS_RECEIVING;
		buffer[0] = 0;
		BaseType_t xTaskWokenByReceive = pdFALSE;
		sNotifySystemCoreFromISR(EVENT_CORE_JOYSTICK_UPDATED, &xTaskWokenByReceive);
		portEND_SWITCHING_ISR(xTaskWokenByReceive)
	} else {
		state = IBUS_INITIALIZING;
	}
	HAL_UART_Receive_DMA(&huart2, buffer, IBUS_MSG_LENGTH);
	/*if (state == IBUS_INITIALIZING){
		if (buffer[0] == 0x20){
			state = IBUS_RECEIVING;
			HAL_UART_Receive_IT(&huart2, &(buffer[1]), IBUS_MSG_LENGTH - 1);
		} else {
			HAL_UART_Receive_IT(&huart2, buffer, 1);
		}
	} else if (state == IBUS_RECEIVING){
		if (buffer[0] != 0x20 || buffer[1] != 0x40){
			state = IBUS_INITIALIZING;
			HAL_UART_Receive_IT(&huart2, buffer, 1);
		} else {
			HAL_UART_Receive_IT(&huart2, buffer, IBUS_MSG_LENGTH);
		}
	}*/
}

uint8_t receiverValid(){
	return state == IBUS_RECEIVING;
}
