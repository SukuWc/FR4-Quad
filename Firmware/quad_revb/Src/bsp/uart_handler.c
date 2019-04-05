/*
 * uart_handler.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */

#include "bsp/logger.h"
#include "bsp/ibus_handler.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart1){
		messageSentFromISR();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2){
		onUartReceiveFinished();
	}
}

HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2){
		onUartReceiveFinished();
	}
}
