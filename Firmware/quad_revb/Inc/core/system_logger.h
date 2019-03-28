/*
 * system_logger.h
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */

#ifndef CORE_SYSTEM_LOGGER_H_
#define CORE_SYSTEM_LOGGER_H_

#include "cmsis_os.h"

/// From main.c
extern osThreadId systemLoggerTaskHandle;

enum LoggerNotificationFlags_T {
	ACCELERATION_SENSOR_UPDATED = 0
};

#define sNotifyLogger(event) xTaskNotify(systemLoggerTaskHandle, (1 << (31 - (int)event)), eSetBits)
#define sNotifyLoggerFromISR(event, pxHigherPriorityTaskWoken) xTaskNotifyFromISR(systemLoggerTaskHandle, (1 << (31 - (int)event)), eSetBits, pxHigherPriorityTaskWoken)

void messageFinishedFromISR();

#endif /* CORE_SYSTEM_LOGGER_H_ */
