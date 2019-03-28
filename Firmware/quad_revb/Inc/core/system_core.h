/*
 * system_core.h
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */

#ifndef CORE_SYSTEM_CORE_H_
#define CORE_SYSTEM_CORE_H_

#include "cmsis_os.h"

/// From main.c
extern osThreadId systemCoreTaskHandle;
extern osTimerId controlTimerHandle;


#define CONTROL_LOOP_PERIOD_MS 2

enum SystemCoreNotificationFlags_T {
	EVENT_CONTROLLER_UPDATE = 0
};

#define sNotifySystemCore(event) xTaskNotify(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits)
#define sNotifySystemCoreFromISR(event, pxHigherPriorityTaskWoken) xTaskNotifyFromISR(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits, pxHigherPriorityTaskWoken)
#define vSystemTimeMs() (configTICK_RATE_HZ / 1000.0f * xTaskGetTickCount())

#endif /* CORE_SYSTEM_CORE_H_ */
