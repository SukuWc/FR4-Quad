/*
 * init.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "core/init.h"
#include "core/system_core.h"

void initCore(){
	xTimerChangePeriod(positionUpdateTimerHandle, pdMS_TO_TICKS(POSITION_LOOP_PERIOD_MS), 0);
	xTimerChangePeriod(controlTimerHandle, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS), 0);
	xTimerChangePeriod(sendLogTimerHandle, pdMS_TO_TICKS(LOGGER_LOOP_PERIOD_MS), 0);
}
