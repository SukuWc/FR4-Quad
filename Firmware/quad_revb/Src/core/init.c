/*
 * init.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "core/init.h"
#include "core/system_core.h"

void initCore(){
	xTimerChangePeriod(controlTimerHandle, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS), 0);
}
