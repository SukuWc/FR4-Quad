/*
 * pid.h
 *
 *  Created on: 18 Mar 2019
 *      Author: danim
 */

#ifndef CORE_PID_H_
#define CORE_PID_H_
#include "main.h"

typedef struct pid_t{
	float p;
	float d;
	float i;
	float clamp_min;
	float clamp_max;
	float last_error;
	float integral_part;
	uint8_t first_value;
} PIDStruct;

void initializePIDStruct(PIDStruct* pid, float p, float d, float i, float clamp_min, float clamp_max);
float calculatePIDLoop(PIDStruct* pid, float inValue);

#endif /* CORE_PID_H_ */
