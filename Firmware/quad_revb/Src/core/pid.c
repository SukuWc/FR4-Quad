/*
 * pid.c
 *
 *  Created on: 18 Mar 2019
 *      Author: danim
 */
#include "core/pid.h"
#include "math.h"

void initializePIDStruct(PIDStruct* pid, float p, float d, float i, float clamp_min, float clamp_max){
	pid->last_error = INFINITY;
	pid->p = p;
	pid->d = d;
	pid->i = i;
	pid->integral_part = 0;
	pid->clamp_min = clamp_min;
	pid->clamp_max = clamp_max;
}
float calculatePIDLoop(PIDStruct* pid, float inError){
	float pPart = pid->p * inError;

	float dPart = 0;
	if (pid->last_error != INFINITY){
		dPart = (inError - pid->last_error) * pid->d;
	}

	pid->integral_part += pid->i * inError;
	if (pid->integral_part > pid->clamp_max){
		pid->integral_part = pid->clamp_max;
	} else if (pid->integral_part < pid->clamp_min){
		pid->integral_part = pid->clamp_min;
	}

	pid->last_error = inError;
	return pPart + dPart + pid->integral_part;
}
