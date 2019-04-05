/*
 * motors.h
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */

#ifndef BSP_MOTORS_H_
#define BSP_MOTORS_H_

#define PWM_MIN 0
#define PWM_MAX 1023

void initMotors();
void setMotorSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

#endif /* BSP_MOTORS_H_ */
