/*
 * motors.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "main.h"
#include "bsp/motors.h"
#include "tim.h"
#define CLAMP_PWM(pwm) (((pwm) < PWM_MIN) ? (PWM_MIN) : ((pwm > PWM_MAX) ? (PWM_MAX) : (pwm)))

void initMotors(){
	setMotorSpeed(0, 0, 0, 0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void setMotorSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4){
    /*htim2.Instance->CCR1 = CLAMP_PWM(PWM_MAX - m3);
    htim3.Instance->CCR2 = CLAMP_PWM(PWM_MAX - m2);
    htim3.Instance->CCR4 = CLAMP_PWM(PWM_MAX - m1);
    htim4.Instance->CCR4 = CLAMP_PWM(PWM_MAX - m4);*/
	htim3.Instance->CCR1 = CLAMP_PWM(PWM_MAX - m1);
	htim3.Instance->CCR2 = CLAMP_PWM(PWM_MAX - m2);
	htim3.Instance->CCR3 = CLAMP_PWM(PWM_MAX - m3);
	htim3.Instance->CCR4 = CLAMP_PWM(PWM_MAX - m4);
	htim4.Instance->CCR1 = CLAMP_PWM(PWM_MAX - m1);
	htim4.Instance->CCR2 = CLAMP_PWM(PWM_MAX - m2);
	htim4.Instance->CCR3 = CLAMP_PWM(PWM_MAX - m3);
	htim4.Instance->CCR4 = CLAMP_PWM(PWM_MAX - m4);
}
