/*
 * motors.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "main.h"
#include "bsp/motors.h"
#define CLAMP_PWM(pwm) (((pwm) < PWM_MIN) ? (PWM_MIN) : ((pwm > PWM_MAX) ? (PWM_MAX) : (pwm)))

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


void setMotorSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4){
   /* htim2.Instance->CCR1 = CLAMP_PWM(PWM_MAX - m3);
    htim3.Instance->CCR2 = CLAMP_PWM(PWM_MAX - m2);
    htim3.Instance->CCR4 = CLAMP_PWM(PWM_MAX - m1);
    htim4.Instance->CCR4 = CLAMP_PWM(PWM_MAX - m4);*/
}
