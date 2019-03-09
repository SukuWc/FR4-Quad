/*
 * system_core.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "limits.h"
#include "core/system_core.h"
#include "core/system_logger.h"
#include "bsp/ppm_handler.h"
#include "bsp/motors.h"
#include "math.h"

void ControlLoop(const void* argument){
	sNotifySystemCore(EVENT_CONTROLLER_UPDATE);
}

int16_t ax, ay, az, rotx, roty, rotz;

uint32_t lastMotorValid = 0;
volatile float pitch = 0;
volatile float roll = 0;
volatile float yawn = 0;

int16_t accel_x, accel_y, accel_z, gyro_ro, gyro_pi, gyro_ya;
int16_t user_ro, user_pi, user_ya;
int16_t user_th;
int32_t pwm_1, pwm_2, pwm_3, pwm_4;

void core_updateController(){
	//sNotifyLogger(ACCELERATION_SENSOR_UPDATED);


	uint32_t currentTick = HAL_GetTick();
	if (ppm_valid){
		ppm_valid = 0;
		lastMotorValid = xTaskGetTickCount();
	}

	//MX_RESET_I2C();
	mpu9250_getMotion6(&ax, &ay, &az, &rotx, &roty, &rotz);

	accel_x = -ax;
	accel_y = ay;
	accel_z = az;

	gyro_pi = -roty;
	gyro_ro = rotx;
	gyro_ya = rotz;


#define GYRO_SENS 16.384
#define M_PI 3.14159265359
#define ALPHA 0.998
#define dt 0.002
	pitch += (float)(gyro_pi) / GYRO_SENS * dt;
	roll +=  gyro_ro/ GYRO_SENS * dt;
	yawn += gyro_ya / GYRO_SENS * dt;

	int forceMagnitudeApprox = abs(accel_x) + abs(accel_y) + abs(accel_z);
	if (forceMagnitudeApprox > 1024 && forceMagnitudeApprox < 4096){
		float pitchAcc = atan2f((float)accel_x, (float)accel_z) * 180 / M_PI;
		pitch = pitch * ALPHA + pitchAcc * (1 - ALPHA);

		float rollAcc = atan2f((float)accel_y, (float)accel_z) * 180 / M_PI;
		roll = roll * ALPHA + rollAcc * (1 - ALPHA);
	}

	sNotifyLogger(ACCELERATION_SENSOR_UPDATED);

	int16_t userInputMin = 1000;
	int16_t userInputMax = 2000;
	int16_t userInputAvg = (userInputMin + userInputMax) / 2;
	int16_t inOutRatio = 4;
	int16_t userOutputMin = -256;
	int16_t userOutputMax = 256;
	user_th = (ppm_values[2] - userInputMin) / inOutRatio;
	if (user_th < 0) user_th = 0; else if (user_th > userOutputMax*2) user_th = userOutputMax*2;
	user_ro = -((ppm_values[0] - userInputAvg) / inOutRatio + 1);
	if (user_ro < userOutputMin) user_ro = userOutputMin; else if (user_ro > userOutputMax) user_ro = userOutputMax;
	user_pi = (ppm_values[1] - userInputAvg) / inOutRatio;
	if (user_pi < userOutputMin) user_pi = userOutputMin; else if (user_pi > userOutputMax) user_pi = userOutputMax;
	user_ya = -((ppm_values[3] - userInputAvg) / inOutRatio);
	if (user_ya < userOutputMin) user_ya = userOutputMin; else if (user_ya > userOutputMax) user_ya = userOutputMax;


#define ANGLE_GAIN 7.01
#define GYRO_GAIN 0

	int32_t ro = 0; // signed int, 0 is center, positive rolls left
	int32_t pi = 0; // signed int, 0 is center, positive pitches forward
	int32_t th = 0; // unsigned, 0 is minimum
	int32_t ya = 0; // signed int, 0 is center, positive turns left

	int16_t mulResX = -ANGLE_GAIN * roll;
	int16_t mulResY = ANGLE_GAIN * pitch;
	int16_t gyroYawn = GYRO_GAIN * gyro_ya;
	ro = 9 * user_ro + /*GYRO_GAIN * gyro_ro*/ + mulResX;
	pi = 9 * user_pi + /*GYRO_GAIN * gyro_pi*/ + mulResY;
	ya = 9 * user_ya + gyroYawn;

	//pi = 0; ya = 0;
	th = user_th * 4;

	int32_t pwm_4 = th - ro - pi + ya  ;
	int32_t pwm_3 = th - ro + pi - ya  ;

	int32_t pwm_1 = th + ro + pi + ya  ;
	int32_t pwm_2 = th + ro - pi - ya  ;

	if (user_th < 5 || lastMotorValid == 0 || xTaskGetTickCount() - lastMotorValid >= pdMS_TO_TICKS(100)){
		pwm_1 = 0;
		pwm_2 = 0;
		pwm_3 = 0;
		pwm_4 = 0;
	}
	setMotorSpeed(pwm_1, pwm_2, pwm_3, pwm_4);
}

void SystemCoreTask(void const * argument){
	xTimerStart(controlTimerHandle, 0);
	uint32_t notifiedValue;
	uint8_t leadingZeroIndex;
	for(;;){
		if (xTaskNotifyWait(0x00, ULONG_MAX, &notifiedValue, portMAX_DELAY)){
			while ((leadingZeroIndex = __CLZ(notifiedValue)) != 32){
				notifiedValue &= UINT32_MAX >> (leadingZeroIndex + 1);
				switch(leadingZeroIndex){
					case EVENT_CONTROLLER_UPDATE: core_updateController(); break;
				}
			}
		}
	}
}
