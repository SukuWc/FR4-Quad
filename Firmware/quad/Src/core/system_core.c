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
#include "core/pid.h"

int16_t ax, ay, az, rotx, roty, rotz;

uint32_t lastMotorValid = 0;
volatile float pitch = 0;
volatile float roll = 0;
volatile float yawn = 0;
volatile float pitchGyro = 0;
volatile float rollGyro = 0;
volatile float pitchAcc = 0;
volatile float rollAcc = 0;

int16_t accel_x, accel_y, accel_z, gyro_ro, gyro_pi, gyro_ya;
int16_t user_ro, user_pi, user_ya;
int16_t user_th;
int32_t pwm_1, pwm_2, pwm_3, pwm_4;

#define AVG_LENGTH 1
int16_t avg[3][AVG_LENGTH] = {0};
uint16_t avgIndex[3] = {0};
int32_t avgSum[3] = {0};

volatile uint8_t inProcess = 0;
volatile uint8_t errorState = 0;
uint8_t thrustWasInZero = 0;

#define PID_CLAMP_MIN -320
#define PID_CLAMP_MAX 320

#define ANGLE_KP 4.1
#define ANGLE_KD 320
#define ANGLE_KI 0.008

#define YAWSPEED_KP 5.0
#define YAWSPEED_KD 30.0
#define YAWSPEED_KI 0.0

PIDStruct rollPid;
PIDStruct pitchPid;
PIDStruct yawPid;

int32_t ro = 0; // signed int, 0 is center, positive rolls left
int32_t pi = 0; // signed int, 0 is center, positive pitches forward
int32_t th = 0; // unsigned, 0 is minimum
int32_t ya = 0; // signed int, 0 is center, positive turns left

void ControlLoop(const void* argument){
	if (inProcess){
		errorState = 1;
	}
	sNotifySystemCore(EVENT_CONTROLLER_UPDATE);
}

int16_t getAverage(int16_t *buffer, uint16_t* index, int32_t* sum, int16_t inValue){
	*sum -= buffer[*index];
	*sum += inValue;
	buffer[*index] = inValue;
	if (*index + 1 >= AVG_LENGTH){
		*index = 0;
	} else {
		(*index)++;
	}
	return (*sum / AVG_LENGTH);
}

#define USER_ANGLE_MAX 60.0
#define USER_INPUT_MAX 256.0;
#define POLY_EXP 2
float getTargetAngleFromUserInput(int16_t user){
	float input = user / USER_INPUT_MAX;
	float result = 1;
	for(int i = 0; i < POLY_EXP; i++){
		result *= input;
	}
	if (POLY_EXP % 2 == 0 && user < 0){
		result = -result;
	}
	return result * USER_ANGLE_MAX;
}

#define USER_YAW_MAX 360.0
float getTargetYawSpeedFromUserInput(int16_t user){
	float input = user / USER_INPUT_MAX;
	return input * USER_YAW_MAX;
}

void core_updateController(){
	//sNotifyLogger(ACCELERATION_SENSOR_UPDATED);


	uint32_t currentTick = HAL_GetTick();
	if (ppm_valid){
		ppm_valid = 0;
		lastMotorValid = xTaskGetTickCount();
	}
	uint8_t motorValid = !(lastMotorValid == 0 || xTaskGetTickCount() - lastMotorValid >= pdMS_TO_TICKS(100));

	inProcess = 1;

	//MX_RESET_I2C();
	mpu9250_getMotion6(&ax, &ay, &az, &rotx, &roty, &rotz);

	accel_x = -getAverage(avg[0], &avgIndex[0], &avgSum[0], ax);
	accel_y = -getAverage(avg[1], &avgIndex[1], &avgSum[1], ay);
	accel_z = getAverage(avg[2], &avgIndex[2], &avgSum[2], az);

	gyro_pi = roty/*getAverage(gyroPitchAvg, &gyroPitchAvgIndex, &gyroPitchSum, roty)*/;
	gyro_ro = -rotx/*getAverage(gyroRollAvg, &gyroRollAvgIndex, &gyroRollSum, rotx)*/;
	gyro_ya = -rotz;


#define GYRO_SENS 16.384
#define ALPHA 0.9985
#define dt 0.002

	pitchGyro = (float)(gyro_pi) / GYRO_SENS * dt;
	pitch += pitchGyro;
	rollGyro = (gyro_ro/ GYRO_SENS * dt);
	roll += rollGyro;
	float yawSpeed = gyro_ya / GYRO_SENS;
	yawn += yawSpeed * dt;

	int forceMagnitudeApprox = abs(accel_x) + abs(accel_y) + abs(accel_z);
	if (forceMagnitudeApprox > 1024 && forceMagnitudeApprox < 4096){
		if (abs(accel_x) < abs(accel_z)){
			pitchAcc = atan2f((float)accel_x, (float)accel_z) * 180 / M_PI;
			pitch = pitch * ALPHA + pitchAcc * (1 - ALPHA);
		}

		if (abs(accel_y < abs(accel_z))){
			rollAcc = atan2f((float)accel_y, (float)accel_z) * 180 / M_PI;
			roll = roll * ALPHA + rollAcc * (1 - ALPHA);
		}
	}

	int16_t userInputMin = 1000;
	int16_t userInputMax = 2000;
	int16_t userInputAvg = (userInputMin + userInputMax) / 2;
	int16_t inOutRatio = 4;
	int16_t userOutputMin = -256;
	int16_t userOutputMax = 256;
	int16_t userDeadband = 12;
	if (motorValid){
		user_th = (ppm_values[2] - userInputMin) / inOutRatio;
		if (user_th < 0) user_th = 0; else if (user_th > userOutputMax*2) user_th = userOutputMax*2;
		user_ro = -((ppm_values[0] - userInputAvg) / inOutRatio + 1);
		if (user_ro < userOutputMin) user_ro = userOutputMin; else if (user_ro > userOutputMax) user_ro = userOutputMax;
		if (user_ro <= userDeadband && user_ro >= -userDeadband) user_ro = 0;
		user_pi = (ppm_values[1] - userInputAvg) / inOutRatio;
		if (user_pi < userOutputMin) user_pi = userOutputMin; else if (user_pi > userOutputMax) user_pi = userOutputMax;
		if (user_pi <= userDeadband && user_pi >= -userDeadband) user_pi = 0;
		user_ya = -((ppm_values[3] - userInputAvg) / inOutRatio);
		if (user_ya < userOutputMin) user_ya = userOutputMin; else if (user_ya > userOutputMax) user_ya = userOutputMax;
		if (user_ya <= userDeadband && user_ya >= -userDeadband) user_ya = 0;
	} else {
		user_th = 0;
		user_ro = 0;
		user_pi = 0;
		user_ya = 0;
	}

	if (user_th == 0 && motorValid){
		thrustWasInZero = 1;
	}

	volatile int32_t pwm_4;
	volatile int32_t pwm_3;
	volatile int32_t pwm_1;
	volatile int32_t pwm_2;

	if (user_th < 80){
		rollPid.integral_part = 0;
		pitchPid.integral_part = 0;
	}

	ro = calculatePIDLoop(&rollPid, getTargetAngleFromUserInput(user_ro) + roll);
	pi = calculatePIDLoop(&pitchPid, getTargetAngleFromUserInput(user_pi) + pitch);
	ya = calculatePIDLoop(&yawPid, getTargetYawSpeedFromUserInput(user_ya) + yawSpeed);
	th = user_th * 4;
	if (user_th < 5 || !motorValid || errorState || !thrustWasInZero){
		pwm_1 = 0;
		pwm_2 = 0;
		pwm_3 = 0;
		pwm_4 = 0;
	} else {

		pwm_4 = th - ro - pi + ya;
		pwm_3 = th - ro + pi - ya;
		pwm_1 = th + ro + pi + ya;
		pwm_2 = th + ro - pi - ya;
	}
	setMotorSpeed(pwm_1, pwm_2, pwm_3, pwm_4);
	sNotifyLogger(ACCELERATION_SENSOR_UPDATED);
	inProcess = 0;
}

void SystemCoreTask(void const * argument){
	xTimerStart(controlTimerHandle, 0);
	uint32_t notifiedValue;
	uint8_t leadingZeroIndex;
	initializePIDStruct(&rollPid, ANGLE_KP, ANGLE_KD, ANGLE_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	initializePIDStruct(&pitchPid, ANGLE_KP, ANGLE_KD, ANGLE_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	initializePIDStruct(&yawPid, YAWSPEED_KP, YAWSPEED_KD, YAWSPEED_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
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
