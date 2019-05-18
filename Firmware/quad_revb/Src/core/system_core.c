/*
 * system_core.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "limits.h"
#include "core/system_core.h"
#include "core/system_position.h"
#include "bsp/motors.h"
#ifndef __SIMULATOR__

#include "core/system_logger.h"
#include "bsp/devices.h"
#include "bsp/mpu9250.h"
#include "bsp/bmp280.h"

#endif
#include "core/pid.h"
#include "bsp/ibus_handler.h"

int16_t ax, ay, az, rotx, roty, rotz;

uint32_t lastReceiverValid = 0;


/*#define AVG_LENGTH 1
int16_t avg[3][AVG_LENGTH] = {0};
uint16_t avgIndex[3] = {0};
int32_t avgSum[3] = {0};*/

volatile uint8_t inProcess = 0;
volatile uint8_t errorState = 0;
uint8_t thrustWasInZero = 0;

#define USER_INPUT_TO_THRUST 0.3f

#define PID_CLAMP_MIN -1024
#define PID_CLAMP_MAX 1024

#ifndef __SIMULATOR__
#define ANGLE_SPEED_KP 3.3f //1.5
#define ANGLE_SPEED_KD (50.0 / CONTROL_LOOP_PERIOD_MS) //350
#define ANGLE_SPEED_KI (0.00 * CONTROL_LOOP_PERIOD_MS)

#define ANGLE_KP 3.0f
#define ANGLE_KD (0.0 / CONTROL_LOOP_PERIOD_MS)
#define ANGLE_KI (0.00 * CONTROL_LOOP_PERIOD_MS)

#define YAWSPEED_KP 3.2f
#define YAWSPEED_KD (0.0 / CONTROL_LOOP_PERIOD_MS)
#define YAWSPEED_KI (0.000 * CONTROL_LOOP_PERIOD_MS)
#else
#define ANGLE_SPEED_KP 3.3f //1.5
#define ANGLE_SPEED_KD (50.0 / CONTROL_LOOP_PERIOD_MS) //350
#define ANGLE_SPEED_KI (0.00 * CONTROL_LOOP_PERIOD_MS)

#define ANGLE_KP 3.0f
#define ANGLE_KD (0.0 / CONTROL_LOOP_PERIOD_MS)
#define ANGLE_KI (0.00 * CONTROL_LOOP_PERIOD_MS)

#define YAWSPEED_KP 3.3f
#define YAWSPEED_KD (50.0 / CONTROL_LOOP_PERIOD_MS)
#define YAWSPEED_KI (0.000 * CONTROL_LOOP_PERIOD_MS)
#endif

#ifdef __SIMULATOR__
//#define HEIGHT_CONTROL
//#define SAFETY_ANGLE
#endif // __SIMULATOR__

#ifdef HEIGHT_CONTROL
static float targetHeight = 0.0f;
static PIDStruct heightPid; 
static uint8_t reached_center = 0;
#define HEIGHT_KP 2000.0f
#define HEIGHT_KD 20000.0f
#define HEIGHT_KI 0.0f
#define HEIGHT_TARGET_DIFF_MAX 1.0f
#define HEIGHT_MAX_ADDITION (5.0f * CONTROL_LOOP_PERIOD_MS / 1000.0f)
#define HEIGHT_STAY_DEADBAND (240)
#endif // HEIGHT_CONTROL

#ifdef SAFETY_ANGLE
#define SAFETY_MAX_ANGLE (20.0f)
#define SAFETY_RETURN_TO_ANGLE (5.0f)
static uint8_t inSafetyMode = 0;
#endif


PIDStruct rollPid;
PIDStruct pitchPid;
PIDStruct rollGyroPid;
PIDStruct pitchGyroPid;
PIDStruct yawPid;

#define GYRO_OFFSET_AVG_LENGTH 1024
int32_t gyroOffsetSum[3] = {0};
uint16_t gyroOffsetIndex = 0;
int16_t gyroOffsets[3] = {18, 23, -32};

float orientationSum[3] = {0};
uint8_t orientationCount = 0;

void ControlEvent(const void* argument){
	if (inProcess){
		errorState = 1;
		while(1){}
	}
	sNotifySystemCore(EVENT_CORE_CONTROLLER_UPDATE);
}

/*int16_t getAverage(int16_t *buffer, uint16_t* index, int32_t* sum, int16_t inValue){
	*sum -= buffer[*index];
	*sum += inValue;
	buffer[*index] = inValue;
	if (*index + 1 >= AVG_LENGTH){
		*index = 0;
	} else {
		(*index)++;
	}
	return (*sum / AVG_LENGTH);
}*/

#define USER_ANGLE_MAX 30.0
#define USER_INPUT_MAX 256.0;
#define POLY_EXP 1
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

#define USER_YAW_MAX 180.0
float getTargetYawSpeedFromUserInput(int16_t user){
	float input = user / USER_INPUT_MAX;
	return input * USER_YAW_MAX;
}

static int16_t user_th;
static int16_t user_ro;
static int16_t user_pi;
static int16_t user_ya;
void core_joystickUpdated(){
	int16_t userInputMin = 1000;
	int16_t userInputMax = 2000;
	int16_t userInputAvg = (userInputMin + userInputMax) / 2;
	int16_t inOutRatio = 2;
	int16_t userOutputMin = -256;
	int16_t userOutputMax = 256;
	int16_t userDeadband = 12;

	user_th = (channel_values[2] - userInputMin) / inOutRatio;
	if (user_th < 0) user_th = 0; else if (user_th > userOutputMax*2) user_th = userOutputMax*2;
	user_ro = -((channel_values[0] - userInputAvg) / inOutRatio);
	if (user_ro < userOutputMin) user_ro = userOutputMin; else if (user_ro > userOutputMax) user_ro = userOutputMax;
	if (user_ro <= userDeadband && user_ro >= -userDeadband) user_ro = 0;
	user_pi = (channel_values[1] - userInputAvg) / inOutRatio;
	if (user_pi < userOutputMin) user_pi = userOutputMin; else if (user_pi > userOutputMax) user_pi = userOutputMax;
	if (user_pi <= userDeadband && user_pi >= -userDeadband) user_pi = 0;
	user_ya = -((channel_values[3] - userInputAvg) / inOutRatio);
	if (user_ya < userOutputMin) user_ya = userOutputMin; else if (user_ya > userOutputMax) user_ya = userOutputMax;
	if (user_ya <= userDeadband && user_ya >= -userDeadband) user_ya = 0;

	lastReceiverValid = xTaskGetTickCount();
}


static float roll = 0;
static float pitch = 0;
static float yawSpeed = 0;
static float rollSpeed = 0;
static float pitchSpeed = 0;
static float height = 0;
void core_positionUpdated(){
	if (xSemaphoreTake(positionDataMutexHandle, pdMS_TO_TICKS(2)) == pdFALSE){
		errorState = 1;
	}
	roll = position_roll;
	pitch = position_pitch;
	yawSpeed = position_yawSpeed;
	rollSpeed = position_rollSpeed;
	pitchSpeed = position_pitchSpeed;
	height = position_height;
	xSemaphoreGive(positionDataMutexHandle);
}

void core_updateController(){
#ifdef __SIMULATOR__
	uint8_t motorValid = 1;
#else
	uint8_t motorValid = !(lastReceiverValid == 0 || xTaskGetTickCount() - lastReceiverValid >= pdMS_TO_TICKS(100));
#endif
	
	if (user_th == 0 && motorValid){
		thrustWasInZero = 1;
	}

	int32_t pwm_4;
	int32_t pwm_3;
	int32_t pwm_1;
	int32_t pwm_2;

	if (user_th < 80){
		rollGyroPid.integral_part = 0;
		pitchGyroPid.integral_part = 0;
		yawPid.integral_part = 0;
	}

#ifdef SAFETY_ANGLE
	if (user_th == 0) {
		inSafetyMode = 0;
	}
	if ((roll > SAFETY_MAX_ANGLE || roll < -SAFETY_MAX_ANGLE || pitch > SAFETY_MAX_ANGLE || pitch < -SAFETY_MAX_ANGLE) && !inSafetyMode) {
		inSafetyMode = 1;
	} else if (inSafetyMode && roll > -SAFETY_RETURN_TO_ANGLE && roll < SAFETY_RETURN_TO_ANGLE && pitch > -SAFETY_RETURN_TO_ANGLE && pitch < SAFETY_RETURN_TO_ANGLE) {
		inSafetyMode = 0;
	}
	float rollAngle = inSafetyMode ? 0.0f : getTargetAngleFromUserInput(user_ro);
	float pitchAngle = inSafetyMode ? 0.0f : getTargetAngleFromUserInput(user_pi);
#else
	float rollAngle = getTargetAngleFromUserInput(user_ro);
	float pitchAngle = getTargetAngleFromUserInput(user_pi);
#endif// SAFETY_ANGLE

	float targetRollGyro = calculatePIDLoop(&rollPid, rollAngle + roll);
	float targetPitchGyro = calculatePIDLoop(&rollPid, pitchAngle + pitch);

	int32_t ro = calculatePIDLoop(&rollGyroPid, rollSpeed + targetRollGyro); // signed int, 0 is center, positive rolls left
	int32_t pi = calculatePIDLoop(&pitchGyroPid, pitchSpeed + targetPitchGyro); // signed int, 0 is center, positive pitches forward
	int32_t ya = calculatePIDLoop(&yawPid, getTargetYawSpeedFromUserInput(user_ya) + position_yawSpeed); // signed int, 0 is center, positive turns left

#ifdef HEIGHT_CONTROL
	int16_t user_center = 256;
	if (user_th == 0) {
		reached_center = 0;
	} else if (user_th > user_center - HEIGHT_STAY_DEADBAND) {
		reached_center = 1;
	}
	
	if (user_th > user_center + HEIGHT_STAY_DEADBAND || user_th < user_center - HEIGHT_STAY_DEADBAND) {
		if (reached_center) {
			targetHeight += ((user_th - 256)) / 256.0f * HEIGHT_MAX_ADDITION;
			if (targetHeight > height + HEIGHT_TARGET_DIFF_MAX) {
				targetHeight = height + HEIGHT_TARGET_DIFF_MAX;
			} else if (targetHeight < height - HEIGHT_TARGET_DIFF_MAX) {
				targetHeight = height - HEIGHT_TARGET_DIFF_MAX;
			}
		}
	}
	int32_t th = calculatePIDLoop(&heightPid, targetHeight - height); // unsigned, 0 is minimum
	if (th > 1024) {
		th = 1024;
	} else if (th < 0) {
		th = 0;
	}
#else
	int32_t th = user_th * 2 + (abs(user_ro) + abs(user_pi)) * USER_INPUT_TO_THRUST; // unsigned, 0 is minimum
#endif // HEIGHT_CONTROL

	if (user_th == 0 || !motorValid || errorState || !thrustWasInZero){
		pwm_1 = 0;
		pwm_2 = 0;
		pwm_3 = 0;
		pwm_4 = 0;

		if (abs(rotx) + abs(roty) + abs(rotz) < 100 && user_th == 0){
			gyroOffsetSum[0] += rotx;
			gyroOffsetSum[1] += roty;
			gyroOffsetSum[2] += rotz;
			gyroOffsetIndex++;
			if (gyroOffsetIndex >= GYRO_OFFSET_AVG_LENGTH){
				for (int i = 0; i < 3; i++){
					//gyroOffsets[i] = (int16_t)(gyroOffsetSum[i] / GYRO_OFFSET_AVG_LENGTH);
					gyroOffsetSum[i] = 0;
				}
				gyroOffsetIndex = 0;
			}
		}
	} else {
		pwm_4 = th - ro - pi + ya;
		pwm_3 = th - ro + pi - ya;
		pwm_1 = th + ro + pi + ya;
		pwm_2 = th + ro - pi - ya;
	}
	setMotorSpeed(pwm_1, pwm_2, pwm_3, pwm_4);
#ifdef __SIMULATOR__
	controlFinished();
#endif
}

void SystemCoreTask(void const * argument){
	/*xTimerStart(positionUpdateTimerHandle, 0);*/
	/*xTimerStart(controlTimerHandle, 0);*/
	uint32_t notifiedValue;
	uint8_t leadingZeroIndex;
	initializePIDStruct(&rollGyroPid, ANGLE_SPEED_KP, ANGLE_SPEED_KD, ANGLE_SPEED_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	initializePIDStruct(&pitchGyroPid, ANGLE_SPEED_KP, ANGLE_SPEED_KD, ANGLE_SPEED_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	initializePIDStruct(&yawPid, YAWSPEED_KP, YAWSPEED_KD, YAWSPEED_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	initializePIDStruct(&rollPid, ANGLE_KP, ANGLE_KD, ANGLE_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	initializePIDStruct(&pitchPid, ANGLE_KP, ANGLE_KD, ANGLE_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
#ifdef HEIGHT_CONTROL
	initializePIDStruct(&heightPid, HEIGHT_KP, HEIGHT_KD, HEIGHT_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
#endif // HEIGHT_CONTROL

	for(;;){
		if (xTaskNotifyWait(0x00, ULONG_MAX, &notifiedValue, portMAX_DELAY)){
			while ((leadingZeroIndex = __CLZ(notifiedValue)) != 32){
				notifiedValue &= UINT32_MAX >> (leadingZeroIndex + 1);
				switch(leadingZeroIndex){
					case EVENT_CORE_CONTROLLER_UPDATE: core_updateController(); break;
					case EVENT_CORE_POSITION_UPDATED: core_positionUpdated(); break;
					case EVENT_CORE_JOYSTICK_UPDATED: core_joystickUpdated(); break;
				}
			}
		}
	}
}
