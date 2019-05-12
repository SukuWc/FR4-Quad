/*
 * system_core.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "limits.h"
#include "core/system_core.h"
#include "bsp/motors.h"
#include "math.h"
#ifndef __SIMULATOR__

#include "core/system_logger.h"
#include "bsp/devices.h"
#include "bsp/mpu9250.h"
#include "bsp/bmp280.h"

#endif
#include "core/pid.h"
#include "core/kalman/MadgwickAHRS.h"
#include "bsp/ibus_handler.h"

int16_t ax, ay, az, rotx, roty, rotz;

uint32_t lastMotorValid = 0;
volatile float pitch = 0;
volatile float roll = 0;
volatile float yawn = 0;
volatile float yawSpeed = 0;
volatile float pitchGyro = 0;
volatile float rollGyro = 0;
volatile float pitchAcc = 0;
volatile float rollAcc = 0;
volatile float height = 0;

uint32_t lastPressureTick = 0;

int16_t accel_x, accel_y, accel_z, gyro_ro, gyro_pi, gyro_ya, mag_x, mag_y, mag_z;
uint32_t pressure, temperature;
int16_t user_ro, user_pi, user_ya;
int16_t user_th;
int32_t pwm_1, pwm_2, pwm_3, pwm_4;

#define MEASUREMENT_LENGTH 1368
volatile int16_t accX[MEASUREMENT_LENGTH];
volatile int16_t accY[MEASUREMENT_LENGTH];
volatile int16_t accZ[MEASUREMENT_LENGTH];
uint16_t measurementIndex = 0;


/*#define AVG_LENGTH 1
int16_t avg[3][AVG_LENGTH] = {0};
uint16_t avgIndex[3] = {0};
int32_t avgSum[3] = {0};*/

volatile uint8_t inProcess = 0;
volatile uint8_t errorState = 0;
uint8_t thrustWasInZero = 0;

#define USER_INPUT_TO_THRUST 0.3f

#define PID_CLAMP_MIN -128
#define PID_CLAMP_MAX 128

#ifndef __SIMULATOR__
#define ANGLE_KP 2.5f
#define ANGLE_KD (300 / CONTROL_LOOP_PERIOD_MS)
#define ANGLE_KI (0.00 * CONTROL_LOOP_PERIOD_MS)
#else
#define ANGLE_KP 2.5
#define ANGLE_KD (640 / CONTROL_LOOP_PERIOD_MS)
#define ANGLE_KI (0.0 * CONTROL_LOOP_PERIOD_MS)
#endif

#define YAWSPEED_KP 2.0f
#define YAWSPEED_KD (0 / CONTROL_LOOP_PERIOD_MS)
#define YAWSPEED_KI (0.000 * CONTROL_LOOP_PERIOD_MS)

PIDStruct rollPid;
PIDStruct pitchPid;
PIDStruct yawPid;

int32_t ro = 0; // signed int, 0 is center, positive rolls left
int32_t pi = 0; // signed int, 0 is center, positive pitches forward
int32_t th = 0; // unsigned, 0 is minimum
int32_t ya = 0; // signed int, 0 is center, positive turns left

#define GYRO_OFFSET_AVG_LENGTH 1024
int32_t gyroOffsetSum[3] = {0};
uint16_t gyroOffsetIndex = 0;
int16_t gyroOffsets[3] = {0};

float orientationSum[3] = {0};
uint8_t orientationCount = 0;

void ControlEvent(const void* argument){
	if (inProcess){
		errorState = 1;
		while(1){}
	}
	sNotifySystemCore(EVENT_CORE_CONTROLLER_UPDATE);
}

void PositionUpdateEvent(const void* argument){
	if (inProcess){
		errorState = 1;
		while(1){}
	}
	sNotifySystemCore(EVENT_CORE_POSITION_UPDATED);
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

#define USER_ANGLE_MAX 45.0
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

#ifndef __SIMULATOR__
extern Mpu9250Device mpu_device;
extern Bmp280Device bmp_device;

void core_updatePosition(){
	inProcess = 0;
	//mpu9250_getMotion6(&mpu_device, &ax, &ay, &az, &rotx, &roty, &rotz);
	static uint8_t motionBuffer[28];
	mpu9250_getMotionAndExternalBytes(&mpu_device, motionBuffer, 28);

    ax = (((int16_t)motionBuffer[0]) << 8) | motionBuffer[1];
    ay = (((int16_t)motionBuffer[2]) << 8) | motionBuffer[3];
    az = (((int16_t)motionBuffer[4]) << 8) | motionBuffer[5];
    rotx = (((int16_t)motionBuffer[8]) << 8) | motionBuffer[9];
    roty = (((int16_t)motionBuffer[10]) << 8) | motionBuffer[11];
    rotz = (((int16_t)motionBuffer[12]) << 8) | motionBuffer[13];

    if (channel_values[4] < 1250){
		accX[measurementIndex] = ax;
		accY[measurementIndex] = ay;
		accZ[measurementIndex] = az;
		measurementIndex++;
		if (measurementIndex == MEASUREMENT_LENGTH){
			measurementIndex = 0;
		}
	}

    uint8_t magnetStatus = motionBuffer[14];
    /*if (magnetStatus & 1){
        mag_x = (((int16_t)motionBuffer[16]) << 8 ) | motionBuffer[15];
        mag_y = (((int16_t)motionBuffer[18]) << 8 ) | motionBuffer[17];
        mag_z = (((int16_t)motionBuffer[20]) << 8 ) | motionBuffer[19];

        if (channel_values[4] < 1250){
			measureMagX[measurementIndex] = mag_x;
			measureMagY[measurementIndex] = mag_y;
			measureMagZ[measurementIndex] = mag_z;
			measurementIndex++;
			if (measurementIndex == MEASUREMENT_LENGTH){
				measurementIndex = 0;
			}
        }
    }*/

    /*if (xTaskGetTickCount() - lastPressureTick > pdMS_TO_TICKS(40)){
        pressure = (((uint32_t)motionBuffer[22]) << 24) | (((uint32_t)motionBuffer[23]) << 16) | (((uint32_t)motionBuffer[24]) << 8);
        pressure >>= 8;

        temperature = (((uint32_t)motionBuffer[25]) << 24) | (((uint32_t)motionBuffer[26]) << 16) | (((uint32_t)motionBuffer[27]) << 8);
        temperature >>= 8;

        bmp_device.readPressure = pressure;
        bmp_device.readTemp = temperature;

        height = bmp280_calcAltitude(&bmp_device);
    }*/


	accel_x = ax;
	accel_y = ay;
	accel_z = az;

	gyro_pi = roty;// - gyroOffsets[1];//(roty - gyroOffsets[1]);//getAverage(gyroPitchAvg, &gyroPitchAvgIndex, &gyroPitchSum, roty);
	gyro_ro = rotx;// - gyroOffsets[0];//-rotx;//-(rotx - gyroOffsets[0]);//getAverage(gyroRollAvg, &gyroRollAvgIndex, &gyroRollSum, rotx);
	gyro_ya = rotz;// - gyroOffsets[2];//-rotz;//-(rotz - gyroOffsets[2]);


	/*#define ALPHA 0.9985
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
	}*/


	#define GYRO_SENS 16.384f
	#define ACC_SENS 1.0f//317.141f //any unit works
	#define M_RAD_TO_DEG (180.0f/M_PI)
	#define M_DEG_TO_RAD (M_PI/180.f)
	/*if (user_th == 0){
		MadgwickAHRSupdateIMU(gyro_ro/GYRO_SENS * M_DEG_TO_RAD, gyro_pi/GYRO_SENS * M_DEG_TO_RAD, gyro_ya/GYRO_SENS * M_DEG_TO_RAD, accel_x/ACC_SENS, accel_y/ACC_SENS, accel_z/ACC_SENS);
		q0 = 1;
		q1 = 0;
		q2 = 0;
		q3 = 0;
	} else {*/
		MadgwickAHRSupdateIMU(gyro_ro/GYRO_SENS * M_DEG_TO_RAD, gyro_pi/GYRO_SENS * M_DEG_TO_RAD, gyro_ya/GYRO_SENS * M_DEG_TO_RAD, accel_x/ACC_SENS, accel_y/ACC_SENS, accel_z/ACC_SENS);
		//MadgwickAHRSupdate(gyro_ro/GYRO_SENS * M_DEG_TO_RAD, gyro_pi/GYRO_SENS * M_DEG_TO_RAD, gyro_ya/GYRO_SENS * M_DEG_TO_RAD, accel_x/ACC_SENS, accel_y/ACC_SENS, accel_z/ACC_SENS, mag_x, mag_y, mag_z);
	//}
	float roll = -atan2f(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2)) * M_RAD_TO_DEG;
	float pitch = asin(2*(q0*q2-q3*q1)) * M_RAD_TO_DEG;
	orientationSum[0] += roll;
	orientationSum[1] += pitch;
	orientationCount++;
	yawSpeed = -gyro_ya / GYRO_SENS;
	inProcess = 0;
}
#else
extern float simulatorRollAngle, simulatorPitchAngle, simulatorYawAngle;
static float oldYawnAngle = 0;
void core_updatePosition() {
	roll = simulatorRollAngle;
	pitch = -simulatorPitchAngle;
	yawSpeed = (simulatorYawAngle - oldYawnAngle) / 0.01f;
	oldYawnAngle = simulatorYawAngle;
}
#endif

void core_updateController(){
	if (receiverValid()){
		//ppm_valid = 0;
		lastMotorValid = xTaskGetTickCount();
	}
	uint8_t motorValid = !(lastMotorValid == 0 || xTaskGetTickCount() - lastMotorValid >= pdMS_TO_TICKS(100));

	//inProcess = 1;

	int16_t userInputMin = 1000;
	int16_t userInputMax = 2000;
	int16_t userInputAvg = (userInputMin + userInputMax) / 2;
	int16_t inOutRatio = 2;
	int16_t userOutputMin = -256;
	int16_t userOutputMax = 256;
	int16_t userDeadband = 12;
	if (motorValid){
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
		yawPid.integral_part = 0;
	}

	roll = orientationSum[0] / orientationCount;
	pitch = orientationSum[1] / orientationCount;
	orientationSum[0] = 0;
	orientationSum[1] = 0;
	orientationCount = 0;

	ro = calculatePIDLoop(&rollPid, getTargetAngleFromUserInput(user_ro) + roll);
	pi = calculatePIDLoop(&pitchPid, getTargetAngleFromUserInput(user_pi) + pitch);
	ya = calculatePIDLoop(&yawPid, getTargetYawSpeedFromUserInput(user_ya) + yawSpeed);
	th = user_th * 2 + (abs(user_ro) + abs(user_pi)) * USER_INPUT_TO_THRUST;
	if (user_th < 5 || !motorValid || errorState || !thrustWasInZero){
		pwm_1 = 0;
		pwm_2 = 0;
		pwm_3 = 0;
		pwm_4 = 0;

		if (abs(rotx) + abs(roty) + abs(rotz) < 100){
			gyroOffsetSum[0] += rotx;
			gyroOffsetSum[1] += roty;
			gyroOffsetSum[2] += rotz;
			gyroOffsetIndex++;
			if (gyroOffsetIndex >= GYRO_OFFSET_AVG_LENGTH){
				for (int i = 0; i < 3; i++){
					gyroOffsets[i] = (int16_t)(gyroOffsetSum[i] / GYRO_OFFSET_AVG_LENGTH);
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
	initializePIDStruct(&rollPid, ANGLE_KP, ANGLE_KD, ANGLE_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	initializePIDStruct(&pitchPid, ANGLE_KP, ANGLE_KD, ANGLE_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	initializePIDStruct(&yawPid, YAWSPEED_KP, YAWSPEED_KD, YAWSPEED_KI, PID_CLAMP_MIN, PID_CLAMP_MAX);
	for(;;){
		if (xTaskNotifyWait(0x00, ULONG_MAX, &notifiedValue, portMAX_DELAY)){
			while ((leadingZeroIndex = __CLZ(notifiedValue)) != 32){
				notifiedValue &= UINT32_MAX >> (leadingZeroIndex + 1);
				switch(leadingZeroIndex){
					case EVENT_CORE_CONTROLLER_UPDATE: core_updateController(); break;
					case EVENT_CORE_POSITION_UPDATED: core_updatePosition(); break;
				}
			}
		}
	}
}
