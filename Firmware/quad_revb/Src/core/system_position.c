/*
 * system_position.c
 *
 *  Created on: 12 May 2019
 *      Author: danim
 */
#include "core/system_position.h"
#include "bsp/devices.h"
#include "main.h"

#define MEASUREMENT_LENGTH 1368
volatile int16_t accX[MEASUREMENT_LENGTH];
volatile int16_t accY[MEASUREMENT_LENGTH];
volatile int16_t accZ[MEASUREMENT_LENGTH];
uint16_t measurementIndex = 0;

//Protected by positionDataMutex
volatile float pitch = 0;
volatile float roll = 0;
volatile float yawn = 0;
volatile float yawSpeed = 0;
volatile float pitchSpeed = 0;
volatile float rollSpeed = 0;
volatile float height = 0;

/*
volatile float pitchAcc = 0;
volatile float rollAcc = 0;
*/
uint32_t lastPressureTick = 0;

int16_t accel_x, accel_y, accel_z, gyro_ro, gyro_pi, gyro_ya, mag_x, mag_y, mag_z;
uint32_t pressure, temperature;

void PositionUpdateEvent(const void* argument){
	sNotifySystemCore(EVENT_POSITION_UPDATE);
}

#ifndef __SIMULATOR__
extern Mpu9250Device mpu_device;
extern Bmp280Device bmp_device;
void core_updatePosition(){
	inProcess = 0;
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
	MadgwickAHRSupdateIMU(gyro_ro/GYRO_SENS * M_DEG_TO_RAD, gyro_pi/GYRO_SENS * M_DEG_TO_RAD, gyro_ya/GYRO_SENS * M_DEG_TO_RAD, accel_x/ACC_SENS, accel_y/ACC_SENS, accel_z/ACC_SENS);
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

void SystemPosition(void const * argument){
	for(;;){
		if (xTaskNotifyWait(0x00, ULONG_MAX, &notifiedValue, portMAX_DELAY)){
			while ((leadingZeroIndex = __CLZ(notifiedValue)) != 32){
				notifiedValue &= UINT32_MAX >> (leadingZeroIndex + 1);
				switch(leadingZeroIndex){
					case EVENT_POSITION_UPDATE: core_updatePosition(); break;
				}
			}
		}
	}
}

