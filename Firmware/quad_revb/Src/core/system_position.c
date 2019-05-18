/*
 * system_position.c
 *
 *  Created on: 12 May 2019
 *      Author: danim
 */
#include "core/system_position.h"
#include "core/system_core.h"
#ifndef __SIMULATOR__
#include "bsp/devices.h"
#include "main.h"
#endif
#include "limits.h"

/*#define MEASUREMENT_LENGTH 1368
volatile int16_t accX[MEASUREMENT_LENGTH];
volatile int16_t accY[MEASUREMENT_LENGTH];
volatile int16_t accZ[MEASUREMENT_LENGTH];
uint16_t measurementIndex = 0;*/

//Protected by positionDataMutex
volatile float position_pitch = 0;
volatile float position_roll = 0;
volatile float position_yaw = 0;
volatile float position_yawSpeed = 0;
volatile float position_pitchSpeed = 0;
volatile float position_rollSpeed = 0;
volatile float position_height = 0;

/*
volatile float pitchAcc = 0;
volatile float rollAcc = 0;

	ro = calculatePIDLoop(&rollGyroPid, -(gyro_ro) + targetRollGyro/*getTargetAngleFromUserInput(user_ro) + roll);
	//pi = calculatePIDLoop(&pitchGyroPid, gyro_pi + targetPitchGyro/*getTargetAngleFromUserInput(user_pi) + pitch);
	//ya = calculatePIDLoop(&yawPid, getTargetYawSpeedFromUserInput(user_ya) + position_yawSpeed);
*/
uint32_t lastPressureTick = 0;

int16_t accel_x, accel_y, accel_z, gyro_ro, gyro_pi, gyro_ya, mag_x, mag_y, mag_z;
float height = 0;
uint32_t pressure, temperature;

#define AVERAGE_LENGTH 10
static int16_t averageGyroX[AVERAGE_LENGTH] = {0};
static int16_t averageGyroY[AVERAGE_LENGTH] = {0};
static int16_t averageGyroZ[AVERAGE_LENGTH] = {0};
static int32_t sumGyroX = 0;
static int32_t sumGyroY = 0;
static int32_t sumGyroZ = 0;
static uint16_t index = 0;

void PositionUpdateEvent(const void* argument){
	sNotifySystemPosition(EVENT_POSITION_UPDATE);
}

#ifndef __SIMULATOR__
extern Mpu9250Device mpu_device;
void getSensorValues(uint8_t* buffer, uint8_t length) {
	mpu9250_getMotionAndExternalBytes(&mpu_device, buffer, 28);


	mpu9250_setClockSource(&mpu_device, MPU9250_CLOCK_PLL_XGYRO);
	mpu9250_setSleepEnabled(&mpu_device, 0);
	mpu9250_setFullScaleAccelRange(&mpu_device, MPU9250_ACCEL_FS_16);
	mpu9250_setFullScaleGyroRange(&mpu_device, MPU9250_GYRO_FS_2000);
	mpu9250_setFChoice_b(&mpu_device, 0);
	mpu9250_setDLPFMode(&mpu_device, 3);
	mpu9250_setAccelDPFL(&mpu_device, 6);
	mpu9250_setAccelF_b(&mpu_device, 0);
}
#endif // !__SIMULATOR__


void core_updatePosition(){
	static uint8_t motionBuffer[28];
	getSensorValues(motionBuffer, 28);

    int16_t ax = (((int16_t)motionBuffer[0]) << 8) | motionBuffer[1];
    int16_t ay = (((int16_t)motionBuffer[2]) << 8) | motionBuffer[3];
    int16_t az = (((int16_t)motionBuffer[4]) << 8) | motionBuffer[5];
    int16_t rotx = (((int16_t)motionBuffer[8]) << 8) | motionBuffer[9];
    int16_t roty = (((int16_t)motionBuffer[10]) << 8) | motionBuffer[11];
    int16_t rotz = (((int16_t)motionBuffer[12]) << 8) | motionBuffer[13];

    /*if (channel_values[4] < 1250){
		accX[measurementIndex] = ax;
		accY[measurementIndex] = ay;
		accZ[measurementIndex] = az;
		measurementIndex++;
		if (measurementIndex == MEASUREMENT_LENGTH){
			measurementIndex = 0;
		}
	}*/

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

#ifdef __SIMULATOR__
	height = ((((int16_t)motionBuffer[25]) << 8) | motionBuffer[26]) * 0.09f;
#endif // __SIMULATOR

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


	sumGyroX += (gyro_ro - averageGyroX[index]);
	sumGyroY += (gyro_pi - averageGyroY[index]);
	sumGyroZ += (gyro_ya - averageGyroZ[index]);
	averageGyroX[index] = gyro_ro;
	averageGyroY[index] = gyro_pi;
	averageGyroZ[index] = gyro_ya;
	index++;
	if (index == AVERAGE_LENGTH){
		index = 0;
	}

	float rollSpeed = -sumGyroX / (GYRO_SENS * AVERAGE_LENGTH);
	float pitchSpeed = sumGyroY / (GYRO_SENS * AVERAGE_LENGTH);
	float yawSpeed = -sumGyroZ / (GYRO_SENS * AVERAGE_LENGTH);

	xSemaphoreTake(positionDataMutexHandle, pdMS_TO_TICKS(10000));
	position_roll = roll;
	position_pitch = pitch;
	position_rollSpeed = rollSpeed;
	position_pitchSpeed = pitchSpeed;
	position_yawSpeed = yawSpeed;
	position_height = height;
	xSemaphoreGive(positionDataMutexHandle);
	sNotifySystemCore(EVENT_CORE_POSITION_UPDATED);
}

void SystemPosition(void const * argument){
	uint32_t notifiedValue = 0;
	uint8_t leadingZeroIndex;
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

