/*
 * devices.c
 *
 *  Created on: 2019. �pr. 5.
 *      Author: danim
 */
#include "bsp/devices.h"
#include "spi.h"
#include "communication_interface.h"
#include "bsp/mpu9250_regs.h"
#include "bsp/bmp280.h"

Mpu9250Device mpu_device;
Bmp280Device bmp_device;


static SPICommunicationInterface spi_comm;
static SPIDeviceCommunicationInterface spi_dev;
static DeviceInterface mpu_dev;

static MPU9250_I2CDeviceInterface bmp_mpu_dev;
static DeviceInterface bmp_dev;

static MPU9250_I2CDeviceInterface ak_mpu_dev;
static DeviceInterface ak_dev;

void initDevices(){
	initSPICommunicationInterface(&spi_comm, &hspi1);
	initSPIDeviceCommunicationInterface(&spi_dev, &spi_comm, GPIOA, GPIO_PIN_15);
	initSPIDeviceInterface(&mpu_dev, &spi_dev);
	mpu_device.interface = &mpu_dev;
	mpu_device.currentSlave4Address = 0;
	mpu9250_switchSPIEnabled(&mpu_device, 1);

	while (mpu9250_testConnection(&mpu_device) == 0){

	}
	mpu9250_initialize(&mpu_device);

	mpu9250_setClockSource(&mpu_device, MPU9250_CLOCK_PLL_XGYRO);
	while(mpu9250_getClockSource(&mpu_device) != MPU9250_CLOCK_PLL_XGYRO){
	}
	mpu9250_setSleepEnabled(&mpu_device, 0);
	while(mpu9250_getSleepEnabled(&mpu_device) != 0){
	}
	mpu9250_setFullScaleAccelRange(&mpu_device, MPU9250_ACCEL_FS_16);
	while (mpu9250_getFullScaleAccelRange(&mpu_device) != 3){
	}
	mpu9250_setFullScaleGyroRange(&mpu_device, MPU9250_GYRO_FS_2000);
	while (mpu9250_getFullScaleGyroRange(&mpu_device) != 3){
	}
	mpu9250_setFChoice_b(&mpu_device, 0);
	while (mpu9250_getFChoice_b(&mpu_device) != 0){
	}
	mpu9250_setDLPFMode(&mpu_device, 3);
	while (mpu9250_getDLPFMode(&mpu_device) != 3){
	}
	mpu9250_setAccelDPFL(&mpu_device, 3);
	while (mpu9250_getAccelDPFL(&mpu_device) != 3){
	}
	mpu9250_setAccelF_b(&mpu_device, 0);
	while (mpu9250_getAccelF_b(&mpu_device) != 0){
	}

	mpu9250_setSlaveReadWriteTransitionEnabled(&mpu_device, 1);
	mpu9250_setI2CBypassEnabled(&mpu_device, 0);
	mpu9250_setWaitForExternalSensorEnabled(&mpu_device, 1);
	mpu9250_setI2CMasterModeEnabled(&mpu_device, 1);
	mpu9250_setSlave4WriteMode(&mpu_device, 0);


	//AK configuration
	mpu9250_initDeviceInterface(&ak_mpu_dev, &mpu_device, 0x0c);
	mpu9250_initMpuDeviceInterface(&ak_dev, &ak_mpu_dev);
	uint8_t temp_buffer = 0;
	while (1){
		readByte(&ak_dev, 0, &temp_buffer);
		if (temp_buffer == 0x48){
			break;
		}
	}
	writeByte(&ak_dev, 0x0a, 0x16); //Continous measurement2, 16 bit output

	mpu9250_setSlaveAddress(&mpu_device, 0, 0x8c);
	mpu9250_setSlaveRegister(&mpu_device, 0, 0x02);
	mpu9250_setSlaveDataLength(&mpu_device, 0, 8);
	mpu9250_setSlaveWriteMode(&mpu_device, 0, 0);
	mpu9250_setSlaveEnabled(&mpu_device, 0, 1);

	//BMP280 configuration
	mpu9250_initDeviceInterface(&bmp_mpu_dev, &mpu_device, 0x76);
	mpu9250_initMpuDeviceInterface(&bmp_dev, &bmp_mpu_dev);
	bmp_device.interface = &bmp_dev;

	bmp280_init(&bmp_device);

	mpu9250_setSlaveAddress(&mpu_device, 1, 0xf6);
	mpu9250_setSlaveRegister(&mpu_device, 1, 0xf7);
	mpu9250_setSlaveDataLength(&mpu_device, 1, 6);
	mpu9250_setSlaveWriteMode(&mpu_device, 1, 0);
	mpu9250_setSlaveEnabled(&mpu_device, 1, 1);

	/*static volatile uint8_t buffer[14];
	while(1){
		mpu9250_getExternalSensorBytes(&mpu_device, 0, buffer, 14);
		HAL_Delay(10);
	}*/
}
