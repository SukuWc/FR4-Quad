/*
 * devices.c
 *
 *  Created on: 2019. ápr. 5.
 *      Author: danim
 */
#include "bsp/devices.h"
#include "spi.h"
#include "communication_interface.h"
#include "bsp/mpu9250_regs.h"

Mpu9250Device mpu_device;

static SPICommunicationInterface spi_comm;
static SPIDeviceCommunicationInterface spi_dev;
static DeviceInterface mpu_dev;

void initDevices(){
	initSPICommunicationInterface(&spi_comm, &hspi1);
	initSPIDeviceCommunicationInterface(&spi_dev, &spi_comm, GPIOA, GPIO_PIN_15);
	initSPIDeviceInterface(&mpu_dev, &spi_dev);
	mpu_device.interface = &mpu_dev;
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
/*
	while (mpu9250_getSlaveAddress(&mpu_device, 0) != 0x8c){
		MX_RESET_I2C();
		mpu9250_setSlaveAddress(&mpu_device, 0, 0x8c);
	}
	while (mpu9250_getSlaveDataLength(&mpu_device, 0) != 1){
		MX_RESET_I2C();
		mpu9250_setSlaveDataLength(&mpu_device, 0, 1);
	}
	while (mpu9250_getSlaveRegister(&mpu_device, 0) != 0){
		MX_RESET_I2C();
		mpu9250_setSlaveRegister(&mpu_device, 0, 0);
	}
	while (mpu9250_getSlaveWriteMode(&mpu_device, 0) != 0){
		MX_RESET_I2C();
		mpu9250_setSlaveWriteMode(&mpu_device, 0, 0);
	}
	while (mpu9250_getSlaveEnabled(&mpu_device, 0) != 1){
		MX_RESET_I2C();
		mpu9250_setSlaveEnabled(&mpu_device, 0, 1);
	}
	while (mpu9250_getMasterClockSpeed(&mpu_device) != 8){
		MX_RESET_I2C();
		mpu9250_setMasterClockSpeed(&mpu_device, 8);
	}
	while (mpu9250_getSlaveReadWriteTransitionEnabled(&mpu_device) != 1){
		MX_RESET_I2C();
		mpu9250_setSlaveReadWriteTransitionEnabled(&mpu_device, 1);
	}
	while(mpu9250_getI2CBypassEnabled(&mpu_device) != 0){
		MX_RESET_I2C();
		mpu9250_setI2CBypassEnabled(&mpu_device, 0);
	}
	while(mpu9250_getWaitForExternalSensorEnabled(&mpu_device) != 1){
		MX_RESET_I2C();
		mpu9250_setWaitForExternalSensorEnabled(&mpu_device, 1);
	}
	while (mpu9250_getI2CMasterModeEnabled(&mpu_device) != 1){
		MX_RESET_I2C();
		mpu9250_setI2CMasterModeEnabled(&mpu_device, 1);
	}
	for (uint8_t i = 0; i < 6; i++){
		MX_RESET_I2C();
		mpu9250_resetI2CMaster(&mpu_device);
	}

	while(mpu9250_getSlaveAddress(&mpu_device, 0) != 0x0c){
		MX_RESET_I2C();
		mpu9250_setSlaveAddress(&mpu_device, 0, 0x0c);
	}
	while(mpu9250_getSlaveRegister(&mpu_device, 0) != 0x0a){
		MX_RESET_I2C();
		mpu9250_setSlaveRegister(&mpu_device, 0, 0xa);
	}
	for (uint8_t i = 0; i < 12; i++){
		MX_RESET_I2C();
		mpu9250_setSlaveOutputByte(&mpu_device, 0, 0x06);
	}
	HAL_Delay(10);

	while(mpu9250_getSlaveAddress(&mpu_device, 0) != 0x8c){
		MX_RESET_I2C();
		mpu9250_setSlaveAddress(&mpu_device, 0, 0x8c);
	}
	while(mpu9250_getSlaveRegister(&mpu_device, 0) != 0x02){
		MX_RESET_I2C();
		mpu9250_setSlaveRegister(&mpu_device, 0, 0x02);
	}
	while(mpu9250_getSlaveDataLength(&mpu_device, 0) != 8){
		MX_RESET_I2C();
		mpu9250_setSlaveDataLength(&mpu_device, 0, 8);
	}*/
}
