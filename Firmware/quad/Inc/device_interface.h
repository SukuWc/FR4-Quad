/*
 * device_interface.h
 *
 *  Created on: 2019. márc. 28.
 *      Author: danim
 */

#ifndef DEVICE_INTERFACE_H_
#define DEVICE_INTERFACE_H_

#include "main.h"

typedef struct DeviceInterface_T{
	int16_t (*read)(uint16_t address, uint8_t* buffer, uint16_t length);
	int16_t (*write)(uint16_t address, uint8_t* buffer, uint16_t length);
	int16_t (*lock)();
	int16_t (*unlock)();
} DeviceInterface;

int16_t readBit(DeviceInterface* device, uint16_t regAddr, uint8_t bitNum, uint8_t *data);
int16_t readBits(DeviceInterface* device, uint16_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
int16_t readByte(DeviceInterface* device, uint16_t regAddr, uint8_t *data);
int16_t readBytes(DeviceInterface* device, uint16_t regAddr, uint8_t length, uint8_t *data);
int16_t writeBit(DeviceInterface* device, uint8_t regAddr, uint8_t bitNum, uint8_t data);
int16_t writeBits(DeviceInterface* device, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
int16_t writeByte(DeviceInterface* device, uint16_t regAddr, uint8_t data);
int16_t writeWord(DeviceInterface* device, uint16_t regAddr, uint16_t data);
int16_t writeBytes(DeviceInterface* device, uint16_t regAddr, uint8_t length, uint8_t *data);

#endif /* DEVICE_INTERFACE_H_ */
