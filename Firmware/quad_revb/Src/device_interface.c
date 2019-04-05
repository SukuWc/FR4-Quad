/*
 * device_interface.c
 *
 *  Created on: 2019. márc. 28.
 *      Author: danim
 */
#include "device_interface.h"


int16_t readBit(DeviceInterface* device, uint16_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = device->read(device, regAddr, &b, 1);
    *data = (b >> bitNum) & 1;
    return count;
}

int16_t readBits(DeviceInterface* device, uint16_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data){
    uint8_t count, b;
    if ((count = device->read(device, regAddr, &b, 1)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

int16_t readByte(DeviceInterface* device, uint16_t regAddr, uint8_t *data){
	return device->read(device, regAddr, data, 1);
}

int16_t readBytes(DeviceInterface* device, uint16_t regAddr, uint8_t length, uint8_t *data){
	return device->read(device, regAddr, data, length);
}

int16_t writeBit(DeviceInterface* device, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(device, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(device, regAddr, b);
}

int16_t writeBits(DeviceInterface* device, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(device, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(device, regAddr, b);
    } else {
        return 0;
    }
}

int16_t writeByte(DeviceInterface* device, uint16_t regAddr, uint8_t data){
	return device->write(device, regAddr, &data, 1);
}

int16_t writeWord(DeviceInterface* device, uint16_t regAddr, uint16_t data){
	return device->write(device, regAddr, &data, 2);
}

int16_t writeBytes(DeviceInterface* device, uint16_t regAddr, uint8_t length, uint8_t *data){
	return device->write(device, regAddr, data, length);
}
