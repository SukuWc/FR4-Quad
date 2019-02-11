/*
 * i2cif.h
 *
 *  Created on: 10 Feb 2019
 *      Author: danim
 */

#ifndef I2CIF_H_
#define I2CIF_H_

#include "main.h"

int8_t i2c_readBit(I2C_HandleTypeDef *i2c, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
int8_t i2c_readBits(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
int8_t i2c_readBitsW(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data);
int8_t i2c_readBitW(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data);
int8_t i2c_readByte(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t *data);
int8_t i2c_readBytes(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t i2c_readWord(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint16_t *data);
int8_t i2c_readWords(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);
uint8_t i2c_writeBits(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t i2c_writeBitsW(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
uint8_t i2c_writeBit(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
uint8_t i2c_writeBitW(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
uint8_t i2c_writeBytes(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);
uint8_t i2c_writeByte(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t data);
uint8_t i2c_writeWords(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data);
uint8_t i2c_writeWord(I2C_HandleTypeDef *i2c,uint8_t devAddr, uint8_t regAddr, uint16_t data);

#endif /* I2CIF_H_ */
