/*
 * bmp280.h
 *
 *  Created on: 11 Feb 2019
 *      Author: danim
 */

#ifndef BMP280_H_
#define BMP280_H_

#include "main.h"
#include "i2c.h"

#define I2C_HANDLE &hi2c2

#define BMP280_ADDRESS   0x77

#define BMP280_REG_DIG_T1    0x88
#define BMP280_REG_DIG_T2    0x8A
#define BMP280_REG_DIG_T3    0x8C

#define BMP280_REG_DIG_P1    0x8E
#define BMP280_REG_DIG_P2    0x90
#define BMP280_REG_DIG_P3    0x92
#define BMP280_REG_DIG_P4    0x94
#define BMP280_REG_DIG_P5    0x96
#define BMP280_REG_DIG_P6    0x98
#define BMP280_REG_DIG_P7    0x9A
#define BMP280_REG_DIG_P8    0x9C
#define BMP280_REG_DIG_P9    0x9E

#define BMP280_REG_CHIPID          0xD0
#define BMP280_REG_VERSION         0xD1
#define BMP280_REG_SOFTRESET       0xE0

#define BMP280_REG_CONTROL         0xF4
#define BMP280_REG_CONFIG          0xF5
#define BMP280_REG_PRESSUREDATA    0xF7
#define BMP280_REG_TEMPDATA        0xFA

uint16_t bmp280_bmp280Read16LE(uint8_t reg);
uint16_t bmp280_bmp280Read16(uint8_t reg);
int32_t bmp280_bmp280Read24(uint8_t reg);
uint8_t bmp280_bmp280Read8(uint8_t reg);
int16_t bmp280_bmp280ReadS16(uint8_t reg);
int16_t bmp280_bmp280ReadS16LE(uint8_t reg);
float bmp280_calcAltitude(float pressure);
float bmp280_getPressure(void);
float bmp280_getTemperature(void);
uint8_t bmp280_init(void);
void bmp280_writeRegister(uint8_t reg, uint8_t val);

#endif /* BMP280_H_ */
