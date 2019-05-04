/*
 * bmp280.h
 *
 *  Created on: 11 Feb 2019
 *      Author: danim
 */

#ifndef BMP280_H_
#define BMP280_H_

#include "main.h"
#include "device_interface.h"

typedef struct Bmp280Device_T{
	DeviceInterface* interface;
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	float lastPressure;

	int32_t t_fine;
} Bmp280Device;
extern I2C_HandleTypeDef hi2c2;
#define I2C_HANDLE &hi2c2

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

static uint16_t bmp280_bmp280Read16LE(Bmp280Device* bmp280, uint8_t reg);
static uint16_t bmp280_bmp280Read16(Bmp280Device* bmp280, uint8_t reg);
static int32_t bmp280_bmp280Read24(Bmp280Device* bmp280, uint8_t reg);
static uint8_t bmp280_bmp280Read8(Bmp280Device* bmp280, uint8_t reg);
static int16_t bmp280_bmp280ReadS16LE(Bmp280Device* bmp280, uint8_t reg);
//static int16_t bmp280_bmp280ReadS16(Bmp280Device* bmp280, uint8_t reg);
float bmp280_calcAltitude(Bmp280Device* bmp280);
float bmp280_getPressure(Bmp280Device* bmp280);
float bmp280_getTemperature(Bmp280Device* bmp280);
uint8_t bmp280_init(Bmp280Device* bmp280);
static void bmp280_writeRegister(Bmp280Device* bmp280, uint8_t reg, uint8_t val);

#endif /* BMP280_H_ */
