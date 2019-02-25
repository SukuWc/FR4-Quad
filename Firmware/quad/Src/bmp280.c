/*
 * bmp280.c
 *
 *  Created on: 11 Feb 2019
 *      Author: danim
 */

#include "bmp280.h"
#include "i2cif.h"

static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;

static int32_t t_fine;

uint8_t bmp280_init(void)
{
  if(bmp280_bmp280Read8(BMP280_REG_CHIPID) != 0x58)
    return 0;

  dig_T1 = bmp280_bmp280Read16LE(BMP280_REG_DIG_T1);
  dig_T2 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_T2);
  dig_T3 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_T3);

  dig_P1 = bmp280_bmp280Read16LE(BMP280_REG_DIG_P1);
  dig_P2 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_P2);
  dig_P3 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_P3);
  dig_P4 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_P4);
  dig_P5 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_P5);
  dig_P6 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_P6);
  dig_P7 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_P7);
  dig_P8 = bmp280_bmp280ReadS16LE(BMP280_REG_DIG_P8);
  dig_P9 =bmp280_bmp280ReadS16LE(BMP280_REG_DIG_P9);

  bmp280_writeRegister(BMP280_REG_CONFIG, 0);
  bmp280_writeRegister(BMP280_REG_CONTROL, 0x3F);
  return 1;
}

float bmp280_getTemperature(void)
{
  int32_t var1, var2;

  int32_t adc_T = bmp280_bmp280Read24(BMP280_REG_TEMPDATA);
  adc_T >>= 4;
  var1 = (((adc_T >> 3) - ((int32_t)(dig_T1 << 1))) *
    ((int32_t)dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
    ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
    ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return T/100;
}

float bmp280_getPressure(void)
{
  int64_t var1, var2, p;

  // Call getTemperature to get t_fine
  bmp280_getTemperature();

  int32_t adc_P = bmp280_bmp280Read24(BMP280_REG_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
  var2 = var2 + (((int64_t)dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
  if (var1 == 0)
  {
  return 0; // avoid exception caused by division by zero
  }
  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
  return p/256;
}

float bmp280_calcAltitude(float pressure)
{
  float A = pressure/101325;
  float C = 44330 * (1 - pow(A,0.1903));
  return C;
}

uint8_t bmp280_bmp280Read8(uint8_t reg)
{
  uint8_t ret;
  i2c_readByte(I2C_HANDLE, BMP280_ADDRESS, reg, &ret);
  return ret;
}

uint16_t bmp280_bmp280Read16(uint8_t reg)
{
  uint16_t ret;
  i2c_readWord(I2C_HANDLE, BMP280_ADDRESS, reg, &ret);
  return ret;
}

uint16_t bmp280_bmp280Read16LE(uint8_t reg)
{
  uint16_t data = bmp280_bmp280Read16(reg);
  return (data >> 8) | (data << 8);
}

int16_t bmp280_bmp280ReadS16(uint8_t reg)
{
  return (int16_t)bmp280_bmp280Read16(reg);
}

int16_t bmp280_bmp280ReadS16LE(uint8_t reg)
{
  return (int16_t)bmp280_bmp280Read16LE(reg);
}

uint32_t bmp280_bmp280Read24(uint8_t reg)
{
  uint32_t data;
  i2c_readBytes(I2C_HANDLE, BMP280_ADDRESS, reg, 3, (uint8_t*)&data);
  return data;
}

void bmp280_writeRegister(uint8_t reg, uint8_t val)
{
  i2c_writeByte(I2C_HANDLE, BMP280_ADDRESS, reg, val);
}
