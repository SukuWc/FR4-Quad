/*
 * bmp280.c
 *
 *  Created on: 11 Feb 2019
 *      Author: danim
 */

#include "bsp/bmp280.h"
#include "i2cif.h"
#include "math.h"

uint8_t bmp280_init(Bmp280Device* bmp280)
{
  while(bmp280_bmp280Read8(bmp280, BMP280_REG_CHIPID) != 0x58){}


  bmp280->dig_T1 = bmp280_bmp280Read16LE(bmp280, BMP280_REG_DIG_T1);
  bmp280->dig_T2 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_T2);
  bmp280->dig_T3 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_T3);

  bmp280->dig_P1 = bmp280_bmp280Read16LE(bmp280, BMP280_REG_DIG_P1);
  bmp280->dig_P2 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_P2);
  bmp280->dig_P3 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_P3);
  bmp280->dig_P4 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_P4);
  bmp280->dig_P5 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_P5);
  bmp280->dig_P6 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_P6);
  bmp280->dig_P7 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_P7);
  bmp280->dig_P8 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_P8);
  bmp280->dig_P9 = bmp280_bmp280ReadS16LE(bmp280, BMP280_REG_DIG_P9);

  bmp280_writeRegister(bmp280, BMP280_REG_CONFIG, 0);
  bmp280_writeRegister(bmp280, BMP280_REG_CONTROL, 0x3F);
  return 1;
}

float bmp280_getTemperature(Bmp280Device* bmp280)
{
  volatile int32_t var1, var2;

  volatile int32_t adc_T = bmp280_bmp280Read24(bmp280, BMP280_REG_TEMPDATA);
  adc_T >>= 4;
  var1 = (((adc_T >> 3) - ((int32_t)(bmp280->dig_T1 << 1))) *
    ((int32_t)bmp280->dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)bmp280->dig_T1)) *
    ((adc_T >> 4) - ((int32_t)bmp280->dig_T1))) >> 12) *
    ((int32_t)bmp280->dig_T3)) >> 14;

  bmp280->t_fine = var1 + var2;
  float T = (bmp280->t_fine * 5 + 128) >> 8;
  return T / 100;
}

float bmp280_getPressure(Bmp280Device* bmp280)
{
  int64_t var1, var2, p;

  // Call getTemperature to get t_fine
  bmp280_getTemperature(bmp280);

  int32_t adc_P = bmp280_bmp280Read24(bmp280, BMP280_REG_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)bmp280->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)bmp280->dig_P6;
  var2 = var2 + ((var1*(int64_t)bmp280->dig_P5)<<17);
  var2 = var2 + (((int64_t)bmp280->dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)bmp280->dig_P3)>>8) + ((var1 * (int64_t)bmp280->dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp280->dig_P1)>>33;
  if (var1 == 0)
  {
  return 0; // avoid exception caused by division by zero
  }
  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((int64_t)bmp280->dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)bmp280->dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280->dig_P7)<<4);
  bmp280->lastPressure = p/256;
  return p/256;
}

float bmp280_calcAltitude(Bmp280Device* bmp280)
{
  float A = (bmp280->lastPressure)/101325;
  float C = 44330 * (1 - pow(A,0.1903));
  return C;
}

static uint8_t bmp280_bmp280Read8(Bmp280Device* bmp280, uint8_t reg)
{
  uint8_t ret;

  readByte(bmp280->interface, reg, &ret);
  return ret;
}

static uint16_t bmp280_bmp280Read16(Bmp280Device* bmp280, uint8_t reg)
{
  uint8_t buffer[2];
  readBytes(bmp280->interface, reg, 2, buffer);
  uint16_t ret = 0;
  ret |= buffer[0] << 8;
  ret |= buffer[1];
  return ret;
}

static uint16_t bmp280_bmp280Read16LE(Bmp280Device* bmp280, uint8_t reg)
{
  uint16_t data = bmp280_bmp280Read16(bmp280, reg);
  return (data >> 8) | (data << 8);
}

/*static int16_t bmp280_bmp280ReadS16(Bmp280Device* bmp280, uint8_t reg)
{
  return (int16_t)bmp280_bmp280Read16(bmp280, reg);
}*/

static int16_t bmp280_bmp280ReadS16LE(Bmp280Device* bmp280, uint8_t reg)
{
  return (int16_t)bmp280_bmp280Read16LE(bmp280, reg);
}

static int32_t bmp280_bmp280Read24(Bmp280Device* bmp280, uint8_t reg)
{
  uint8_t buffer[4];
  bmp280->interface->read(bmp280->interface, reg, buffer, 3);
  uint32_t ret = 0;
  ret |= buffer[0] << 24;
  ret |= buffer[1] << 16;
  ret |= buffer[2] << 8;
  ret >>= 8;
  return ret;
}

static void bmp280_writeRegister(Bmp280Device* bmp280, uint8_t reg, uint8_t val)
{
  bmp280->interface->write(bmp280->interface, reg, &val, 1);
}
