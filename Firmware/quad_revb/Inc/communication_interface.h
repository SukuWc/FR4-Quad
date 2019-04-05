/*
 * communication_interface.h
 *
 *  Created on: 2019. ápr. 5.
 *      Author: danim
 */

#ifndef COMMUNICATION_INTERFACE_H_
#define COMMUNICATION_INTERFACE_H_
#include "main.h"

typedef struct I2CCommunicationInterface_T{
	I2C_HandleTypeDef* hi2c;
	uint8_t inBlockingMode;
	//SemaphoreHandle_t* blockingMutex;
	uint8_t buffer[32];
} I2CCommunicationInterface;

typedef struct I2CDeviceCommunicationInterface_T{
	uint8_t address;
	I2CCommunicationInterface* i2c_if;
} I2CDeviceCommunicationInterface;

typedef struct SPICommunicationInterface_T{
	SPI_HandleTypeDef* hspi;
	uint8_t inBlockingMode;
	//SemaphoreHandle_t* blockingMutex;
	uint8_t spi_recv_buffer[64];
	uint8_t spi_trans_buffer[64];
} SPICommunicationInterface;

typedef struct SPIDeviceCommunicationInterface_T{
	GPIO_TypeDef* csn_port;
	uint16_t csn_pin;
	SPICommunicationInterface* spi_if;
} SPIDeviceCommunicationInterface;

typedef struct DeviceInterface_T {
	int16_t (*read)(struct DeviceInterface_T* dev, uint16_t address, uint8_t* buffer, uint16_t length);
	int16_t (*write)(struct DeviceInterface_T* dev, uint16_t address, uint8_t* buffer, uint16_t length);
	void* communication_dev_if;
} DeviceInterface;


int16_t i2c_comm_receive(I2CCommunicationInterface* c_if, uint8_t address, uint8_t* buffer, uint16_t length);
int16_t i2c_comm_transfer(I2CCommunicationInterface* c_if, uint8_t address, uint8_t* buffer, uint16_t length);
int16_t i2c_dev_receive(I2CDeviceCommunicationInterface* c_if, uint8_t* buffer, uint16_t length);
int16_t i2c_dev_transfer(I2CDeviceCommunicationInterface* c_if, uint8_t* buffer, uint16_t length);
int16_t i2c_read(DeviceInterface* dev_comm_if, uint16_t address, uint8_t* buffer, uint16_t length);
int16_t i2c_write(DeviceInterface* dev_comm_if, uint16_t address, uint8_t* buffer, uint16_t length);
int16_t spi_comm_receive(SPICommunicationInterface* c_if, uint8_t* buffer, uint16_t length);
int16_t spi_comm_tramsfer_receive(SPICommunicationInterface* c_if, uint8_t* buffer,uint8_t* rx_buffer, uint16_t length);
int16_t spi_comm_transfer(SPICommunicationInterface* c_if, uint8_t* buffer, uint16_t length);
int16_t spi_dev_receive(SPIDeviceCommunicationInterface* spi_dev_comm, uint8_t* buffer, uint16_t length);
int16_t spi_dev_transfer_receive(SPIDeviceCommunicationInterface* spi_dev_comm, uint8_t* buffer, uint8_t* rx_buffer, uint16_t length);
int16_t spi_dev_transfer(SPIDeviceCommunicationInterface* spi_dev_comm, uint8_t* buffer, uint16_t length);
int16_t spi_read(DeviceInterface* dev_comm_if, uint16_t address, uint8_t* buffer, uint16_t length);
int16_t spi_write(DeviceInterface* dev_comm_if, uint16_t address, uint8_t* buffer, uint16_t length);


void initI2cCommunicationInterface(I2CCommunicationInterface* i2c_if, I2C_HandleTypeDef* hi2c);
void initI2cDeviceCommunicationInterface(I2CDeviceCommunicationInterface* i2c_dev_if, I2CCommunicationInterface* i2c_comm_if, uint8_t address);
void initI2cDeviceInterface(DeviceInterface* dev_if, I2CDeviceCommunicationInterface* i2c_dev_if);
void initSPICommunicationInterface(SPICommunicationInterface* spi_if, SPI_HandleTypeDef* hspi);
void initSPIDeviceCommunicationInterface(SPIDeviceCommunicationInterface* spi_dev_if, SPICommunicationInterface* spi_comm_if, GPIO_TypeDef* csn_port, uint16_t csn_pin);
void initSPIDeviceInterface(DeviceInterface* dev_if, SPIDeviceCommunicationInterface* spi_dev_if);
#endif /* COMMUNICATION_INTERFACE_H_ */
