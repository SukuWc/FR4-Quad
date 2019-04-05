/*
 * communication_interface.c
 *
 *  Created on: 2019. ápr. 5.
 *      Author: danim
 */
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "communication_interface.h"

 int16_t i2c_comm_transfer (I2CCommunicationInterface* c_if, uint8_t address, uint8_t* buffer, uint16_t length){
	if (c_if->inBlockingMode){
		HAL_I2C_Master_Transmit(c_if->hi2c, address << 1, buffer, length, HAL_MAX_DELAY);
	}
	return length;
}
 int16_t i2c_comm_receive (I2CCommunicationInterface* c_if, uint8_t address, uint8_t* buffer, uint16_t length){
	if (c_if->inBlockingMode){
		HAL_I2C_Master_Receive(c_if->hi2c, address << 1, buffer, length, HAL_MAX_DELAY);
	}
	return length;
}
 int16_t i2c_dev_transfer (I2CDeviceCommunicationInterface* c_if, uint8_t* buffer, uint16_t length){
	return i2c_comm_transfer(c_if->i2c_if, c_if->address, buffer, length);
}
 int16_t i2c_dev_receive (I2CDeviceCommunicationInterface* c_if, uint8_t* buffer, uint16_t length){
	return i2c_comm_receive(c_if->i2c_if, c_if->address, buffer, length);
}
 int16_t spi_comm_transfer (SPICommunicationInterface* c_if, uint8_t* buffer, uint16_t length){
	if (c_if->inBlockingMode){
		HAL_SPI_Transmit(c_if->hspi, buffer, length, HAL_MAX_DELAY);
	}
	return length;
}

 int16_t spi_comm_receive (SPICommunicationInterface* c_if, uint8_t* buffer, uint16_t length){
	if (c_if->inBlockingMode){
		HAL_SPI_Receive(c_if->hspi, buffer, length, HAL_MAX_DELAY);
	}
	return length;
}

 int16_t spi_comm_tramsfer_receive (SPICommunicationInterface* c_if, uint8_t* buffer,uint8_t* rx_buffer, uint16_t length){
	if (c_if->inBlockingMode){
		HAL_SPI_TransmitReceive(c_if->hspi, buffer, rx_buffer, length, HAL_MAX_DELAY);
	}
	return length;
}

 int16_t spi_dev_transfer (SPIDeviceCommunicationInterface* spi_dev_comm, uint8_t* buffer, uint16_t length){
	HAL_GPIO_WritePin(spi_dev_comm->csn_port, spi_dev_comm->csn_pin, GPIO_PIN_RESET);
	int16_t ret = spi_comm_transfer(spi_dev_comm->spi_if, buffer, length);
	HAL_GPIO_WritePin(spi_dev_comm->csn_port, spi_dev_comm->csn_pin, GPIO_PIN_SET);
	return ret;
}

 int16_t spi_dev_receive (SPIDeviceCommunicationInterface* spi_dev_comm, uint8_t* buffer, uint16_t length){
	HAL_GPIO_WritePin(spi_dev_comm->csn_port, spi_dev_comm->csn_pin, GPIO_PIN_RESET);
	int16_t ret = spi_comm_receive(spi_dev_comm->spi_if, buffer, length);
	HAL_GPIO_WritePin(spi_dev_comm->csn_port, spi_dev_comm->csn_pin, GPIO_PIN_SET);
	return ret;
}

 int16_t spi_dev_transfer_receive (SPIDeviceCommunicationInterface* spi_dev_comm, uint8_t* buffer, uint8_t* rx_buffer, uint16_t length) {
	HAL_GPIO_WritePin(spi_dev_comm->csn_port, spi_dev_comm->csn_pin, GPIO_PIN_RESET);
	int16_t ret = spi_comm_tramsfer_receive(spi_dev_comm->spi_if, buffer, rx_buffer, length);
	HAL_GPIO_WritePin(spi_dev_comm->csn_port, spi_dev_comm->csn_pin, GPIO_PIN_SET);
	return ret;
}

 int16_t i2c_read(DeviceInterface* dev_comm_if, uint16_t address, uint8_t* buffer, uint16_t length){
	I2CDeviceCommunicationInterface* c_if = (I2CDeviceCommunicationInterface*)(dev_comm_if->communication_dev_if);
	uint8_t addr = address;
	if (i2c_dev_transfer(c_if, &addr, 1) == 1){
		if (i2c_dev_receive(c_if, buffer, length)){
			return length;
		} else {
			return -1;
		}
	} else {
		return -1;
	}
}

 int16_t i2c_write(DeviceInterface* dev_comm_if, uint16_t address, uint8_t* buffer, uint16_t length){
	I2CDeviceCommunicationInterface* c_if = (I2CDeviceCommunicationInterface*)(dev_comm_if->communication_dev_if);
	uint8_t *i2c_buffer = c_if->i2c_if->buffer;
	i2c_buffer[0] = address;
	for (uint16_t i = 0; i < length; i++){
		i2c_buffer[i+1] = buffer[i];
	}
	if (i2c_dev_transfer(c_if, i2c_buffer, length + 1) == length + 1){
		return length;
	} else {
		return -1;
	}
}

 int16_t spi_read(DeviceInterface* dev_comm_if, uint16_t address, uint8_t* buffer, uint16_t length){
	SPIDeviceCommunicationInterface* c_if = (SPIDeviceCommunicationInterface*)(dev_comm_if->communication_dev_if);
	uint8_t* spi_buffer = c_if->spi_if->spi_trans_buffer;
	uint8_t* rx_buffer = c_if->spi_if->spi_recv_buffer;
	spi_buffer[0] = address;
	spi_buffer[0] |= 0x80;
	if (spi_dev_transfer_receive(c_if, spi_buffer, rx_buffer, length + 1) == length + 1){
		for (uint16_t i = 0; i < length; i++){
			buffer[i] = rx_buffer[i + 1];
		}
		return length;
	} else {
		return -1;
	}
}
 int16_t spi_write(DeviceInterface* dev_comm_if, uint16_t address, uint8_t* buffer, uint16_t length){
	SPIDeviceCommunicationInterface* c_if = (SPIDeviceCommunicationInterface*)(dev_comm_if->communication_dev_if);
	uint8_t* spi_buffer = c_if->spi_if->spi_recv_buffer;
	spi_buffer[0] = address;
	spi_buffer[0] &= 0x7f;
	for (uint16_t i = 0; i < length; i++){
		spi_buffer[i+1] = buffer[i];
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	if (spi_dev_transfer(c_if, spi_buffer, length + 1) == length + 1){
		return length;
	} else {
		return -1;
	}
}




void initSPICommunicationInterface(SPICommunicationInterface* spi_if, SPI_HandleTypeDef* hspi){
	spi_if->hspi = hspi;
	spi_if->inBlockingMode = 1;
}
void initSPIDeviceCommunicationInterface(SPIDeviceCommunicationInterface* spi_dev_if, SPICommunicationInterface* spi_comm_if, GPIO_TypeDef* csn_port, uint16_t csn_pin){
	spi_dev_if->spi_if = spi_comm_if;
	spi_dev_if->csn_pin = csn_pin;
	spi_dev_if->csn_port = csn_port;
}
void initSPIDeviceInterface(DeviceInterface* dev_if, SPIDeviceCommunicationInterface* spi_dev_if){
	dev_if->read = spi_read;
	dev_if->write = spi_write;
	dev_if->communication_dev_if = spi_dev_if;
}
void initI2cCommunicationInterface(I2CCommunicationInterface* i2c_if, I2C_HandleTypeDef* hi2c){
	i2c_if->hi2c = hi2c;
	i2c_if->inBlockingMode = 1;
}
void initI2cDeviceCommunicationInterface(I2CDeviceCommunicationInterface* i2c_dev_if, I2CCommunicationInterface* i2c_comm_if, uint8_t address){
	i2c_dev_if->address = address;
	i2c_dev_if->i2c_if = i2c_comm_if;
}
void initI2cDeviceInterface(DeviceInterface* dev_if, I2CDeviceCommunicationInterface* i2c_dev_if){
	dev_if->read = i2c_read;
	dev_if->write = i2c_write;
	dev_if->communication_dev_if = i2c_dev_if;
}
