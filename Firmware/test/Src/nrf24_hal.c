#include "nrf24_hal.h"


// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately
void nRF24_GPIO_Init(void) {
    GPIO_InitTypeDef PORT;

    // Enable the nRF24L01 GPIO peripherals
	//RCC->APB2ENR |= nRF24_GPIO_PERIPHERALS;

    // Configure CSN pin
	PORT.Mode = GPIO_MODE_OUTPUT_PP;
	PORT.Speed = GPIO_SPEED_FREQ_HIGH;
	PORT.Pin = nRF24_CSN_PIN;
	HAL_GPIO_Init(nRF24_CSN_PORT, &PORT);
	nRF24_CSN_H();

	// Configure CE pin
	PORT.Pin = nRF24_CE_PIN;
	HAL_GPIO_Init(nRF24_CE_PORT, &PORT);
	nRF24_CE_L();
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
	 // Wait until TX buffer is empty
	while (HAL_SPI_GetState(nRF24_SPI_PORT) == HAL_SPI_STATE_BUSY_TX);
	// Send byte to SPI (TXE cleared)
	HAL_SPI_Transmit(nRF24_SPI_PORT, &data, sizeof(uint8_t), HAL_MAX_DELAY);
	// Wait while receive buffer is empty
	HAL_SPI_Receive(nRF24_SPI_PORT, &data, sizeof(uint8_t), HAL_MAX_DELAY);
	// Return received byte
	return data;
}
