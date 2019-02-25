#include "nrf24_hal.h"


// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately
void nRF24_GPIO_Init(void) {

}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
	 // Wait until TX buffer is empty
	while (__HAL_SPI_GET_FLAG(nRF24_SPI_HANDLE, SPI_FLAG_TXE) == RESET);
	// Send byte to SPI (TXE cleared)
  uint8_t ret;
	HAL_SPI_TransmitReceive(nRF24_SPI_HANDLE, &data, &ret, 1, HAL_MAX_DELAY);
	// Wait while receive buffer is empty
	// while (__HAL_SPI_GET_FLAG(nRF24_SPI_HANDLE, SPI_FLAG_RXNE) == SET);

	// Return received byte
	return ret;
}
