/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t receiver = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the address for RX pipe
  if (receiver){
    nRF24_DisableAA(0xFF); // disable ShockBurst
    nRF24_SetRFChannel(115); // set RF channel to 2490MHz
    nRF24_SetDataRate(nRF24_DR_250kbps); // 2Mbit/s data rate
    nRF24_SetCRCScheme(nRF24_CRC_off); // 1-byte CRC scheme
    nRF24_SetAddrWidth(5); // address width is 5 bytes
    nRF24_SetAddr(nRF24_PIPE1, ADDR); // program pipe address
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 13); // enable RX pipe#1 with Auto-ACK: disabled, payload length: 10 bytes
    nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
    nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
    // then pull CE pin to HIGH, and the nRF24 will start a receive...
  } else {
    nRF24_DisableAA(0xFF); // disable ShockBurst
    nRF24_SetRFChannel(115); // set RF channel to 2490MHz
    nRF24_SetDataRate(nRF24_DR_250kbps); // 2Mbit/s data rate
    nRF24_SetCRCScheme(nRF24_CRC_off); // 1-byte CRC scheme
    nRF24_SetAddrWidth(5); // address width is 5 bytes
    nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power
    nRF24_SetAddr(nRF24_PIPETX, ADDR); // program TX address
    nRF24_SetOperationalMode(nRF24_MODE_TX); // switch transceiver to the TX mode
    nRF24_ClearIRQFlags(); // clear any pending IRQ bits
    nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
    // the nRF24 is ready for transmission, upload a payload, then pull CE pin to HIGH and it will transmit a packet...
  }

  volatile uint8_t nrf24_status = nRF24_Check();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  volatile uint8_t status;
  volatile uint8_t ret;
  volatile uint8_t cnt = 0;
  while (1)
  {
    /* USER CODE END WHILE */
    if (receiver){
      uint8_t nRF24_payload[32]; // buffer for payload
      uint8_t payload_length; // variable to store a length of received payload
      uint8_t pipe; // pipe number
      nRF24_CE_H(); // start receiving
      while (1) {
          // constantly poll the status of RX FIFO...
          if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
              // the RX FIFO have some data, take a note what nRF24 can hold up to three payloads of 32 bytes...
              pipe = nRF24_ReadPayload(nRF24_payload, &payload_length); // read a payload to buffer
              nRF24_ClearIRQFlags(); // clear any pending IRQ bits
              // now the nRF24_payload buffer holds received data
              // payload_length variable holds a length of received data
              // pipe variable holds a number of the pipe which has received the data
              // ... do something with received data ...
              cnt++;
          }
      }
    } else {
      uint8_t payload[] = "Hello World!\n";
      payload[0] = cnt++;
      nRF24_CE_L();
      nRF24_WritePayload(payload, 13); // transfer payload data to transceiver
      nRF24_CE_H(); // assert CE pin (transmission starts)
      while (1) {
          status = nRF24_GetStatus();
          if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
              // transmission ended, exit loop
              break;
          }
      }
      nRF24_CE_L(); // de-assert CE pin (nRF24 goes to StandBy-I mode)
      nRF24_ClearIRQFlags(); // clear any pending IRQ flags
      if (status & nRF24_FLAG_MAX_RT) {
          // Auto retransmit counter exceeds the programmed maximum limit (payload in FIFO is not removed)
          // Also the software can flush the TX FIFO here...
          ret = -1;
          continue;
      }
      if (status & nRF24_FLAG_TX_DS) {
          // Successful transmission
          ret = 0;
          continue;
      }
      nRF24_FlushTX();
      // In fact that should not happen
      ret = -2;
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
