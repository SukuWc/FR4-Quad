/*
 * logger.h
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */

#ifndef BSP_LOGGER_H_
#define BSP_LOGGER_H_
#include "main.h"

void sendMessage(uint8_t *buffer, uint16_t bufferLength);
void mesageSentFromISR();

#endif /* BSP_LOGGER_H_ */
