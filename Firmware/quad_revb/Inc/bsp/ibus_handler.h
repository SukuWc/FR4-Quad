/*
 * ibus_handler.h
 *
 *  Created on: 2019. ápr. 5.
 *      Author: danim
 */

#ifndef BSP_IBUS_HANDLER_H_
#define BSP_IBUS_HANDLER_H_

extern uint16_t channel_values[14];

void initIbusHandler();
void onUartReceiveFinished();
uint8_t receiverValid();

#endif /* BSP_IBUS_HANDLER_H_ */
