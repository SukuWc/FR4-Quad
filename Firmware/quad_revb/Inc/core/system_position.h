/*
 * system_position.h
 *
 *  Created on: 12 May 2019
 *      Author: danim
 */

#ifndef CORE_SYSTEM_POSITION_H_
#define CORE_SYSTEM_POSITION_H_

enum SystemPositionNotificationFlags_T {
	EVENT_POSITION_UPDATE = 0
};

#define sNotifySystemPosition(event) xTaskNotify(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits)
#define sNotifySystemPositionFromISR(event, pxHigherPriorityTaskWoken) xTaskNotifyFromISR(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits, pxHigherPriorityTaskWoken)

//Protected by positionDataMutex
extern volatile float pitch = 0;
extern volatile float roll = 0;
extern volatile float yawn = 0;
extern volatile float yawSpeed = 0;
extern volatile float pitchSpeed = 0;
extern volatile float rollSpeed = 0;
extern volatile float height = 0;

#endif /* CORE_SYSTEM_POSITION_H_ */
