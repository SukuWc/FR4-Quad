#include "stdio.h"
#include "extApi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "core\system_core.h"
#include "core\system_position.h"
#include "math.h"

#define M_RAD_TO_DEG (180.0/M_PI)

int clientId;
xTaskHandle systemCoreTaskHandle;
xTaskHandle simulationTaskHandle;
xTaskHandle systemPositionHandle;
xSemaphoreHandle positionDataMutexHandle;
extern void SystemCoreTask(void const * argument);
extern void SystemPosition(void const * argument);
extern void joystickMain(void const * params);


#define ACC_SENS (32768 / 156.96f)
#define GYRO_SENS (32768 / 2000.0f)

float simulatorRollAngle, simulatorPitchAngle, simulatorYawAngle; 
float simulatorAccX, simulatorAccY, simulatorAccZ;
float simulatorHeight;

void putIntInBuffer(uint8_t *buffer, int16_t value) {
	buffer[0] = (value >> 8) & 0x00ff;
	buffer[1] = value & 0x00ff;
}

void getSensorValues(uint8_t *buffer, uint8_t length) {
	int16_t ax = (simulatorAccX) * ACC_SENS;
	int16_t ay = (simulatorAccY) * ACC_SENS;
	int16_t az = -(simulatorAccZ) * ACC_SENS;
	int16_t gx = -simulatorRollAngle * GYRO_SENS;
	int16_t gy = -simulatorPitchAngle * GYRO_SENS;
	int16_t gz = -simulatorYawAngle * GYRO_SENS;
	int16_t h = (simulatorHeight / 0.09);

	putIntInBuffer(&(buffer[0]), ax);
	putIntInBuffer(&(buffer[2]), ay);
	putIntInBuffer(&(buffer[4]), az);
	putIntInBuffer(&(buffer[8]), gx);
	putIntInBuffer(&(buffer[10]), gy);
	putIntInBuffer(&(buffer[12]), gz);
	putIntInBuffer(&(buffer[25]), h);
	/*int16_t ax = (((int16_t)motionBuffer[0]) << 8) | motionBuffer[1];
	int16_t ay = (((int16_t)motionBuffer[2]) << 8) | motionBuffer[3];
	int16_t az = (((int16_t)motionBuffer[4]) << 8) | motionBuffer[5];
	int16_t rotx = (((int16_t)motionBuffer[8]) << 8) | motionBuffer[9];
	int16_t roty = (((int16_t)motionBuffer[10]) << 8) | motionBuffer[11];
	int16_t rotz = (((int16_t)motionBuffer[12]) << 8) | motionBuffer[13];*/
}

void mainSimulationTask(void *params) {
	simxSynchronous(clientId, 1);
	simxSynchronousTrigger(clientId);
	int quadHandle;
	simxGetObjectHandle(clientId, "Quadricopter", &quadHandle, simx_opmode_blocking);
	while (1)
	{
		simxInt floatCnt;
		simxFloat *sensorRawData;
		simxCallScriptFunction(clientId, "Quadricopter", sim_scripttype_childscript, "getSensorData", 0, NULL, 0, NULL, 0, NULL, 0, NULL, 0, NULL, &floatCnt, &sensorRawData, 0, NULL, 0, NULL, simx_opmode_blocking);
		simulatorAccX = sensorRawData[0];
		simulatorAccY = sensorRawData[1];
		simulatorAccZ = sensorRawData[2];
		simulatorRollAngle = sensorRawData[3] * M_RAD_TO_DEG;
		simulatorPitchAngle = sensorRawData[4] * M_RAD_TO_DEG;
		simulatorYawAngle = sensorRawData[5] * M_RAD_TO_DEG;
		simulatorHeight = sensorRawData[6];

		for (int i = 0; i < 10; i++) {
			sNotifySystemPosition(EVENT_POSITION_UPDATE);
		}
		sNotifySystemCore(EVENT_CORE_CONTROLLER_UPDATE);
		/*sNotifySystemCore(EVENT_CORE_POSITION_UPDATED);
		sNotifySystemCore(EVENT_CONTROLLER_UPDATE);*/
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		simxSynchronousTrigger(clientId);
	}
}

void controlFinished() {
	xTaskNotify(simulationTaskHandle, 0, eNoAction);
}
void main() {


	clientId = simxStart("127.0.0.1", 15678, 1, 0, 50000, 5);
	if (clientId != -1) {
		
		positionDataMutexHandle = xSemaphoreCreateMutex();
		xTaskCreate(mainSimulationTask,			/* The function that implements the task. */
			"SimTask", 							/* The text name assigned to the task - for debug only as it is not used by the kernel. */
			configMINIMAL_STACK_SIZE, 		/* The size of the stack to allocate to the task. */
			NULL, 							/* The parameter passed to the task - not used in this simple case. */
			tskIDLE_PRIORITY + 1,/* The priority assigned to the task. */
			&simulationTaskHandle);
		xTaskCreate(SystemCoreTask,			/* The function that implements the task. */
			"SimTask", 							/* The text name assigned to the task - for debug only as it is not used by the kernel. */
			configMINIMAL_STACK_SIZE, 		/* The size of the stack to allocate to the task. */
			NULL, 							/* The parameter passed to the task - not used in this simple case. */
			tskIDLE_PRIORITY + 2,/* The priority assigned to the task. */
			&systemCoreTaskHandle);
		xTaskCreate(SystemPosition,			/* The function that implements the task. */
			"SimTask", 							/* The text name assigned to the task - for debug only as it is not used by the kernel. */
			configMINIMAL_STACK_SIZE, 		/* The size of the stack to allocate to the task. */
			NULL, 							/* The parameter passed to the task - not used in this simple case. */
			tskIDLE_PRIORITY + 2,/* The priority assigned to the task. */
			&systemPositionHandle);
		xTaskCreate(joystickMain,			/* The function that implements the task. */
			"Joystick", 							/* The text name assigned to the task - for debug only as it is not used by the kernel. */
			configMINIMAL_STACK_SIZE, 		/* The size of the stack to allocate to the task. */
			NULL, 							/* The parameter passed to the task - not used in this simple case. */
			tskIDLE_PRIORITY + 3,/* The priority assigned to the task. */
			NULL);

		vTaskStartScheduler();
	} else {
		printf("Could not connect!");
	}

	for (;;) {}
	/*printf("Hello World!\r\n");
	int result = simxStart("127.0.0.1", 15678, 1, 0, 5000, 5);
	printf("%d\n", result);
	if (result != -1) {
		simxSynchronous(result, 1);
		int counter = 0;
		int inUpState = 1;
		while (1) {
			counter++;
			if (counter == 100 && inUpState) {
				inUpState = 0;
				counter = 0;
			}
			else if (inUpState == 0 && counter == 200) {
				inUpState = 1;
				counter = 0;
			}
			if (inUpState) {
				int callRes = simxCallScriptFunction(result, "Quadricopter", sim_scripttype_childscript, "setAirVelocities", 0, NULL, 0, NULL, 4, "5.8\0 5.8\0 5.8\0 5.8", 0, NULL, 0, NULL, 0, NULL, 0, NULL, 0, NULL, simx_opmode_blocking);
			} else {
				int callRes = simxCallScriptFunction(result, "Quadricopter", sim_scripttype_childscript, "setAirVelocities", 0, NULL, 0, NULL, 4, "0\0 0\0 0\0 0", 0, NULL, 0, NULL, 0, NULL, 0, NULL, 0, NULL, simx_opmode_blocking);
			}
			simxSynchronousTrigger(result);
		}
		/*simxInt scriptHandle = -1;
		int test = sim_handle_all;
		float speed = 1000.0f;
		//simxCallScriptFunction(result, "", sim_scripttype_mainscript, "sim.getScriptHandle", 0, NULL, 0, NULL, 1, "Quadricopter_propeller_respondable1\0", 0, NULL, 1, &(scriptHandles[0]), 0, NULL, 0, NULL, 0, NULL, simx_opmode_blocking);
		int callRes = simxCallScriptFunction(result, "Quadricopter", sim_scripttype_childscript, "setAirVelocities", 0, NULL, 0, NULL, 4, "5.8\0 5.8\0 5.8\0 5.8", 0, NULL, 0, NULL, 0, NULL, 0, NULL, 0, NULL, simx_opmode_blocking);
		simxFinish(result);*/
}