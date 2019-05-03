#include "stdio.h"
#include "extApi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "core\system_core.h"
#include "math.h"

#define M_RAD_TO_DEG (180.0/M_PI)

int clientId;
xTaskHandle systemCoreTaskHandle;
xTaskHandle simulationTaskHandle;
extern void SystemCoreTask(void const * argument);
extern void joystickMain(void const * params);

float simulatorRollAngle, simulatorPitchAngle, simulatorYawAngle;
void mainSimulationTask(void *params) {
	simxSynchronous(clientId, 1);
	simxSynchronousTrigger(clientId);
	int quadHandle;
	simxGetObjectHandle(clientId, "Quadricopter", &quadHandle, simx_opmode_blocking);
	while (1)
	{
		static float angles[3];
		simxGetObjectOrientation(clientId, quadHandle, -1, angles, simx_opmode_blocking);
		simulatorRollAngle = angles[0] * M_RAD_TO_DEG;
		simulatorPitchAngle = angles[1] * M_RAD_TO_DEG;
		simulatorYawAngle = angles[2] * M_RAD_TO_DEG;

		sNotifySystemCore(EVENT_POSITION_UPDATE);
		sNotifySystemCore(EVENT_CONTROLLER_UPDATE);
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
			tskIDLE_PRIORITY + 1,/* The priority assigned to the task. */
			&systemCoreTaskHandle);
		xTaskCreate(joystickMain,			/* The function that implements the task. */
			"Joystick", 							/* The text name assigned to the task - for debug only as it is not used by the kernel. */
			configMINIMAL_STACK_SIZE, 		/* The size of the stack to allocate to the task. */
			NULL, 							/* The parameter passed to the task - not used in this simple case. */
			tskIDLE_PRIORITY + 2,/* The priority assigned to the task. */
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