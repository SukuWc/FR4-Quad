#include "stdio.h"
#include "extApi.h"
#include "FreeRTOS.h"
#include "task.h"


extern uint32_t SystemCoreClock;
void testFun(void *params) {
	while (1)
	{
		printf("%d", SystemCoreClock);
	}
}
void main() {
	xTaskCreate(testFun,			/* The function that implements the task. */
		"Rx", 							/* The text name assigned to the task - for debug only as it is not used by the kernel. */
		configMINIMAL_STACK_SIZE, 		/* The size of the stack to allocate to the task. */
		NULL, 							/* The parameter passed to the task - not used in this simple case. */
		tskIDLE_PRIORITY + 1,/* The priority assigned to the task. */
		NULL);							/* The task handle is not required, so NULL is passed. */

	vTaskStartScheduler();

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