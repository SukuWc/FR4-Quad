#include "stdint.h"
#include "extApi.h"
#include "stdio.h"

extern int clientId;

static char buffer[256];

#define MAX_VELOCITY 7.5f

float getAirVelocityValue(uint16_t pwm) {
	if (pwm > 1024) {
		pwm = 1024;
	}
	return (pwm / 1024.0) * MAX_VELOCITY;
}

void setMotorSpeed(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4) {
	float v1 = getAirVelocityValue(pwm4);
	float v2 = getAirVelocityValue(pwm3);
	float v3 = getAirVelocityValue(pwm1);
	float v4 = getAirVelocityValue(pwm2);
	snprintf(buffer, 256, "%f \n %f \n %f \n %f", v1, v2, v3, v4); //using /n, because /0 will short-circuit, and only print the first value
	int foundCounter = 0;
	for (int i = 0; i < 256; i++) {
		if (buffer[i] == '\n') {
			buffer[i] = '\0';
			foundCounter++;
			if (foundCounter == 3) {
				break;
			}
		}
	}
	simxCallScriptFunction(clientId, "Quadricopter", sim_scripttype_childscript, "setAirVelocities", 0, NULL, 0, NULL, 4, buffer, 0, NULL, 0, NULL, 0, NULL, 0, NULL, 0, NULL, simx_opmode_blocking);
}