/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "tim.h"
#include "stdio.h"
#include "usart.h"
#include "mpu9250.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	volatile uint8_t valid = 0;
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */

uint32_t lastMotorValid;
static uint8_t buffer[256];
extern uint16_t motorValue;
extern uint16_t values[8];
extern uint16_t rawValues[8];
extern uint8_t motorValid;
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  //uint16_t counter = 800;
  /*mpu9250_initialize();
  volatile uint8_t status2 = bmp280_init();*/

	htim2.Instance->CCR1 = 1023;
	htim3.Instance->CCR2 = 1023;
	htim3.Instance->CCR4 = 1023;
	htim4.Instance->CCR4 = 1023;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	mpu9250_initialize();
	mpu9250_setFChoice_b(0);
	mpu9250_setDLPFMode(3);
	mpu9250_setAccelDPFL(3);
	//bmp280_init();

	int16_t ax, ay, az, rotx, roty, rotz, mgx, mgy, mgz;
	uint8_t msgLength;
  for(;;)
  {
    /* Infinite loop */
	uint32_t currentTick = HAL_GetTick();
	if (motorValid){
		motorValid = 0;
		lastMotorValid = xTaskGetTickCount();
	}
    mpu9250_getMotion6(&ax, &ay, &az, &rotx, &roty, &rotz/*, &mgx, &mgy, &mgz*/);

    int16_t accel_x, accel_y, accel_z, gyro_ro, gyro_pi, gyro_ya;
    int16_t user_ro, user_pi, user_ya;
    int16_t user_th;

    accel_x = -ay;
    accel_y = -ax;
	accel_z =  az;

	gyro_ro = -rotx;
    gyro_pi =  roty;
	gyro_ya = -rotz;

	int16_t userInputMin = 1000;
	int16_t userInputMax = 2000;
	int16_t userInputAvg = (userInputMin + userInputMax) / 2;
	int16_t inOutRatio = 4;
	int16_t userOutputMin = -256;
	int16_t userOutputMax = 256;
	user_th = (rawValues[2] - userInputMin) / inOutRatio;
	if (user_th < 0) user_th = 0; else if (user_th > userOutputMax*2) user_th = userOutputMax*2;
	user_ro = -((rawValues[0] - userInputAvg) / inOutRatio + 1);
	if (user_ro < userOutputMin) user_ro = userOutputMin; else if (user_ro > userOutputMax) user_ro = userOutputMax;
	user_pi = (rawValues[1] - userInputAvg) / inOutRatio;
	if (user_pi < userOutputMin) user_pi = userOutputMin; else if (user_pi > userOutputMax) user_pi = userOutputMax;
	user_ya = -((rawValues[3] - userInputAvg) / inOutRatio);
	if (user_ya < userOutputMin) user_ya = userOutputMin; else if (user_ya > userOutputMax) user_ya = userOutputMax;

	/* TOP VIEW OF THE QUADCOPTER


	              < +Roll +Yaw

	              Y

	              ^
	        >     |     <
	      ((4))   |   ((2))
	        <     |     >
	              |
	              |
	              ----------> X    ^ +Pitch

	        <           >
	      ((3))       ((1))
	        >           <


	*/


	// gyro range: +- 500 °/s
	// acce range: +- 20 m/s^2


	#define PWM_MAX 1023
	#define PWM_MIN 0

#define GYRO_GAIN 0.15
#define ACCEL_GAIN 0.1

	int32_t ro = 0; // signed int, 0 is center, positive rolls left
	int32_t pi = 0; // signed int, 0 is center, positive pitches forward
	int32_t th = 0; // unsigned, 0 is minimum
	int32_t ya = 0; // signed int, 0 is center, positive turns left

	int16_t mulResX = ACCEL_GAIN * accel_x;
	int16_t mulResY = ACCEL_GAIN * accel_y;
	int16_t gyroYawn = GYRO_GAIN * gyro_ya;
	ro = 9 * user_ro + /*GYRO_GAIN * gyro_ro +*/ mulResX;
	pi = 9 * user_pi + /*GYRO_GAIN * gyro_pi +*/ mulResY;
	ya = 9 * user_ya + gyroYawn;

	//pi = 0; ya = 0;
	th = user_th * 4;

	int32_t pwm_4 = th - ro - pi + ya  ;
	int32_t pwm_3 = th - ro + pi - ya  ;

	int32_t pwm_1 = th + ro + pi + ya  ;
	int32_t pwm_2 = th + ro - pi - ya  ;

	pwm_1 = (pwm_1>PWM_MAX)? PWM_MAX: pwm_1;
	pwm_2 = (pwm_2>PWM_MAX)? PWM_MAX: pwm_2;
	pwm_3 = (pwm_3>PWM_MAX)? PWM_MAX: pwm_3;
	pwm_4 = (pwm_4>PWM_MAX)? PWM_MAX: pwm_4;

	pwm_1 = (pwm_1<PWM_MIN)? PWM_MIN: pwm_1;
	pwm_2 = (pwm_2<PWM_MIN)? PWM_MIN: pwm_2;
	pwm_3 = (pwm_3<PWM_MIN)? PWM_MIN: pwm_3;
	pwm_4 = (pwm_4<PWM_MIN)? PWM_MIN: pwm_4;

	if (user_th < 10){
		pwm_1 = 0;
		pwm_2 = 0;
		pwm_3 = 0;
		pwm_4 = 0;
	}
	//msgLength = snprintf(buffer, 256, "PWM1: %+4d PWM2: %+4d PWM3: %+4d PWM4: %+4d\r\n", pwm_1, pwm_2, pwm_3, pwm_4);

	//msgLength = snprintf(buffer, 255, "Roll: %+4d UserRoll: %+4d PWM1: %+4d PWM3: %+4d, Accelx: %+4d, GAIN: %+4d\r\n", ro, user_ro, pwm_1, pwm_3, accel_x, mulRes);

	pwm_1 = 1023 - pwm_1;
	pwm_2 = 1023 - pwm_2;
	pwm_3 = 1023 - pwm_3;
	pwm_4 = 1023 - pwm_4;



	if (currentTick > 100 && currentTick - lastMotorValid < pdMS_TO_TICKS(100)){
		int16_t value = 1000 - motorValue;
		if (value < 0){
			value = 0;
		}
	    htim2.Instance->CCR1 = pwm_3;
	    htim3.Instance->CCR2 = pwm_2;
	    htim3.Instance->CCR4 = pwm_1;
	    htim4.Instance->CCR4 = pwm_4;
	} else {
	    htim2.Instance->CCR1 = 1023;
	    htim3.Instance->CCR2 = 1023;
	    htim3.Instance->CCR4 = 1023;
	    htim4.Instance->CCR4 = 1023;
	}

	//msgLength = snprintf(buffer, 256, "PWM1: %+4d PWM2: %+4d PWM3: %+4d PWM4: %+4d\r\n", pwm_1, pwm_2, pwm_3, pwm_4);


    //msgLength = snprintf(buffer, 256, "%+8d %+8d %+8d %+8d %+8d %+8d %+8d %+8d %+8d \r\n", ax , ay , az , rotx , roty , rotz , mgx , mgy , mgz );
    //msgLength = snprintf(buffer, 256, "Roll: %+4d Pitch: %+4d Yaw: %+4d Throttle: %+4d\r\n", user_ro, user_pi, user_ya, user_th );
	//msgLength = snprintf(buffer, 256, "Roll: %+4d Pitch: %+4d Yaw: %+4d Throttle: %+4d\r\n", ro, pi, ya, th );
	HAL_UART_Transmit(&huart1, buffer, msgLength, HAL_MAX_DELAY);
	uint16_t deltaTick = HAL_GetTick() - currentTick;
	if (deltaTick < pdMS_TO_TICKS(1)){
		vTaskDelay(pdMS_TO_TICKS(1) - deltaTick);
	}
    /*if(counter == 999){
      counter = 0;
    } else {
      counter++;
    }

    volatile int16_t ax, ay, az, rotx, roty, rotz, mgx, mgy, mgz;
    mpu9250_getMotion9(&ax, &ay, &az, &rotx, &roty, &rotz, &mgx, &mgy, &mgz);
    volatile float temp = bmp280_getTemperature();
    volatile float pressure = bmp280_getPressure();
    volatile float height = bmp280_calcAltitude(pressure);
    osDelay(10);*/
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
