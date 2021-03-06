/**
  ******************************************************************************
  * File Name          : freertos.c
  * Date               : 10/04/2015 16:29:22
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "my_free_rtos.h"
#include <stdio.h>
#include "i2c2_sensors.h"
#include "algorithm_ahrs.h"
#include "algorithm_moveAve.h"
#include "tim2_motor.h"
#include "usart3_ble.h"
#include "leds.h"
#include "algorithm_pid2.h"
#include "pid.h"
#include "usart1_dbg.h"
#include "DataScope_DP.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osTimerId getSensorDataTimerHandle;

/* USER CODE BEGIN Variables */
osThreadId sensorTaskHandle;
osSemaphoreId sensorSemaphore;
osSemaphoreId sensorOKSemaphore;
osSemaphoreId sensorWorkSemaphore;
osThreadId bleRecvTaskHandle;
osSemaphoreId bleRecvOKSemaphore;
osThreadId uartTaskHandle;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void getSensorDataTimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
#define USART_Send(x) USART_DBG_Send(x)

void StartSensorTask(void const * argument);
void StartBleRecvTask(void const * argument);
void StartUartTask(void const * argument);
void EullerReport(EulerAngle* pAngE);
/* USER CODE END FunctionPrototypes */
/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  printf("%s\r\n", __func__);
  Sensor_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(sensorSemaphore);
  sensorSemaphore = osSemaphoreCreate(osSemaphore(sensorSemaphore), 1);
  if(sensorSemaphore == NULL){
	printf("osSemaphoreCreate sensorSemaphore Error\r\n");
	return;
  }
  osSemaphoreWait(sensorSemaphore, 0);

  osSemaphoreDef(sensorWorkSemaphore);
  sensorWorkSemaphore = osSemaphoreCreate(osSemaphore(sensorWorkSemaphore), 1);
  if(sensorWorkSemaphore == NULL){
	printf("osSemaphoreCreate sensorWorkSemaphore Error\r\n");
	return;
  }
  osSemaphoreWait(sensorWorkSemaphore, 0);

  osSemaphoreDef(sensorOKSemaphore);
  sensorOKSemaphore = osSemaphoreCreate(osSemaphore(sensorOKSemaphore), 1);
  if(sensorOKSemaphore == NULL){
	printf("osSemaphoreCreate sensorOKSemaphore Error\r\n");
	return;
  }
  osSemaphoreWait(sensorOKSemaphore, 0);

  osSemaphoreDef(bleRecvOKSemaphore);
  bleRecvOKSemaphore = osSemaphoreCreate(osSemaphore(bleRecvOKSemaphore), 1);
  if(bleRecvOKSemaphore == NULL){
	printf("osSemaphoreCreate bleRecvOKSemaphore Error\r\n");
	return;
  }
  osSemaphoreWait(bleRecvOKSemaphore, 0);

  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of getSensorDataTimer */
  osTimerDef(getSensorDataTimer, getSensorDataTimerCallback);
  getSensorDataTimerHandle = osTimerCreate(osTimer(getSensorDataTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */


  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 512);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  osThreadDef(bleRecvTask, StartBleRecvTask, osPriorityNormal, 0, 256);
  bleRecvTaskHandle = osThreadCreate(osThread(bleRecvTask), NULL);

//  osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
//  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  printf("%s\r\n", __func__);

//  osDelay(1000);
  /* USER CODE END StartDefaultTask */
}

/* getSensorDataTimerCallback function */
void getSensorDataTimerCallback(void const * argument)
{
  /* USER CODE BEGIN getSensorDataTimerCallback */

#define MoveAveSize 8
  static float Acc_FIFO[3][MoveAveSize] = {0};
  static float Gyr_FIFO[3][MoveAveSize] = {0};
  static float Mag_FIFO[3][MoveAveSize] = {0};

  static float OffsetSum[3] = {0};
  static uint16_t Correction_Time = 0;
  static uint8_t cycle = 0;
//  printf("%s\r\n", __func__);
LED_BlueOn();
  StartReadSensors(Gyr,
						Acc,
#ifdef USE_MAGNETOMETER
						Mag,
#else
						NULL,
#endif

#ifdef USE_BAROMETER
						NULL
#else
						NULL
#endif
						  );
  if(osSemaphoreWait(sensorOKSemaphore, 100) != 0)
	return;
  LED_BlueOff();

//  printf("Gyr[X]:%7.2f\tGyr[Y]:%7.2f\tGyr[Z]:%7.2f\r\n", Gyr[0], Gyr[1], Gyr[2]);
//  printf("Acc[X]:%7.2f\tAcc[Y]:%7.2f\tAcc[Z]:%7.2f\r\n", Acc[0], Acc[1], Acc[2]);
//  printf("Mag[X]:%7.2f\tMag[Y]:%7.2f\tMag[Z]:%7.2f\r\n", Mag[0], Mag[1], Mag[2]);

  Gyr[0] = Gyr[0] - Gyro_Offset.Offset_X;
  Gyr[1] = Gyr[1] - Gyro_Offset.Offset_Y;
  Gyr[2] = Gyr[2] - Gyro_Offset.Offset_Z;

  Acc[0] = Acc[0] - Acc_Parameter.Acc_Offset.Offset_X;
  Acc[1] = Acc[1] - Acc_Parameter.Acc_Offset.Offset_Y;
  Acc[2] = Acc[2] - Acc_Parameter.Acc_Offset.Offset_Z;

  Acc[0] = Acc[0] * Acc_Parameter.Acc_Coupling.K_X      \
	+ Acc[1] * Acc_Parameter.Acc_Coupling.K_YX     \
	  + Acc[2] * Acc_Parameter.Acc_Coupling.K_ZX;
  Acc[1] = Acc[0] * Acc_Parameter.Acc_Coupling.K_XY     \
	+ Acc[1] * Acc_Parameter.Acc_Coupling.K_Y      \
	  + Acc[2] * Acc_Parameter.Acc_Coupling.K_ZY;
  Acc[2] = Acc[0] * Acc_Parameter.Acc_Coupling.K_XZ     \
	+ Acc[1] * Acc_Parameter.Acc_Coupling.K_YZ     \
	  + Acc[2] * Acc_Parameter.Acc_Coupling.K_Z;

#ifdef	USE_MAGNETOMETER

  Mag[0] = Mag[0] - Mag_Parameter.Mag_Offset.Offset_X;
  Mag[1] = Mag[1] - Mag_Parameter.Mag_Offset.Offset_Y;
  Mag[2] = Mag[2] - Mag_Parameter.Mag_Offset.Offset_Z;

  Mag[0] = Mag[0] * Mag_Parameter.Mag_Coupling.K_X      \
	+ Mag[1] * Mag_Parameter.Mag_Coupling.K_YX     \
	  + Mag[2] * Mag_Parameter.Mag_Coupling.K_ZX;
  Mag[1] = Mag[0] * Mag_Parameter.Mag_Coupling.K_XY     \
	+ Mag[1] * Mag_Parameter.Mag_Coupling.K_Y      \
	  + Mag[2] * Mag_Parameter.Mag_Coupling.K_ZY;
  Mag[2] = Mag[0] * Mag_Parameter.Mag_Coupling.K_XZ     \
	+ Mag[1] * Mag_Parameter.Mag_Coupling.K_YZ     \
	  + Mag[2] * Mag_Parameter.Mag_Coupling.K_Z;
  /* Due to IC position in PCB*/
  //Mag[0] = -Mag[0];
  //Mag[1] = -Mag[1];
#endif

  switch(SensorMode)
  {
	/************************** Mode_CorrectGyr **************************************/
  case Mode_GyrCorrect:

	//	  YELLOW_ON;
	/* Offset Correction */
#define GyroSampleSize 1024
	OffsetSum[0] += Gyr[0];
	OffsetSum[1] += Gyr[1];
	OffsetSum[2] += Gyr[2];
	Correction_Time++;
	if(Correction_Time == GyroSampleSize)
	{
	  Gyro_Offset.Offset_X = OffsetSum[0] / GyroSampleSize;
	  Gyro_Offset.Offset_Y = OffsetSum[1] / GyroSampleSize;
	  Gyro_Offset.Offset_Z = OffsetSum[2] / GyroSampleSize;

	  Correction_Time = 0;
	  /* Next Procedure */
	  SensorMode = Mode_AccCorrect;
	}
	break;

	/************************** Mode_CorrectAcc **************************************/
  case Mode_AccCorrect:
#if 0
	/* Offset Correction */
	Acc_Parameter.Acc_Offset.Offset_X = -0.007970729553635;
	Acc_Parameter.Acc_Offset.Offset_Y = 0.019836643844817;
	Acc_Parameter.Acc_Offset.Offset_Z = 0.006183107591687;
	/* Coupling Correction */
	Acc_Parameter.Acc_Coupling.K_X  = 0.970990949467151;
	Acc_Parameter.Acc_Coupling.K_Y  = 0.980792462583293;
	Acc_Parameter.Acc_Coupling.K_Z  = 0.965535220154680;
	Acc_Parameter.Acc_Coupling.K_YX = 0.002437447263339;
	Acc_Parameter.Acc_Coupling.K_ZX = 0.014915966356573;
	Acc_Parameter.Acc_Coupling.K_XY = 0;
	Acc_Parameter.Acc_Coupling.K_ZY = -0.005387874488198;
	Acc_Parameter.Acc_Coupling.K_XZ = 0;
	Acc_Parameter.Acc_Coupling.K_YZ = 0;
#else
	/* Offset Correction */
	Acc_Parameter.Acc_Offset.Offset_X = 0;
	Acc_Parameter.Acc_Offset.Offset_Y = 0;
	Acc_Parameter.Acc_Offset.Offset_Z = 0;
	/* Coupling Correction */
	Acc_Parameter.Acc_Coupling.K_X  = 1;
	Acc_Parameter.Acc_Coupling.K_Y  = 1;
	Acc_Parameter.Acc_Coupling.K_Z  = 1;
	Acc_Parameter.Acc_Coupling.K_YX = 0;
	Acc_Parameter.Acc_Coupling.K_ZX = 0;
	Acc_Parameter.Acc_Coupling.K_XY = 0;
	Acc_Parameter.Acc_Coupling.K_ZY = 0;
	Acc_Parameter.Acc_Coupling.K_XZ = 0;
	Acc_Parameter.Acc_Coupling.K_YZ = 0;
#endif
	/* Wait until FIFO is filled up */
	MoveAve_SMA(Acc[0], Acc_FIFO[0], MoveAveSize);
	MoveAve_SMA(Acc[1], Acc_FIFO[1], MoveAveSize);
	MoveAve_SMA(Acc[2], Acc_FIFO[2], MoveAveSize);
	Correction_Time++;
	if (Correction_Time == MoveAveSize+1)	//beacuse the first data is zero
	{
	  Correction_Time = 0;
	  /* Next Procedure */
#ifdef	USE_MAGNETOMETER
	  SensorMode = Mode_MagCorrect;
#else
	  SensorMode = Mode_Quaternion;
#endif

	}
	break;

	/************************** Mode_CorrectMag **************************************/
#ifdef	USE_MAGNETOMETER
  case Mode_MagCorrect:

	/* Offset Correction */
	Mag_Parameter.Mag_Offset.Offset_X = 0.347320519123196;
	Mag_Parameter.Mag_Offset.Offset_Y = -0.441676977320650;
	Mag_Parameter.Mag_Offset.Offset_Z = 0.623926096014853;
	/* Coupling Correction */
	Mag_Parameter.Mag_Coupling.K_X  = 3.213533040206366;
	Mag_Parameter.Mag_Coupling.K_Y  = 3.241421222828024;
	Mag_Parameter.Mag_Coupling.K_Z  = 3.215694520594453;
	Mag_Parameter.Mag_Coupling.K_YX = 0.081451183121051;
	Mag_Parameter.Mag_Coupling.K_ZX = 0.006184918582682;
	Mag_Parameter.Mag_Coupling.K_XY = 0;
	Mag_Parameter.Mag_Coupling.K_ZY = -0.049199718330043;
	Mag_Parameter.Mag_Coupling.K_XZ = 0;
	Mag_Parameter.Mag_Coupling.K_YZ = 0;
	/* Wait until FIFO is filled up */
	MoveAve_SMA(Mag[0], Mag_FIFO[0], MoveAveSize);
	MoveAve_SMA(Mag[1], Mag_FIFO[1], MoveAveSize);
	MoveAve_SMA(Mag[2], Mag_FIFO[2], MoveAveSize);
	Correction_Time++;
	if (Correction_Time == MoveAveSize+1)	//beacuse the first data is zero
	{
	  Correction_Time = 0;
	  /* Next Procedure */
	  SensorMode = Mode_Quaternion;
	}
	break;
#endif

	/************************** Quaternion Mode **************************************/
  case Mode_Quaternion:

	/* Weighted Moving Average */
	Acc[0] = MoveAve_WMA(Acc[0], Acc_FIFO[0], MoveAveSize);
	Acc[1] = MoveAve_WMA(Acc[1], Acc_FIFO[1], MoveAveSize);
	Acc[2] = MoveAve_WMA(Acc[2], Acc_FIFO[2], MoveAveSize);
#ifdef	USE_MAGNETOMETER
	Mag[0] = MoveAve_WMA(Mag[0], Mag_FIFO[0], MoveAveSize);
	Mag[1] = MoveAve_WMA(Mag[1], Mag_FIFO[1], MoveAveSize);
	Mag[2] = MoveAve_WMA(Mag[2], Mag_FIFO[2], MoveAveSize);
#endif
	AHRS_Init(Acc, Mag, &AngE);
	SensorMode = Mode_Algorithm;
	break;

	/************************** Algorithm Mode ****************************************/
  case Mode_Algorithm:
#if 1
	/* Weighted Moving Average */
	Gyr[0] = MoveAve_WMA(Gyr[0], Gyr_FIFO[0], MoveAveSize);
	Gyr[1] = MoveAve_WMA(Gyr[1], Gyr_FIFO[1], MoveAveSize);
	Gyr[2] = MoveAve_WMA(Gyr[2], Gyr_FIFO[2], MoveAveSize);
	Acc[0] = MoveAve_WMA(Acc[0], Acc_FIFO[0], MoveAveSize);
	Acc[1] = MoveAve_WMA(Acc[1], Acc_FIFO[1], MoveAveSize);
	Acc[2] = MoveAve_WMA(Acc[2], Acc_FIFO[2], MoveAveSize);

#ifdef	USE_MAGNETOMETER
	Mag[0] = MoveAve_WMA(Mag[0], Mag_FIFO[0], MoveAveSize);
	Mag[1] = MoveAve_WMA(Mag[1], Mag_FIFO[1], MoveAveSize);
	Mag[2] = MoveAve_WMA(Mag[2], Mag_FIFO[2], MoveAveSize);
#endif
#else
		/* Slide Moving Average */
	Gyr[0] = MoveAve_SMA(Gyr[0], Gyr_FIFO[0], MoveAveSize);
	Gyr[1] = MoveAve_SMA(Gyr[1], Gyr_FIFO[1], MoveAveSize);
	Gyr[2] = MoveAve_SMA(Gyr[2], Gyr_FIFO[2], MoveAveSize);
	Acc[0] = MoveAve_SMA(Acc[0], Acc_FIFO[0], MoveAveSize);
	Acc[1] = MoveAve_SMA(Acc[1], Acc_FIFO[1], MoveAveSize);
	Acc[2] = MoveAve_SMA(Acc[2], Acc_FIFO[2], MoveAveSize);

#ifdef	USE_MAGNETOMETER
	Mag[0] = MoveAve_SMA(Mag[0], Mag_FIFO[0], MoveAveSize);
	Mag[1] = MoveAve_SMA(Mag[1], Mag_FIFO[1], MoveAveSize);
	Mag[2] = MoveAve_SMA(Mag[2], Mag_FIFO[2], MoveAveSize);
#endif
#endif
	cycle ++;
	if(cycle >= 10){
	  cycle = 0;
	  osSemaphoreRelease(sensorSemaphore);
	  osSemaphoreWait(sensorWorkSemaphore, 5);
	}

	break;
  }

  /* USER CODE END getSensorDataTimerCallback */
}

/* USER CODE BEGIN Application */
void SensorsReadOK()
{
  osSemaphoreRelease(sensorOKSemaphore);
}
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  printf("%s\r\n", __func__);

//  Motor_Out(MOTOR_SPEED_MAX,MOTOR_SPEED_MAX,MOTOR_SPEED_MAX,MOTOR_SPEED_MAX);
//  osDelay(2500);
//  Motor_Out(MOTOR_SPEED_MIN,MOTOR_SPEED_MIN,MOTOR_SPEED_MIN,MOTOR_SPEED_MIN);
//  osDelay(2500);

  Motor_Out(MOTOR_SPEED_MIN + 300,0,0,0);
  osDelay(50);
  Motor_Out(0,0,0,0);
  osDelay(500);

  Motor_Out(0,MOTOR_SPEED_MIN + 300,0,0);
  osDelay(50);
  Motor_Out(0,0,0,0);
  osDelay(500);

  Motor_Out(0,0,MOTOR_SPEED_MIN + 300,0);
  osDelay(50);
  Motor_Out(0,0,0,0);
  osDelay(500);

  Motor_Out(0,0,0,MOTOR_SPEED_MIN + 300);
  osDelay(50);
  Motor_Out(0,0,0,0);
  osTimerStart(getSensorDataTimerHandle, 1000 / 500);


//  Motor_Out(1000,1000,1000,1000);

  /* Infinite loop */
  for(;;)
  {

	if(osSemaphoreWait(sensorSemaphore, 100) == 0){
	  uint8_t Send_Count = 0, i =0;
	  LED_RedOn();
	  AHRS_Update(Gyr, Acc, Mag, &AngE);
#if 1
#if 1
	  DataScope_Get_Channel_Data( Acc[0], 1 );
	  DataScope_Get_Channel_Data( Acc[1], 2 );
	  DataScope_Get_Channel_Data( Acc[2], 3 );
	  DataScope_Get_Channel_Data( Gyr[0] , 4 );
	  DataScope_Get_Channel_Data( Gyr[1] , 5 );
	  DataScope_Get_Channel_Data( Gyr[2] , 6 );
	  DataScope_Get_Channel_Data( toDeg(AngE.Pitch) , 7 );
	  DataScope_Get_Channel_Data( toDeg(AngE.Roll) , 8 );
	  DataScope_Get_Channel_Data( toDeg(AngE.Yaw) , 9 );

	  Send_Count = DataScope_Data_Generate(9);
	  for(i = 0;i < Send_Count;i++)
	  {
		USART_DBG_Send(DataScope_OutPut_Buffer[i]);
	  }
#else
	  DataScope_Get_Channel_Data( toDeg(AngE.Pitch) , 1 );
	  DataScope_Get_Channel_Data( toDeg(AngE.Roll) , 2 );
	  DataScope_Get_Channel_Data( toDeg(AngE.Yaw) , 3 );

	  Send_Count = DataScope_Data_Generate(3);
	  for(i = 0;i < Send_Count;i++)
	  {
		USART_DBG_Send(DataScope_OutPut_Buffer[i]);
	  }
#endif
#endif

	  osSemaphoreRelease(sensorWorkSemaphore);

#if 1
	  if(BLE_CONNECTED){
#if 0
		//		  PID2_Process(toDeg(AngE.Pitch - AngE_Zero.Pitch) - gCommand_Packet.mRoll,
		//					   toDeg(AngE.Roll - AngE_Zero.Roll) - gCommand_Packet.mPitch,
		//					   toDeg(AngE.Yaw - AngE_Zero.Yaw) - gCommand_Packet.mYaw,
		//					   gCommand_Packet.mThrust);
		PID2_Process(0,
					 0,
					 0,
					 gCommand_Packet.mThrust);
#else
		//			expect.Roll_expect = gCommand_Packet.mPitch;
		//			expect.Pitch_expect = gCommand_Packet.mRoll;
		//			expect.Yaw_expect = gCommand_Packet.mYaw;

//		expect.Roll_expect = 0;
//		expect.Pitch_expect = 0;
//		expect.Yaw_expect = AngE.Yaw;
//		expect.Throttle_expect = gCommand_Packet.mThrust;//1500;
//		Control_Angle(&AngE,&expect);
//		Control_Gyro(Gyr,&AngE);
#endif
	  }
	  else{
		Motor_Out(0,0,0,0);
	  }
//	  printf("+ AngE.Roll:%7f AngE.Pitch:%7f AngE.Yaw:%7f\r\n", AngE.Roll, AngE.Pitch, AngE.Yaw);

#endif
	  //EullerReport(&AngE);
	  LED_RedOff();
	}
	else{
	}

  }
  /* USER CODE END StartDefaultTask */
}

void BLERecvOK()
{
  osSemaphoreRelease(bleRecvOKSemaphore);
}

void StartBleRecvTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  printf("%s\r\n", __func__);
  /* Infinite loop */
  BLE_Mode( 0 );
  BLE_Power_Enable( 0 );
  osDelay(1);
  BLE_Power_Enable( 1 );
  osDelay(1);
  BLE_BroadCast_Enable( 1 );
  //BLE_Input_Enable(1);
  printf("BLE init OK\r\n");
  BLE_StartRead();
  for(;;)
  {
	if(osSemaphoreWait(bleRecvOKSemaphore, 1000) == 0)
	{
	  if(BLE_GetPacket(&gCommand_Packet)){
		Motor_Out(gCommand_Packet.mThrust,gCommand_Packet.mThrust,
				  gCommand_Packet.mThrust,gCommand_Packet.mThrust);
//		Motor_Out(gCommand_Packet.mThrust,0,
//				  0,0);

//		printf("mRoll:%f mPitch:%f mYaw:%f mThrust:%u\r\n",
//			   gCommand_Packet.mRoll, gCommand_Packet.mPitch, gCommand_Packet.mYaw, gCommand_Packet.mThrust);
	  }
	}
  }
  /* USER CODE END StartDefaultTask */
}

void StartUartTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(200);
//	printf("%s\r\n", __func__);
  }
  /* USER CODE END StartDefaultTask */
}

/**
* @brief  Report Euler angles(Degree) to PC.
*
* @param  pAngE: Pointer to the buffer containing the Euler angles(radias).
*
* @retval None.
*
*/
void EullerReport(EulerAngle* pAngE)
{
//  int16_t  yaw 	 = (int16_t)(toDeg(pAngE->Yaw - AngE_Zero.Yaw)*10);
//  int16_t  pitch = (int16_t)(toDeg(pAngE->Roll - AngE_Zero.Roll)*10);
//  int16_t  roll	 = (int16_t)(toDeg(pAngE->Pitch - AngE_Zero.Pitch)*10);
  int16_t  yaw 	 = (int16_t)(toDeg(pAngE->Yaw)*10);
  int16_t  pitch = (int16_t)(toDeg(pAngE->Roll)*10);
  int16_t  roll	 = (int16_t)(toDeg(pAngE->Pitch)*10);
  uint8_t  ctemp = 0;
  uint16_t verification = 0xB1;

  USART_Send(0xA5);
  USART_Send(0x5A);
  USART_Send(0x10);
  USART_Send(0xA1);

  if(yaw < 0)
	yaw = 32768 - yaw;
  ctemp = yaw >> 8;

  USART_Send(ctemp);
  verification += ctemp;
  ctemp = yaw;

  USART_Send(ctemp);
  verification += ctemp;

  if(pitch < 0)
	pitch = 32768 - pitch;
  ctemp = pitch >> 8;

  USART_Send(ctemp);
  verification += ctemp;
  ctemp = pitch;

  USART_Send(ctemp);
  verification += ctemp;

  if(roll < 0)
	roll = 32768 - roll;
  ctemp = roll >> 8;

  USART_Send(ctemp);
  verification += ctemp;
  ctemp = roll;

  USART_Send(ctemp);
  verification += ctemp;

  USART_Send(verification % 256);
  USART_Send(0xAA);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
