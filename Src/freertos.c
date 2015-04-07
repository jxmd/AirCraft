/**
  ******************************************************************************
  * File Name          : freertos.c
  * Date               : 02/04/2015 18:01:29
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
#include <stdio.h>
#include "i2c2_sensors.h"
#include "algorithm_ahrs.h"
#include "tim2_motor.h"
#include "usart3_ble.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
osThreadId sensorTaskHandle;
osThreadId bleRecvTaskHandle;
osThreadId uartTaskHandle;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void StartSensorTask(void const * argument);
void StartBleRecvTask(void const * argument);
void StartUartTask(void const * argument);
/* USER CODE END FunctionPrototypes */
/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  printf("%s\r\n", __func__);
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

  osThreadDef(sensorTask, StartSensorTask, osPriorityHigh, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  osThreadDef(bleRecvTask, StartBleRecvTask, osPriorityHigh, 0, 128);
  bleRecvTaskHandle = osThreadCreate(osThread(bleRecvTask), NULL);

  osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

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
#define MoveAveSize 8
//  static float Acc_FIFO[3][MoveAveSize] = {0};
//  static float Gyr_FIFO[3][MoveAveSize] = {0};
//  static float Mag_FIFO[3][MoveAveSize] = {0};
//
//  static float OffsetSum[3] = {0};
//  static uint16_t Correction_Time = 0;
  /* Infinite loop */
  for(;;)
  {}
  for(;;)
  {
	/* Read Gyro data from LSM330DLC */
	LSM330DLC_GyroReadAngRate(Gyr);
//	printf("Gyr[X]:%f\tGyr[Y]:%f\tGyr[Z]:%f\r\n", Gyr[0], Gyr[1], Gyr[2]);
#if 0
	Gyr[0] = Gyr[0] - Gyro_Offset.Offset_X;
	Gyr[1] = Gyr[1] - Gyro_Offset.Offset_Y;
	Gyr[2] = Gyr[2] - Gyro_Offset.Offset_Z;
#endif
	/* Read Acc data from LSM330DLC */
	LSM330DLC_AcceleroReadAcc(Acc);
//	printf("Acc[X]:%f\tAcc[Y]:%f\tAcc[Z]:%f\r\n", Acc[0], Acc[1], Acc[2]);
#if 0
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
#endif
#ifdef	USE_MAGNETOMETER
	LIS3MDL_CompassReadMag(Mag);

//	printf("Mag[X]:%f\tMag[Y]:%f\tMag[Z]:%f\r\n", Mag[0], Mag[1], Mag[2]);
#if 0
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
#endif


#if 0
	switch(SensorMode)
	{
	  /************************** Mode_CorrectGyr **************************************/
	case Mode_GyrCorrect:

	  YELLOW_ON;
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
	  UpdateFlag_AHRS ++;

	  break;
	}
#endif
    osDelay(500);
//	printf("%s\r\n", __func__);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void StartSensorTask(void const * argument)
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

void StartBleRecvTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
//  Motor_Out(500,0,0,0);
//  osDelay(2000);
//  Motor_Out(0,500,0,0);
//  osDelay(2000);
//  Motor_Out(0,0,500,0);
//  osDelay(2000);
//  Motor_Out(0,0,0,500);
//  osDelay(2000);
//  Motor_Out(0,0,0,0);
  for(;;)
  {
	osDelay(200);
	BLE_StartRead();
//	printf("%s\r\n", __func__);
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
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
