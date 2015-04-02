/****************************************************************
 *                                                              *
 *    Copyright (c) Linker Chip Corp. All rights reserved.      *
 *    														    													*
 *    Created by Anakin																					*
 *                                                      				*
 ****************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __ALGORITHM_AHRS_H
#define __ALGORITHM_AHRS_H


/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
  Mode_GyrCorrect,
  Mode_AccCorrect,  
  Mode_MagCorrect,
  Mode_Quaternion,
  Mode_Algorithm
} Sensor_Mode;


typedef struct {
  float Yaw;
  float Pitch;
  float Roll;
} EulerAngle;



typedef struct {
  int16_t Yaw;
  int16_t Pitch;
  int16_t Roll;
} UploadAngle;



typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;




/* Gyroscope */
typedef struct {
  float Offset_X;
  float Offset_Y;
  float Offset_Z; 
} GYRO_OffsetTypeDef;


/* Accelerometer */
typedef struct {
  float K_X;  
  float K_Y;
  float K_Z;
  float K_YX;
  float K_ZX;
  float K_XY;
  float K_ZY;
  float K_XZ;
  float K_YZ; 
} ACC_CouplingTypeDef;

typedef struct {
  float Offset_X;
  float Offset_Y;
  float Offset_Z;
} ACC_OffsetTypeDef;


typedef struct {
  ACC_CouplingTypeDef Acc_Coupling; 
  ACC_OffsetTypeDef   Acc_Offset;
} ACC_ParameterTypeDef;


/* Magnetometer */
typedef struct {
  float K_X;  
  float K_Y;
  float K_Z;
  float K_YX;
  float K_ZX;
  float K_XY;
  float K_ZY;
  float K_XZ;
  float K_YZ; 
} MAG_CouplingTypeDef;


typedef struct {
  float Offset_X;
  float Offset_Y;
  float Offset_Z;
} MAG_OffsetTypeDef;



typedef struct {
  MAG_CouplingTypeDef Mag_Coupling; 
  MAG_OffsetTypeDef   Mag_Offset;
} MAG_ParameterTypeDef;


/* Exported constants --------------------------------------------------------*/

#define squa( Sq )          (((float)Sq)*((float)Sq))
#define invSqrtf( iSq ) 		(1.0f/sqrtf((float)iSq))
#define toRad( Math_D )	    ((float)(Math_D)*0.0174532925f)
#define toDeg( Math_R )	    ((float)(Math_R)*57.2957795f)

#define 	PA_INITIAL 		0.001
#define 	RA_INITIAL 		0.01
#define 	QA_INITIAL 		0.00002

#define 	PM_INITIAL 		0.001
#define 	RM_INITIAL 		0.02
#define 	QM_INITIAL 		0.00002

/* Exported macro ------------------------------------------------------------*/

extern Sensor_Mode 						SensorMode;
extern Quaternion 						NumQ;
extern EulerAngle 						AngE;
extern UploadAngle   					iAngE;
extern float 									Gyr[3];
extern float 									Acc[3];
extern float 									Mag[3];
extern GYRO_OffsetTypeDef			Gyro_Offset;
extern ACC_ParameterTypeDef		Acc_Parameter;
extern MAG_ParameterTypeDef		Mag_Parameter;


/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Measure the initial attitude and heading.
  *					in order to make sure the initial quaternion.
  *
  * @param  pAcc:  Pointer to the buffer containing the data of accelerometer.
  * @param  pMag:  Pointer to the buffer containing the data of magnetometer.
  * @param  pAngE: Pointer to the buffer containing the Euler angles(radias).       					 		
  *
  * @retval None.
  *					
  */
void AHRS_Init(float* pAcc, float* pMag, EulerAngle* pAngE);

/**
  * @brief  Update the attitude and heading. 
  *
  * @param  pGyr:  Pointer to the buffer containing the data of gyroscope.
  * @param  pAcc:  Pointer to the buffer containing the data of accelerometer.
  * @param  pMag:  Pointer to the buffer containing the data of magnetometer.
  * @param  pAngE: Pointer to the buffer containing the Euler angles(radias).       					 		
  *
  * @retval None.
  *					
  */
void AHRS_Update(float* pGyr, float* pAcc, float* pMag, EulerAngle* pAngE);


/**
  * @brief  Report Euler angles(Degree) to PC.
  *
  * @param  pAngE: Pointer to the buffer containing the Euler angles(radias).       					 		
  *
  * @retval None.
  *					
  */
void EullerReport(EulerAngle* pAngE);














#endif

