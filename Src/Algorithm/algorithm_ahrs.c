/****************************************************************
*                                                               *
*    Copyright (c) Linker Chip Corp. All rights reserved.       *
*    														                                *
*    Created by Anakin											                    *
*                                                      		      *
****************************************************************/

// Date					Author					Notes
// 16/07/2014		Anakin    			Initial release
// 17/07/2014   Anakin					Optimise the method of updating quaternion by RungeKutta



/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "algorithm_ahrs.h"
#include "pid.h"

#ifdef  USE_EKF_ALGORITHM
#include "algorithm_kalman.h"
#endif


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

Sensor_Mode 				SensorMode = Mode_GyrCorrect;
Quaternion 					NumQ = {1, 0, 0, 0};
UploadAngle   			iAngE = {0};
float 							Gyr[3] = {0};
float 							Acc[3] = {0};
float 							Mag[3] = {0};
EulerAngle 					AngE = {0};
EulerAngle 					AngE_Zero = {0};


#ifdef USE_MAGNETOMETER
float					Mag_X = 0;	// X -> East, Mag_X ~= 0
float					Mag_Y = 0;	// Y -> North, Mag_Y reaches the max
float					Mag_Z = 0;	// Z -> Up, Mag_Y^2 + Mag_Z^2 = 1
#endif

GYRO_OffsetTypeDef			Gyro_Offset = {0};
ACC_ParameterTypeDef		Acc_Parameter = {0};
MAG_ParameterTypeDef		Mag_Parameter = {0};

#ifdef  USE_EKF_ALGORITHM
float 		PA[16];				//posteriori estimate of acc
float 		PM[16];				//posteriori estimate of mag


const float QA[16] = {QA_INITIAL,0		   ,0		  	,0,
0		    ,QA_INITIAL,0		  	,0,
0		    ,0		   ,QA_INITIAL,0,
0		    ,0		   ,0		  ,QA_INITIAL};

float RA[9] = {RA_INITIAL_X,0,			0,
0,			RA_INITIAL_Y,	0,
0,			0,			RA_INITIAL_Z};


#ifdef	USE_MAGNETOMETER
const float RM[9] = {RM_INITIAL,0,			0,
0,			RM_INITIAL,	0,
0,			0,			RM_INITIAL};

const float QM[16] = {QM_INITIAL,0		   ,0		  	,0,
0		    ,QM_INITIAL,0		  	,0,
0		    ,0		   ,QM_INITIAL,0,
0		    ,0		   ,0		  ,QM_INITIAL};
#endif

#endif

/* Private function prototypes -----------------------------------------------*/

static void QuaternionToNumQ( Quaternion *pNumQ, EulerAngle *pAngE );
static void QuaternionNormalize( Quaternion *pNumQ );
static void QuaternionToAngE( Quaternion *pNumQ, EulerAngle *pAngE );


/* Private functions ---------------------------------------------------------*/

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

void AHRS_Init(float* pAcc, float* pMag, EulerAngle* pAngE)
{
#ifdef USE_MAGNETOMETER
  float hx, hy, hz;
  float sinP, cosP;
  float sinR, cosR;
  float Normalize;

  /* Determin initial Euller angles according to Acc */
  pAngE->Pitch  = -asinf(pAcc[0]);
  pAngE->Roll 	= atan2f(pAcc[1], pAcc[2]);

  /* Compensate for dip angle */
  sinP = arm_sin_f32(pAngE->Pitch);
  cosP = arm_cos_f32(pAngE->Pitch);
  sinR = arm_sin_f32(pAngE->Roll);
  cosR = arm_cos_f32(pAngE->Roll);

  hx =  pMag[0]*cosP + pMag[1]*sinP*sinR + pMag[2]*sinP*cosR;
  hy =  							 pMag[1]     *cosR - pMag[2]*     sinR;
  hz = -pMag[0]*sinP + pMag[1]*cosP*sinR + pMag[2]*cosR*cosR;

  /* Compute yaw with magnetometer */
  Normalize = invSqrtf(squa(hx) + squa(hy) + squa(hz));
  hx = hx*Normalize;
  hy = hy*Normalize;
  hz = hz*Normalize;
  pAngE->Yaw = atan2f(hx, hy);
  /* Initialize quaternion */
  QuaternionToNumQ(&NumQ, &AngE);

  /* Compute magnetic flux on each axis in the reasonable positon(East North Up) */
  Mag_X = 0;
  Mag_Y = hx*arm_sin_f32(pAngE->Yaw) + hy*arm_cos_f32(pAngE->Yaw);
  Mag_Z = hz;

#else
  /* Determin initial Euller angles according to Acc */
  AngE_Zero.Yaw = pAngE->Yaw 	 = 	0;
  AngE_Zero.Pitch = pAngE->Pitch = -asinf(pAcc[0]);
  AngE_Zero.Roll = pAngE->Roll  = atan2f(pAcc[1], pAcc[2]);

  /* Initialize quaternion */
  QuaternionToNumQ(&NumQ, &AngE);
#endif

}




#ifdef USE_PI_FUSION_ALGORITHM			// 480us in 6axes; 720us in 9axes;

#ifdef USE_MAGNETOMETER

#define 	Kp 							3.0f
#define 	Ki 							0.0f

#else

#define 	Kp 							5.0f
#define 	Ki 							0.0f

#endif

#define   IntervalHalf		 0.01f
#define 	Threshold					1.0f

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

void AHRS_Update(float* pGyr, float* pAcc, float* pMag, EulerAngle* pAngE)
{
  static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
  float	gx, gy, gz;
  float q0, q1, q2, q3;
#ifdef USE_MAGNETOMETER
  float mx, my, mz;
#endif
  float ErrX, ErrY, ErrZ;
  float AccX, AccY, AccZ;
#ifdef USE_MAGNETOMETER
  float MagX, MagY, MagZ;
#endif
  float GyrX, GyrY, GyrZ;
  float Mq13, Mq23, Mq33;
#ifdef USE_MAGNETOMETER
  float Mq12, Mq22, Mq32;
#endif
  float Normalize;

  /* Processing the measurements of gyroscope */
  if(pGyr[2] < Threshold && pGyr[2] > -Threshold)
  {
	GyrX = pGyr[0];
	GyrY = pGyr[1];
	GyrZ = 0;
  }
  else
  {
	GyrX = pGyr[0];
	GyrY = pGyr[1];
	GyrZ = pGyr[2];
  }
  /* Direction Cosine Matrix at last time  */

  //	 Mq11 = NumQ.q0*NumQ.q0 + NumQ.q1*NumQ.q1 - NumQ.q2*NumQ.q2 - NumQ.q3*NumQ.q3;
#ifdef USE_MAGNETOMETER
  Mq12 = 2.0f*(NumQ.q1*NumQ.q2 + NumQ.q0*NumQ.q3);
#endif
  Mq13 = 2.0f*(NumQ.q1*NumQ.q3 - NumQ.q0*NumQ.q2);
  //	 Mq21 = 2.0f*(NumQ.q1*NumQ.q2 - NumQ.q0*NumQ.q3);
#ifdef USE_MAGNETOMETER
  Mq22 = NumQ.q0*NumQ.q0 - NumQ.q1*NumQ.q1 + NumQ.q2*NumQ.q2 - NumQ.q3*NumQ.q3;
#endif
  Mq23 = 2.0f*(NumQ.q0*NumQ.q1 + NumQ.q2*NumQ.q3);
  //	 Mq31 = 2.0f*(NumQ.q0*NumQ.q2 + NumQ.q1*NumQ.q3);
#ifdef USE_MAGNETOMETER
  Mq32 = 2.0f*(NumQ.q2*NumQ.q3 - NumQ.q0*NumQ.q1);
#endif
  Mq33 = NumQ.q0*NumQ.q0 - NumQ.q1*NumQ.q1 - NumQ.q2*NumQ.q2 + NumQ.q3*NumQ.q3;

  /* Normalise the measurements of accelerometer */
  Normalize = invSqrtf(squa(pAcc[0]) + squa(pAcc[1]) + squa(pAcc[2]));
  AccX = pAcc[0]*Normalize;
  AccY = pAcc[1]*Normalize;
  AccZ = pAcc[2]*Normalize;

#ifdef USE_MAGNETOMETER
  /* Normalise the measurements of magnetometer */
  Normalize = invSqrtf(squa(pMag[0]) + squa(pMag[1]) + squa(pMag[2]));
  MagX = pMag[0]*Normalize;
  MagY = pMag[1]*Normalize;
  MagZ = pMag[2]*Normalize;
#endif

  /* Estimate the direction of gravity */
  gx = Mq13;
  gy = Mq23;
  gz = Mq33;

#ifdef USE_MAGNETOMETER
  /* Estimate the direction of magnetic gravity */
  mx = Mq12*Mag_Y + Mq13*Mag_Z;
  my = Mq22*Mag_Y + Mq23*Mag_Z;
  mz = Mq32*Mag_Y + Mq33*Mag_Z;
#endif

#ifdef USE_MAGNETOMETER
  /* error is sum of cross product between reference direction of field and direction measured by sensor */
  ErrX = (AccY*gz - AccZ*gy) + (MagY*mz - MagZ*my);
  ErrY = (AccZ*gx - AccX*gz) + (MagZ*mx - MagX*mz);
  ErrZ = (AccX*gy - AccY*gx) + (MagX*my - MagY*mx);
#else
  /* error is sum of cross product between reference direction of field and direction measured by sensor */
  ErrX = (AccY*gz - AccZ*gy);
  ErrY = (AccZ*gx - AccX*gz);
  ErrZ = (AccX*gy - AccY*gx);
#endif


  /* integral error scaled integral gain */
  exInt = exInt + ErrX*Ki;
  eyInt = eyInt + ErrY*Ki;
  ezInt = ezInt + ErrZ*Ki;

  /* adjusted gyroscope measurements */
  GyrX = toRad(GyrX);
  GyrY = toRad(GyrY);
  GyrZ = toRad(GyrZ);

  GyrX = GyrX + Kp*ErrX + exInt;
  GyrY = GyrY + Kp*ErrY + eyInt;
  GyrZ = GyrZ + Kp*ErrZ + ezInt;

  /* Update quaternion by RungeKutta */
  // 17/07/2014 Update
  q0 = NumQ.q0+(-NumQ.q1*GyrX - NumQ.q2*GyrY - NumQ.q3*GyrZ)*IntervalHalf;
  q1 = NumQ.q1+( NumQ.q0*GyrX - NumQ.q3*GyrY + NumQ.q2*GyrZ)*IntervalHalf;
  q2 = NumQ.q2+( NumQ.q3*GyrX + NumQ.q0*GyrY - NumQ.q1*GyrZ)*IntervalHalf;
  q3 = NumQ.q3+(-NumQ.q2*GyrX + NumQ.q1*GyrY + NumQ.q0*GyrZ)*IntervalHalf;
  NumQ.q0 = q0;
  NumQ.q1 = q1;
  NumQ.q2 = q2;
  NumQ.q3 = q3;
  /* Normalise quaternion */
  QuaternionNormalize(&NumQ);

  /* Convert Quaternion to Euler */
  QuaternionToAngE(&NumQ, &AngE);

}

#endif





#ifdef USE_BALANCED_FILTER_ALGORITHM  		// 520us in 6axes; 800us in 9axes;

#define   IntervalHalf		 0.01f
#define 	Threshold					1.0f
#define		FACTOR_ACC			0.015f
#define		FACTOR_MAG			0.010f

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

void AHRS_Update(float* pGyr, float* pAcc, float* pMag, EulerAngle* pAngE)
{
  float AccX, AccY, AccZ;
  float GyrX, GyrY, GyrZ;
  float q0, q1, q2, q3;
#ifdef USE_MAGNETOMETER
  float MagX, MagY, MagZ;
  float HorizonX, HorizonY, HorizonZ;
  float sinP, cosP;
  float sinR, cosR;
#endif
  float Normalize;
  float newGyroOrientation[3] = {0};
  float newAccMagOrientation[3] = {0};

  /* Processing the measurements of gyroscope */
  if(pGyr[2] < Threshold && pGyr[2] > -Threshold)
  {
	GyrX = pGyr[0];
	GyrY = pGyr[1];
	GyrZ = 0;
  }
  else
  {
	GyrX = pGyr[0];
	GyrY = pGyr[1];
	GyrZ = pGyr[2];
  }

  /* Normalise the measurements of accelerometer */
  Normalize = invSqrtf(squa(pAcc[0]) + squa(pAcc[1]) + squa(pAcc[2]));
  AccX = pAcc[0]*Normalize;
  AccY = pAcc[1]*Normalize;
  AccZ = pAcc[2]*Normalize;

  /* Determin initial Euller angles according to Acc */
  newAccMagOrientation[1] = -asinf(AccX);
  newAccMagOrientation[2] = atan2f(AccY, AccZ);

  /* adjusted gyroscope measurements */
  GyrX = toRad(GyrX);
  GyrY = toRad(GyrY);
  GyrZ = toRad(GyrZ);

  /* Update quaternion by RungeKutta */
  // 17/07/2014 Update
  q0 = NumQ.q0+(-NumQ.q1*GyrX - NumQ.q2*GyrY - NumQ.q3*GyrZ)*IntervalHalf;
  q1 = NumQ.q1+( NumQ.q0*GyrX - NumQ.q3*GyrY + NumQ.q2*GyrZ)*IntervalHalf;
  q2 = NumQ.q2+( NumQ.q3*GyrX + NumQ.q0*GyrY - NumQ.q1*GyrZ)*IntervalHalf;
  q3 = NumQ.q3+(-NumQ.q2*GyrX + NumQ.q1*GyrY + NumQ.q0*GyrZ)*IntervalHalf;
  NumQ.q0 = q0;
  NumQ.q1 = q1;
  NumQ.q2 = q2;
  NumQ.q3 = q3;

  /* Normalise quaternion */
  QuaternionNormalize(&NumQ);

  /* Convert Quaternion to Euler */
  QuaternionToAngE(&NumQ, &AngE);

  newGyroOrientation[0] = pAngE->Yaw;
  newGyroOrientation[1] = pAngE->Pitch;
  newGyroOrientation[2] = pAngE->Roll;

  /* The Complementary Filter for Pitch and Roll*/
  pAngE->Pitch = FACTOR_ACC*newAccMagOrientation[1] + (1-FACTOR_ACC)*newGyroOrientation[1];
  pAngE->Roll  = FACTOR_ACC*newAccMagOrientation[2] + (1-FACTOR_ACC)*newGyroOrientation[2];

#ifdef USE_MAGNETOMETER
  /* Normalise the measurements of magnetometer */
  Normalize = invSqrtf(squa(pMag[0]) + squa(pMag[1]) + squa(pMag[2]));
  MagX = pMag[0]*Normalize;
  MagY = pMag[1]*Normalize;
  MagZ = pMag[2]*Normalize;

  /* Compensate for dip angle */
  sinP = arm_sin_f32(pAngE->Pitch);
  cosP = arm_cos_f32(pAngE->Pitch);
  sinR = arm_sin_f32(pAngE->Roll);
  cosR = arm_cos_f32(pAngE->Roll);

  HorizonX =  MagX*cosP + MagY*sinP*sinR + MagZ*sinP*cosR;
  HorizonY =  					  MagY     *cosR - MagZ*     sinR;
  HorizonZ = -MagX*sinP + MagY*cosP*sinR + MagZ*cosR*cosR;

  /* Compute yaw with magnetometer */
  Normalize = invSqrtf(squa(HorizonX) + squa(HorizonY) + squa(HorizonZ));
  HorizonX = HorizonX*Normalize;
  HorizonY = HorizonY*Normalize;
  HorizonZ = HorizonZ*Normalize;
  newAccMagOrientation[0] = atan2f(HorizonX, HorizonY);
#endif

#ifdef USE_MAGNETOMETER
  /* The Complementary Filter for Pitch and Roll*/
  pAngE->Yaw   = FACTOR_MAG*newAccMagOrientation[0] + (1-FACTOR_MAG)*newGyroOrientation[0];
#else
  pAngE->Yaw   = newGyroOrientation[0];
#endif

}

#endif





#ifdef  USE_EKF_ALGORITHM

#define   Interval    		 0.02f
#define 	Threshold					1.0f

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

void AHRS_Update(float* pGyr, float* pAcc, float* pMag, EulerAngle* pAngE)
{
  float	cbn[9];
  float xerror_a[4];
  float fa[16];
  float h_a[12];
  float zerror_a[3];
  float Normalize;
  float q0, q1, q2, q3;
  float dx, dy, dz;
  float AccX, AccY, AccZ;
  float GyrX, GyrY, GyrZ;
#ifdef USE_MAGNETOMETER
  float MagX, MagY, MagZ;
  float zerror_m[3];
  float h_m[12];
  float xerror_m[4];
#endif

  /* Processing the measurements of gyroscope */
  if(pGyr[2] < Threshold && pGyr[2] > -Threshold)
  {
	GyrX = pGyr[0];
	GyrY = pGyr[1];
	GyrZ = 0;
  }
  else
  {
	GyrX = pGyr[0];
	GyrY = pGyr[1];
	GyrZ = pGyr[2];
  }
  
  gyo.Pitch_gy = GyrY;
  gyo.Roll_gy = GyrX;
  gyo.Yaw_gy = GyrZ;
  
 
  /* Normalise the measurements of accelerometer */
  Normalize = invSqrtf(squa(pAcc[0]) + squa(pAcc[1]) + squa(pAcc[2]));
  AccX = pAcc[0]*Normalize;
  AccY = pAcc[1]*Normalize;
  AccZ = pAcc[2]*Normalize;
  
   accsource.ACCX = asinf(-AccX);
   accsource.ACCY = atan2f(AccY,AccZ);
  
  
  
  
#ifdef USE_MAGNETOMETER
  /* Normalise the measurements of magnetometer */
  Normalize = invSqrtf(squa(pMag[0]) + squa(pMag[1]) + squa(pMag[2]));
  MagX = pMag[0]*Normalize;
  MagY = pMag[1]*Normalize;
  MagZ = pMag[2]*Normalize;
#endif
  /* Update quaternion */
  dx = toRad(GyrX) * Interval;
  dy = toRad(GyrY) * Interval;
  dz = toRad(GyrZ) * Interval;

  Normalize = squa(dx) + squa(dy) + squa(dz);
  fa[0]  =  (1 - 0.125*Normalize);
  fa[1]  = -(0.5 - Normalize*0.020833333)*dx;
  fa[2]  = -(0.5 - Normalize*0.020833333)*dy;
  fa[3]  = -(0.5 - Normalize*0.020833333)*dz;
  fa[4]  =  (0.5 - Normalize*0.020833333)*dx;
  fa[5]  =  (1 - 0.125*Normalize);
  fa[6]  =  (0.5 - Normalize*0.020833333)*dz;
  fa[7]  = -(0.5 - Normalize*0.020833333)*dy;
  fa[8]  =  (0.5 - Normalize*0.020833333)*dy;
  fa[9]  = -(0.5 - Normalize*0.020833333)*dz;
  fa[10] =  (1 - 0.125*Normalize);
  fa[11] =  (0.5 - Normalize*0.020833333)*dx;
  fa[12] =  (0.5 - Normalize*0.020833333)*dz;
  fa[13] =  (0.5 - Normalize*0.020833333)*dy;
  fa[14] = -(0.5 - Normalize*0.020833333)*dx;
  fa[15] =  (1 - 0.125*Normalize);

  q0 = fa[0]* NumQ.q0 + fa[1]* NumQ.q1 + fa[2]* NumQ.q2 + fa[3] *NumQ.q3;
  q1 = fa[4]* NumQ.q0 + fa[5]* NumQ.q1 + fa[6]* NumQ.q2 + fa[7] *NumQ.q3;
  q2 = fa[8]* NumQ.q0 + fa[9]* NumQ.q1 + fa[10]*NumQ.q2 + fa[11]*NumQ.q3;
  q3 = fa[12]*NumQ.q0 + fa[13]*NumQ.q1 + fa[14]*NumQ.q2 + fa[15]*NumQ.q3;
  NumQ.q0 = q0;
  NumQ.q1 = q1;
  NumQ.q2 = q2;
  NumQ.q3 = q3;
  QuaternionNormalize(&NumQ);

  /********************** Sensor Fusion for Pitch and Roll **********************/
  //cbn[0] = squa(NumQ.q0) + squa(NumQ.q1) - squa(NumQ.q2) - squa(NumQ.q3);
  //cbn[1] = 2*(NumQ.q1*NumQ.q2 - NumQ.q0*NumQ.q3);
  //cbn[2] = 2*(NumQ.q1*NumQ.q3 + NumQ.q0*NumQ.q2);
  //cbn[3] = 2*(NumQ.q1*NumQ.q2 + NumQ.q0*NumQ.q3);
  //cbn[4] = squa(NumQ.q0) - squa(NumQ.q1) + squa(NumQ.q2) - squa(NumQ.q3);
  //cbn[5] = 2*(NumQ.q2*NumQ.q3 - NumQ.q0*NumQ.q1);
  cbn[6] = 2*(NumQ.q1*NumQ.q3 - NumQ.q0*NumQ.q2);
  cbn[7] = 2*(NumQ.q2*NumQ.q3 + NumQ.q0*NumQ.q1);
  cbn[8] = squa(NumQ.q0) - squa(NumQ.q1) - squa(NumQ.q2) + squa(NumQ.q3);

  h_a[0] = -2*NumQ.q2;
  h_a[1] =  2*NumQ.q3;
  h_a[2] = -2*NumQ.q0;
  h_a[3] =  2*NumQ.q1;
  h_a[4] =  2*NumQ.q1;
  h_a[5] =  2*NumQ.q0;
  h_a[6] =  2*NumQ.q3;
  h_a[7] =  2*NumQ.q2;
  h_a[8] =  2*NumQ.q0;
  h_a[9] = -2*NumQ.q1;
  h_a[10]= -2*NumQ.q2;
  h_a[11]=  2*NumQ.q3;

  zerror_a[0] = AccX - cbn[6];
  zerror_a[1] = AccY - cbn[7];
  zerror_a[2] = AccZ - cbn[8];

  KalmanFilterUpdate(fa, h_a, PA, RA, QA, zerror_a, xerror_a);
  NumQ.q0 = NumQ.q0 + xerror_a[0];
  NumQ.q1 = NumQ.q1 + xerror_a[1];
  NumQ.q2 = NumQ.q2 + xerror_a[2];
  NumQ.q3 = NumQ.q3 + xerror_a[3];
  QuaternionNormalize(&NumQ);

#ifndef	USE_MAGNETOMETER
  QuaternionToAngE(&NumQ, &AngE);
#else
  //cbn[0] = squa(NumQ.q0) + squa(NumQ.q1) - squa(NumQ.q2) - squa(NumQ.q3);
  //cbn[1] = 2*(NumQ.q1*NumQ.q2 - NumQ.q0*NumQ.q3);
  //cbn[2] = 2*(NumQ.q1*NumQ.q3 + NumQ.q0*NumQ.q2);
  cbn[3] = 2*(NumQ.q1*NumQ.q2 + NumQ.q0*NumQ.q3);
  cbn[4] = squa(NumQ.q0) - squa(NumQ.q1) + squa(NumQ.q2) - squa(NumQ.q3);
  cbn[5] = 2*(NumQ.q2*NumQ.q3 - NumQ.q0*NumQ.q1);
  cbn[6] = 2*(NumQ.q1*NumQ.q3 - NumQ.q0*NumQ.q2);
  cbn[7] = 2*(NumQ.q2*NumQ.q3 + NumQ.q0*NumQ.q1);
  cbn[8] = squa(NumQ.q0) - squa(NumQ.q1) - squa(NumQ.q2) + squa(NumQ.q3);

  pAngE->Pitch = asinf(-cbn[6]);
  pAngE->Roll  = atan2f(cbn[7], cbn[8]);

  /********************** Sensor Fusion for Yaw **********************/
  h_m[0]  =  2*NumQ.q3*Mag_Y - 2*NumQ.q2*Mag_Z;
  h_m[1]  =  2*NumQ.q2*Mag_Y + 2*NumQ.q3*Mag_Z;
  h_m[2]  =  2*NumQ.q1*Mag_Y - 2*NumQ.q0*Mag_Z;
  h_m[3]  =  2*NumQ.q0*Mag_Y + 2*NumQ.q1*Mag_Z;
  h_m[4]  =  2*NumQ.q0*Mag_Y + 2*NumQ.q1*Mag_Z;
  h_m[5]  = -2*NumQ.q1*Mag_Y + 2*NumQ.q0*Mag_Z;
  h_m[6]  =  2*NumQ.q2*Mag_Y + 2*NumQ.q3*Mag_Z;
  h_m[7]  = -2*NumQ.q3*Mag_Y + 2*NumQ.q2*Mag_Z;
  h_m[8]  = -2*NumQ.q1*Mag_Y + 2*NumQ.q0*Mag_Z;
  h_m[9]  = -2*NumQ.q0*Mag_Y - 2*NumQ.q1*Mag_Z;
  h_m[10] =  2*NumQ.q3*Mag_Y - 2*NumQ.q2*Mag_Z;
  h_m[11] =  2*NumQ.q2*Mag_Y + 2*NumQ.q3*Mag_Z;

  zerror_m[0] = MagX - cbn[3]*Mag_Y - cbn[6]*Mag_Z;
  zerror_m[1] = MagY - cbn[4]*Mag_Y - cbn[7]*Mag_Z;
  zerror_m[2] = MagZ - cbn[5]*Mag_Y - cbn[8]*Mag_Z;


  KalmanFilterUpdate(fa, h_m, PM, RM, QM, zerror_m, xerror_m);
  NumQ.q0 = NumQ.q0 + xerror_m[0];
  NumQ.q1 = NumQ.q1 + xerror_m[1];
  NumQ.q2 = NumQ.q2 + xerror_m[2];
  NumQ.q3 = NumQ.q3 + xerror_m[3];
  QuaternionNormalize(&NumQ);

  cbn[0] = squa(NumQ.q0) + squa(NumQ.q1) - squa(NumQ.q2) - squa(NumQ.q3);
  //cbn[1] = 2*(NumQ.q1*NumQ.q2 - NumQ.q0*NumQ.q3);
  //cbn[2] = 2*(NumQ.q1*NumQ.q3 + NumQ.q0*NumQ.q2);
  cbn[3] = 2*(NumQ.q1*NumQ.q2 + NumQ.q0*NumQ.q3);
  //cbn[4] = squa(NumQ.q0) - squa(NumQ.q1) + squa(NumQ.q2) - squa(NumQ.q3);
  //cbn[5] = 2*(NumQ.q2*NumQ.q3 - NumQ.q0*NumQ.q1);
  //cbn[6] = 2*(NumQ.q1*NumQ.q3 - NumQ.q0*NumQ.q2);
  //cbn[7] = 2*(NumQ.q2*NumQ.q3 + NumQ.q0*NumQ.q1);
  //cbn[8] = squa(NumQ.q0) - squa(NumQ.q1) - squa(NumQ.q2) + squa(NumQ.q3);
  pAngE->Yaw = atan2f(cbn[3], cbn[0]);

  QuaternionToNumQ(&NumQ, &AngE);

//  pAngE->Yaw -= AngE_Zero.Yaw;
//  pAngE->Pitch -= AngE_Zero.Pitch;
//  pAngE->Roll -= AngE_Zero.Roll;

#endif
}

#endif





#ifdef  USE_GRADIENT_DESCENT_ALGORITHM

#define   Interval    		 0.02f
#define   IntervalHalf		 0.01f
#define 	Threshold				  1.0f
#define 	Beta 						0.041f

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

void AHRS_Update(float* pGyr, float* pAcc, float* pMag, EulerAngle* pAngE)
{
  float AccX, AccY, AccZ;
  float GyrX, GyrY, GyrZ;
  float Normalize;
  float Mq13, Mq23, Mq33;
  float f_1, f_2, f_3; 																																// objective function elements
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; 							// quaternion derrivative from gyroscopes elements
  float SEqDot_gradient_1, SEqDot_gradient_2, SEqDot_gradient_3, SEqDot_gradient_4; 	// estimated direction of the gyroscope error
#ifdef USE_MAGNETOMETER
  float MagX, MagY, MagZ;
  float Mq12, Mq22, Mq32;
  float f_4, f_5, f_6;
#endif
//  if(gCommand_Packet.mThrust != 0){
//	RA[0] = 0.01;
//	RA[4] = 0.01;
//	RA[8] = 0.01;
//  }
//  else{
//	RA[0] = 0.8;
//	RA[4] = 0.8;
//	RA[8] = 0.8;
//  }

  /* Processing the measurements of gyroscope */
  if(pGyr[2] < Threshold && pGyr[2] > -Threshold)
  {
	GyrX = pGyr[0];
	GyrY = pGyr[1];
	GyrZ = 0;
  }
  else
  {
	GyrX = pGyr[0];
	GyrY = pGyr[1];
	GyrZ = pGyr[2];
  }
  /* Normalise the measurements of accelerometer */
  Normalize = invSqrtf(squa(pAcc[0]) + squa(pAcc[1]) + squa(pAcc[2]));
  AccX = pAcc[0]*Normalize;
  AccY = pAcc[1]*Normalize;
  AccZ = pAcc[2]*Normalize;
#ifdef USE_MAGNETOMETER
  /* Normalise the measurements of magnetometer */
  Normalize = invSqrtf(squa(pMag[0]) + squa(pMag[1]) + squa(pMag[2]));
  MagX = pMag[0]*Normalize;
  MagY = pMag[1]*Normalize;
  MagZ = pMag[2]*Normalize;
#endif

  /* Direction Cosine Matrix at last time  */

  //	 Mq11 = NumQ.q0*NumQ.q0 + NumQ.q1*NumQ.q1 - NumQ.q2*NumQ.q2 - NumQ.q3*NumQ.q3;
#ifdef USE_MAGNETOMETER
  Mq12 = 2.0f*(NumQ.q1*NumQ.q2 + NumQ.q0*NumQ.q3);
#endif
  Mq13 = 2.0f*(NumQ.q1*NumQ.q3 - NumQ.q0*NumQ.q2);
  //	 Mq21 = 2.0f*(NumQ.q1*NumQ.q2 - NumQ.q0*NumQ.q3);
#ifdef USE_MAGNETOMETER
  Mq22 = NumQ.q0*NumQ.q0 - NumQ.q1*NumQ.q1 + NumQ.q2*NumQ.q2 - NumQ.q3*NumQ.q3;
#endif
  Mq23 = 2.0f*(NumQ.q0*NumQ.q1 + NumQ.q2*NumQ.q3);
  //	 Mq31 = 2.0f*(NumQ.q0*NumQ.q2 + NumQ.q1*NumQ.q3);
#ifdef USE_MAGNETOMETER
  Mq32 = 2.0f*(NumQ.q2*NumQ.q3 - NumQ.q0*NumQ.q1);
#endif
  Mq33 = NumQ.q0*NumQ.q0 - NumQ.q1*NumQ.q1 - NumQ.q2*NumQ.q2 + NumQ.q3*NumQ.q3;

  /* Compute the objective function and Jacobian */
  f_1 = Mq13 - AccX;
  f_2 = Mq23 - AccY;
  f_3 = Mq33 - AccZ;
#ifdef USE_MAGNETOMETER
  f_4 = Mq12*Mag_Y + Mq13*Mag_Z - MagX;
  f_5 = Mq22*Mag_Y + Mq23*Mag_Z - MagY;
  f_6 = Mq32*Mag_Y + Mq33*Mag_Z - MagZ;
#endif

  /* Compute the gradient */
  SEqDot_gradient_1 = -2*NumQ.q2*f_1 + 2*NumQ.q1*f_2 + 2*NumQ.q0*f_3;
  SEqDot_gradient_2 =  2*NumQ.q3*f_1 + 2*NumQ.q0*f_2 - 2*NumQ.q1*f_3;
  SEqDot_gradient_3 = -2*NumQ.q0*f_1 + 2*NumQ.q3*f_2 - 2*NumQ.q2*f_3;
  SEqDot_gradient_4 =  2*NumQ.q1*f_1 + 2*NumQ.q2*f_2 + 2*NumQ.q3*f_3;
#ifdef USE_MAGNETOMETER
  SEqDot_gradient_1 += (2*NumQ.q3*Mag_Y - 2*NumQ.q2*Mag_Z)*f_4 + ( 2*NumQ.q0*Mag_Y + 2*NumQ.q1*Mag_Z)*f_5 + (-2*NumQ.q1*Mag_Y + 2*NumQ.q0*Mag_Z)*f_6;
  SEqDot_gradient_2 += (2*NumQ.q2*Mag_Y + 2*NumQ.q3*Mag_Z)*f_4 + (-2*NumQ.q1*Mag_Y + 2*NumQ.q0*Mag_Z)*f_5 + (-2*NumQ.q0*Mag_Y - 2*NumQ.q1*Mag_Z)*f_6;
  SEqDot_gradient_3 += (2*NumQ.q1*Mag_Y - 2*NumQ.q0*Mag_Z)*f_4 + ( 2*NumQ.q2*Mag_Y + 2*NumQ.q3*Mag_Z)*f_5 + ( 2*NumQ.q3*Mag_Y - 2*NumQ.q2*Mag_Z)*f_6;
  SEqDot_gradient_4 += (2*NumQ.q0*Mag_Y + 2*NumQ.q1*Mag_Z)*f_4 + (-2*NumQ.q3*Mag_Y + 2*NumQ.q2*Mag_Z)*f_5 + ( 2*NumQ.q2*Mag_Y + 2*NumQ.q3*Mag_Z)*f_6;
#endif

  /* Normalise the gradient */
  Normalize = invSqrtf(squa(SEqDot_gradient_1) + squa(SEqDot_gradient_2) + squa(SEqDot_gradient_3) + squa(SEqDot_gradient_4));
  SEqDot_gradient_1 = SEqDot_gradient_1*Normalize;
  SEqDot_gradient_2 = SEqDot_gradient_2*Normalize;
  SEqDot_gradient_3 = SEqDot_gradient_3*Normalize;
  SEqDot_gradient_4 = SEqDot_gradient_4*Normalize;

  /* Compute the quaternion derrivative measured by gyroscopes */
  GyrX = toRad(GyrX);
  GyrY = toRad(GyrY);
  GyrZ = toRad(GyrZ);
  SEqDot_omega_1 = -NumQ.q1*GyrX - NumQ.q2*GyrY - NumQ.q3*GyrZ;
  SEqDot_omega_2 =  NumQ.q0*GyrX - NumQ.q3*GyrY + NumQ.q2*GyrZ;
  SEqDot_omega_3 =  NumQ.q3*GyrX + NumQ.q0*GyrY - NumQ.q1*GyrZ;
  SEqDot_omega_4 = -NumQ.q2*GyrX + NumQ.q1*GyrY + NumQ.q0*GyrZ;

  /* Compute then integrate the estimated quaternion derrivative */
  NumQ.q0 += SEqDot_omega_1*IntervalHalf - Beta*SEqDot_gradient_1*Interval;
  NumQ.q1 += SEqDot_omega_2*IntervalHalf - Beta*SEqDot_gradient_2*Interval;
  NumQ.q2 += SEqDot_omega_3*IntervalHalf - Beta*SEqDot_gradient_3*Interval;
  NumQ.q3 += SEqDot_omega_4*IntervalHalf - Beta*SEqDot_gradient_4*Interval;

  /* Normalise quaternion */
  QuaternionNormalize(&NumQ);

  /* Convert Quaternion to Euler */
  QuaternionToAngE(&NumQ, &AngE);


}

#endif





/**
* @brief  Convert quaternion to Euler angles(radias).
*
* @param  pNumQ: Pointer to the buffer containing the quaternion.
* @param  pAngE: Pointer to the buffer containing the Euler angles(radias).
*
* @retval None.
*
*/

static void QuaternionToAngE( Quaternion *pNumQ, EulerAngle *pAngE )
{
  float NumQ_T11 = pNumQ->q0*pNumQ->q0 + pNumQ->q1*pNumQ->q1 - pNumQ->q2*pNumQ->q2 - pNumQ->q3*pNumQ->q3;
  float NumQ_T12 = 2.0f*(pNumQ->q0*pNumQ->q3 + pNumQ->q1*pNumQ->q2);
  float NumQ_T13 = 2.0f*(pNumQ->q1*pNumQ->q3 - pNumQ->q0*pNumQ->q2);
  float NumQ_T23 = 2.0f*(pNumQ->q0*pNumQ->q1 + pNumQ->q2*pNumQ->q3);
  float NumQ_T33 = pNumQ->q0*pNumQ->q0 - pNumQ->q1*pNumQ->q1 - pNumQ->q2*pNumQ->q2 + pNumQ->q3*pNumQ->q3;

  pAngE->Yaw   = atan2f(NumQ_T12, NumQ_T11);
  pAngE->Pitch = -asinf(NumQ_T13);
  pAngE->Roll  = atan2f(NumQ_T23, NumQ_T33);

}



/**
* @brief  Normalize quaternion.
*
* @param  pNumQ: Pointer to the buffer containing the quaternion.
*
* @retval None.
*
*/

static void QuaternionNormalize( Quaternion *pNumQ )
{
  float Normalize = 0.0f;

  Normalize = invSqrtf(squa(pNumQ->q0) + squa(pNumQ->q1) + squa(pNumQ->q2) + squa(pNumQ->q3));

  pNumQ->q0 = pNumQ->q0*Normalize;
  pNumQ->q1 = pNumQ->q1*Normalize;
  pNumQ->q2 = pNumQ->q2*Normalize;
  pNumQ->q3 = pNumQ->q3*Normalize;

}



/**
* @brief  Convert Euler angles(radias) to quaternion.
*
* @param  pNumQ: Pointer to the buffer containing the quaternion.
* @param  pAngE: Pointer to the buffer containing the Euler angles(radias).
*
* @retval None.
*
*/

static void QuaternionToNumQ( Quaternion *pNumQ, EulerAngle *pAngE )
{
  float halfY = pAngE->Yaw/2.0f;
  float halfP = pAngE->Pitch/2.0f;
  float halfR = pAngE->Roll/2.0f;

  float sinY = arm_sin_f32(halfY);
  float cosY = arm_cos_f32(halfY);
  float sinP = arm_sin_f32(halfP);
  float cosP = arm_cos_f32(halfP);
  float sinR = arm_sin_f32(halfR);
  float cosR = arm_cos_f32(halfR);

  pNumQ->q0 = cosR*cosP*cosY + sinR*sinP*sinY;
  pNumQ->q1 = sinR*cosP*cosY - cosR*sinP*sinY;
  pNumQ->q2 = cosR*sinP*cosY + sinR*cosP*sinY;
  pNumQ->q3 = cosR*cosP*sinY - sinR*sinP*cosY;

}