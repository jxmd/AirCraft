/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __PID_H
#define __PID_H

/* Includes ------------------------------------------------------------------*/
#include "algorithm_ahrs.h"
#include "tim2_motor.h"
#include "usart1_dbg.h"
#include "DataScope_DP.h"
/* Exported types ------------------------------------------------------------*/
struct _out_angle{
  float roll;
  float pitch;
  float yaw;
} ;

struct pid{
  float kp;
  float ki;
  float kd;
  float output;
  float integral;
};

typedef struct {
  float Yaw_expect;
  float Pitch_expect;
  float Roll_expect;
  uint16_t Throttle_expect;
} Expect;

/* Exported constants --------------------------------------------------------*/
extern Expect expect;

/* Exported functions ------------------------------------------------------- */
void Control_Angle(EulerAngle* pAngE,Expect *expect);
void Control_Gyro(float* pGyr,EulerAngle *pAngE);

#endif