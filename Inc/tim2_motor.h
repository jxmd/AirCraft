/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __TIM2_MOTOR_H
#define __TIM2_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include <stdio.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define MOTOR_SPEED_MIN 0
#define MOTOR_SPEED_MAX 1800
/* Exported functions ------------------------------------------------------- */
void Motor_Out( int16_t, int16_t, int16_t, int16_t );
#endif