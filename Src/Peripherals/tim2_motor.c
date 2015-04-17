/****************************************************************
*                                                              *
*    Created by Xuejilong					*
*                                                      	*
****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "tim2_motor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Set Motor Outputs.
* @param  motor pwr percent
* @retval None
*/
void Motor_Out( int16_t motor_1, int16_t motor_2, int16_t motor_3, int16_t motor_4 )
{
  int16_t motor1_Pulse = motor_1;
  int16_t motor2_Pulse = motor_2;
  int16_t motor3_Pulse = motor_3;
  int16_t motor4_Pulse = motor_4;
#if 0
  printf("motor_1:%d motor_2:%d motor_3:%d motor_4:%d\r\n",
         motor_1, motor_2, motor_3, motor_4);
#endif

  if(motor1_Pulse > MOTOR_SPEED_MAX) motor1_Pulse = MOTOR_SPEED_MAX;
  if(motor2_Pulse > MOTOR_SPEED_MAX) motor2_Pulse = MOTOR_SPEED_MAX;
  if(motor3_Pulse > MOTOR_SPEED_MAX) motor3_Pulse = MOTOR_SPEED_MAX;
  if(motor4_Pulse > MOTOR_SPEED_MAX) motor4_Pulse = MOTOR_SPEED_MAX;
  if(motor1_Pulse < MOTOR_SPEED_MIN) motor1_Pulse = MOTOR_SPEED_MIN;
  if(motor2_Pulse < MOTOR_SPEED_MIN) motor2_Pulse = MOTOR_SPEED_MIN;
  if(motor3_Pulse < MOTOR_SPEED_MIN) motor3_Pulse = MOTOR_SPEED_MIN;
  if(motor4_Pulse < MOTOR_SPEED_MIN) motor4_Pulse = MOTOR_SPEED_MIN;
#if 0
  MX_TIM2_PWM_SetChannelPulse(TIM_CHANNEL_1, motor1_Pulse);
  MX_TIM2_PWM_SetChannelPulse(TIM_CHANNEL_2, motor2_Pulse);
  MX_TIM2_PWM_SetChannelPulse(TIM_CHANNEL_3, motor3_Pulse);
  MX_TIM2_PWM_SetChannelPulse(TIM_CHANNEL_4, motor4_Pulse);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
#else
  htim2.Instance->CCR1 = motor1_Pulse;
  htim2.Instance->CCR2 = motor2_Pulse;
  htim2.Instance->CCR3 = motor3_Pulse;
  htim2.Instance->CCR4 = motor4_Pulse;
#endif
}