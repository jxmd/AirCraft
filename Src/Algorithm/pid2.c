#include "pid.h"
#include "arm_math.h"
#include "DataScope_DP.h"
#include <stdio.h>
#define angle_max       10.0f
#define angle_integral_max  1000.0f
#define DertT 0.02                ////////////////////////
uint8_t Lock = 0;
Expect expect={0,0,0,0};
ACCSOURCE accsource={0,0,0,0,0,0};
Gyo gyo={0,0,0};
struct pid roll,pitch,yaw,gyro_pitch,gyro_yaw,gyro_roll;
/****************************角度PID参数设置***********************************/
#define PID_ROLL_P 10
#define PID_ROLL_I 0
#define PID_ROLL_D 0
#define PID_PITCH_P 10
#define PID_PITCH_I 0
#define PID_PITCH_D 0
#define PID_YAW_P 0
#define PID_YAW_I 0
#define PID_YAW_D 0
struct pid roll={PID_ROLL_P,PID_ROLL_I,PID_ROLL_D,0,0};
struct pid pitch={PID_PITCH_P,PID_PITCH_I,PID_PITCH_D,0,0};
struct pid yaw={PID_YAW_P,PID_YAW_I,PID_YAW_D,0,0};
/****************************角速度PID参数设置*********************************/
#define GYRO_ROLL_P 4.5
#define GYRO_ROLL_I 0
#define GYRO_ROLL_D 20
#define GYRO_PITCH_P 4.5
#define GYRO_PITCH_I 0
#define GYRO_PITCH_D 20
#define GYRO_YAW_P 0
#define GYRO_YAW_I 0
#define GYRO_YAW_D 0
struct pid gyro_roll={GYRO_ROLL_P,GYRO_ROLL_I,GYRO_ROLL_D,0,0};
struct pid gyro_pitch={GYRO_PITCH_P,GYRO_PITCH_I,GYRO_PITCH_D,0,0};
struct pid gyro_yaw={GYRO_YAW_P,GYRO_YAW_I,GYRO_YAW_D,0,0};
/******************************************************************************
函数原型：   void Control_Angle(struct _out_angle *angle,struct _Rc *rc)
功    能： PID控制器(外环)
*******************************************************************************/
void Control_Angle(EulerAngle *pAngE,Expect *expect)
{
    static struct _out_angle control_angle;
    static struct _out_angle last_angle;
    float pitch_expect = expect->Pitch_expect;
    float roll_expect = expect->Roll_expect;
    float yaw_expect = expect->Yaw_expect;
    int16_t  yaw1 	 = (int16_t)(pAngE->Yaw);
    int16_t  pitch1      = (int16_t)(pAngE->Pitch);
    int16_t  roll1	 = (int16_t)(pAngE->Roll);
    //printf("yaw1=%d   pitch1=%d  roll1=%d\r\n",yaw1,pitch1,roll1);
//////////////////////////////////////////////////////////////////
//          以下为角度环
    control_angle.roll  = roll_expect - roll1;
    control_angle.pitch = pitch_expect - pitch1;
    //control_angle.yaw = yaw_expect - yaw1;
//////////////////////////////////////////////////////////////////
    if(control_angle.roll >  angle_max)  //ROLL
        roll.integral +=  angle_max;
    if(control_angle.roll < -angle_max)
        roll.integral += -angle_max;
    else
        roll.integral += control_angle.roll;

    if(roll.integral >  angle_integral_max)
       roll.integral =  angle_integral_max;
    if(roll.integral < -angle_integral_max)
       roll.integral = -angle_integral_max;
//////////////////////////////////////////////////////////////////
    if(control_angle.pitch >  angle_max)//PITCH
       pitch.integral +=  angle_max;
    if(control_angle.pitch < -angle_max)
       pitch.integral += -angle_max;
    else
        pitch.integral += control_angle.pitch;

    if(pitch.integral >  angle_integral_max)
       pitch.integral =  angle_integral_max;
    if(pitch.integral < -angle_integral_max)
       pitch.integral = -angle_integral_max;


    if((control_angle.pitch < -5)||(control_angle.pitch > 5))//油门较小时，积分清零
    {
        pitch.integral = 0;
    }
    if((control_angle.roll < -5)||(control_angle.roll >5))//油门较小时，积分清零
    {
        roll.integral  = 0;
    }

    roll.output  = roll.kp *control_angle.roll  + roll.ki *roll.integral  + roll.kd *(control_angle.roll -last_angle.roll );
    pitch.output = pitch.kp*control_angle.pitch + pitch.ki*pitch.integral + pitch.kd*(control_angle.pitch-last_angle.pitch);
    //yaw.output = yaw.kp*control_angle.yaw + yaw.ki*yaw.integral + yaw.kd*(control_angle.yaw-last_angle.yaw);
//////////////////////////////////////////////////////////////////
    last_angle.roll =control_angle.roll;
    last_angle.pitch=control_angle.pitch;
}

//下面是内环代码：
#define gyro_max        50.0f
#define gyro_integral_max   5000.0f
/******************************************************************************
函数原型：   void Control_Gyro(struct _SI_float *gyro,struct _Rc *rc,uint8_t Lock)
功    能： PID控制器(内环)
*******************************************************************************/
void Control_Gyro(Gyo *gyo,EulerAngle *pAngE)
{
    static struct _out_angle control_gyro;
    static struct _out_angle last_gyro;

    int16_t throttle1,throttle2,throttle3,throttle4;
//////////////////////////////////////////////////////////////////
//          以下为角速度环
/*
    if(rc->YAW>1400 && rc->YAW<1600)
        rc->YAW=1500;
*/ 
    control_gyro.pitch =  gyo->Pitch_gy + pitch.output;
    control_gyro.roll = gyo->Roll_gy + roll.output;
    //control_gyro.yaw   = yaw.output - gyro[1]*Radian_to_Angle;
    control_gyro.yaw   = gyo->Yaw_gy - 0;


    //printf("%f,%f \r\n",gyo->Pitch_gy,gyo->Roll_gy);
//////////////////////////////////////////////////////////////////
    if(control_gyro.roll >  gyro_max)    //GYRO_ROLL
        gyro_roll.integral +=  gyro_max;
    if(control_gyro.roll < -gyro_max)
        gyro_roll.integral += -gyro_max;
    else
        gyro_roll.integral += control_gyro.roll;

    if(gyro_roll.integral >  gyro_integral_max)
       gyro_roll.integral =  gyro_integral_max;
    if(gyro_roll.integral < -gyro_integral_max)
       gyro_roll.integral = -gyro_integral_max;
//////////////////////////////////////////////////////////////////
    if(control_gyro.pitch >  gyro_max)//GYRO_PITCH
        gyro_pitch.integral +=  gyro_max;
    if(control_gyro.pitch < -gyro_max)
        gyro_pitch.integral += -gyro_max;
    else
        gyro_pitch.integral += control_gyro.pitch;

    if(gyro_pitch.integral >  gyro_integral_max)
       gyro_pitch.integral =  gyro_integral_max;
    if(gyro_pitch.integral < -gyro_integral_max)
       gyro_pitch.integral = -gyro_integral_max;
//////////////////////////////////////////////////////////////////
//  if(control_gyro.yaw >  gyro_max)//GYRO_YAW
//      gyro_yaw.integral +=  gyro_max;
//  if(control_gyro.yaw < -gyro_max)
//      gyro_yaw.integral += -gyro_max;
//    else
        gyro_yaw.integral += control_gyro.yaw;

    if(gyro_yaw.integral >  gyro_integral_max)
       gyro_yaw.integral =  gyro_integral_max;
    if(gyro_yaw.integral < -gyro_integral_max)
       gyro_yaw.integral = -gyro_integral_max;

    if((control_gyro.roll < -40)||(control_gyro.roll > 40))//油门较小时，积分清零
    {
        gyro_roll.integral = 0;
    }
    if((control_gyro.pitch < -40)||(control_gyro.pitch >40))//油门较小时，积分清零
    {
        gyro_pitch.integral  = 0;
    }
/*
    if(rc->THROTTLE<1200)//油门较小时，积分清零
    {
        gyro_yaw.integral  = 0;
    }
*/
    gyro_roll.output  = gyro_roll.kp *control_gyro.roll  + gyro_roll.ki *gyro_roll.integral  + gyro_roll.kd *(control_gyro.roll -last_gyro.roll );
    gyro_pitch.output = gyro_pitch.kp*control_gyro.pitch + gyro_pitch.ki*gyro_pitch.integral + gyro_pitch.kd*(control_gyro.pitch-last_gyro.pitch);
    //gyro_yaw.output   = gyro_yaw.kp  *control_gyro.yaw   + gyro_yaw.ki  *gyro_yaw.integral   + gyro_yaw.kd  *(control_gyro.yaw  -last_gyro.yaw  );
//////////////////////////////////////////////////////////////////
    last_gyro.roll =control_gyro.roll;
    last_gyro.pitch=control_gyro.pitch;
    last_gyro.yaw  =control_gyro.yaw;
//////////////////////////////////////////////////////////////////
    if(Lock==0)
    {
        //float temp = (1100/arm_cos_f32(toRad(pAngE->Pitch))/arm_cos_f32(toRad(pAngE->Roll)));
	  float temp = 1100;
        //printf("gyro_pitch:%f gyro_roll:%f  gyro_yaw:%f\r\n",
        //       gyro_pitch.output, gyro_roll.output, gyro_yaw.output);
        //printf("%f\r\n",temp);
        throttle1 = (int16_t)(temp + gyro_pitch.output + gyro_roll.output + gyro_yaw.output);
        throttle2 = (int16_t)(temp + gyro_pitch.output - gyro_roll.output - gyro_yaw.output);
        throttle3 = (int16_t)(temp - gyro_pitch.output - gyro_roll.output + gyro_yaw.output);
        throttle4 = (int16_t)(temp - gyro_pitch.output + gyro_roll.output - gyro_yaw.output);
//        throttle1 = (int16_t)(temp + gyro_pitch.output - gyro_roll.output - gyro_yaw.output);
//        throttle2 = (int16_t)(temp + gyro_pitch.output + gyro_roll.output + gyro_yaw.output);
//        throttle3 = (int16_t)(temp - gyro_pitch.output + gyro_roll.output - gyro_yaw.output);
//        throttle4 = (int16_t)(temp - gyro_pitch.output - gyro_roll.output + gyro_yaw.output);
    }
    else
    {
        throttle1=1100;
        throttle2=1100;
        throttle3=1100;
        throttle4=1100;
    }
    Motor_Out(throttle1,throttle2,throttle3,throttle4);
#if 0
	
	  uint8_t Send_Count = 0, i =0;
//	  float Fthrottle1 = throttle1;
//	  float Fthrottle2 = throttle2;
//	  float Fthrottle3 = throttle3;
//	  float Fthrottle4 = throttle4;
//	  DataScope_Get_Channel_Data( Fthrottle1, 1 );
//	  DataScope_Get_Channel_Data( Fthrottle2, 2 );
//	  DataScope_Get_Channel_Data( Fthrottle3, 3 );
//	  DataScope_Get_Channel_Data( Fthrottle4, 4 );
	  DataScope_Get_Channel_Data( (pAngE->Roll), 1 );
	  DataScope_Get_Channel_Data( (pAngE->Pitch), 2 );
//	  DataScope_Get_Channel_Data( toDeg(pAngE->Yaw), 7 );
//	  DataScope_Get_Channel_Data( gyro_pitch.output, 3 );
//	  DataScope_Get_Channel_Data( gyro_roll.output, 4 );
//	  DataScope_Get_Channel_Data( gyro_yaw.output, 7 );
//	  DataScope_Get_Channel_Data( pitch.output, 5 );
//	  DataScope_Get_Channel_Data( roll.output, 6 );
//	  DataScope_Get_Channel_Data( yaw.output, 10 );

//	  printf("%f %f %f %f\r\n", Fthrottle1, Fthrottle2, Fthrottle3, Fthrottle4);
	  Send_Count = DataScope_Data_Generate(2);
	  for(i = 0;i < Send_Count;i++)
	  {
		USART_DBG_Send(DataScope_OutPut_Buffer[i]);
	  }
	
#endif
}