/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __USART3_BLE_H
#define __USART3_BLE_H

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
/* Exported types ------------------------------------------------------------*/
struct _Command_Packet{
  float mRoll;
  float mPitch;
  float mYaw;
  uint16_t mThrust;
};
typedef struct _Command_Packet Command_Packet;
/* Exported define -----------------------------------------------------------*/
//PIN5 模块复位，低有效
#define BLE_RES_PIN						GPIO_PIN_12
#define BLE_RES_GPIO_PORT				GPIOC
#define BLE_RES_GPIO_CLK					RCC_AHBPeriph_GPIOC
#define BLE_RES_SOURCE					GPIO_PINSource12

//PIN6
/*
* 模块使能控制线，默认为电平触发模式
* >>>>> 电平触发模式，低电平有效，带内部上拉。
* 		0：模块开始广播，直到连接到移动设备
* 		1：无论模块当前状态，立即进入完全睡眠状态 (0.4uA)
* >>>>> 脉冲触发模式，每收到一次脉冲（ W>200ms） ,
*       模块会在开机（进行广播，允许被发现和连接）以及关机（完全睡眠
*       状态）之间循环切换
*/
#define BLE_EN_PIN						GPIO_PIN_2
#define BLE_EN_GPIO_PORT					GPIOD
#define BLE_EN_GPIO_CLK					RCC_AHBPeriph_GPIOD
#define BLE_EN_SOURCE					GPIO_PINSource2

//PIN10
/*
* >>>>> 上电后 30 秒内，
* 		保持此引脚低电平 5s ，系统会恢复部分参数（ 浅恢复） ，
* 		若保持20s 以上则将会恢复全部参数（深度恢复）（见《系统复位与恢复》章节）
* >>>>> 可编程双向IO
*/
#define BLE_REST_PIN

//PIN11
/*
* >>>>> 上电时刻，做测试模式触发端。仅在上电时刻检测一次，内部上拉
* 		0： 进入透传测试模式(对直驱功能无影响)
* 		1：进入正常工作模式
* >>>>> 上电之后，做PWM 输出通道1
*/
#define BLE_PWM1_OR_MODE_PIN				GPIO_PIN_5
#define BLE_PWM1_OR_MODE_GPIO_PORT		GPIOB
#define BLE_PWM1_OR_MODE_GPIO_CLK		RCC_AHBPeriph_GPIOB
#define BLE_PWM1_OR_MODE_SOURCE			GPIO_PINSource5

//PIN14
/*
* 作为数据发送请求（用来唤醒模块）
* 0： 主机有数据发送， 模块将等待接收来自主机的数据, 此时模块不睡眠
* 1：主机无数据发送，或主机数据发送完毕之后，应该将此信号线置1
*/
#define BLE_BRTS_PIN						GPIO_PIN_6
#define BLE_BRTS_GPIO_PORT				GPIOB
#define BLE_BRTS_GPIO_CLK				RCC_AHBPeriph_GPIOB
#define BLE_BRTS_SOURCE					GPIO_PINSource6

//PIN15
/*
* 数据输入信号（用来唤醒主机，可选）
* 0： 模块有数据发送到主机，主机接收模块数据
* 1：模块无数据发送到主机，或模块数据发送完毕之后，会将此信号置1
*/
#define BLE_BCTS_PIN

/* Exported constants --------------------------------------------------------*/
extern Command_Packet gCommand_Packet;
/* Exported functions ------------------------------------------------------- */
void BLE_Init( void );
void BLE_BroadCast_Enable( uint8_t enable );
void BLE_Input_Enable( uint8_t enable );
void BLE_Power_Reset( void );
void BLE_Mode( uint8_t mode );
void BLE_StartRead(void);
void BLE_ReadOK(void);
#endif