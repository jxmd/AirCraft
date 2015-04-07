/****************************************************************
*																*
*    Created by Xuejilong										*
*		Driver for BLE Chip RF-BM-S02						*
****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usart3_ble.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define GPIO_ResetBits(x, y) HAL_GPIO_WritePin(x, y, GPIO_PIN_RESET)
#define GPIO_SetBits(x, y) HAL_GPIO_WritePin(x, y, GPIO_PIN_SET)
/* Private variables ---------------------------------------------------------*/
Command_Packet gCommand_Packet;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Initialize BLE.
* @param  None
* @retval None
*/

void BLE_Init( void )
{
  BLE_Mode( 0 );
  BLE_Power_Reset(  );
  BLE_BroadCast_Enable( 1 );
}

/**
* @brief  Set BLE Mode, must be set before Power On(not EN pin).
* @param  0 test;other normal
* @retval None
*/

void BLE_Mode( uint8_t mode )
{
  if(mode){
	GPIO_ResetBits(BLE_PWM1_OR_MODE_GPIO_PORT, BLE_PWM1_OR_MODE_PIN);
  }
  else{
	GPIO_SetBits(BLE_PWM1_OR_MODE_GPIO_PORT, BLE_PWM1_OR_MODE_PIN);
  }
}


/**
* @brief  Reset Power.
* @param  None
* @retval None
*/

void BLE_Power_Reset( void )
{
	//reset
	GPIO_ResetBits(BLE_RES_GPIO_PORT, BLE_RES_PIN);
	//resume
	GPIO_SetBits(BLE_RES_GPIO_PORT, BLE_RES_PIN);
}

/**
* @brief  Enable BLE BroadCast.
* @param  0 disable;other enable
* @retval None
*/

void BLE_BroadCast_Enable( uint8_t enable )
{
  if(enable){
	GPIO_ResetBits(BLE_EN_GPIO_PORT, BLE_EN_PIN);
  }
  else{
	GPIO_SetBits(BLE_EN_GPIO_PORT, BLE_EN_PIN);
  }
}

/**
* @brief  Enable BLE Input.
* @param  0 disable;other enable
* @retval None
*/

void BLE_Input_Enable( uint8_t enable )
{
  if(enable){
	GPIO_ResetBits(BLE_BRTS_GPIO_PORT, BLE_BRTS_PIN);
  }
  else{
	GPIO_SetBits(BLE_BRTS_GPIO_PORT, BLE_BRTS_PIN);
  }
}

/**
* @brief  Start Read Packet.
* @param  None
* @retval None
*/

void BLE_StartRead(void)
{
  printf("**\r\n");
  while(HAL_UART_Receive_DMA(&huart3, (uint8_t*)&gCommand_Packet, sizeof(gCommand_Packet)) != HAL_OK);
}

/**
* @brief  Start Read Packet.
* @param  None
* @retval None
*/

void BLE_ReadOK(void)
{
  printf("mRoll:%f mPitch:%f mYaw:%f mThrust:%u\r\n",
			 gCommand_Packet.mRoll, gCommand_Packet.mPitch, gCommand_Packet.mYaw, gCommand_Packet.mThrust);
}