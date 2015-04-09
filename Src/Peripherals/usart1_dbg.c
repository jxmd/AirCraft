/****************************************************************
*																*
*    Created by Xuejilong										*
*		Driver for USART_DBG Chip RF-BM-S02						*
****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usart1_dbg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t volatile STATE = 0;
static uint16_t volatile USART1_IS_INITED = 0;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Initialize USART_DBG.
* @param  None
* @retval None
*/

void USART_DBG_Init( void )
{
  UartTxRxBuf_Reg(COM1, BUFFER_TX, 1024);
  USART1_IS_INITED = 1;
}

/**
* @brief  Start Send byte.
* @param  None
* @retval None
*/

void USART_DBG_Send(uint8_t ch)
{
  if(!USART1_IS_INITED) return;
  uint8_t c = 0;
  Uart_Put_Char(COM1, ch);
  if(STATE == 0){
	c = Uart_Get_Char_HW(COM1);
	HAL_UART_Transmit_IT(&huart1, &c, 1);
	STATE = 1;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t c = 0;
  if(Uart_Get_TXSize_HW(COM1)!=0){
	c = Uart_Get_Char_HW(COM1);
	HAL_UART_Transmit_IT(&huart1, &c, 1);
  }
  else{
	STATE = 0;
  }

}