/****************************************************************
*																*
*    Created by Xuejilong										*
*		Driver for USART_DBG Chip RF-BM-S02						*
****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usart1_dbg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TXBUFFER_SIZE 512
/* Private variables ---------------------------------------------------------*/
static uint16_t volatile STATE = 0;
static uint16_t volatile USART1_IS_INITED = 0;
static uint8_t arrayDMABuffer[TXBUFFER_SIZE] = {0};
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Initialize USART_DBG.
* @param  None
* @retval None
*/

void USART_DBG_Init( void )
{
  UartTxRxBuf_Reg(COM1, BUFFER_TX, TXBUFFER_SIZE);
  USART1_IS_INITED = 1;
}

/**
* @brief  Insert byte to Buffer.
* @param  None
* @retval None
*/

void USART_DBG_Send(uint8_t ch)
{
  if(!USART1_IS_INITED) return;
  Uart_Put_Char(COM1, ch);
}

/**
* @brief  Start Send bytes.
* @param  None
* @retval None
*/

void USART_DBG_StartSend(void)
{
  if(!USART1_IS_INITED && STATE == 1) return;
  uint16_t count = Uart_Get_TXSize_HW(COM1);
  uint16_t i = 0;
  if(count){
	STATE = 1;
	if(count >= TXBUFFER_SIZE) count = TXBUFFER_SIZE;
	for(i = 0;i < count;i++)
	{
	  arrayDMABuffer[i] = Uart_Get_Char_HW(COM1);
	}
	HAL_UART_Transmit_DMA(&huart1, arrayDMABuffer, count);
  }
  else{
	STATE = 0;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t count = Uart_Get_TXSize_HW(COM1);
  uint16_t i = 0;
  if(count){
	STATE = 1;
	if(count >= TXBUFFER_SIZE) count = TXBUFFER_SIZE;
	for(i = 0;i < count;i++)
	{
	  arrayDMABuffer[i] = Uart_Get_Char_HW(COM1);
	}
	HAL_UART_Transmit_DMA(&huart1, arrayDMABuffer, count);
  }
  else{
	STATE = 0;
  }
}