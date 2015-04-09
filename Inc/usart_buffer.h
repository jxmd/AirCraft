/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __USART_BUFFER_H
#define __USART_BUFFER_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint16_t volatile Wd_Indx;
  uint16_t volatile Rd_Indx;
  uint16_t Mask;
  uint16_t MaxSize;
  uint8_t *pbuf;
}UartBuf;
/* Exported define -----------------------------------------------------------*/
#define COM1 0
#define COM2 1
#define COM3 2
#define BUFFER_TX 0
#define BUFFER_RX 1
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
bool UartTxRxBuf_Reg(uint8_t com, uint8_t txrx, uint16_t size);
bool UartTxRxBuf_UnReg(uint8_t com, uint8_t txrx);
bool UartTxBuf_GetState(uint8_t com, uint8_t txrx);

bool UartIsDataReceived( uint8_t com );
uint8_t Uart_Get_Char( uint8_t com );
uint8_t Uart_Put_Char(uint8_t com, uint8_t DataToSend);
void Uart_Put_Char_HW(uint8_t com, uint8_t DataRead);
uint8_t Uart_Get_Char_HW( uint8_t com );
uint16_t Uart_Get_TXSize_HW( uint8_t com );
#endif