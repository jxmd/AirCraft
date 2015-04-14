#ifndef __USART_PING_PONG_H
#define __USART_PING_PONG_H
#define USEUSART3

#define USART_BufferChannel 8
#define USART_BufferLength 48


/* 整个结构体对内核而言都是只读的，内核不能进行任何写操作 */
typedef struct
{
  /* 表示最先接收到数据的通道 */
  /* EG.3，表示通道3是最先收到的数据 */

  uint16_t volatile RecBufChaOldest;

  /* 表示当前通道是否被选中作为注入通道 */
  /* EG.3，表示被选中的是通道3 */

  uint16_t volatile RecBufCurrent;

  /* 表示该通道由于硬件原因导致数据错误 */

  uint32_t RecBufChaErr;

  /* 数据溢出，标志为READY的通道未在规定的时间移出，而新的数据需要使用该通道。*/

  ErrorStatus RecBufOverFlow;

  /* 缓冲区 */
  uint8_t RecBuf[USART_BufferChannel][USART_BufferLength];

  uint16_t RecBufValLen[USART_BufferChannel];
} USARTRecBufDef;


#define MOVE_OLDEST_BUFFER_TO_NEXT_CHANNEL(x) do{\
	if(x.RecBufChaOldest == (USART_BufferChannel - 1)) \
  		x.RecBufChaOldest=0;\
	else \
		x.RecBufChaOldest++;\
}while(0)

#define MOVE_CURRENT_BUFFER_TO_NEXT_CHANNEL(x) do{\
	if(x.RecBufCurrent == (USART_BufferChannel - 1)) \
  		x.RecBufCurrent=0;\
	else \
		x.RecBufCurrent++;\
	if(x.RecBufChaOldest == x.RecBufCurrent){\
	  MOVE_OLDEST_BUFFER_TO_NEXT_CHANNEL(x);\
	}\
}while(0)

#ifdef USEUSART1
  extern USARTRecBufDef USART1Buf;
#endif
#ifdef USEUSART2
  extern USARTRecBufDef USART2Buf;
#endif
#ifdef USEUSART3
  extern USARTRecBufDef USART3Buf;
#endif
#ifdef USEUART4
  extern USARTRecBufDef UART4Buf;
#endif
#ifdef USEUART5
  extern USARTRecBufDef UART5Buf;
#endif

#endif