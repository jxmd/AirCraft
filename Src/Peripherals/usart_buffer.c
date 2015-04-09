/****************************************************************
*																*
*    Created by Xuejilong										*
*		Driver for Usart 										*
****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usart_buffer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define REGISTERED true
#define UNREGISTERED false
bool UartBufIsRegistered[2][3]={UNREGISTERED};

/* Private variables ---------------------------------------------------------*/
static UartBuf UartTxRxbuf[2][3];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void UartTxBuf_SetState(uint8_t com, uint8_t txrx, bool state)
{
  UartBufIsRegistered[txrx][com] = state;
}

bool UartTxBuf_GetState(uint8_t com, uint8_t txrx)
{
  return UartBufIsRegistered[txrx][com];
}

bool UartTxRxBuf_Reg(uint8_t com, uint8_t txrx, uint16_t size)
{
  if(UartTxBuf_GetState(com, txrx) == UNREGISTERED){
	uint8_t* p = 0;
	p = (uint8_t *)malloc(size);
	if(p){
	  UartTxRxbuf[txrx][com].Wd_Indx = 0;
	  UartTxRxbuf[txrx][com].Rd_Indx = 0;
	  UartTxRxbuf[txrx][com].MaxSize = size;
	  UartTxRxbuf[txrx][com].Mask = size - 1;
	  UartTxRxbuf[txrx][com].pbuf = p;
	  UartTxBuf_SetState(com, txrx, true);
	  return true;
	}
	else{
	  printf("Malloc Memory Error\r\n");
	  return false;
	}
  }
  else{
	return false;
  }
}

bool UartTxRxBuf_UnReg(uint8_t com, uint8_t txrx)
{
  if(UartTxBuf_GetState(com, txrx) == REGISTERED){
	UartTxRxbuf[txrx][com].Wd_Indx = 0;
	UartTxRxbuf[txrx][com].Rd_Indx = 0;
	UartTxRxbuf[txrx][com].MaxSize = 0;
	UartTxRxbuf[txrx][com].Mask = 0;
	free(UartTxRxbuf[txrx][com].pbuf);
	UartTxBuf_SetState(com, txrx, false);
	return true;
  }
  else{
	return false;
  }
}

//读取环形数据中的一个字节
static uint8_t UartBuf_RD(UartBuf *Ringbuf)
{
  uint8_t temp;
  temp = Ringbuf->pbuf[Ringbuf->Rd_Indx & Ringbuf->Mask];//数据长度掩码很重要，这是决定数据环形的关键
  Ringbuf->Rd_Indx++;//读取完成一次，读指针加1，为下一次 读取做 准备
  return temp;
}
//将一个字节写入一个环形结构体中
static void UartBuf_WD(UartBuf *Ringbuf,uint8_t DataIn)
{
  Ringbuf->pbuf[Ringbuf->Wd_Indx & Ringbuf->Mask] = DataIn;//数据长度掩码很重要，这是决定数据环形的关键
  Ringbuf->Wd_Indx++;//写完一次，写指针加1，为下一次写入做准备
}
//环形数据区的可用字节长度，当写指针写完一圈，追上了读指针
//那么证明数据写满了，此时应该增加缓冲区长度，或者缩短外围数据处理时间
static uint16_t UartBuf_Cnt(UartBuf *Ringbuf)
{
  return (Ringbuf->Wd_Indx - Ringbuf->Rd_Indx) & Ringbuf->Mask;//数据长度掩码很重要，这是决定数据环形的关键
}

static uint16_t UartBuf_MaxSize(UartBuf *Ringbuf)
{
  return Ringbuf->MaxSize;
}

static void UartBufClear(UartBuf *Ringbuf)
{
	Ringbuf->Rd_Indx=Ringbuf->Wd_Indx;
}

uint8_t Uart_Put_Char(uint8_t com, uint8_t DataToSend)
{
  if (!UartTxBuf_GetState(com, BUFFER_TX))
	return 0;
  UartBuf_WD(&UartTxRxbuf[BUFFER_TX][com],DataToSend);//将待发送数据放在环形缓冲数组中
  return DataToSend;
}

void Uart_Put_Char_HW(uint8_t com, uint8_t DataRead)
{
  if (!UartTxBuf_GetState(com, BUFFER_RX))
	return;
  UartBuf_WD(&UartTxRxbuf[BUFFER_RX][com],DataRead);//将待发送数据放在环形缓冲数组中
}

uint8_t Uart_Get_Char( uint8_t com )
{
  if (!UartTxBuf_GetState(com, BUFFER_RX))
	return 0;
  return UartBuf_RD(&UartTxRxbuf[BUFFER_RX][com]);
}

uint8_t Uart_Get_Char_HW( uint8_t com )
{
  if (!UartTxBuf_GetState(com, BUFFER_TX))
	return 0;
  return UartBuf_RD(&UartTxRxbuf[BUFFER_TX][com]);
}

bool UartIsDataReceived( uint8_t com )
{
  if (!UartTxBuf_GetState(com, BUFFER_RX))
	return 0;
  return UartBuf_Cnt(&UartTxRxbuf[BUFFER_RX][com]);
}

uint16_t Uart_Get_TXSize_HW( uint8_t com )
{
  if (!UartTxBuf_GetState(com, BUFFER_TX))
	return 0;
  return UartBuf_Cnt(&UartTxRxbuf[BUFFER_TX][com]);
}