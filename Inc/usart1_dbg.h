/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __USART1_DBG_H
#define __USART1_DBG_H

/* Includes ------------------------------------------------------------------*/
#include "usart_buffer.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
/* Exported types ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void USART_DBG_Init( void );
void USART_DBG_Send(uint8_t ch);
void USART_DBG_StartSend(void);
#endif