/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @date    10/04/2015 16:29:23
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "leds.h"
#include "usart.h"
#include "usart_pingpong.h"
#include "my_free_rtos.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern void xPortSysTickHandler(void);
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    xPortSysTickHandler();
  }
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel3 global interrupt.
*/
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXT line 28.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  /* USER CODE END USART3_IRQn 0 */
  //HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  if((__HAL_UART_GET_IT(&huart3, UART_IT_IDLE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE) != RESET)){//IDLE
	  LED_RedOn();
	  __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_IDLEF);
	  __HAL_UART_DISABLE_IT(&huart3, UART_IT_IDLE);
	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	  /* 设置新数据的长度 */
	  USART3Buf.RecBufValLen[USART3Buf.RecBufCurrent] = USART_BufferLength -
		hdma_usart3_rx.Instance->CNDTR;

	  /* 指向下一个通道号 */
	  MOVE_CURRENT_BUFFER_TO_NEXT_CHANNEL(USART3Buf);

	  /* 停止DMA传输 */
	  HAL_UART_DMAStop(&huart3);

	  __HAL_UART_ENABLE(&huart3);
	  while(HAL_UART_Receive_DMA(&huart3,
								 USART3Buf.RecBuf[USART3Buf.RecBufCurrent],
								 USART_BufferLength) != HAL_OK);
	  BLERecvOK();
	  LED_RedOff();
  }
  //busy
  else{
	if(__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE) != RESET){
	  __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
	  __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_IDLEF);
	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	}
  }

  /* UART parity error interrupt occurred -------------------------------------*/
  if((__HAL_UART_GET_IT(&huart3, UART_IT_PE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_PE) != RESET))
  {
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_PEF);
  }

  /* UART frame error interrupt occured --------------------------------------*/
  if((__HAL_UART_GET_IT(&huart3, UART_IT_FE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_ERR) != RESET))
  {
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_FEF);
  }

  /* UART noise error interrupt occured --------------------------------------*/
  if((__HAL_UART_GET_IT(&huart3, UART_IT_NE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_ERR) != RESET))
  {
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_NEF);
  }

  /* UART Over-Run interrupt occured -----------------------------------------*/
  if((__HAL_UART_GET_IT(&huart3, UART_IT_ORE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_ERR) != RESET))
  {
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_OREF);
  }
  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel2 global interrupt.
*/
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXT line 25.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
