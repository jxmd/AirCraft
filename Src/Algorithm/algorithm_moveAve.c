/****************************************************************
*                                                               *
*    Copyright (c) Linker Chip Corp. All rights reserved.       *
*    														                                *
*    Created by Anakin											                    *
*                                                      		      *
****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "algorithm_moveAve.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Simple Moving Average.
  *
  * @param  NewData: 			The new data which would enter the FIFO.
  * @param  MoveAve_FIFO: Pointer to the FIFO.
  * @param  SampleNum:   	pointer to the buffer containing the data to be received from 
  *         					 		the sensor.
  *
  * @retval The average of the datas in FIFO.
  *					
  */

float MoveAve_SMA( float NewData, float *MoveAve_FIFO, uint8_t SampleNum )
{
  uint8_t i = 0;
  float AveData = 0;
  float MoveAve_Sum = 0;

  for(i=0; i<SampleNum-1; i++)              
    MoveAve_FIFO[i] = MoveAve_FIFO[i+1];
  MoveAve_FIFO[SampleNum-1] = NewData;      
  for(i=0; i<SampleNum; i++)                
    MoveAve_Sum += MoveAve_FIFO[i];
  AveData = MoveAve_Sum / (float)SampleNum;  

  return AveData;
}



/**
  * @brief  Weighted Moving Average.
  *
  * @param  NewData: 			The new data which would enter the FIFO.
  * @param  MoveAve_FIFO: Pointer to the FIFO.
  * @param  SampleNum:   	pointer to the buffer containing the data to be received from 
  *         					 		the sensor.
  *
  * @retval The weighted average of the datas in FIFO, 
  *					which may indicate the estimated data this time.
  */

float MoveAve_WMA( float NewData, float *MoveAve_FIFO, uint8_t SampleNum )
{
  uint8_t i = 0;
  float AveData = 0;
  float SampleSum = 0;
  float MoveAve_Sum = 0;

  for(i=0; i<SampleNum-1; i++)                
    MoveAve_FIFO[i] = MoveAve_FIFO[i+1];
  MoveAve_FIFO[SampleNum-1] = NewData;        
  for(i=0; i<SampleNum; i++)                  
    MoveAve_Sum += MoveAve_FIFO[i]*(i+1);
  SampleSum = (SampleNum*(SampleNum+1))/2;    
  AveData = MoveAve_Sum / (float)SampleSum;     

  return AveData;
}














