/****************************************************************
 *                                                              *
 *    Copyright (c) Linker Chip Corp. All rights reserved.      *
 *    														    													*
 *    Created by Anakin																					*
 *                                                      				*
 ****************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __ALGORITHM_MOVEAVE_H
#define __ALGORITHM_MOVEAVE_H


/* Includes ------------------------------------------------------------------*/

#include <stdint.h>


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

float MoveAve_SMA( float NewData, float *MoveAve_FIFO, uint8_t SampleNum );
float MoveAve_WMA( float NewData, float *MoveAve_FIFO, uint8_t SampleNum );













#endif

