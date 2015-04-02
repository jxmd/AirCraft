/****************************************************************
*                                                               *
*    Copyright (c) Linker Chip Corp. All rights reserved.       *
*    														                                *
*    Created by Anakin											                    *
*                                                      		      *
****************************************************************/
 
#ifndef __ALGORITHM_KALMAN_H
#define __ALGORITHM_KALMAN_H

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"


/* Exported macro ------------------------------------------------------------*/

#define  	state_num 			4
#define  	obs_num   			3



/* Typedefs ------------------------------------------------------------------*/



/* Exported functions ------------------------------------------------------- */

void KalmanFilterUpdate(float* fa, float* h, float* p, float const* R,	\
							  float const* Q, float* zk, float* xerror);

static void PrioriEstimate_Pk(float* fa, float* p, float const* Q);

static void PrioriEstimate_Wk(float* h, float const* R);

static void Covariance_ExternalDisturb(float* zk);

static void Get_Lambda(float* eigenvalues, float* eigenvector, float* mu_matrix);




















#endif