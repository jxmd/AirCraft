/****************************************************************
*                                                               *
*    Copyright (c) Linker Chip Corp. All rights reserved.       *
*    														                                *
*    Created by Anakin											                    *
*                                                      		      *
****************************************************************/

 
/* Includes ------------------------------------------------------------------*/
#include "algorithm_kalman.h"
#include "algorithm_matrix.h"






/* Globals -------------------------------------------------------------------*/

static float 	Qab[obs_num*obs_num];				//time-varying covariance of external acceleration
static float 	Wk[obs_num*obs_num];				//H*P*H'+R = H*G+R
static float 	Wk_inv[obs_num*obs_num];			//1/Wk
static float 	Fk[state_num*obs_num];				//D*U'*H'
static float 	Gk[state_num*obs_num];				//U*Fk
static float	Pk_Estimate[state_num*state_num];	//P(k/k-1)	Priori estimate
static float 	K[state_num*obs_num];				//Gk/Wk
static float 	U[state_num*state_num];
static float 	UT[state_num*state_num];
static float 	D[state_num*state_num];



/* Extern Globals ------------------------------------------------------------*/
/* Kalman Update Functions ---------------------------------------------------*/

void KalmanFilterUpdate(float* fa, float* h, float* p, float const* R,
							  float const* Q, float* zk, float* xerror)
{ 
  uint32_t 	i=0;
  float 	FW[state_num*obs_num]={0};
  float 	Fk_t[state_num*obs_num]={0};
  float 	FWFT[state_num*state_num]={0};
  float 	DSubFWFT[state_num*state_num]={0};
  float 	UDSubFWFT[state_num*state_num]={0};

  PrioriEstimate_Pk(fa, p, Q);
  PrioriEstimate_Wk(h, R);
  Covariance_ExternalDisturb(zk);
	
  /* HPH'+R+Qab */
  for(i=0;i<obs_num*obs_num;i++)
  {
  	Wk[i]=Wk[i]+Qab[i];
  }
  
  /* Wk = 1/Wk */
  for(i=0;i<obs_num*obs_num;i++)
  {
  	Wk_inv[i]=Wk[i];
  }
  MatrixInverse(Wk_inv,obs_num);
  
  /* K=G*W^-1 */
  MatrixMultiply(Gk,state_num,obs_num,Wk_inv,obs_num,obs_num,K);

  /* Update Pk */
  MatrixMultiply(Fk,state_num,obs_num,Wk_inv,obs_num,obs_num,FW);
  MatrixTranspose(Fk,state_num,obs_num,Fk_t);
  MatrixMultiply(FW,state_num,obs_num,Fk_t,obs_num,state_num,FWFT);
  MatrixSub(D,FWFT,DSubFWFT,state_num,state_num);
  MatrixMultiply(U,state_num,state_num,DSubFWFT,state_num,state_num,UDSubFWFT);
  MatrixMultiply(UDSubFWFT,state_num,state_num,UT,state_num,state_num,p);
  
  /* Update xerror */
  MatrixMultiply(K,state_num,obs_num,zk,obs_num,1,xerror);
 
}



static void PrioriEstimate_Pk(float* fa, float* p, float const* Q)
{
  uint32_t 	i;
  float 	fa_t[state_num*state_num]={0};
  float 	demp[state_num*state_num]={0};

  /* fa*P(k-1)*fa' */
  MatrixTranspose(fa,state_num,state_num,fa_t);
  MatrixMultiply(fa,state_num,state_num,p,state_num,state_num,demp);
  MatrixMultiply(demp,state_num,state_num,fa_t,state_num,state_num,Pk_Estimate);

  /* P(k/k-1) = fa*P(k-1)*fa' + Q */
  for(i=0;i<state_num*state_num;i++)
  {
	Pk_Estimate[i]=Pk_Estimate[i]+Q[i];
  }
  
}


static void PrioriEstimate_Wk(float* h, float const* R)
{
  uint32_t 	i;
  float 	h_t[state_num*obs_num]={0};
  float 	temp[state_num*state_num]={0};	//D*U'
  
  /* UD Factorization */
  UD(Pk_Estimate,state_num,U,D);
  
  /* Fk = D*U'*H' */
  MatrixTranspose(h,obs_num,state_num,h_t);
  MatrixTranspose(U,state_num,state_num,UT);
  MatrixMultiply(D,state_num,state_num,UT,state_num,state_num,temp);
  MatrixMultiply(temp,state_num,state_num,h_t,state_num,obs_num,Fk);
  
  /* Gk = U*Fk */
  MatrixMultiply(U,state_num,state_num,Fk,state_num,obs_num,Gk);
  
  /* H*G */
  MatrixMultiply(h,obs_num,state_num,Gk,state_num,obs_num,Wk);
  
  /* Wk = H*G + R */
  for(i=0;i<obs_num*obs_num;i++)
  {
	Wk[i]=Wk[i]+R[i];
  }

}






/* Shielded From Interference Functions ------------------------------------------------------------*/

#define		THRESHOLD			0.5

static void Covariance_ExternalDisturb(float* zk)
{
  uint32_t 	i;
  uint32_t 	flag = 0;
  float 	eigenvalues[obs_num]={0};					
  float 	eigenvector[obs_num*obs_num]={0};			
  float 	eigenvector_t[obs_num*obs_num]={0};		
  float 	lambda_matrix[obs_num*obs_num] = {0};			
  float 	mu_matrix[obs_num*obs_num] = {0};  
  float 	difference[obs_num*obs_num] = {0};
  float 	temp[obs_num*obs_num]={0};
  
  Qab[0]=zk[0]*zk[0];
  Qab[1]=zk[0]*zk[1];
  Qab[2]=zk[0]*zk[2];
  Qab[3]=zk[1]*zk[0];
  Qab[4]=zk[1]*zk[1];
  Qab[5]=zk[1]*zk[2];
  Qab[6]=zk[2]*zk[0];
  Qab[7]=zk[2]*zk[1];
  Qab[8]=zk[2]*zk[2];

  Jacobi_Cyclic_Method(eigenvalues,eigenvector,Qab,obs_num);

  MatrixTranspose(eigenvector,obs_num,obs_num,eigenvector_t);
  lambda_matrix[0]=eigenvalues[0];
  lambda_matrix[4]=eigenvalues[1];
  lambda_matrix[8]=eigenvalues[2];

  Get_Lambda(eigenvalues, eigenvector, mu_matrix);

  for(i=0;i<obs_num*obs_num;i+=obs_num)
  {
	difference[i]=lambda_matrix[i]-mu_matrix[i];
  }

  for(i=0;i<obs_num*obs_num;i+=obs_num)
  {
	if(difference[i] > THRESHOLD)
    {
	  flag = 1;
	  break;
	}
  }


  if(flag)
  {
	for(i=0;i<obs_num*obs_num;i++)
    {
  	  Qab[i]=0;
	}
  }
  else
  {
	for(i=0;i<obs_num*obs_num;i+=obs_num)
	{
	  if(difference[i]<0)
	  {
		difference[i]=0;
	  }
  }
	
	/* Qab=U*max(Lambda-Mu,0)*U' */
	MatrixMultiply(eigenvector,obs_num,obs_num,difference,obs_num,obs_num,temp);
	MatrixMultiply(temp,obs_num,obs_num,eigenvector_t,obs_num,obs_num,Qab);
  }
  
}


static void Get_Lambda(float* eigenvalues, float* eigenvector, float* mu_matrix)
{
  uint32_t 	i;
  float 	temp[obs_num]={0};
  float 	temp_eigenvector[obs_num]={0};									
  float 	temp_eigenvector_t[obs_num]={0};
  float 	mu1[1]={0};
  float 	mu2[1]={0};
  float 	mu3[1]={0};

  for(i=0;i<obs_num;i++)
  {
	temp_eigenvector[i]=eigenvector[obs_num*i];
  }
  MatrixTranspose(temp_eigenvector,obs_num,1,temp_eigenvector_t);
  MatrixMultiply(temp_eigenvector_t,1,obs_num,Wk,obs_num,obs_num,temp);
  MatrixMultiply(temp,1,obs_num,temp_eigenvector,obs_num,1,mu1);
  
  for(i=0;i<obs_num;i++)
  {
	temp_eigenvector[i]=eigenvector[obs_num*i+1];
  }
  MatrixTranspose(temp_eigenvector,obs_num,1,temp_eigenvector_t);
  MatrixMultiply(temp_eigenvector_t,1,obs_num,Wk,obs_num,obs_num,temp);
  MatrixMultiply(temp,1,obs_num,temp_eigenvector,obs_num,1,mu2);

  for(i=0;i<obs_num;i++)
  {
	temp_eigenvector[i]=eigenvector[obs_num*i+2];
  }  
  MatrixTranspose(temp_eigenvector,obs_num,1,temp_eigenvector_t);
  MatrixMultiply(temp_eigenvector_t,1,obs_num,Wk,obs_num,obs_num,temp);
  MatrixMultiply(temp,1,obs_num,temp_eigenvector,obs_num,1,mu3);
  
  mu_matrix[0]=mu1[0];
  mu_matrix[4]=mu2[0];
  mu_matrix[8]=mu3[0];

}





