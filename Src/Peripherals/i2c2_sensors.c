/****************************************************************
*                                                              *
*    Copyright (c) Linker Chip Corp. All rights reserved.      *
*    														    													*
*    Created by Anakin																					*
*                                                      				*
****************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "i2c2_sensors.h"
#include <string.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//HAL_I2C_Mem_Read(&hi2c2, SlaveAddr, ReadAddr, 1, ReadBuf, NumByte, SENSOR_FLAG_TIMEOUT);
#define I2C_ReadReg(SlaveAddr, ReadAddr, ReadBuf, NumByte) do{\
while(HAL_I2C_Mem_Read_DMA(&hi2c2, SlaveAddr, ReadAddr, 1, ReadBuf, NumByte) != HAL_OK);\
  while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);\
  }while(0)
/* Private variables ---------------------------------------------------------*/
uint8_t xBuffer[6];
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
* @brief  Initialise the LSM330DLC and LIS3MDL.
* @param  None
* @retval ERROR(0) if operation is NOT correctly performed, else return value SUCCESS
*					different from ERROR(0).
*/

HAL_StatusTypeDef Sensor_Init( void )
{
  uint8_t i = 0;
  uint8_t ReadID = 0;
  HAL_StatusTypeDef TransResult = HAL_OK;
  uint8_t LSM330DLC_G_InitData[5][2] =
  {
	{0x00, CTRL_REG2_G},
	{0x00, CTRL_REG3_G},	// Disable Interrupt
	{0x20, CTRL_REG4_G},	// FS: +-500dps, BlockData_Update: Continous, Endianness: Little Endian
	{0x08, CTRL_REG5_G},	// LPF1 -> LPF2 -> DataReg
	{0xCF, CTRL_REG1_G},	// ODR: 100Hz, BW: 30Hz, PowerMode: Normal, Axes: X/Y/Z Enable
  };
  uint8_t LSM330DLC_A_InitData[6][2] =
  {
	{0x00, CTRL_REG2_A},	// High pass filter disable
	{0x00, CTRL_REG3_A},	// Disable Interrupt
	{0x00, CTRL_REG4_A},	// FS: +-2g, High_Resolution: Enable, Endianness: Little Endian
	{0x00, CTRL_REG5_A},	// FIFO Mode: Disable
	{0x00, CTRL_REG6_A},
	{0x77, CTRL_REG1_A},	// ODR: 400Hz, PowerMode: Normal, Axes: X/Y/Z Enable
  };

#ifdef USE_MAGNETOMETER
  uint8_t LIS3MDL_InitData[5][2] =
  {
	{0x00, CTRL_REG2_M},	// FS: +-4gauss
	{0x00, CTRL_REG3_M},	// PowerMode: Normal, OperatingMode: Continuous-Conversion
	{0x00, CTRL_REG4_M},	// Endianness: Little Endian
	{0x00, CTRL_REG5_M},	// BlockData_Update: Continous
	{0x7C, CTRL_REG1_M},	// ODR: 80Hz, OperativeMode: Ultra-high Performance
  };
#endif

  /* Vertify LSM330DLC */
  HAL_I2C_Mem_Read(&hi2c2, ADDR_LSM330DLC_G, WHO_AM_I_G, 1, &ReadID, 1, SENSOR_FLAG_TIMEOUT);
  if(ReadID != LSM330DLC_Device_ID){
	printf("LSM330DLC Init Failed!\r\n");
	return HAL_ERROR;
  }
  else{
	printf("LSM330DLC Init OK!\r\n");
  }

#ifdef USE_MAGNETOMETER
  /* Vertify LIS3MDL */
  HAL_I2C_Mem_Read(&hi2c2, ADDR_LIS3MDL, WHO_AM_I_G, 1, &ReadID, 1, SENSOR_FLAG_TIMEOUT);
  if(ReadID != LIS3MDL_Device_ID){
	printf("LIS3MDL Init Failed!\r\n");
	return HAL_ERROR;
  }
  else{
	printf("LIS3MDL Init OK!\r\n");
  }
#endif

  /* Vertify LPS331AP */
  HAL_I2C_Mem_Read(&hi2c2, ADDR_LPS331AP, WHO_AM_I_G, 1, &ReadID, 1, SENSOR_FLAG_TIMEOUT);
  if(ReadID != LPS331AP_Device_ID){
	printf("LPS331AP Init Failed!\r\n");
	return HAL_ERROR;
  }
  else{
	printf("LPS331AP Init OK!\r\n");
  }

  /* Initialise the gyroscope */
  for(i=0; i<5; i++)
  {
	TransResult = HAL_I2C_Mem_Write(&hi2c2, ADDR_LSM330DLC_G, LSM330DLC_G_InitData[i][1], 1, LSM330DLC_G_InitData[i], 1, SENSOR_FLAG_TIMEOUT);
	if (TransResult != HAL_OK)	return TransResult;
  }

  /* Initialise the accelerometer */
  for(i=0; i<6; i++)
  {
	TransResult = HAL_I2C_Mem_Write(&hi2c2, ADDR_LSM330DLC_A, LSM330DLC_A_InitData[i][1], 1, LSM330DLC_A_InitData[i], 1, SENSOR_FLAG_TIMEOUT);
	if (TransResult != HAL_OK)	return TransResult;
  }

#ifdef USE_MAGNETOMETER
  /* Initialise the magnetometer */
  for(i=0; i<5; i++)
  {
	TransResult = HAL_I2C_Mem_Write(&hi2c2, ADDR_LIS3MDL, LIS3MDL_InitData[i][1], 1, LIS3MDL_InitData[i], 1, SENSOR_FLAG_TIMEOUT);
	if (TransResult != HAL_OK)	return TransResult;
  }
#endif

  return HAL_OK;
}

/**
* @brief  Calculate the angular Data rate Gyroscope.
* @param  pfData : Pointer to the data out.
* @retval None
*/

void LSM330DLC_GyroReadAngRate (float* pfData)
{
  uint8_t i = 0;
  int16_t RawData[3] = {0};
  float sensitivity = LSM330DLC_Gyr_Sensitivity_500dps;
  memset(xBuffer, 0, 6);
  I2C_ReadReg(ADDR_LSM330DLC_G, OUT_X_L_G, xBuffer, 6);

  for(i=0; i<3; i++)
  {
	RawData[i]=(int16_t)(((uint16_t)xBuffer[2*i+1] << 8) + xBuffer[2*i]);
  }

  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
	pfData[i]=(float)RawData[i] * sensitivity;
  }
}

/**
* @brief Read LSM330DLC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: Pointer to the data out.
* @retval None
*/

void LSM330DLC_AcceleroReadAcc(float* pfData)
{
  uint8_t i = 0;
  uint8_t cDivider = 16;
  int16_t RawData[3] = {0};
  float sensitivity = LSM330DLC_Acc_Sensitivity_2g;
  memset(xBuffer, 0, 6);
  I2C_ReadReg(ADDR_LSM330DLC_A, OUT_X_L_A, xBuffer, 6);

  for(i=0; i<3; i++)
  {
	RawData[i]=((int16_t)((uint16_t)xBuffer[2*i+1] << 8) + xBuffer[2*i])/cDivider;
  }

  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
	pfData[i]=(float)RawData[i] * sensitivity;
  }
}

/**
* @brief  Calculate the magnetic field Magn.
* @param  pfData : Pointer to the data out.
* @retval None
*/

void LIS3MDL_CompassReadMag (float* pfData)
{
  uint8_t i = 0;
  int16_t RawData[3] = {0};
  float sensitivity = LIS3MDL_Mag_Sensitivity_4guass;
  memset(xBuffer, 0, 6);
  I2C_ReadReg(ADDR_LIS3MDL, OUT_X_L_M, xBuffer, 6);

  for(i=0; i<3; i++)
  {
	RawData[i]=(int16_t)(((uint16_t)xBuffer[2*i+1] << 8) + xBuffer[2*i]);
  }

  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
	pfData[i]=(float)RawData[i] / sensitivity;
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
//  printf("%s\n\r", __func__);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
//  printf("%s\n\r", __func__);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  printf("%s\n\r", __func__);
}