/****************************************************************
 *                                                              *
 *    Copyright (c) Linker Chip Corp. All rights reserved.      *
 *    														    													*
 *    Created by Anakin																					*
 *                                                      				*
 ****************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __PERIPHERALS_I2C2_SENSORS_H
#define __PERIPHERALS_I2C2_SENSORS_H

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted.
 */
#define SENSOR_FLAG_TIMEOUT         ((uint32_t)100)
#define SENSOR_LONG_TIMEOUT         ((uint32_t)(10 * SENSOR_FLAG_TIMEOUT))

#define LSM330DLC_Gyr_Sensitivity_250dps         0.00875f        /* gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define LSM330DLC_Gyr_Sensitivity_500dps          0.0175f        /* gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define LSM330DLC_Gyr_Sensitivity_2000dps           0.07f      	 /* gyroscope sensitivity with 2000 dps full scale [LSB/dps] */

#define LSM330DLC_Acc_Sensitivity_2g     		       0.001f        /* accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM330DLC_Acc_Sensitivity_4g               0.002f        /* accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM330DLC_Acc_Sensitivity_8g               0.004f        /* accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM330DLC_Acc_Sensitivity_16g              0.012f        /* accelerometer sensitivity with 12 g full scale [LSB/mg] */

#define LIS3MDL_Mag_Sensitivity_4guass     	       6842.0f       /* magnetometer sensitivity with 4 guass full scale [guass/LSB] */
#define LIS3MDL_Mag_Sensitivity_8guass             3421.0f       /* magnetometer sensitivity with 8 guass full scale [guass/LSB] */
#define LIS3MDL_Mag_Sensitivity_12guass            2281.0f       /* magnetometer sensitivity with 12 guass full scale [guass/LSB] */
#define LIS3MDL_Mag_Sensitivity_16guass            1711.0f       /* magnetometer sensitivity with 16 guass full scale [guass/LSB] */

/************************** LSM330DLC Register **************************/

// address of ADDR_LSM330DLC_G: 11010100b
#define ADDR_LSM330DLC_G    0xD4

// address of ADDR_LSM330DLC_A: 00110000b
#define ADDR_LSM330DLC_A    0x30

// address of LIS3MDL: 00111000b
#define ADDR_LIS3MDL    		0x38

// address of LPS331AP: 10111000b
#define ADDR_LPS331AP    		0XB8

// LSM330DLC device ID: 11010100b
#define LSM330DLC_Device_ID	0xD4

// LIS3MDL device ID: 11010100b
#define LIS3MDL_Device_ID		0x3D

// LPS331AP device ID: 10111011b
#define LPS331AP_Device_ID	0xBB

//Gyroscope
#define WHO_AM_I_G            0x0F
#define CTRL_REG1_G           0x20
#define CTRL_REG2_G           0x21
#define CTRL_REG3_G           0x22
#define CTRL_REG4_G           0x23
#define CTRL_REG5_G           0x24
#define REFERENCE_G           0x25
#define OUT_TEMP_G            0x26
#define STATUS_REG_G          0x27
#define OUT_X_L_G             0x28
#define OUT_X_H_G             0x29
#define OUT_Y_L_G             0x2A
#define OUT_Y_H_G             0x2B
#define OUT_Z_L_G             0x2C
#define OUT_Z_H_G             0x2D
#define FIFO_CTRL_REG_G       0x2E
#define FIFO_SRC_REG_G        0x2F
#define INT1_CFG_G            0x30
#define INT1_SRC_G            0x31
#define INT1_TSH_XH_G         0x32
#define INT1_TSH_XL_G         0x33
#define INT1_TSH_YH_G         0x34
#define INT1_TSH_YL_G         0x35
#define INT1_TSH_ZH_G         0x36
#define INT1_TSH_ZL_G         0x37
#define INT1_DURATION_G       0x38

//Accelerometer
#define CTRL_REG1_A           0x20
#define CTRL_REG2_A           0x21
#define CTRL_REG3_A           0x22
#define CTRL_REG4_A           0x23
#define CTRL_REG5_A           0x24
#define CTRL_REG6_A           0x25
#define REFERENCE_A           0x26
#define STATUS_REG_A          0x27
#define OUT_X_L_A             0x28
#define OUT_X_H_A             0x29
#define OUT_Y_L_A             0x2A
#define OUT_Y_H_A             0x2B
#define OUT_Z_L_A             0x2C
#define OUT_Z_H_A             0x2D
#define FIFO_CTRL_REG_A       0x2E
#define FIFO_SRC_REG_A        0x2F
#define INT1_CFG_A            0x30
#define INT1_SOURCE_A         0x31
#define INT1_THS_A            0x32
#define INT1_DURATION_A       0x33
#define INT2_CFG_A            0x34
#define INT2_SOURCE_A         0x35
#define INT2_THS_A            0x36
#define INT2_DURATION_A       0x37
#define CLICK_CFG_A           0x38
#define CLICK_SRC_A			  		0x39
#define CLICK_THS_A			  		0x3A
#define TIME_LIMIT_A		  		0x3B
#define TIME_LATENCY_A		  	0x3C
#define TIME_WINDOW_A 		  	0x3D
#define Act_THS 		      		0x3E
#define Act_DUR 		      		0x3F

//Magnetometer
#define WHO_AM_I_M            0x0F
#define CTRL_REG1_M           0x20
#define CTRL_REG2_M           0x21
#define CTRL_REG3_M	          0x22
#define CTRL_REG4_M		        0x23
#define CTRL_REG5_M           0x24
#define STATUS_REG_M          0x27
#define OUT_X_L_M             0x28
#define OUT_X_H_M             0x29
#define OUT_Y_L_M             0x2A
#define OUT_Y_H_M             0x2B
#define OUT_Z_L_M             0x2C
#define OUT_Z_H_M             0x2D
#define TEMP_OUT_L_M					0x2E
#define TEMP_OUT_H_M					0x2F
#define INT_CFG_M							0x30
#define INT_SRC_M							0x31
#define INT_THS_L_M						0x32
#define INT_THS_H_M						0x33

//Barometer
#define WHO_AM_I_B            0x0F
#define REF_P_XL_B            0x08
#define REF_P_L_B             0x09
#define REF_P_H_B             0x0A
#define RES_CONF_B            0x10
#define CTRL_REG1_B           0x20
#define CTRL_REG2_B           0x21
#define CTRL_REG3_B           0x22
#define INT_CFG_REG_B         0x23
#define INT_SOURCE_REG_B      0x24
#define THS_P_LOW_REG_B       0x25
#define THS_P_HIGH_REG        0x26
#define STATUS_REG_B          0x27
#define PRESS_POUT_XL_REH_B   0x28
#define PRESS_OUT_L_B         0x29
#define PRESS_OUT_H_B         0x2A
#define TEMP_OUT_L_B          0x2B
#define TEMP_OUT_H_B          0x2C
#define AMP_CTRL_B            0x30

/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef Sensor_Init( void );
void LSM330DLC_GyroReadAngRate (float* pfData);
void LSM330DLC_AcceleroReadAcc(float* pfData);
void LIS3MDL_CompassReadMag (float* pfData);
#endif