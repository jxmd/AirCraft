/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __LEDS_H
#define __LEDS_H

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include <stdio.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void LED_RedToggle( void );

/**
* @brief  Set RED LED On.
* @param  None
* @retval None
*/
void LED_RedOn( void );

/**
* @brief  Set RED LED Off.
* @param  None
* @retval None
*/
void LED_RedOff( void );

void LED_GreenToggle( void );

/**
* @brief  Set GREEN LED On.
* @param  None
* @retval None
*/
void LED_GreenOn( void );

/**
* @brief  Set GREEN LED Off.
* @param  None
* @retval None
*/
void LED_GreenOff( void );

void LED_BlueToggle( void );
/**
* @brief  Set BLUE LED On.
* @param  None
* @retval None
*/
void LED_BlueOn( void );
/**
* @brief  Set BLUE LED Off.
* @param  None
* @retval None
*/
void LED_BlueOff( void );
#endif