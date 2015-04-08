/****************************************************************
*                                                              *
*    Created by Xuejilong					*
*                                                      	*
****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "leds.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LEDS_RED_PIN				GPIO_PIN_4
#define LEDS_RED_GPIO_PORT			GPIOA

#define LEDS_GREEN_PIN				GPIO_PIN_5
#define LEDS_GREEN_GPIO_PORT			GPIOA

#define LEDS_BLUE_PIN				GPIO_PIN_6
#define LEDS_BLUE_GPIO_PORT			GPIOA

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void LED_RedToggle( void )
{
  HAL_GPIO_TogglePin(LEDS_RED_GPIO_PORT, LEDS_RED_PIN);
}

/**
* @brief  Set RED LED On.
* @param  None
* @retval None
*/
void LED_RedOn( void )
{
  HAL_GPIO_WritePin(LEDS_RED_GPIO_PORT, LEDS_RED_PIN, GPIO_PIN_SET);
}

/**
* @brief  Set RED LED Off.
* @param  None
* @retval None
*/
void LED_RedOff( void )
{
  HAL_GPIO_WritePin(LEDS_RED_GPIO_PORT, LEDS_RED_PIN, GPIO_PIN_RESET);
}

void LED_GreenToggle( void )
{
  HAL_GPIO_TogglePin(LEDS_GREEN_GPIO_PORT, LEDS_GREEN_PIN);
}

/**
* @brief  Set GREEN LED On.
* @param  None
* @retval None
*/
void LED_GreenOn( void )
{
  HAL_GPIO_WritePin(LEDS_GREEN_GPIO_PORT, LEDS_GREEN_PIN, GPIO_PIN_SET);
}

/**
* @brief  Set GREEN LED Off.
* @param  None
* @retval None
*/
void LED_GreenOff( void )
{
  HAL_GPIO_WritePin(LEDS_GREEN_GPIO_PORT, LEDS_GREEN_PIN, GPIO_PIN_RESET);
}

void LED_BlueToggle( void )
{
  HAL_GPIO_TogglePin(LEDS_BLUE_GPIO_PORT, LEDS_BLUE_PIN);
}

/**
* @brief  Set BLUE LED On.
* @param  None
* @retval None
*/
void LED_BlueOn( void )
{
  HAL_GPIO_WritePin(LEDS_BLUE_GPIO_PORT, LEDS_BLUE_PIN, GPIO_PIN_SET);
}

/**
* @brief  Set BLUE LED Off.
* @param  None
* @retval None
*/
void LED_BlueOff( void )
{
  HAL_GPIO_WritePin(LEDS_BLUE_GPIO_PORT, LEDS_BLUE_PIN, GPIO_PIN_RESET);
}