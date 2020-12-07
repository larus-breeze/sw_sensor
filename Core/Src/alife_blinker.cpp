/*
 * alife_blinker.cpp
 *
 *  Created on: Dec 6, 2020
 *      Author: schaefer
 */
#include "main.h"
#include "FreeRTOS_wrapper.h"

void blink( void *)
{
	for( synchronous_timer t(500); true; )
	{
		HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS3_Pin, GPIO_PIN_RESET);
		t.sync();
		HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS3_Pin, GPIO_PIN_SET);
		t.sync();
	}
}

RestrictedTask alife_blinker( blink, "BLINK");


