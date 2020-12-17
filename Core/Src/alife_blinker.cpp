#include "main.h"
#include "FreeRTOS_wrapper.h"

void heartbeat(void)
{
	bool set = GPIO_PIN_SET == HAL_GPIO_ReadPin( LED_STATUS1_GPIO_Port, LED_STATUS2_Pin);
	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, set ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, set ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

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


