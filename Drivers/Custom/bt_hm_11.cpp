#include "FreeRTOS_wrapper.h"
#include "main.h"
#include "uart6.h"
#include "stdio.h"


static char altitude[]  = "$PGRMZ,246,f,3*1B\r\n";

void bluetooth_hm_11(void*)
{
	acquire_privileges();
	UART6_Init();

	// Reset HM 11. Pull low for 100ms to reset
	HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_RESET);
	delay(100);
	HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_SET);
	delay(200);

	uint8_t len  = strlen(altitude);
	uint8_t rxData = 0;
	for(;;)
	{
		UART6_Transmit((uint8_t *)altitude, len);

		delay(1000);

		while(UART6_Receive(&rxData) == true)
		{
			ITM_SendChar(rxData);
		}
	}
}







RestrictedTask bluetooth_handling(bluetooth_hm_11, "BT_HM_11", 256);
