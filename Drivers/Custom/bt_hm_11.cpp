#include "FreeRTOS_wrapper.h"
#include "main.h"
#include "uart6.h"
#include "stdio.h"

void bluetooth_hm_11(void*)
{
	// Reset HM 11. Pull low for 100ms to reset
	HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_RESET);
	delay(100);
	HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_SET);
	delay(200);

	char altitude[] = "$PGRMZ,246,f,3*1B\r\n";
	uint8_t len  = strlen(altitude);
	for(;;)
	{
		UART6_Transmit((uint8_t *)altitude, len);
		delay(1000);

		//if(UART6_Receive(&rxData, 1) == true)
		//{
		//	ITM_SendChar(rxData);
		//}
	}
}







RestrictedTask bluetooth_handling(bluetooth_hm_11, "BT_HM_11", 256);
