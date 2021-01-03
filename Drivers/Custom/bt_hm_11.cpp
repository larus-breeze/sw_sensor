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

	char wakeupquery[] = "#WAKEMEUP#";
	uint8_t lenwakeup = strlen(wakeupquery);
	for(int i = 0; i<10; i++)
	{
		UART6_Transmit((uint8_t *)wakeupquery, lenwakeup);
	}
	delay(50);

	char query[] = "AT+ADDR?";
	uint8_t len = strlen(query);
	UART6_Transmit((uint8_t *)query, len);

	uint8_t rxData;
	for(;;)
	{
		if(UART6_Receive(&rxData, 1) == true)
		{
			ITM_SendChar(rxData);
		}
	}
}







RestrictedTask bluetooth_handling(bluetooth_hm_11, "BT_HM_11", 256);
