#include "uart6.h"
#include "my_assert.h"

extern UART_HandleTypeDef huart6;
#define UART6_DEFAULT_TIMEOUT 100

void UART6_Init(void)
{

}

void UART6_Transmit(uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_UART_Transmit(&huart6, pData, Size, UART6_DEFAULT_TIMEOUT);
	ASSERT(HAL_OK == status);
}

bool UART6_Receive(uint8_t *pData, uint8_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_UART_Receive(&huart6, pData, Size, UART6_DEFAULT_TIMEOUT);
	if (HAL_OK == status)
	{
		return true;
	}
	else
	{
		return false;
	}
}
