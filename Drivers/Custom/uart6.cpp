#include "uart6.h"
#include "my_assert.h"
#include "FreeRTOS.h"
#include "queue.h"

#define UART6_DEFAULT_TIMEOUT 100
#define UART6_RX_QUEUE_SIZE 256

extern UART_HandleTypeDef huart6;
static QueueHandle_t UART6_CPL_Message_Id = NULL;
static QueueHandle_t UART6_Rx_Queue = NULL;



static uint8_t uart6_rx_byte = 0; //
void UART6_Init(void)
{
	if (UART6_CPL_Message_Id == NULL)
	{
		UART6_CPL_Message_Id =  xQueueCreate(1,0);
	}
	if (UART6_Rx_Queue == NULL)
	{
		UART6_Rx_Queue =  xQueueCreate(UART6_RX_QUEUE_SIZE, sizeof(uint8_t));
	}

	HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);
}

void UART6_Transmit(uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status = pdFALSE;

	status = HAL_UART_Transmit_DMA(&huart6, pData, Size);
	ASSERT(HAL_OK == status);
	queue_status = xQueueReceive(UART6_CPL_Message_Id, 0, UART6_DEFAULT_TIMEOUT);
	ASSERT(pdTRUE == queue_status);
}

bool UART6_Receive(uint8_t *pRxByte)
{
	BaseType_t queue_status = pdFALSE;

	queue_status = xQueueReceive(UART6_Rx_Queue, pRxByte, 0);
	if(pdTRUE == queue_status)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (huart->Instance == USART6)
	{
		queue_status = xQueueSendFromISR(UART6_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (huart->Instance == USART6)
	{
		/*Get Byte and enable interrupt again*/
		queue_status = xQueueSendFromISR(UART6_Rx_Queue, &uart6_rx_byte, &xHigherPriorityTaskWokenByPost);
		HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{

}


