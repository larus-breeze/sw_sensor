#include "i2c.h"
#include "main.h"
#include "my_assert.h"
#include "FreeRTOS.h"
#include "queue.h"
#define I2C_DEFAULT_TIMEOUT_MS  100

static QueueHandle_t I2C1_CPL_Message_Id = NULL;
static QueueHandle_t I2C2_CPL_Message_Id = NULL;
volatile unsigned I2C_error_count;

void I2C_Init(void)
{
	if (I2C1_CPL_Message_Id == NULL)
	{
		I2C1_CPL_Message_Id =  xQueueCreate(1,0);
	}
	if (I2C2_CPL_Message_Id == NULL)
	{
		I2C2_CPL_Message_Id =  xQueueCreate(1,0);
	}

	/*  Resolve busy flag problem if it apears.
	__HAL_RCC_I2C1_FORCE_RESET();
	vTaskDelay(5);
	__HAL_RCC_I2C1_RELEASE_RESET();
  	*/
}



void I2C_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status;
	status = HAL_I2C_Master_Receive_IT(hi2c, DevAddress, pData, Size);
	ASSERT(HAL_OK == status);
	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueReceive(I2C1_CPL_Message_Id, 0, I2C_DEFAULT_TIMEOUT_MS);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueReceive(I2C2_CPL_Message_Id, 0, I2C_DEFAULT_TIMEOUT_MS);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
}


void I2C_ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status;
	status = HAL_I2C_Mem_Read_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
	ASSERT(HAL_OK == status);
	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueReceive(I2C1_CPL_Message_Id, 0, I2C_DEFAULT_TIMEOUT_MS);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueReceive(I2C2_CPL_Message_Id, 0, I2C_DEFAULT_TIMEOUT_MS);
	}
	else
	{
		ASSERT(0);
	}
//	ASSERT(pdTRUE == queue_status); patch
}


void I2C_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status;
	status = HAL_I2C_Mem_Write_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
	ASSERT(HAL_OK == status);
	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueReceive(I2C1_CPL_Message_Id, 0, I2C_DEFAULT_TIMEOUT_MS);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueReceive(I2C2_CPL_Message_Id, 0, I2C_DEFAULT_TIMEOUT_MS);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
}


void I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status;
	status = HAL_I2C_Master_Transmit_IT(hi2c, DevAddress, pData, Size);
	ASSERT(HAL_OK == status);
	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueReceive(I2C1_CPL_Message_Id, 0, I2C_DEFAULT_TIMEOUT_MS);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueReceive(I2C2_CPL_Message_Id, 0, I2C_DEFAULT_TIMEOUT_MS);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}


void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  ++I2C_error_count;
//  ASSERT(0);
}


void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
  ++I2C_error_count;
//  ASSERT(0);
}
