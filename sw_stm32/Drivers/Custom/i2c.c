/** *****************************************************************************
 * @file    	i2c.c
 * @brief   	Basic I2C driver
 * @author  	Maximilian Betz
 * @copyright 	Copyright 2021 Maximilian Betz. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/
#include "i2c.h"
#include "main.h"
#include "my_assert.h"
#include "FreeRTOS_wrapper.h"
#include "queue.h"
#define I2C_DEFAULT_TIMEOUT_MS  100

static COMMON QueueHandle_t I2C1_CPL_Message_Id = NULL;
static COMMON QueueHandle_t I2C2_CPL_Message_Id = NULL;

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/*USER CODE END I2C2_Init 1*/
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

static void I2C1_ResolveStuckSlave(void)
{
	/* Reset a slave which potentially blocks the bus by pulling the data line to low. Very sporadic issue.*/
	HAL_I2C_DeInit(&hi2c1);

	/* Configure SDA to input to monitor success*/
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = I2C1_SDA_GPIOPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(I2C1_SDA_GPIOX, &GPIO_InitStruct);

	/* Configure SCL to output to generate a pseudo clock signal */
	GPIO_InitStruct.Pin = I2C1_SCL_GPIOPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(I2C1_SCL_GPIOX, &GPIO_InitStruct);

	/* Generate clock pulses until slave releases the data line */
	while(HAL_GPIO_ReadPin(I2C1_SDA_GPIOX, I2C1_SDA_GPIOPIN) == GPIO_PIN_RESET)
	{
		HAL_GPIO_TogglePin(I2C1_SCL_GPIOX, I2C1_SCL_GPIOPIN);
		vTaskDelay(1);
	}

	/* Initialize I2C for normal operation.*/
	MX_I2C1_Init();
}

static void I2C2_ResolveStuckSlave(void)
{
	/* Reset a slave which potentially blocks the bus by pulling the data line to low. Very sporadic issue.*/
	HAL_I2C_DeInit(&hi2c2);

	/* Configure SDA to input to monitor success*/
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = I2C2_SDA_GPIOPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(I2C2_SDA_GPIOX, &GPIO_InitStruct);

	/* Configure SCL to output to generate a pseudo clock signal */
	GPIO_InitStruct.Pin = I2C2_SCL_GPIOPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(I2C2_SCL_GPIOX, &GPIO_InitStruct);

	/* Generate clock pulses until slave releases the data line */
	while(HAL_GPIO_ReadPin(I2C2_SDA_GPIOX, I2C2_SDA_GPIOPIN) == GPIO_PIN_RESET)
	{
		HAL_GPIO_TogglePin(I2C2_SCL_GPIOX, I2C2_SCL_GPIOPIN);
		vTaskDelay(1);
	}

	/* Initialize I2C for normal operation.*/
	MX_I2C2_Init();
}

I2C_StatusTypeDef I2C_Init(I2C_HandleTypeDef *hi2c)
{
	I2C_StatusTypeDef status = I2C_OK;
	if (hi2c->Instance == I2C1)
	{
		if (I2C1_CPL_Message_Id == NULL)
		{
			I2C1_CPL_Message_Id =  xQueueCreate(1, sizeof(I2C_StatusTypeDef));
			I2C1_ResolveStuckSlave();
			if (NULL == I2C1_CPL_Message_Id)
			{
				status = I2C_ERROR;
			}
		}
		else
		{
			status = I2C_OK; //Already initialized
		}

	}
	else if (hi2c->Instance == I2C2)
	{
		if (I2C2_CPL_Message_Id == NULL)
		{
			I2C2_CPL_Message_Id =  xQueueCreate(1, sizeof(I2C_StatusTypeDef));
			I2C2_ResolveStuckSlave();
			if (NULL == I2C2_CPL_Message_Id)
			{
				status = I2C_ERROR;
			}
		}
		else
		{
			status = I2C_OK; //Already initialized
		}

	}
	else
	{
		status = I2C_ERROR;
		ASSERT(0);
	}

	return status;
}


I2C_StatusTypeDef I2C_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status = pdFALSE;
	I2C_StatusTypeDef flag = I2C_ERROR;
	status = HAL_I2C_Master_Receive_IT(hi2c, DevAddress, pData, Size);
	if(HAL_OK != status)
	{
		return I2C_ERROR;  // Return false in case HAL did not accept the command.
	}
	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueReceive(I2C1_CPL_Message_Id, &flag, I2C_DEFAULT_TIMEOUT_MS);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueReceive(I2C2_CPL_Message_Id, &flag, I2C_DEFAULT_TIMEOUT_MS);
	}

	if((I2C_OK != flag) | (pdTRUE != queue_status))
	{
		return I2C_ERROR;  // Return false in case queue timeout or false flag value.
	}
	return I2C_OK;
}


I2C_StatusTypeDef I2C_ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status = pdFALSE;
	I2C_StatusTypeDef flag = false;
	status = HAL_I2C_Mem_Read_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
	if(HAL_OK != status)
	{
		return I2C_ERROR;  // Return false in case HAL did not accept the command.
	}
	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueReceive(I2C1_CPL_Message_Id, &flag, I2C_DEFAULT_TIMEOUT_MS);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueReceive(I2C2_CPL_Message_Id, &flag, I2C_DEFAULT_TIMEOUT_MS);
	}

	if((I2C_OK != flag) | (pdTRUE != queue_status))
	{
		return I2C_ERROR;  // Return false in case queue timeout or false flag value.
	}
	return I2C_OK;
}


I2C_StatusTypeDef I2C_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status = pdFALSE;
	I2C_StatusTypeDef flag = false;
	status = HAL_I2C_Mem_Write_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
	if(HAL_OK != status)
	{
		return I2C_ERROR;  // Return false in case HAL did not accept the command.
	}
	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueReceive(I2C1_CPL_Message_Id, &flag, I2C_DEFAULT_TIMEOUT_MS);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueReceive(I2C2_CPL_Message_Id, &flag, I2C_DEFAULT_TIMEOUT_MS);
	}

	if((I2C_OK != flag) | (pdTRUE != queue_status))
	{
		return I2C_ERROR;  // Return false in case queue timeout or false flag value.
	}
	return I2C_OK;
}


I2C_StatusTypeDef I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status = pdFALSE;
	I2C_StatusTypeDef flag = I2C_ERROR;
	status = HAL_I2C_Master_Transmit_IT(hi2c, DevAddress, pData, Size);
	if(HAL_OK != status)
	{
		return I2C_ERROR;  // Return false in case HAL did not accept the command.
	}
	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueReceive(I2C1_CPL_Message_Id, &flag, I2C_DEFAULT_TIMEOUT_MS);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueReceive(I2C2_CPL_Message_Id, &flag, I2C_DEFAULT_TIMEOUT_MS);
	}

	if((I2C_OK != flag) | (pdTRUE != queue_status))
	{
		return I2C_ERROR;  // Return false in case queue timeout or false flag value.
	}
	return I2C_OK;
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;
	I2C_StatusTypeDef flag = I2C_OK;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
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
	I2C_StatusTypeDef flag = I2C_OK;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
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
	I2C_StatusTypeDef flag = I2C_OK;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
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
	I2C_StatusTypeDef flag = I2C_OK;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
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
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;
	I2C_StatusTypeDef flag = I2C_ERROR;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}


void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;
	I2C_StatusTypeDef flag = I2C_ERROR;

	if (hi2c->Instance == I2C1)
	{
		queue_status = xQueueSendFromISR(I2C1_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
	}
	else if (hi2c->Instance == I2C2)
	{
		queue_status = xQueueSendFromISR(I2C2_CPL_Message_Id, &flag, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}
