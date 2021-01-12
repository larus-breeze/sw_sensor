/*
 * spi.c
 *
 *  Created on: 24.11.2020
 *      Author: mbetz
 */

#include "spi.h"
#include "main.h"
#include "my_assert.h"
#include "FreeRTOS.h"
#include "queue.h"
#define SPI_DEFAULT_TIMEOUT_MS  100

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
static QueueHandle_t SPI1_CPL_Message_Id = NULL;
static QueueHandle_t SPI2_CPL_Message_Id = NULL;


void SPI_Init(SPI_HandleTypeDef *hspi)
{
	if (SPI1_CPL_Message_Id == NULL)
	{
		SPI1_CPL_Message_Id =  xQueueCreate(1,0);
	}
	if (SPI2_CPL_Message_Id == NULL)
	{
		SPI2_CPL_Message_Id =  xQueueCreate(1,0);
	}
}


void SPI_Transceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status = pdFALSE;
	//status = HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, 100 );
	//ASSERT(HAL_OK == status);
	//return;


	status = HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, Size );
	ASSERT(HAL_OK == status);
	if (hspi->Instance == SPI1)
	{
		queue_status = xQueueReceive(SPI1_CPL_Message_Id, 0, SPI_DEFAULT_TIMEOUT_MS);
	}
	else if (hspi->Instance == SPI2)
	{
		queue_status = xQueueReceive(SPI2_CPL_Message_Id, 0, SPI_DEFAULT_TIMEOUT_MS);
	}
	else
	{
		ASSERT(0);
	}

	ASSERT(pdTRUE == queue_status);
}


void SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status = pdFALSE;
	//status = HAL_SPI_Transmit(hspi, pTxData, Size, 100); //Test
	//ASSERT(HAL_OK == status);
	//return;

	status = HAL_SPI_Transmit_DMA(hspi, pTxData, Size);
	ASSERT(HAL_OK == status);
	if (hspi->Instance == SPI1)
	{
		queue_status = xQueueReceive(SPI1_CPL_Message_Id, 0, SPI_DEFAULT_TIMEOUT_MS);
	}
	else if (hspi->Instance == SPI2)
	{
		queue_status = xQueueReceive(SPI2_CPL_Message_Id, 0, SPI_DEFAULT_TIMEOUT_MS);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
}


void SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	BaseType_t queue_status = pdFALSE;
	//status = HAL_SPI_Receive(hspi, pRxData, Size, 100);
	//ASSERT(HAL_OK == status);
	//return;

	status = HAL_SPI_Receive_DMA(hspi, pRxData, Size);
	ASSERT(HAL_OK == status);
	if (hspi->Instance == SPI1)
	{
		queue_status = xQueueReceive(SPI1_CPL_Message_Id, 0, SPI_DEFAULT_TIMEOUT_MS);
	}
	else if (hspi->Instance == SPI2)
	{
		queue_status = xQueueReceive(SPI2_CPL_Message_Id, 0, SPI_DEFAULT_TIMEOUT_MS);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (hspi->Instance == SPI1)
	{
		queue_status = xQueueSendFromISR(SPI1_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else if (hspi->Instance == SPI2)
	{
		queue_status = xQueueSendFromISR(SPI2_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (hspi->Instance == SPI1)
	{
		queue_status = xQueueSendFromISR(SPI1_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else if (hspi->Instance == SPI2)
	{
		queue_status = xQueueSendFromISR(SPI2_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
	BaseType_t queue_status;

	if (hspi->Instance == SPI1)
	{
		queue_status = xQueueSendFromISR(SPI1_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else if (hspi->Instance == SPI2)
	{
		queue_status = xQueueSendFromISR(SPI2_CPL_Message_Id, 0, &xHigherPriorityTaskWokenByPost);
	}
	else
	{
		ASSERT(0);
	}
	ASSERT(pdTRUE == queue_status);
	portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);
}


void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);
}
