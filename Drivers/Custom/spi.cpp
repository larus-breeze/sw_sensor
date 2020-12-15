/*
 * spi.c
 *
 *  Created on: 24.11.2020
 *      Author: mbetz
 */

#include "spi.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"

#define SPI_DEFAULT_TIMEOUT_MS  100

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
COMMON  static TaskHandle_t SPI1_task_Id = NULL;
COMMON  static TaskHandle_t SPI2_task_Id = NULL;

static inline void SPI_sync(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
		SPI1_task_Id = xTaskGetCurrentTaskHandle();
	else
		SPI2_task_Id = xTaskGetCurrentTaskHandle();
	uint32_t pulNotificationValue;
	BaseType_t result = xTaskNotifyWait( 0xffffffff, 0, &pulNotificationValue, SPI_DEFAULT_TIMEOUT_MS);
	ASSERT( result == pdTRUE);
}

void SPI_Transceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, Size );
	ASSERT(HAL_OK == status);
	SPI_sync(hspi);
}

void SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_Transmit_DMA(hspi, pTxData, Size);
	ASSERT(HAL_OK == status);
	SPI_sync(hspi);
}


void SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_Receive_DMA(hspi, pRxData, Size);
	ASSERT(HAL_OK == status);
	SPI_sync(hspi);
}


void HAL_SPI_CpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t HigherPriorityTaskWoken=0;

	if (hspi->Instance == SPI1)
	{
		ASSERT( SPI1_task_Id);
		vTaskNotifyGiveFromISR( SPI1_task_Id, &HigherPriorityTaskWoken);
	}
	else if (hspi->Instance == SPI2)
	{
		ASSERT( SPI2_task_Id);
		vTaskNotifyGiveFromISR( SPI2_task_Id, &HigherPriorityTaskWoken);
	}
	else
	{
		ASSERT(0);
	}
	portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_CpltCallback( hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_CpltCallback( hspi);
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_CpltCallback( hspi);
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);
}


void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);
}
