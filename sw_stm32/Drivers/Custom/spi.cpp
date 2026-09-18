/**
 @file 		spi.c
 @brief 	Basic SPI driver
 @author: 	Maximilian Betz
 @copyright 	Copyright 2021 Maximilian Betz. All rights reserved.
 @license 	This project is released under the GNU Public License GPL-3.0

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

void register_SPI_usertask(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
		SPI1_task_Id = xTaskGetCurrentTaskHandle();
	else
		SPI2_task_Id = xTaskGetCurrentTaskHandle();
}

void notify_SPI2_task_from_ISR(uint32_t notification_value)
{
	if (SPI2_task_Id != 0)
	{
		BaseType_t xYieldRequired = pdFALSE;
		xTaskNotifyFromISR( SPI2_task_Id, notification_value, eSetValueWithOverwrite, &xYieldRequired);
		portEND_SWITCHING_ISR(xYieldRequired);
	}
}
static inline void SPI_sync(SPI_HandleTypeDef *hspi)
{
	// A DMA half-transfer callback also fires (HAL_SPI_RxHalfCpltCallback(),
	// notify value 0) even for a one-shot receive - only value 1
	// (HAL_SPI_CpltCallback()) is the real completion. Without filtering,
	// this used to return on the half-transfer, reading a half-DMA'd,
	// effectively garbage buffer.
	uint32_t pulNotificationValue;
	do
	{
		BaseType_t result = xTaskNotifyWait( 0xffffffff, 0, &pulNotificationValue, SPI_DEFAULT_TIMEOUT_MS);
		ASSERT( result == pdTRUE);
	}
	while (pulNotificationValue == 0);
}

void SPI_Transceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	register_SPI_usertask( hspi);
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, Size );
	ASSERT(HAL_OK == status);
	SPI_sync(hspi);
}

void SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size, uint32_t)
{
	register_SPI_usertask( hspi);
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_Transmit_DMA(hspi, pTxData, Size);
	ASSERT(HAL_OK == status);
	SPI_sync(hspi);
}


void SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size, uint32_t)
{
	register_SPI_usertask( hspi);
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_Receive_DMA(hspi, pRxData, Size);
	ASSERT(HAL_OK == status);
	SPI_sync(hspi);
}

bool SPI_Receive_Timeout(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size, uint32_t timeout_ms)
{
	register_SPI_usertask( hspi);
	HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(hspi, pRxData, Size);
	ASSERT(HAL_OK == status);

	// see SPI_sync()'s comment: a notification value of 0 is only the DMA
	// half-transfer notice, not the real completion - keep waiting for a
	// non-zero value, still bounded by timeout_ms per wait.
	uint32_t pulNotificationValue;
	for (;;)
	{
		BaseType_t result = xTaskNotifyWait( 0xffffffff, 0, &pulNotificationValue, pdMS_TO_TICKS(timeout_ms));
		if( result != pdTRUE)
		{
			HAL_SPI_Abort(hspi); // cancel the pending DMA request so the next call starts clean
			return false;
		}
		if (pulNotificationValue != 0)
			return true;
	}
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
		ASSERT( SPI2_task_Id !=0);

		BaseType_t xYieldRequired = pdFALSE;;
		xTaskNotifyFromISR( SPI2_task_Id, WLAN_LINK_SPI2_NOTIFY_FULL_COMPLETE, eSetValueWithOverwrite, &xYieldRequired);
		portEND_SWITCHING_ISR(xYieldRequired);
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

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance != SPI2)
	  return;

	BaseType_t xYieldRequired = pdFALSE;;
	ASSERT( SPI2_task_Id !=0);
	xTaskNotifyFromISR( SPI2_task_Id, WLAN_LINK_SPI2_NOTIFY_HALF_COMPLETE, eSetValueWithOverwrite, &xYieldRequired);
	portEND_SWITCHING_ISR(xYieldRequired);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_CpltCallback( hspi);
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	// SPI2 (WLAN link) hardware errors recover instead of the unconditional
	// ASSERT(0) other SPI instances get: notify the waiting arm_*() call
	// with a distinct sentinel (WLAN_LINK_SPI2_NOTIFY_ERROR, spi.h) so it
	// retries via the same path as a timeout, matching the WLAN link's own
	// recover-and-retry design. SPI1 (IMU) stays fatal. See
	// documentation/wlan_link.md.
	if (hspi->Instance == SPI2)
	{
		if (SPI2_task_Id != 0)
		{
			BaseType_t xYieldRequired = pdFALSE;
			xTaskNotifyFromISR( SPI2_task_Id, WLAN_LINK_SPI2_NOTIFY_ERROR, eSetValueWithOverwrite, &xYieldRequired);
			portEND_SWITCHING_ISR(xYieldRequired);
		}
		return;
	}

	ASSERT(0);
}


void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);
}
