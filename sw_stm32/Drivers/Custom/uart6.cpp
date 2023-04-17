/** *****************************************************************************
 * @file    	uart6.cpp
 * @brief   	Driver for uart 6
 * @author  	Dr. Klaus Schaefer
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
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
#include "system_configuration.h"
#include "uart6.h"
#include "my_assert.h"
#include "FreeRTOS_wrapper.h"

#if ACTIVATE_BLUETOOTH_HM19

#define UART6_DEFAULT_TIMEOUT 250
#define UART6_RX_QUEUE_SIZE 64

static COMMON QueueHandle_t UART6_CPL_Message_Id = NULL;
static COMMON QueueHandle_t UART6_Rx_Queue = NULL;

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
  HAL_UART_Init(&huart6);
  HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1); // Activate interrupt for rx data
}

void UART6_DeInit(void)
{
  HAL_UART_Abort(&huart6);
  HAL_UART_DeInit(&huart6);
}

void UART6_ChangeBaudRate(uint32_t rate)
{
  HAL_UART_Abort(&huart6);
  HAL_UART_DeInit(&huart6);
  huart6.Init.BaudRate = rate;
  HAL_UART_Init(&huart6);

  HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1); // Activate interrupt for rx data
}

void UART6_Transmit(const uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef status = HAL_OK;
  BaseType_t queue_status = pdFALSE;

  status = HAL_UART_Transmit_IT(&huart6, (uint8_t *)pData, Size);
  ASSERT(HAL_OK == status);

  queue_status = xQueueReceive(UART6_CPL_Message_Id, 0, UART6_DEFAULT_TIMEOUT);
  ASSERT(pdTRUE == queue_status);
}

bool UART6_Receive(uint8_t *pRxByte, uint32_t timeout)
{
  BaseType_t queue_status = pdFALSE;

  queue_status = xQueueReceive(UART6_Rx_Queue, pRxByte, timeout);
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
      ASSERT(pdTRUE == queue_status);
    }
  else
    {
//      ASSERT(0); todo patch needs to be reworked
    }
  portEND_SWITCHING_ISR( xHigherPriorityTaskWokenByPost);
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
      ASSERT(pdTRUE == queue_status);
    }
  else
    {
      //		ASSERT(0);
    }
  portYIELD_FROM_ISR(xHigherPriorityTaskWokenByPost);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{

}


#endif
