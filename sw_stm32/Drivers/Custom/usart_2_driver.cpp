/** *****************************************************************************
 * @file    	usart2_driver.cpp
 * @brief   	Driver for DMA transmission using USART 2
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
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "stm32f4xx_hal.h"
#include "GNSS.h"

COMMON UART_HandleTypeDef huart2;
COMMON DMA_HandleTypeDef hdma_USART_2_TX;
COMMON static TaskHandle_t USART_2_task_ID = NULL;

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
void USART_2_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  __HAL_RCC_USART2_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART1 GPIO Configuration
   PA9     ------> USART1_TX
   PA10     ------> USART1_RX
   */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  hdma_USART_2_TX.Instance = DMA1_Stream6;
  hdma_USART_2_TX.Init.Channel = DMA_CHANNEL_4;
  hdma_USART_2_TX.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_USART_2_TX.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_USART_2_TX.Init.MemInc = DMA_MINC_ENABLE;
  hdma_USART_2_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_USART_2_TX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_USART_2_TX.Init.Mode = DMA_NORMAL;
  hdma_USART_2_TX.Init.Priority = DMA_PRIORITY_LOW;
  hdma_USART_2_TX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init (&hdma_USART_2_TX) != HAL_OK)
    {
      ASSERT(0);
    }

    HAL_NVIC_SetPriority (DMA1_Stream6_IRQn, STANDARD_ISR_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream6_IRQn);

  __HAL_LINKDMA(&huart2, hdmatx, hdma_USART_2_TX);

    HAL_NVIC_SetPriority (USART2_IRQn, STANDARD_ISR_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (USART2_IRQn);

  huart2.Instance = USART2;

  huart2.Init.BaudRate = 38400; //115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init (&huart2) != HAL_OK)
    {
      ASSERT(0);
    }
}

void USART_2_transmit_DMA( uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_DMA (&huart2, pData, Size);
}

/**
 * @brief This function handles USART 2 global interrupt.
 */
extern "C" void USART2_IRQHandler (void)
{
  HAL_UART_IRQHandler (&huart2);
}

/**
 * @brief This function handles DMA1 stream6 channel 4 global interrupt.
 */
extern "C" void DMA1_Stream6_IRQHandler (void)
{
  HAL_DMA_IRQHandler (&hdma_USART_2_TX);
#if RUN_USART_2_TEST
  BaseType_t HigherPriorityTaskWoken=0;
  vTaskNotifyGiveFromISR( USART_2_task_ID, &HigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
#endif
}

#if RUN_USART_2_TEST

void USART_2_runnable (void*)
{
  USART_2_task_ID = xTaskGetCurrentTaskHandle();

  acquire_privileges();
  USART_2_Init ();
  drop_privileges();

  while (true)
    {
      uint32_t pulNotificationValue;
      BaseType_t notify_result;
      volatile HAL_StatusTypeDef result;

      result = HAL_UART_Transmit_DMA (&huart2, (uint8_t *)"Hello\r\n", 7);
//    result = HAL_UART_Transmit_IT (&huart2, (uint8_t *)"Hello\r\n", 7);
      ASSERT( result == HAL_OK);

      notify_result = xTaskNotifyWait( 0xffffffff, 0, &pulNotificationValue, INFINITE_WAIT);
      notify_result = xTaskNotifyWait( 0xffffffff, 0, &pulNotificationValue, INFINITE_WAIT);

      delay(1);
    }
}

RestrictedTask usart2_task ( USART_2_runnable, "USART2");

#endif

