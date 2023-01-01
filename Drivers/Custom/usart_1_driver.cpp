#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "stm32f4xx_hal.h"
#include "GNSS.h"
#include "usart_1_driver.h"

COMMON UART_HandleTypeDef huart1;
COMMON DMA_HandleTypeDef hdma_USART_1_TX;
COMMON static TaskHandle_t USART_1_task_ID = NULL;

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
void USART_1_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  __HAL_RCC_USART1_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART1 GPIO Configuration
   PA9     ------> USART1_TX
   PA10     ------> USART1_RX
   */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  hdma_USART_1_TX.Instance = DMA2_Stream7;
  hdma_USART_1_TX.Init.Channel = DMA_CHANNEL_4;
  hdma_USART_1_TX.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_USART_1_TX.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_USART_1_TX.Init.MemInc = DMA_MINC_ENABLE;
  hdma_USART_1_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_USART_1_TX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_USART_1_TX.Init.Mode = DMA_NORMAL;
  hdma_USART_1_TX.Init.Priority = DMA_PRIORITY_LOW;
  hdma_USART_1_TX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init (&hdma_USART_1_TX) != HAL_OK)
    {
      ASSERT(0);
    }

    HAL_NVIC_SetPriority (DMA2_Stream7_IRQn, STANDARD_ISR_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (DMA2_Stream7_IRQn);

  __HAL_LINKDMA(&huart1, hdmatx, hdma_USART_1_TX);

    HAL_NVIC_SetPriority (USART1_IRQn, STANDARD_ISR_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (USART1_IRQn);

  huart1.Instance = USART1;

  huart1.Init.BaudRate = 38400; //115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init (&huart1) != HAL_OK)
    {
      ASSERT(0);
    }
}

void USART_1_transmit_DMA( uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_DMA (&huart1, pData, Size);
}

/**
 * @brief This function handles USART 2 global interrupt.
 */
extern "C" void USART1_IRQHandler (void)
{
  HAL_UART_IRQHandler (&huart1);
}

/**
 * @brief This function handles DMA1 stream6 channel 4 global interrupt.
 */
extern "C" void DMA2_Stream7_IRQHandler (void)
{
  HAL_DMA_IRQHandler (&hdma_USART_1_TX);
#if RUN_USART_1_TEST
  BaseType_t HigherPriorityTaskWoken=0;
  vTaskNotifyGiveFromISR( USART_1_task_ID, &HigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
#endif
}

#if RUN_USART_1_TEST

void USART_1_runnable (void*)
{
  USART_1_task_ID = xTaskGetCurrentTaskHandle();

  acquire_privileges();
  USART_1_Init ();
  drop_privileges();

  while (true)
    {
      uint32_t pulNotificationValue;
      BaseType_t notify_result;
      volatile HAL_StatusTypeDef result;

     result = HAL_UART_Transmit_DMA (&huart1, (uint8_t *)"Hello\r\n", 7);
//   result = HAL_UART_Transmit_IT (&huart1, (uint8_t *)"Hello\r\n", 7);
      ASSERT( result == HAL_OK);

      notify_result = xTaskNotifyWait( 0xffffffff, 0, &pulNotificationValue, INFINITE_WAIT);
      notify_result = xTaskNotifyWait( 0xffffffff, 0, &pulNotificationValue, INFINITE_WAIT);

      delay(1);
    }
}

RestrictedTask usart1_task ( USART_1_runnable, "USART1");

#endif

