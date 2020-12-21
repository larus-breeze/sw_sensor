#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "stm32f4xx_hal.h"
#include "GNSS.h"

COMMON UART_HandleTypeDef huart1;
COMMON DMA_HandleTypeDef hdma_usart1_rx;
COMMON  static TaskHandle_t USART1_task_Id = NULL;

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_USART1_UART_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };
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

  /* USART1 DMA Init */
  /* USART1_RX Init */
  hdma_usart1_rx.Instance = DMA2_Stream2;
  hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx.Init.Mode = DMA_NORMAL;
  hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init (&hdma_usart1_rx) != HAL_OK)
    {
      ASSERT(0);
    }

    HAL_NVIC_SetPriority (DMA2_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ (DMA2_Stream2_IRQn);

  __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    HAL_NVIC_SetPriority (USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ (USART1_IRQn);

  huart1.Instance = USART1;

  huart1.Init.BaudRate = 115200;
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

/**
 * @brief This function handles USART1 global interrupt.
 */
extern "C" void
USART1_IRQHandler (void)
{
  HAL_UART_IRQHandler (&huart1);
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
extern "C" void
DMA2_Stream2_IRQHandler (void)
{
  BaseType_t HigherPriorityTaskWoken=0;
  HAL_DMA_IRQHandler (&hdma_usart1_rx);
  vTaskNotifyGiveFromISR( USART1_task_Id, &HigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
}

#define GPS_DMA_buffer_SIZE (sizeof( uBlox_pvt) + 8) // plus "u B class id size1 size2 ... cks1 cks2"

static uint8_t buffer[GPS_DMA_buffer_SIZE];

void
usart_tester_runnable (void*)
{
  USART1_task_Id = xTaskGetCurrentTaskHandle();
  MX_USART1_UART_Init ();
  volatile HAL_StatusTypeDef result;
  synchronous_timer t(100);
  while (true)
    {
      result = HAL_UART_Receive_DMA (&huart1, buffer, GPS_DMA_buffer_SIZE);
      if( result != HAL_OK)
	continue;
      uint32_t pulNotificationValue;
      BaseType_t notify_result = xTaskNotifyWait( 0xffffffff, 0, &pulNotificationValue, 100);
      if( notify_result != pdTRUE)
	continue;
      t.re_synchronize(xTaskGetTickCount() - 11);
      if ((HAL_OK != result) || (buffer[0] != 0xb5) || (buffer[1] != 'b'))
	{
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
	  HAL_UART_Abort (&huart1);
	  delay (50);
	  continue;
	}
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_SET);
      t.sync();
    }
}

Task usart1_task (usart_tester_runnable, "USART1");

