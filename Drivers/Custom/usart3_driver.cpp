/**
 @file usart3_driver.cpp
 @brief GNSS USART driver
 @author: Dr. Klaus Schaefer
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "stm32f4xx_hal.h"
#include "GNSS.h"

#if RUN_GNSS

COMMON UART_HandleTypeDef huart3;
COMMON DMA_HandleTypeDef hdma_usart3_rx;
COMMON  static TaskHandle_t USART3_task_Id = NULL;

/**
 * @brief USART3 Initialization Function
 */
static inline void MX_USART3_UART_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
      ASSERT(0);

    __HAL_LINKDMA( &huart3, hdmarx, hdma_usart3_rx);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
      ASSERT(0);

    HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, STANDARD_ISR_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
extern "C" void
DMA1_Stream1_IRQHandler (void)
{
  BaseType_t HigherPriorityTaskWoken=0;
  HAL_DMA_IRQHandler (&hdma_usart3_rx);
  vTaskNotifyGiveFromISR( USART3_task_Id, &HigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
}

#define GPS_DMA_buffer_SIZE (sizeof( uBlox_pvt) + 8) // plus "u B class id size1 size2 ... cks1 cks2"

static uint8_t buffer[GPS_DMA_buffer_SIZE];

static void GNSS_runnable (void*)
{
  USART3_task_Id = xTaskGetCurrentTaskHandle();
  MX_USART3_UART_Init ();
  volatile HAL_StatusTypeDef result;

  while (true)
    {
      result = HAL_UART_Receive_DMA (&huart3, buffer, GPS_DMA_buffer_SIZE);
      if( result != HAL_OK)
	{
	  HAL_UART_Abort (&huart3);
#if UART3_LED_STATUS
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
#endif
	  continue;
	}
      // wait for half transfer interrupt
      uint32_t pulNotificationValue;
      BaseType_t notify_result = xTaskNotifyWait( 0xffffffff, 0xffffffff, &pulNotificationValue, 100);
      if( notify_result != pdTRUE)
	{
	  HAL_UART_Abort (&huart3);
#if UART3_LED_STATUS
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
#endif
	  continue;
	}
      // wait for transfer complete interrupt
      notify_result = xTaskNotifyWait( 0xffffffff, 0xffffffff, &pulNotificationValue, 10);
      if( notify_result != pdTRUE)
	{
	  HAL_UART_Abort (&huart3);
#if UART3_LED_STATUS
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
#endif
	  continue;
	}
      HAL_UART_Abort (&huart3);

//      if ((buffer[0] != 0xb5) || (buffer[1] != 'b'))
      GNSS_Result result = GNSS.update(buffer);
      if( result == GNSS_ERROR)
      {
#if UART3_LED_STATUS
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
#endif
	  HAL_UART_Abort (&huart3);
	  delay (50);
	  continue;
	}
      if(  result == GNSS_HAVE_FIX)
    	  update_system_state_set( GNSS_AVAILABLE);

#if UART3_LED_STATUS
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_SET);
#endif
      delay( 50);
    }
}

Task usart3_task (GNSS_runnable, "GNSS", 256, 0, STANDARD_TASK_PRIORITY+1);

#endif
