/**
 @file usart4_driver.cpp
 @brief GNSS USART driver
 @author: Dr. Klaus Schaefer
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "stm32f4xx_hal.h"
#include "GNSS.h"

COMMON UART_HandleTypeDef huart4;
COMMON DMA_HandleTypeDef hdma_uart4_rx;
COMMON  static TaskHandle_t USART4_task_Id = NULL;

/**
 * @brief USART4 Initialization Function
 */
static inline void MX_USART4_UART_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
      ASSERT(0);

    __HAL_LINKDMA( &huart4, hdmarx, hdma_uart4_rx);

    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200 * 2; // second channel high speed !
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
      ASSERT(0);

    HAL_NVIC_SetPriority (DMA1_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream2_IRQn);
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
extern "C" void
DMA1_Stream2_IRQHandler (void)
{
  BaseType_t HigherPriorityTaskWoken=0;
  HAL_DMA_IRQHandler (&hdma_uart4_rx);
  vTaskNotifyGiveFromISR( USART4_task_Id, &HigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
}

#define DGNSS_DMA_buffer_SIZE (sizeof( uBlox_relpos_NED) + 8) // plus "u B class id size1 size2 ... cks1 cks2"

static uint8_t buffer[DGNSS_DMA_buffer_SIZE];

static void usart_tester_runnable (void*)
{
  USART4_task_Id = xTaskGetCurrentTaskHandle();
  MX_USART4_UART_Init ();
  volatile HAL_StatusTypeDef result;
  synchronous_timer t(100);
  while (true)
    {
      result = HAL_UART_Receive_DMA (&huart4, buffer, DGNSS_DMA_buffer_SIZE);
      if( result != HAL_OK)
	{
	  HAL_UART_Abort (&huart4);
#if UART4_LED_STATUS
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
#endif
	  continue;
	}
      uint32_t pulNotificationValue;
      BaseType_t notify_result = xTaskNotifyWait( 0xffffffff, 0xffffffff, &pulNotificationValue, 20);
      if( notify_result != pdTRUE)
	{
	  HAL_UART_Abort (&huart4);
#if UART4_LED_STATUS
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
#endif
	  continue;
	}
      notify_result = xTaskNotifyWait( 0xffffffff, 0xffffffff, &pulNotificationValue, 20);
      if( notify_result != pdTRUE)
	{
	  HAL_UART_Abort (&huart4);
#if UART4_LED_STATUS
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
#endif
	  continue;
	}
      HAL_UART_Abort (&huart4);
      t.re_synchronize(xTaskGetTickCount() - 16);
      if ((buffer[0] != 0xb5) || (buffer[1] != 'b'))
	{
#if UART4_LED_STATUS
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
#endif
	  HAL_UART_Abort (&huart4);
	  delay (50);
	  continue;
	}
#if UART4_LED_STATUS
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_SET);
#endif
      t.sync();
    }
}

Task usart4_task (usart_tester_runnable, "D-GNSS");
