#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "stm32f4xx_hal.h"

COMMON UART_HandleTypeDef huart1;
COMMON DMA_HandleTypeDef hdma_usart1_rx;

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        ASSERT(0);
    }

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

	huart1.Instance = USART1;

	huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
      ASSERT(0);
  }
}

/**
  * @brief This function handles USART1 global interrupt.
  */
extern "C" void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

static uint8_t buffer[128];

void usart_tester_runnable( void *)
{
	MX_USART1_UART_Init();
	while( true)
	{
		if( HAL_OK != HAL_UART_Receive_DMA( &huart1, buffer, sizeof( buffer)))
		{
			delay( 2);
			bool set = GPIO_PIN_SET == HAL_GPIO_ReadPin( LED_STATUS2_GPIO_Port, LED_STATUS1_Pin);
			HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, set ? GPIO_PIN_RESET : GPIO_PIN_SET);
			continue;
		}
		bool set = GPIO_PIN_SET == HAL_GPIO_ReadPin( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin);
		HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, set ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
}

Task usart1_task(usart_tester_runnable, "USART1" );


