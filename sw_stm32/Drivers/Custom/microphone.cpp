#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"

#include "generic_CAN_driver.h"
#include "candriver.h"

#include "main.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_spi.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;

void configure_SPI_interface (void)
{
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /*Configure GPIO pin : SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init (SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin (SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET); // disable

  /**SPI2 GPIO Configuration
  PB13     ------> SPI2_SCK
  PB14     ------> SPI2_MISO
  PB15     ------> SPI2_MOSI */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

  /* SPI2 DMA Init */
  /* SPI2_RX Init */
  hdma_spi2_rx.Instance = DMA1_Stream3;
  hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
  hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_spi2_rx.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_spi2_rx.Init.Mode = DMA_CIRCULAR;
  hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
  {
	  Error_Handler();
  }

  __HAL_LINKDMA( &hspi2 , hdmarx, hdma_spi2_rx);
}

#define MIC_DMA_BUFSIZE_HALFWORDS 256

static uint16_t __ALIGNED( MIC_DMA_BUFSIZE_HALFWORDS * 2) mic_DMA_buffer[ MIC_DMA_BUFSIZE_HALFWORDS];

COMMON unsigned isr_counter;
COMMON unsigned bits_seen;

static void runnable (void*)
{
  configure_SPI_interface ();
  register_SPI_usertask( &hspi2);

  volatile HAL_StatusTypeDef status;
  status = HAL_SPI_Receive_DMA( &hspi2, (uint8_t *)mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS);

  uint32_t NotificationValue=0;
  while (true)
    {
    xTaskNotifyWait( 0, 0, &NotificationValue, INFINITE_WAIT);
    bits_seen |= NotificationValue == 1 ? 1 : 2;
    }
}

static TaskParameters_t p =
  {
    runnable,
    "MIC",
    configMINIMAL_STACK_SIZE * 4,
    0,
    STANDARD_TASK_PRIORITY,
    0,
      {
	{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
	{ mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS * 2, 0 },
	{ 0, 0, 0 }
      }
  };

static RestrictedTask mic_task ( p);
