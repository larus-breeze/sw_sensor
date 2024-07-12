#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"

#include "generic_CAN_driver.h"
#include "candriver.h"

#include "main.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_spi.h"
#include "spi.h"
#include "communicator.h"
#include "microphone.h"
#include "embedded_math.h"

#if RUN_MICROPHONE

ROM int8_t bit_count_table[] =
{
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

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
  // bit samplerate will be 168Mhz / 4 / 16 => 2.625 Mhz
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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

uint16_t __ALIGNED( MIC_DMA_BUFSIZE_HALFWORDS * sizeof( uint16_t)) mic_DMA_buffer[ MIC_DMA_BUFSIZE_HALFWORDS];
int8_t __ALIGNED( SAMPLE_BUFSIZE) audio_samples[2][SAMPLE_BUFSIZE];

// our byte samples have 328kHz sampling-rate
// resampling by 64 gives 5.125 kHz sampling-rate
void resample( uint8_t * bytes, int8_t * samples, unsigned output_bytecount)
{
  do
    {
      int accumulator = 0;

      for( unsigned i=0; i<RESAMPLING_RATIO; ++i)
	accumulator += bit_count_table[*bytes++];

      *samples++ = (int8_t)(accumulator - RESAMPLING_RATIO * 4);

      --output_bytecount;
    }
  while( output_bytecount > 0);
}

uint64_t getTime_usec(void);

static void runnable (void*)
{
  configure_SPI_interface ();
  register_SPI_usertask( &hspi2);
  uint32_t double_buffer_index=0;

  delay(3000);

  HAL_SPI_Receive_DMA( &hspi2, (uint8_t *)mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS);

  uint32_t BufferIndex=0; // 0 -> first half, 1 second half

  int8_t * samples_pointer  = audio_samples[double_buffer_index];
  int32_t sum = 0;
  int32_t qsum = 0;

  while (true)
    {
      BufferIndex = 0xffffffff;
      xTaskNotifyWait( 0, 0, &BufferIndex, 40); // trigger on half complete interrupt

      if( BufferIndex != 0) // be sure to be behind the half transfer interrupt
	{
	  HAL_SPI_DMAStop( &hspi2);
	  HAL_SPI_Receive_DMA( &hspi2, (uint8_t *)mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS);

	  double_buffer_index = 0;
	  samples_pointer  = audio_samples[double_buffer_index];
	  sum = 0;
	  qsum = 0;

	  continue; // re-synchronize
	}
      // now we have 128 byte samples @ the beginning of our buffer
      resample( (uint8_t *)mic_DMA_buffer, samples_pointer, SAMPLE_BUFSIZE_HALF);

      // do statistics and advance pointer
      for( unsigned i=0; i < SAMPLE_BUFSIZE_HALF; ++i)
      {
	sum += (int32_t)*samples_pointer;
	qsum += (int32_t)*samples_pointer * *samples_pointer;
	++samples_pointer;
      }

      BufferIndex = 0xffffffff;
      xTaskNotifyWait( 0, 0, &BufferIndex, 40);

      if( BufferIndex != 1) // be sure to be behind the transfer complete interrupt
	{
	  HAL_SPI_DMAStop( &hspi2);
	  HAL_SPI_Receive_DMA( &hspi2, (uint8_t *)mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS);

	  double_buffer_index = 0;
	  samples_pointer  = audio_samples[double_buffer_index];
	  sum = 0;
	  qsum = 0;

	  continue; // re-synchronize
	}
      // now we have 128 byte samples @ the middle of our buffer
      resample( (uint8_t *)(&mic_DMA_buffer[MIC_DMA_BUFSIZE_HALFWORDS / 2]), samples_pointer, SAMPLE_BUFSIZE_HALF);

      // do statistics and advance pointer
      for( unsigned i=0; i < SAMPLE_BUFSIZE_HALF; ++i)
      {
	sum += (int32_t)*samples_pointer;
	qsum += (int32_t)*samples_pointer * *samples_pointer;
	++samples_pointer;
      }

      // summa summarum ...
      if( samples_pointer >= audio_samples[double_buffer_index] + SAMPLE_BUFSIZE)
	{
	  // this scaling delivers ac_power = 13.0 @ 94dB
	  output_data.sound_intensity  = (qsum * SAMPLE_BUFSIZE - sum * sum) / ((float)SAMPLE_BUFSIZE * SAMPLE_BUFSIZE);
	  sum = 0;
	  qsum = 0;

	  double_buffer_index ^= 1; // switch our twin buffer
	  samples_pointer  = audio_samples[double_buffer_index];
	}
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
	{ mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS * 2, portMPU_REGION_READ_ONLY },
	      { audio_samples, sizeof(audio_samples), portMPU_REGION_READ_WRITE}
      }
  };

static RestrictedTask mic_task ( p);

#endif
