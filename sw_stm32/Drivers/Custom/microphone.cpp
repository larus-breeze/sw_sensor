#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"

#include "generic_CAN_driver.h"
#include "candriver.h"

#include "main.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_spi.h"
#include "spi.h"
#include "embedded_math.h"

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

#define MIC_DMA_BUFSIZE_HALFWORDS 256
#define SAMPLE_BUFSIZE 512

uint16_t __ALIGNED( MIC_DMA_BUFSIZE_HALFWORDS * sizeof( uint16_t)) mic_DMA_buffer[ MIC_DMA_BUFSIZE_HALFWORDS];
int8_t __ALIGNED( SAMPLE_BUFSIZE) samples_at_5_kHz[SAMPLE_BUFSIZE];

COMMON uint32_t out_of_range_count;
COMMON uint32_t zero_seen;

// our byte samples have 328kHz sampling-rate
// resampling by 64 gives 5.125 kHz sampling-rate
void resample_by_64( uint8_t * bytes, int8_t * samples, unsigned bytecount)
{
  while( true)
    {
      int accumulator = 0;

      for( unsigned i=0; i<64; ++i)
	{
	  if( *bytes ==0)
	    ++zero_seen;
          accumulator += bit_count_table[*bytes++];
	}

      if( (accumulator > 256 + 127) || ( accumulator < 128))
	    ++out_of_range_count;
      *samples++ = (int8_t)(accumulator - 256);

      bytecount -= 64;
      if( bytecount == 0)
	  return;
    }
}

uint64_t getTime_usec(void);

COMMON int32_t ac_power;
COMMON int32_t dc_content;
COMMON volatile uint64_t time_delta;
COMMON volatile uint64_t time_delta_max;
COMMON volatile uint64_t time_delta_min;
COMMON uint32_t overrun_count;

static void runnable (void*)
{
  configure_SPI_interface ();
  register_SPI_usertask( &hspi2);

  volatile HAL_StatusTypeDef status;
  status = HAL_SPI_Receive_DMA( &hspi2, (uint8_t *)mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS);

  uint32_t BufferIndex; // 0 -> first half, 1 second half

  time_delta_max=0;
  time_delta_min=0xffffffffffffffff;

  int8_t * target = samples_at_5_kHz;
  int32_t sum = 0;
  int32_t qsum = 0;

  while (true)
    {
      BufferIndex = 0xffffffff;

      uint64_t timestamp = getTime_usec();
      xTaskNotifyWait( 0, 0, &BufferIndex, 2);

      time_delta = getTime_usec() - timestamp;
      if( time_delta < time_delta_min)
	time_delta_min = time_delta;
      if( time_delta > time_delta_max)
	time_delta_max = time_delta;

      if( BufferIndex != 0) // be sure to be behind the half transfer interrupt
	{
	  target = samples_at_5_kHz;
	  sum = 0;
	  qsum = 0;
	  ++overrun_count;
	  HAL_SPI_DMAStop( &hspi2);
	  status = HAL_SPI_Receive_DMA( &hspi2, (uint8_t *)mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS);
	  continue; // re-synchronize
	}
      // now we have 256 byte samples @ the beginning of our buffer
      resample_by_64( (uint8_t *)mic_DMA_buffer, target, MIC_DMA_BUFSIZE_HALFWORDS);

      // do statistics and advance pointer
      for( unsigned i=0; i<8; ++i)
      {
	sum += (int32_t)*target;
	qsum += (int32_t)*target * *target;
	++target;
      }

      BufferIndex = 0xffffffff;
      timestamp = getTime_usec();
      xTaskNotifyWait( 0, 0, &BufferIndex, 2);

      time_delta = getTime_usec() - timestamp;
      if( time_delta < time_delta_min)
	time_delta_min = time_delta;
      if( time_delta > time_delta_max)
	time_delta_max = time_delta;

      if( BufferIndex != 1) // be sure to be behind the transfer complete interrupt
	{
	  target = samples_at_5_kHz;
	  sum = 0;
	  qsum = 0;
	  ++overrun_count;
	  HAL_SPI_DMAStop( &hspi2);
	  status = HAL_SPI_Receive_DMA( &hspi2, (uint8_t *)mic_DMA_buffer, MIC_DMA_BUFSIZE_HALFWORDS);
	  continue; // re-synchronize
	}
      // now we have another 256 byte samples @ the middle of our buffer
      resample_by_64( (uint8_t *)(&mic_DMA_buffer[MIC_DMA_BUFSIZE_HALFWORDS / 2]), target, MIC_DMA_BUFSIZE_HALFWORDS);

      // do statistics and advance pointer
      for( unsigned i=0; i<8; ++i)
      {
	sum += (int32_t)*target;
	qsum += (int32_t)*target * *target;
	++target;
      }

      // summa summarum ...
      if( target >= samples_at_5_kHz + SAMPLE_BUFSIZE)
	{
	  //	  ac_energy = qsum - sum * sum;
	  ac_power = qsum / 16;
	  dc_content= sum / 16;
	  target = samples_at_5_kHz;
	  sum = 0;
	  qsum = 0;
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
	{ samples_at_5_kHz,  SAMPLE_BUFSIZE, portMPU_REGION_READ_WRITE }
      }
  };

static RestrictedTask mic_task ( p);
