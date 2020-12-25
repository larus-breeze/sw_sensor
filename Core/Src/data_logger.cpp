/** *****************************************************************************
 * @file    data_logger.cpp
 * @author  Klaus Schaefer
 * @brief   data logging to uSD
 ******************************************************************************/

#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "fatfs.h"
#include "common.h"

#if RUN_DATA_LOGGER

FATFS fatfs;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

#define BUFSIZE 1024
#define RESERVE 256
static uint8_t  __ALIGNED(BUFSIZE) buffer[BUFSIZE+RESERVE];

void data_logger_runnable(void*)
{
  HAL_SD_DeInit (&hsd);
  delay (100);

  FRESULT fresult;
  FIL fp;
  uint8_t sd_card_detect = 0;
  sd_card_detect = BSP_PlatformIsDetected ();

  // wait until sd card is detected
  while( sd_card_detect != 1)
    delay(1000);

  delay(500); // wait until card is plugged correctly

  GPIO_PinState led_state = GPIO_PIN_RESET;

  uint32_t writtenBytes = 0;
  uint8_t *buf_ptr=buffer;

  fresult = f_mount (&fatfs, "", 0);
  if (FR_OK == fresult)
    {
      fresult = f_open (&fp, "data.bin", FA_CREATE_ALWAYS | FA_WRITE);
      if (FR_OK == fresult)
	{
	  for ( synchronous_timer t(10); true; t.sync())
	    {
	      memcpy( buf_ptr, (uint8_t *)&observations, sizeof(observations) );
	      buf_ptr += sizeof(observations);
#if LOG_COORDINATES
	      memcpy( buf_ptr, (uint8_t *)&(GNSS.coordinates), sizeof(coordinates_t) );
	      buf_ptr += sizeof(coordinates_t);
#endif
	      if( buf_ptr < buffer+BUFSIZE)
		  continue; // buffer only filled partially

	      fresult = f_write (&fp, buffer, BUFSIZE, (UINT*) &writtenBytes);
	      ASSERT((fresult == FR_OK) && (writtenBytes == BUFSIZE));

	      uint32_t rest = buf_ptr -(buffer+BUFSIZE);
	      memcpy( buffer, buffer+BUFSIZE, rest);
	      buf_ptr = buffer + rest;

	      f_sync( &fp);
#if uSD_LED_STATUS
	      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, led_state);
	      led_state = led_state == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
	    }
	}
    }
  while( true)
    suspend();
}

#define STACKSIZE 512
static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static TaskParameters_t p =
{
    data_logger_runnable,
    "LOGGER",
    STACKSIZE,
    0,
    STANDARD_TASK_PRIORITY + portPRIVILEGE_BIT,
    stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 },
      { 0, 0, 0 }
    }
};

RestrictedTask data_logger( p);

#endif
