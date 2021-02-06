/** *****************************************************************************
 * @file    data_logger.cpp
 * @author  Klaus Schaefer,  some adaptions by Maximilian Betz
 * @brief   data logging to uSD
 ******************************************************************************/

#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "fatfs.h"
#include "common.h"
#include "ascii_support.h"

#if RUN_DATA_LOGGER

FATFS fatfs;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

#define BUFSIZE 2048 // bytes
#define RESERVE 512
static uint8_t  __ALIGNED(BUFSIZE) buffer[BUFSIZE+RESERVE];
COMMON static char filename[25];

void data_logger_runnable(void*)
{
	HAL_SD_DeInit (&hsd);
	delay (2000); //TODO: Quick consecutive resets cause SD Card to hang. This improved but does not fix the situation.

	FRESULT fresult;
	FIL fp;

	// wait until sd card is detected
	while(! BSP_PlatformIsDetected ())
	{
		delay(1000);
	}
	delay(500); // wait until card is plugged correctly

	// wait until a GNSS timestamp is available.
	while(output_data.c.year == 0)
	{
		delay(100);  /**/
	}

	// generate filename based on timestamp
	int idx = 0;
	itoa(2000 + output_data.c.year, filename, 10);
	while(filename[idx] != 0)
		idx++;
	if (output_data.c.month < 10)
	{
		filename[idx] = '0';
		idx++;
	}
	itoa(output_data.c.month, &filename[idx], 10);
	while(filename[idx] != 0)
		idx++;

	if (output_data.c.day < 10)
	{
		filename[idx] = '0';
		idx++;
	}
	itoa(output_data.c.day, &filename[idx], 10);
	while(filename[idx] != 0)
		idx++;

	if (output_data.c.hour < 10)
	{
		filename[idx] = '0';
		idx++;
	}
	itoa(output_data.c.hour, &filename[idx], 10);
	while(filename[idx] != 0)
		idx++;

	if (output_data.c.minute < 10)
	{
		filename[idx] = '0';
		idx++;
	}
	itoa(output_data.c.minute, &filename[idx], 10);
	while(filename[idx] != 0)
		idx++;

	if (output_data.c.second < 10)
	{
		filename[idx] = '0';
		idx++;
	}
	itoa(output_data.c.second, &filename[idx], 10);
	while(filename[idx] != 0)
		idx++;

	filename[idx] = '.';
	filename[idx+1] = 'f';
	filename[idx+2] = '7';
	filename[idx+3] = '9';
	filename[idx+4] = 0;

	GPIO_PinState led_state = GPIO_PIN_RESET;

	uint32_t writtenBytes = 0;
	uint8_t *buf_ptr=buffer;

	fresult = f_mount (&fatfs, "", 0);
	if (FR_OK == fresult)
	{
		fresult = f_open (&fp, filename, FA_CREATE_ALWAYS | FA_WRITE);
		if (FR_OK == fresult)
		{
			for ( synchronous_timer t(10); true; t.sync())
			{
#if LOG_OBSERVATIONS
				memcpy( buf_ptr, (uint8_t *)&measurement_data, sizeof(measurement_data) );
				buf_ptr += sizeof(measurement_data);
#endif
#if LOG_COORDINATES
				memcpy( buf_ptr, (uint8_t *)&(GNSS.coordinates), sizeof(coordinates_t) );
				buf_ptr += sizeof(coordinates_t);
#endif
#if LOG_OUTPUT_DATA
				memcpy( buf_ptr, (uint8_t *)&( output_data ), sizeof(output_data) );
				buf_ptr += sizeof(output_data);
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
		LOGGER_PRIORITY + portPRIVILEGE_BIT,
		stack_buffer,
		{
				{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
				{ 0, 0, 0 },
				{ 0, 0, 0 }
		}
};

RestrictedTask data_logger( p);

#endif
