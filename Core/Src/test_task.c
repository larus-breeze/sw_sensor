#include "my_assert.h"
#include "main.h"
#include "fatfs.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "stm_l3gd20.h"
#include "stdio.h"
#include "i2c.h"
#include "fxos8700cq.h"

FATFS fatfs;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

#ifndef ROM
#define ROM const __attribute__ ((section (".rodata")))
#endif

ROM char testString[] = "0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n"
		"0123456789abcdef"
		"0123456789abcd\r\n";

void RunFATFSTestTask(void) {
	HAL_SD_DeInit(&hsd);
	osDelay(100);  //Not sure if required.

	/*Register DMA SDIO callbacks*/
	HAL_DMA_RegisterCallback(&hdma_sdio_rx, HAL_DMA_XFER_CPLT_CB_ID,
			(void*) BSP_SD_ReadCpltCallback);
	HAL_DMA_RegisterCallback(&hdma_sdio_rx, HAL_DMA_XFER_ABORT_CB_ID,
			(void*) BSP_SD_AbortCallback);
	HAL_DMA_RegisterCallback(&hdma_sdio_tx, HAL_DMA_XFER_CPLT_CB_ID,
			(void*) BSP_SD_WriteCpltCallback);
	HAL_DMA_RegisterCallback(&hdma_sdio_tx, HAL_DMA_XFER_ABORT_CB_ID,
			(void*) BSP_SD_AbortCallback);

	FRESULT fresult;
	FIL fp;
	uint8_t sd_card_detect = 0;
	sd_card_detect = BSP_PlatformIsDetected();
	ASSERT(sd_card_detect == 1);
	uint32_t start = 0, duration = 0;
	GPIO_PinState led_state = GPIO_PIN_RESET;

	osDelay(2000); //give micro sd card some time to...Not sure if required

	fresult = f_mount(&fatfs, "", 0);

	start = HAL_GetTick();
	uint32_t writtenBytes = 0;
	if (FR_OK == fresult) {
		fresult = f_open(&fp, "pattern.txt", FA_CREATE_ALWAYS | FA_WRITE);

		if (FR_OK == fresult) {
			for (int i = 0; i < 2048 * 8; i++) // write 8 Mbytes
			{
				fresult = f_write(&fp, testString, 512, (UINT*) &writtenBytes);
				ASSERT((fresult == FR_OK) && (writtenBytes == 512));

				if ((i % 200) == 0) {
					HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
							led_state);
					led_state = (~led_state) & 0x01;
				}
			}
		}
	}
	fresult = f_close(&fp);
	duration = HAL_GetTick() - start;
	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
}


void StartTestTask(void const *argument) {
	//osDelay(5000); //Let USB Connect First.

	//RunFATFSTestTask();

	TickType_t tickCount = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&tickCount, 10);
	}
	/* USER CODE END 5 */
}
