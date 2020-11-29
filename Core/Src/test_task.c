#include "my_assert.h"
#include "main.h"
#include "fatfs.h"
#include "string.h"

FATFS fatfs;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

void StartTestTask(void const * argument)
{
	/*Register DMA SDIO callbacks*/
	  HAL_DMA_RegisterCallback(&hdma_sdio_rx, HAL_DMA_XFER_CPLT_CB_ID, (void*)BSP_SD_ReadCpltCallback);
	  HAL_DMA_RegisterCallback(&hdma_sdio_rx, HAL_DMA_XFER_ABORT_CB_ID, (void*)BSP_SD_AbortCallback);
	  HAL_DMA_RegisterCallback(&hdma_sdio_tx, HAL_DMA_XFER_CPLT_CB_ID, (void*)BSP_SD_WriteCpltCallback);
	  HAL_DMA_RegisterCallback(&hdma_sdio_tx, HAL_DMA_XFER_ABORT_CB_ID, (void*)BSP_SD_AbortCallback);

	  FRESULT fresult;
	  FIL fp;
	  uint8_t sd_card_detect = 0;
	  sd_card_detect = BSP_PlatformIsDetected();
	  ASSERT(sd_card_detect == 1);
	  uint32_t start = 0, duration = 0;

	  osDelay(500); //give micro sd card some time to...


	  fresult = f_mount(&fatfs, "", 0);


	  start = HAL_GetTick();
	  char testString[] = "The Soar Instrument.\n";
	  uint8_t testStringLength = strlen(testString);
	  uint32_t writtenBytes = 0;
	  if (FR_OK == fresult)
	  {
		  fresult = f_open(&fp, "test2111.txt",FA_CREATE_ALWAYS | FA_WRITE);

		  if (FR_OK == fresult)
		  {
			  for(int i = 0; i < 1000000; i++)
			  {
				  fresult = f_write (&fp, testString, testStringLength, (UINT*)&writtenBytes);
				  ASSERT(fresult == FR_OK);
			  }
		  }
	  }
	  fresult = f_close (&fp);
	  duration = HAL_GetTick() - start;

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}
