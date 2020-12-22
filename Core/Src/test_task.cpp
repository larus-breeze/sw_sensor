#include "system_configuration.h"
#include "my_assert.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "fatfs.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "stm_l3gd20.h"
#include "stdio.h"
#include "i2c.h"
#include "fxos8700cq.h"

#if RUN_SDIO_TEST

FATFS fatfs;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

#ifndef ROM
#define ROM const __attribute__ ((section (".rodata")))
#endif

ROM char testString[] =
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
  "0123456789abcd\r\n"
  "0123456789abcdef"
  "0123456789abcd\r\n";

void RunFATFSTestTask(void *)
{
	HAL_SD_DeInit(&hsd);
	osDelay(100);  //Not sure if required.

/*
	Register DMA SDIO callbacks
	HAL_DMA_RegisterCallback(&hdma_sdio_rx, HAL_DMA_XFER_CPLT_CB_ID,
			(void (*)(DMA_HandleTypeDef *)) BSP_SD_ReadCpltCallback);
	HAL_DMA_RegisterCallback(&hdma_sdio_rx, HAL_DMA_XFER_ABORT_CB_ID,
			(void (*)(DMA_HandleTypeDef *)) BSP_SD_AbortCallback);
	HAL_DMA_RegisterCallback(&hdma_sdio_tx, HAL_DMA_XFER_CPLT_CB_ID,
			(void (*)(DMA_HandleTypeDef *)) BSP_SD_WriteCpltCallback);
	HAL_DMA_RegisterCallback(&hdma_sdio_tx, HAL_DMA_XFER_ABORT_CB_ID,
			(void (*)(DMA_HandleTypeDef *)) BSP_SD_AbortCallback);

*/
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
					HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, led_state);
					led_state =  led_state == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;
				}
			}
		}
	}
	fresult = f_close(&fp);
	duration = HAL_GetTick() - start;
	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
	while( true)
		suspend();
}

Task SD_tester( RunFATFSTestTask, "SD_IO", 256);

#endif

void RunL3GD20TestTask(void) {
	SPI_Init(&hspi2);
	L3GD20_Initialize();

	osDelay(100); //Let sensor gather some data in FIFO.
#define BUFFERSIZE 100
#define NUMBER_OF_CALIBRATIONRUNS 500
	char printbuf[BUFFERSIZE];
	uint8_t size = 0;
	float gyro_xyz[] = { 0.0, 0.0, 0.0 };
	float gyro_xyz_calib[] = { 0.0, 0.0, 0.0 };
	int16_t gyro_xyzint[] = { 0, 0, 0 };

	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_SET);
	for (int i = 0; i < NUMBER_OF_CALIBRATIONRUNS; i++) {
		L3GD20_ReadData(gyro_xyz);
		for (int n = 0; n < 3; n++) {
			gyro_xyz_calib[n] = gyro_xyz_calib[n] + gyro_xyz[n];
		}
		osDelay(10);
	}
	for (int i = 0; i < 3; i++) {
		gyro_xyz_calib[i] = gyro_xyz_calib[i]
				/ (float) NUMBER_OF_CALIBRATIONRUNS;
	}
	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);

	for (;;) {
		L3GD20_ReadData(gyro_xyz);
		for (int i = 0; i < 3; i++) {
			gyro_xyz[i] = gyro_xyz[i] - gyro_xyz_calib[i];
			gyro_xyzint[i] = (int16_t) (gyro_xyz[i]);
		}
		size = sprintf(printbuf, "Raw Gyro XYZ: %6d   %6d   %6d \r\n",
				gyro_xyzint[0], gyro_xyzint[1], gyro_xyzint[2]);
		ASSERT(BUFFERSIZE >= size);

		for (int i = 0; i < size; i++) {
			ITM_SendChar(printbuf[i]);
		}
		osDelay(10);
	}
}

void RunFXOS8700TestTask(void) {
	I2C_Init();
	float xyz_acc[] = { 0, 0, 0 };
	float xyz_mag[] = { 0, 0, 0 };
#define BUFFERSIZE 100
	char printbuf[BUFFERSIZE];
	uint8_t size = 0;
	uint32_t counter = 0;
	TickType_t tickCount = xTaskGetTickCount();

	FXOS8700_Initialize(ACCEL_RANGE_4G);

	for (;;) {
		FXOS8700_get(xyz_acc, xyz_mag);

		if (counter % 10 == 0) {
			size = sprintf(printbuf, "XYZ: ACC: %6d   %6d   %6d  ",
					(int16_t) (xyz_acc[0] * 100), (int16_t) (xyz_acc[1] * 100),
					(int16_t) (xyz_acc[2] * 100));
			for (int i = 0; i < size; i++) {
				//ITM_SendChar(printbuf[i]);
			}

			size = sprintf(printbuf, "MAG: %6d   %6d   %6d \r\n",
					(int16_t) (xyz_mag[0] * 100), (int16_t) (xyz_mag[1] * 100),
					(int16_t) (xyz_mag[2] * 100));
			for (int i = 0; i < size; i++) {
				//ITM_SendChar(printbuf[i]);
			}
		}
		vTaskDelayUntil(&tickCount, 10);
	}
}
