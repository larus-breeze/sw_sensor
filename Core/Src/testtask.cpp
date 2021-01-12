/*
 * testtask.cpp
 *
 *  Created on: Dec 9, 2020
 *      Author: mbetz
 */
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "i2c.h"
#include "ms5611.h"
#include "stdio.h"
#include "my_assert.h"
#include "fatfs.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "stm_l3gd20.h"
#include "stdio.h"
#include "fxos8700cq.h"
#include "bsp_driver_sd.h"


void getPressure(void*)
{
	acquire_privileges(); /*I2C HAL functions cause MPU exception.*/
	I2C_Init(MS5611_I2C);
#define BUFFERSIZE 100
	char printbuf[BUFFERSIZE];
	uint8_t size = 0;

	MS5611 ms5611_static(0xEE);
	MS5611 ms5611_pitot(0xEC);

	ms5611_static.initialize();
	ms5611_pitot.initialize();

	float pressure_static = 0.0f, pressure_pitot = 0.0f, temperature_static =
			0.0f, temperature_pitot = 0.0f;
	for (;;) {
		delay(500);
		ms5611_static.update();
		ms5611_pitot.update();

		pressure_static = ms5611_static.get_pressure();
		pressure_pitot = ms5611_pitot.get_pressure();
		temperature_static = ms5611_static.get_temperature();
		temperature_pitot = ms5611_pitot.get_temperature();


		size = sprintf(printbuf, "Static, %ld, %ld, Pitot, %ld, %ld\r\n",(int32_t)(pressure_static * 1000),
				(int32_t)(temperature_static * 100), (int32_t)(pressure_pitot * 1000), (int32_t)(temperature_pitot * 100));
		ASSERT(BUFFERSIZE >= size);

		for (int i = 0; i < size; i++)
		{
			//ITM_SendChar(printbuf[i]);
		}

	}
}

void RunL3GD20TestTask(void*)
{
	SPI_Init(&hspi2);

	acquire_privileges(); //TODO: check why?
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

void RunFXOS8700TestTask(void*)
{
	acquire_privileges(); //TODO: check why?
	I2C_Init(&hi2c1);
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


//Run all Test Tasks parallel
RestrictedTask ms5611_reading(getPressure, "Pressure", 256);
RestrictedTask l3gd20_reading(RunL3GD20TestTask, "Gyro", 512);
RestrictedTask fxos8700_reading(RunFXOS8700TestTask, "AccMag", 512);




