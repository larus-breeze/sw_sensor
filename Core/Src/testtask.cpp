/*
 * testtask.cpp
 *
 *  Created on: 12.01.2021
 *      Author: Maximilian Betz
 */
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "my_assert.h"
#include "stdio.h" //sprintf

#include "ms5611.h"
#include "stm_l3gd20.h"
#include "fxos8700cq.h"
#include "eeprom.h"

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

void getRotation(void*)
{
	acquire_privileges(); /*HAL functions cause MPU exception.*/
	SPI_Init(&hspi2);
	L3GD20_Initialize();

	delay(100); //Let sensor gather some data in FIFO.
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
		delay(10);
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
		delay(10);
	}
}

void getAccMag(void*)
{
	acquire_privileges();  /*I2C HAL functions cause MPU exception.*/
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



/* Virtual address defined by the user: 0xFFFF value is prohibited */
COMMON uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
void flashTest(void*)
{
	synchronous_timer myTimer(100);

	//Patch required for running without acquire priviledges: COMMON __IO uint32_t uwTick; in stm32f4xx_hal.c and COMMON FLASH_ProcessTypeDef pFlash;
	acquire_privileges();

	/* Unlock the Flash Program Erase controller */
	HAL_FLASH_Unlock();

	if( EE_Init() != EE_OK)
	{
		Error_Handler();
	}

	if(EE_WriteVariable(VirtAddVarTab[0], 12345)  != EE_OK )
	{
		Error_Handler();
	}

	uint16_t temp = 0;
	for (;;) {
		myTimer.sync();
		if(EE_ReadVariable(VirtAddVarTab[0], &temp)  != EE_OK )
		{
			Error_Handler();
		}
	}
}

COMMON static TaskParameters_t params =
{
		flashTest, // task code
		"FLASH",
		256, // words
		0,  //No parameters
		STANDARD_TASK_PRIORITY,
		(StackType_t*)0,  //No stack provided let FreeRTOS allocate it.
		{
				{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
				{ (void *)0x080F8000, 0x8000, portMPU_REGION_READ_WRITE},  //Allow write access to the last 32KByte of Flash.
				{ 0, 0, 0 }
		}
};
COMMON RestrictedTask FLASH_task( params);

RestrictedTask ms5611_reading(getPressure, "Pressure", 256);
RestrictedTask l3gd20_reading(getRotation, "Gyro", 512);
RestrictedTask fxos8700_reading(getAccMag, "AccMag", 512);




