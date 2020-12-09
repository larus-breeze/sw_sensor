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

void getPressure(void*) {
	acquire_privileges(); /*I2C HAL functions cause MPU exception.*/
	I2C_Init();
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
			ITM_SendChar(printbuf[i]);
		}

	}
}

RestrictedTask ms5611_reading(getPressure, "Pressure", 256);

