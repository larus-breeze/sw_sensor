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

void getPressure( void *)
{
	acquire_privileges();  /*I2C HAL functions cause MPU exception.*/
	I2C_Init();
	MS5611 ms5611;
	ms5611.initialize();
	float pressure = 0.0f, temperature = 0.0f;
	for(;;)
	{
		ms5611.update();
		pressure = ms5611.get_pressure();
		temperature = ms5611.get_temperature();

		delay(100);
	}
}

RestrictedTask ms5611_reading( getPressure, "BLINK");


