/**
 @file FXOS8700.cpp
 @brief  Low Cost  ACC + MAG Sensor
 @author: Maximilian Betz
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "i2c.h"
#include "fxos8700cq.h"
#include "common.h"

#if RUN_FXOS8700

#define SCALING 1.0

void acc_mag_runnable(void *) {
	acquire_privileges();
	I2C_Init(&hi2c1);

	float xyz_acc[] = { 0, 0, 0 };
	float xyz_mag[] = { 0, 0, 0 };

	bool fxos8700_available = false;
	fxos8700_available = FXOS8700_Initialize(ACCEL_RANGE_4G);


	delay(100); //Let sensor gather some data in FIFO.
	synchronous_timer t(10);

	for(;;)
	{
		t.sync();
		if(true == fxos8700_available)
		{

			FXOS8700_get(xyz_acc, xyz_mag);
			for (int i = 0; i < 3; i++)
			{
				output_data.m.lowcost_acc[i] = xyz_acc[i] * SCALING;
				output_data.m.lowcost_mag[i] = xyz_mag[i] * SCALING;
			}
		}
		else
		{
			// FXOS8700 not available, might be due to the known soldering bridge problem
		}
	}

}


#define STACKSIZE 128
static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static TaskParameters_t p =
{
		acc_mag_runnable,
		"ACCMAG",
		STACKSIZE,
		0,
		FXOS8700_PRIORITY + portPRIVILEGE_BIT,
		stack_buffer,
		{
				{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
				{ 0, 0, 0 },
				{ 0, 0, 0 }
		}
};

//RestrictedTask acc_mag_chip_sensor_task( p);    //TODO: pitot_sensor and this task concurrently used the i2c1 without mutex!

#endif




