/**
 @file L3GD20_gyro.cpp
 @brief HCLA pressure sensor driver
 @author: Klaus Schaefer
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "spi.h"
#include "stm_l3gd20.h"
#include "communicator.h"

#if RUN_L3GD20

#define SCALING 1.527e-4f

static void runnable (void*)
{
  if( L3GD20_Initialize ())
	  update_system_state_set( L3GD20_SENSOR_AVAILABLE);
  else
	  suspend(); // discontinue task

  delay(100); //Let sensor gather some data in FIFO.

  float gyro_xyz[3] = { 0};

  for( synchronous_timer t(10); true; t.sync())
    {
      L3GD20_ReadData (gyro_xyz);
      for (int i = 0; i < 3; i++)
	 output_data.m.lowcost_gyro[i] = gyro_xyz[i] * SCALING;
    }
}

static ROM TaskParameters_t p =
{
    runnable,
    "CHIPSNS",
    128,
    0,
    L3GD20_PRIORITY,
    0,
	{
		{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
		{ 0, 0, 0 },
		{ 0, 0, 0 }
	}
};

RestrictedTask gyro_chip_sensor_task( p);

#endif
