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
#include "common.h"

#if RUN_L3GD20

#define SCALING 1.527e-4f

static void runnable (void*)
{
  L3GD20_Initialize ();

  delay(100); //Let sensor gather some data in FIFO.

  float gyro_xyz[3] = { 0};

  for( synchronous_timer t(10); true; t.sync())
    {
      L3GD20_ReadData (gyro_xyz);
      for (int i = 0; i < 3; i++)
	 output_data.m.lowcost_gyro[i] = gyro_xyz[i] * SCALING;
    }
}

#define STACKSIZE 128
static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static TaskParameters_t p =
{
    runnable,
    "CHIPSNS",
    STACKSIZE,
    0,
    L3GD20_PRIORITY,
    stack_buffer,
	{
		{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
		{ 0, 0, 0 },
		{ 0, 0, 0 }
	}
};

RestrictedTask chip_sensor_task( p);

#endif
