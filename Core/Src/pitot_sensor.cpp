/**
 @file pitot_sensor.cpp
 @brief HCLA pressure sensor driver
 @author: Klaus Schaefer
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "i2c.h"
#include "common.h"

#define I2C_ADDRESS (0x28<<1) // 7 bits left-adjusted

static void runnable( void *)
{
  uint8_t data[4];

  I2C_Init();

//  drop_privileges(); later...

  for( synchronous_timer t(10); true; t.sync())
    {
      I2C_Read( &hi2c1, I2C_ADDRESS, data, 2);
      ASSERT(( data[0] & 0xC0)==0); 			// no error flags !
      uint16_t raw_data = (data[0] << 8) | data[1];
      observations.pressure_pitot = raw_data * 0.42082257; // 16384 = 1PSI = 6894.757 Pa
    }
}

RestrictedTask pitot_reading ( runnable, "PITOT", 256, 0, STANDARD_TASK_PRIORITY | portPRIVILEGE_BIT);


