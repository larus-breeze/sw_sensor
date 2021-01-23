/**
 @file ms5611.cpp
 @brief MS5611 pressure sensor driver
 @author: Maximilian Betz
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "i2c.h"
#include "ms5611.h"
#include "common.h"

#if RUN_MS5611_MODULE == 1

void getPressure (void*)
{
  I2C_Init ();
  drop_privileges();

  MS5611 ms5611_static (0xEE);
//  MS5611 ms5611_pitot (0xEC);

//  ms5611_static.initialize ();
  ms5611_static.initialize ();

//  volatile float pressure_static = 0.0f;
//  volatile float temperature_static = 0.0f;
  synchronous_timer t(10);
  while( true)
    {
//    ms5611_static.update ();
//    pressure_static = ms5611_static.get_pressure ();
//    temperature_static = ms5611_static.get_temperature ();
      ms5611_static.update ();
      t.sync();
      ms5611_static.update ();

      output_data.m.static_pressure = ms5611_static.get_pressure ();

      t.sync();
    }
}

RestrictedTask ms5611_reading (getPressure, "P_ABS", 256, 0, MS5611_PRIORITY + portPRIVILEGE_BIT);

#endif
