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

#if STATISTICS
COMMON float temperature_pitot = 0.0f;
COMMON float pressure_squaresum = 0.0f;
COMMON float pressure_sum = 0.0f;
COMMON float pressure_variance = 0.0f;
COMMON uint32_t pressure_samples =0;
#endif

void getPressure (void*)
{
  acquire_privileges(); /*I2C HAL functions cause MPU exception.*/
  I2C_Init ();
#define BUFFERSIZE 100
  char printbuf[BUFFERSIZE];
  uint8_t size = 0;

//  MS5611 ms5611_static (0xEE);
  MS5611 ms5611_pitot (0xEC);

//  ms5611_static.initialize ();
  ms5611_pitot.initialize ();

//  volatile float pressure_static = 0.0f;
//  volatile float temperature_static = 0.0f;
  synchronous_timer t(10);
  while( true)
    {
//    ms5611_static.update ();
//    pressure_static = ms5611_static.get_pressure ();
//    temperature_static = ms5611_static.get_temperature ();
      ms5611_pitot.update ();
      t.sync();
      ms5611_pitot.update ();

      observations.pressure_absolute = ms5611_pitot.get_pressure ();

#if STATISTICS
      temperature_pitot = ms5611_pitot.get_temperature ();
      pressure_squaresum += pressure_pitot*pressure_pitot;
      pressure_sum += pressure_pitot;
      ++pressure_samples;
      float pressure_mean = pressure_sum / (float) pressure_samples;
      pressure_variance =
	  (pressure_squaresum / (float)pressure_samples - pressure_mean * pressure_mean);
#endif

      t.sync();
    }
}

RestrictedTask ms5611_reading (getPressure, "P_ABS", 256);

#endif
