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
#include "communicator.h"

#if RUN_MS5611_MODULE == 1

void getPressure (void*)
{
  while (true) // re-initialization loop
    {
      update_system_state_clear(MS5611_STATIC_AVAILABLE);
      update_system_state_clear(MS5611_PITOT_AVAILABLE);

      delay (2);

      acquire_privileges ();
      I2C_Init (MS5611_I2C);
      drop_privileges();

      MS5611 ms5611_static (0xEE);
      MS5611 ms5611_pitot (0xEC);  // Second ms5611 sensor on PCB.

      bool static_ms5611_available = false;
      bool pitot_ms5611_available = false;

      static_ms5611_available = ms5611_static.initialize ();
      if (static_ms5611_available)
	update_system_state_set (MS5611_STATIC_AVAILABLE);

      pitot_ms5611_available = ms5611_pitot.initialize ();
      if (pitot_ms5611_available)
	update_system_state_set (MS5611_PITOT_AVAILABLE);

      for( synchronous_timer t (10); true; t.sync()) // measurement loop
	{
	  if ( static_ms5611_available)
	    if (ms5611_static.update () == false)
	      break;;

	  if ( pitot_ms5611_available)
	    if (ms5611_pitot.update () == false)
	      break;;

	  t.sync ();

	  if ( static_ms5611_available)
	    if (ms5611_static.update () == false)
	      break;;

	  if (pitot_ms5611_available)
	    if (ms5611_pitot.update () == false)
	      break;;

	  if ( static_ms5611_available)
	    {
	      output_data.m.static_pressure = ms5611_static.get_pressure ();
	      output_data.m.static_sensor_temperature =
		  ms5611_static.get_temperature ();
	    }

	  if (true == pitot_ms5611_available)
	    {
	      output_data.m.absolute_pressure = ms5611_pitot.get_pressure ();
	      output_data.m.absolute_sensor_temperature =
		  ms5611_pitot.get_temperature ();
	    }
	}
    }
}

RestrictedTask ms5611_reading (getPressure, "P_ABS", 256, 0, MS5611_PRIORITY + portPRIVILEGE_BIT);

#endif
