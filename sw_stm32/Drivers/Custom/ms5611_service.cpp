/**
 @file 		ms5611_service.cpp
 @brief 	MS5611 pressure sensor driver
 @author: 	Maximilian Betz
 @copyright 	Copyright 2021 Maximilian Betz. All rights reserved.
 @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "i2c.h"
#include "ms5611.h"
#include "common.h"
#include "communicator.h"
#include "system_state.h"
#include "CRC32.h"

#if RUN_MS5611_MODULE == 1

void getPressure (void*)
{
  delay (123); // de-synchronize task start

  while (true) // re-initialization loop
    {
re_initialize:

      acquire_privileges ();
      I2C_Init (MS5611_I2C);
      drop_privileges();

      MS5611 ms5611_static (0xEE);
      bool static_ms5611_available = false;

      static_ms5611_available = ms5611_static.initialize ();
      if ( not static_ms5611_available)
	{
	  delay( 1000);
	  continue; // try re - initialization
	}

      value_and_timestamp_t static_pressure;

      for( synchronous_timer t (10); true; t.sync()) // measurement loop
	{
	  if ( static_ms5611_available)
	    if (ms5611_static.update () == false)
	      {
		update_system_state_clear(MS5611_STATIC_AVAILABLE);
		goto re_initialize;
	      }

	  t.sync ();

	  if ( static_ms5611_available)
	    if (ms5611_static.update () == false)
	      {
		update_system_state_clear(MS5611_STATIC_AVAILABLE);
		goto re_initialize;
	      }

	  update_system_state_set (MS5611_STATIC_AVAILABLE);

	  static_pressure.value = ms5611_static.get_pressure ();
	  static_pressure.timestamp = ms5611_static.get_timestamp();
	  sensor_temperature = ms5611_static.get_temperature ();
	  static_pressure_Q.send( static_pressure, 1);
	}
    }
}

RestrictedTask ms5611_reading (getPressure, "P_ABS", 256, 0, MS5611_PRIORITY + portPRIVILEGE_BIT);

#endif
