/**
 @file 		ms5611.cpp
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

#if RUN_MS5611_MODULE == 1

void getPressure (void*)
{
  delay (123); // de-synchronize task start

  while (true) // re-initialization loop
    {
      update_system_state_clear(MS5611_STATIC_AVAILABLE);

      acquire_privileges ();
      I2C_Init (MS5611_I2C);
      drop_privileges();

      MS5611 ms5611_static (0xEE);
      bool static_ms5611_available = false;

      static_ms5611_available = ms5611_static.initialize ();
      if (static_ms5611_available)
	update_system_state_set (MS5611_STATIC_AVAILABLE);
      else
	{
	  delay( 1000);
	  continue; // restart this task
	}

      for( synchronous_timer t (10); true; t.sync()) // measurement loop
	{
	  if ( static_ms5611_available)
	    if (ms5611_static.update () == false)
	      break;;

	  t.sync ();

	  if ( static_ms5611_available)
	    if (ms5611_static.update () == false)
	      break;;

	  if ( static_ms5611_available)
	    {
	      output_data.m.static_pressure = ms5611_static.get_pressure ();
	      output_data.m.static_sensor_temperature =
		  ms5611_static.get_temperature ();
	    }
	}
    }
}

RestrictedTask ms5611_reading (getPressure, "P_ABS", 256, 0, MS5611_PRIORITY + portPRIVILEGE_BIT);

#endif
