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
  while (true) // re-initialization loop
    {
      update_system_state_clear(MS5611_STATIC_AVAILABLE);
#if WITH_LOWCOST_SENSORS
      update_system_state_clear(MS5611_PITOT_AVAILABLE);
#endif
      delay (2);

      acquire_privileges ();
      I2C_Init (MS5611_I2C);
      drop_privileges();

      MS5611 ms5611_static (0xEE);
#if WITH_LOWCOST_SENSORS
      MS5611 ms5611_pitot (0xEC);  // Second ms5611 sensor on PCB.
#endif
      bool static_ms5611_available = false;

#if WITH_LOWCOST_SENSORS
      bool pitot_ms5611_available = false;
#endif

      static_ms5611_available = ms5611_static.initialize ();
      if (static_ms5611_available)
	update_system_state_set (MS5611_STATIC_AVAILABLE);

#if WITH_LOWCOST_SENSORS
      pitot_ms5611_available = ms5611_pitot.initialize ();
      if (pitot_ms5611_available)
	update_system_state_set (MS5611_PITOT_AVAILABLE);
#endif

      for( synchronous_timer t (10); true; t.sync()) // measurement loop
	{
	  if ( static_ms5611_available)
	    if (ms5611_static.update () == false)
	      break;;

#if WITH_LOWCOST_SENSORS
	  if ( pitot_ms5611_available)
	    if (ms5611_pitot.update () == false)
	      break;;
#endif
	  t.sync ();

	  if ( static_ms5611_available)
	    if (ms5611_static.update () == false)
	      break;;

#if WITH_LOWCOST_SENSORS
	  if (pitot_ms5611_available)
	    if (ms5611_pitot.update () == false)
	      break;;
#endif
	  if ( static_ms5611_available)
	    {
	      output_data.m.static_pressure = ms5611_static.get_pressure ();
	      output_data.m.static_sensor_temperature =
		  ms5611_static.get_temperature ();
	    }
#if WITH_LOWCOST_SENSORS
	  if (true == pitot_ms5611_available)
	    {
	      output_data.m.absolute_pressure = ms5611_pitot.get_pressure ();
	      output_data.m.absolute_sensor_temperature =
		  ms5611_pitot.get_temperature ();
	    }
#endif
	}
    }
}

RestrictedTask ms5611_reading (getPressure, "P_ABS", 256, 0, MS5611_PRIORITY + portPRIVILEGE_BIT);

#endif
