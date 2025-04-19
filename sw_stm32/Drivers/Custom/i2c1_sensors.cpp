/**
 * @file 	pitot_sensor.cpp
 * @brief 	HCLA pressure sensor driver
 * @author: 	Klaus Schaefer  Low Cost ACC /MAG added by Maximilian Betz
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

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
#include "common.h"
#include "communicator.h"
#include "system_state.h"

#define I2C_ADDRESS (0x28<<1) // 7 bits left-adjusted
/*
 * Bereich: 80% von 16384 counts auf 1PSI verteilt
 * ergibt 6895 Pa / 13107 counts = 0.5261 Pa / count
 */

#define SPAN 0.5261f
#define OFFSET 1638 // exakt 0.1 * 16384

static void runnable (void*)
{
restart:

  uint8_t data[4];

  acquire_privileges ();
  I2C_Init (&hi2c1);
  drop_privileges();

#if RUN_PITOT_MODULE
  if (I2C_OK == I2C_Read (&hi2c1, I2C_ADDRESS, data, 2))
    update_system_state_set (PITOT_SENSOR_AVAILABLE);

  delay (10); // wait for "next measurement available"
#endif
  for (synchronous_timer t (10); true; t.sync ())
    {
#if RUN_PITOT_MODULE
      if ((I2C_OK == I2C_Read (&hi2c1, I2C_ADDRESS, data, 2)
	  && (data[0] & 0xC0) == 0)) // no error flags read
	{
	  uint16_t raw_data = (data[0] << 8) | data[1];
	  output_data.m.pitot_pressure = ((float) (raw_data - OFFSET) * SPAN);
	  update_system_state_set (PITOT_SENSOR_AVAILABLE);
	}
      else
	{
	  output_data.m.pitot_pressure = 0.0f;
	  update_system_state_clear (PITOT_SENSOR_AVAILABLE);
	  goto restart;
	}
#endif
    }
}

RestrictedTask pitot_reading (runnable, "PITOT", 512, 0,
			      PITOT_PRIORITY + portPRIVILEGE_BIT);

