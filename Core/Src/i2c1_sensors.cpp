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
#include "fxos8700cq.h"
#include "communicator.h"
#include "system_state.h"

#define I2C_ADDRESS (0x28<<1) // 7 bits left-adjusted
/*
 * Bereich: 80% von 16384 counts auf 1PSI verteilt
 * ergibt 6895 Pa / 13107 counts = 0.5261 Pa / count
 */

#define SPAN 0.5261f
#define OFFSET 1638 // exakt 0.1 * 16384

static void runnable( void *)
{
	uint8_t data[4];

	I2C_Init( &hi2c1);
	drop_privileges();

#if RUN_FXOS8700
	float xyz_acc[] = { 0, 0, 0 };
	float xyz_mag[] = { 0, 0, 0 };

	bool fxos8700_available = false;

	uint32_t retries = 10;
	while(retries--)
	{
		fxos8700_available = FXOS8700_Initialize(ACCEL_RANGE_4G);
		if (true == fxos8700_available)
		  {
		  update_system_state_set( FXOS_SENSOR_AVAILABLE);
		  break;
		  }
	}
#endif
#if RUN_PITOT_MODULE
	if(I2C_OK == I2C_Read( &hi2c1, I2C_ADDRESS, data, 2))
	    update_system_state_set( PITOT_SENSOR_AVAILABLE);

	delay( 10); // wait for "next measurement available"
#endif
	for( synchronous_timer t(10); true; t.sync())
	{
#if RUN_PITOT_MODULE
		if(I2C_OK == I2C_Read( &hi2c1, I2C_ADDRESS, data, 2))
		{
			ASSERT(( data[0] & 0xC0)==0); 			// no error flags !
			uint16_t raw_data = (data[0] << 8) | data[1];
			output_data.m.pitot_pressure = ((float)( raw_data - OFFSET) * SPAN);		
		}
		else
		{
			//TODO: log Sensor read error. Shall we set Pitot pressure here to 0 to ensure
			// faulty reading does not cause a wrong air speed?
		}
#endif
#if RUN_FXOS8700
		if(true == fxos8700_available)  // Only read value if sensor is available
		{
			if (true == FXOS8700_get(xyz_acc, xyz_mag))
			{
				for (int i = 0; i < 3; i++)
				{
					output_data.m.lowcost_acc[i] = xyz_acc[i];
					output_data.m.lowcost_mag[i] = xyz_mag[i];
				}
			}
			else
			{
				//TODO: log Sensor read error.
			}

		}
#endif
	}
}

RestrictedTask pitot_reading ( runnable, "PITOT", 512, 0, PITOT_PRIORITY + portPRIVILEGE_BIT);


