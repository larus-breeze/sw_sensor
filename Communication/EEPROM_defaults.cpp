/***********************************************************************//**
 * @file		EEPROM_defaults.cpp
 * @brief		Writes default data into the FLASH EEPROM-Emulation
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

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
#include "my_assert.h"
#include "NAV_tuning_parameters.h"
#include "embedded_math.h"

//! helper function, in use until manual configuration is implemented
void write_EEPROM_defaults( void)
{
  unsigned status;
  status = EEPROM_initialize();
  assert( status == false);

  status = lock_EEPROM( false);
  assert( status == false);

  status = 	    write_EEPROM_value( SENS_TILT_ROLL, 0.0f);
  status = status | write_EEPROM_value( SENS_TILT_NICK, 0.0f);
  status = status | write_EEPROM_value( SENS_TILT_YAW,  0.0f);

  status = status | write_EEPROM_value( ANT_BASELENGTH, 1.0f);
  status = status | write_EEPROM_value( ANT_SLAVE_DOWN, 0.0f);
  status = status | write_EEPROM_value( ANT_SLAVE_RIGHT, 0.0f);
  status = status | write_EEPROM_value( GNSS_CONFIGURATION, (float)GNSS_M9N);

  status = status | write_EEPROM_value( PITOT_OFFSET, 0.0f);
  status = status | write_EEPROM_value( PITOT_SPAN, 1.0f);
  status = status | write_EEPROM_value( QNH_OFFSET, 0.0f);

  status = status | write_EEPROM_value( MAG_X_OFF, 0.0f);
  status = status | write_EEPROM_value( MAG_X_SCALE, 1.0f);
  status = status | write_EEPROM_value( MAG_Y_OFF, 0.0f);
  status = status | write_EEPROM_value( MAG_Y_SCALE, 1.0f);
  status = status | write_EEPROM_value( MAG_Z_OFF, 0.0f);
  status = status | write_EEPROM_value( MAG_Z_SCALE, 1.0f);
  status = status | write_EEPROM_value( MAG_STD_DEVIATION, 0.009999f);
  status = status | write_EEPROM_value( MAG_AUTO_CALIB, 1.0f);
  status = status | write_EEPROM_value( MAG_EARTH_AUTO, 0.0f);

  // time constants
  status = status | write_EEPROM_value( VARIO_TC, DEFAULT_VARIO_TC);
  status = status | write_EEPROM_value( VARIO_INT_TC, DEFAULT_AVG_VARIO_TC);
  status = status | write_EEPROM_value( WIND_TC, DEFAULT_WIND_TC);
  status = status | write_EEPROM_value( MEAN_WIND_TC, DEFAULT_WIND_AVG_TC);
  status = status | write_EEPROM_value( VETF, DEFAULT_VETF);

  status = status | write_EEPROM_value( DECLINATION, 0.0f * M_PI_F / 180.0);
  status = status | write_EEPROM_value( INCLINATION, +65.5f * M_PI_F / 180.0);

  assert( status == false); // be sure there was no error

  status = lock_EEPROM( true);
  assert( status == false);
}
