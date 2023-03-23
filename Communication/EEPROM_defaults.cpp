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
#include "EEPROM_defaults.h"

//! helper function, in use until manual configuration is implemented
void write_EEPROM_defaults( void)
{
  unsigned status;
  status = EEPROM_initialize();
  assert( status == false);

  status = lock_EEPROM( false);
  assert( status == false);

  for( unsigned index = 0; index < PERSISTENT_DATA_ENTRIES; ++index)
      status |= write_EEPROM_value( PERSISTENT_DATA[index].id, PERSISTENT_DATA[index].default_value);

  assert( status == false); // be sure there was no error

  status = lock_EEPROM( true);
  assert( status == false);
}
