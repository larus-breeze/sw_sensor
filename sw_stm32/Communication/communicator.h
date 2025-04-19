/***********************************************************************//**
 * @file		communicator.h
 * @brief		Main module for data acquisition and signal output
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
#ifndef COMMUNICATOR_H_
#define COMMUNICATOR_H_

#include "data_structures.h"

typedef enum
{
  NO_COMMAND,
  MEASURE_CALIB_LEFT,
  MEASURE_CALIB_RIGHT,
  MEASURE_CALIB_LEVEL,
  SET_SENSOR_ROTATION,
  FINE_TUNE_CALIB,
  SOME_EEPROM_VALUE_HAS_CHANGED
} communicator_command_t;

extern output_data_t output_data;
extern RestrictedTask communicator_task;
extern Queue < communicator_command_t> communicator_command_queue;

#endif /* COMMUNICATOR_H_ */
