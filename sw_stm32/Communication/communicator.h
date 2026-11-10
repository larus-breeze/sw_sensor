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
#include "reminder_flag.h"
#include "communicator_command.h"

extern D_GNSS_coordinates_t coordinates;
#if SUPPORT_D_GNSS_ACCURACY
extern D_GNSS_accuracy_t accuracy;
#endif
extern measurement_data_t observations;
extern float3vector external_magnetometer;
extern state_vector_t state_vector;

extern RestrictedTask communicator_task;
extern Queue < communicator_command_t> communicator_command_queue;

static inline bool is_airborne (void)
{
  return state_vector.flight_mode != ON_GROUND;
}

#endif /* COMMUNICATOR_H_ */
