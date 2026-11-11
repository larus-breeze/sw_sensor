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
#include <FreeRTOS_wrapper.h>

extern D_GNSS_coordinates_t coordinates;
#if SUPPORT_D_GNSS_ACCURACY
extern D_GNSS_accuracy_t accuracy;
#endif
extern Queue < magnetometer_data_timed_t> external_magnetometer_data_Q;
extern Queue <imu_data_timed_t> imu_data_Q;
extern Queue <value_and_timestamp_t> static_pressure_Q;

extern state_vector_t state_vector;

extern RestrictedTask communicator_task;
extern Queue < communicator_command_t> communicator_command_queue;
extern int32_t clock_TOW_offset;
extern float32_t supply_voltage;
extern float32_t pitot_pressure;
extern float32_t sensor_temperature;
extern measurement_data_timed_t observations;

uint64_t getTime_usec(void);

inline int32_t get_TOW( void)
{
  int64_t time = getTime_usec();
  time /= 1000; // -> ms
  return (int32_t)time - clock_TOW_offset;
}

inline int32_t get_TOW_privileged( void)
{
  int64_t time = getTime_usec_privileged();
  time /= 1000; // -> ms
  return (int32_t)time - clock_TOW_offset;
}

inline void set_TOW( int32_t time_TOW_now)
{
  int64_t time = getTime_usec();
  time /= 1000; // -> ms
  clock_TOW_offset = (int32_t)time - time_TOW_now;
}

static inline bool is_airborne (void)
{
  return state_vector.flight_mode != ON_GROUND;
}

#endif /* COMMUNICATOR_H_ */
