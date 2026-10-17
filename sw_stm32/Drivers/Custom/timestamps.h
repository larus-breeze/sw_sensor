/***********************************************************************//**
 * @file		timestamps.h
 * @brief		precise acquisition times of the observations (#263)
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

#ifndef DRIVERS_CUSTOM_TIMESTAMPS_H_
#define DRIVERS_CUSTOM_TIMESTAMPS_H_

#include <stdint.h>
#include "data_structures.h" // timestamps_t, TIMESTAMP_TICKS_PER_SECOND

//! acquisition times, logged once per 100 Hz cycle as TIMESTAMPS record
extern timestamps_t timestamps;

//! set up the timestamp counter (TIM4 + TIM5) and its input captures, privileged
void timestamps_init (void);

//! current timestamp counter value, privileged (interrupts or privileged code)
uint64_t timestamp_now_privileged (void);

//! timestamp of the latest IMU data-ready edge, called from its EXTI interrupt
uint64_t timestamp_IMU_data_ready_privileged (void);

#endif /* DRIVERS_CUSTOM_TIMESTAMPS_H_ */
