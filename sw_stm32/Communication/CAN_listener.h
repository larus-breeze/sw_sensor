/***********************************************************************//**
 * @file		CAN_listener.cpp
 * @brief		CAN listener for processing incoming CAN Frames
 * @author		Maximilian Betz
 * @copyright 		Copyright 2024. All rights reserved.
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
#ifndef CAN_OUTPUT_TASK_H_
#define CAN_OUTPUT_TASK_H_

#include "FreeRTOS_wrapper.h"

extern Task CAN_listener_task;

bool get_mc_updates(float32_t &value);
bool get_bal_updates(float32_t &value);
bool get_bugs_updates(float32_t &value);
bool get_qnh_updates(float32_t &value);

#endif /* CAN_OUTPUT_TASK_H_ */
