/***********************************************************************//**
 * @file		CAN_output_task.cpp
 * @brief		CAN output manager
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
#include "FreeRTOS_wrapper.h"
#include "CAN_output.h"
#include "communicator.h"

void CAN_task_runnable( void *)
{
  delay(5000); // allow data acquisition setup
  while( true)
    {
      notify_take();
      CAN_output( output_data);
    }
}

COMMON RestrictedTask CAN_task( CAN_task_runnable, "CAN", 256, 0, CAN_PRIORITY);
