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
#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "common.h"
#include "CAN_output.h"
#include "communicator.h"
#include "candriver.h"
#include "CAN_distributor.h"
#include "NMEA_format.h"

COMMON Queue<CANpacket> can_packet_q(10,"CAN_RX");

COMMON static float32_t latest_mc = 0.0, latest_bal = 0.0, latest_bugs = 0.0, latest_qnh = 0.0;
COMMON static bool new_mc = false, new_bal = false, new_bugs = false, new_qnh = false;

bool get_mc_updates(float32_t &value)
{
  if(new_mc){
      new_mc = false;
      value = latest_mc;
      return true;
  }
  return false;
}

bool get_bal_updates(float32_t &value)
{
  if (new_bal){
      new_bal = false;
      value = latest_bal;
      return true;
  }
  return false;
}

bool get_bugs_updates(float32_t &value)
{
  if (new_bugs){
       new_bugs = false;
       value = latest_bugs;
       return true;
    }
  return false;
}

bool get_qnh_updates(float32_t &value)
{
  if (new_qnh){
       new_qnh = false;
       value = latest_qnh;
       return true;
    }
  return false;
}

typedef struct
{
  float3vector acc_left;
  float3vector acc_right;
  float3vector acc_level;
} sensor_rotation_data;;

void
CAN_listener_task_runnable (void*)
{
  sensor_rotation_data calibration_data={0};
  CAN_distributor_entry my_entry
    { 0x040F, 0x0402, &can_packet_q }; // Listen for "Set System Wide Config Item" on CAN
  subscribe_CAN_messages (my_entry);

  my_entry.ID_value = 0x320;
  my_entry.ID_mask = 0x0fff;
  subscribe_CAN_messages (my_entry);

  CANpacket p;
  while (true)
    {
      if (true == can_packet_q.receive(p))

#if WITH_EXTERNAL_IMU

	switch (p.id)
	  {
	  case 0x118:
	    output_data.extra.gyro[0] = p.data_f[0];
	    output_data.extra.gyro[1] = p.data_f[1];
	    break;
	  case 0x119:
	    output_data.extra.gyro[2] = p.data_f[0];
	    break;
	  case 0x11a:
	    output_data.extra.acc[0] = p.data_f[0];
	    output_data.extra.acc[1] = p.data_f[1];
	    break;
	  case 0x11b:
	    output_data.extra.acc[2] = p.data_f[0];
	    output_data.extra.temperature = p.data_f[1];
	    break;
	  case 0x11c:
	    output_data.extra.mag[0] = p.data_f[0];
	    output_data.extra.mag[1] = p.data_f[1];
	    break;
	  case 0x11d:
	    output_data.extra.mag[2] = p.data_f[0];
	    break;
	  }

#endif

      if(( p.id & 0x40F) == 0x402)
        switch (p.data_h[0])
	  {
	  case SYSWIDECONFIG_ITEM_ID_MC:
	    latest_mc = p.data_f[1];
	    new_mc = true;
	    break;

	  case SYSWIDECONFIG_ITEM_ID_BALLAST:
	    latest_bal = p.data_f[1];
	    new_bal = true;
	    break;

	  case SYSWIDECONFIG_ITEM_ID_BUGS:
	    latest_bugs = p.data_f[1];
	    new_bugs = true;
	    break;

	  case SYSWIDECONFIG_ITEM_ID_QNH:
	    latest_qnh = p.data_f[1];
	    new_qnh = true;
	    break;
	  }
      if( p.id ==0x320)
        switch (p.data_b[1] - 1) // audio volume minus 1 -> 1, 2, 3, 4 are relevant
	  {
	  case CMD_MEASURE_LEFT & 0x0f:
	    communicator_command_queue.send( MEASURE_CALIB_LEFT, 1);
	    break;

	  case CMD_MEASURE_RIGHT & 0x0f:
	    communicator_command_queue.send( MEASURE_CALIB_RIGHT, 1);
	    break;

	  case CMD_MEASURE_LEVEL & 0x0f:
	    communicator_command_queue.send( MEASURE_CALIB_LEVEL, 1);
	    break;

	  case CMD_CALCULATE & 0x0f:
	    communicator_command_queue.send( SET_SENSOR_ROTATION, 1);
	    break;

	  default:
	    //0: volume_vario, 5: pilot_weight, 6: vario_mode_control not supported by PLARS Sentence
	    break;
	  }

    }
}

Task CAN_listener_task (CAN_listener_task_runnable, "CAN_RX");





