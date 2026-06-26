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
#include "watchdog_handler.h"
#include "system_state.h"
#include "communicator.h"
#include "uSD_handler.h"
#include "signal_flight_event.h"

#define CAN_Id_Send_Config_Value 0x12f

ROM EEPROM_PARAMETER_ID parameter_list[] =
    {
	SENS_TILT_ROLL,
	SENS_TILT_PITCH,
	SENS_TILT_YAW,
	PITOT_OFFSET,
	PITOT_SPAN,
	QNH_OFFSET,
	MAG_AUTO_CALIB,
	VARIO_TC,
	VARIO_INT_TC,
	WIND_TC,
	MEAN_WIND_TC,
	GNSS_CONFIGURATION,
	ANT_BASELENGTH,
	ANT_SLAVE_DOWN,
	ANT_SLAVE_RIGHT,
	VARIO_P_TC,
	HORIZON
    };

#define PARAMETER_LIST_LENGTH (sizeof( parameter_list) / sizeof(EEPROM_PARAMETER_ID))
#define PARAMETER_OFFSET 0x2000

COMMON Queue< parameter_setting_message> parameter_setting_queue(3);

//! read or write EEPROM value
//! @return true if value read successfully
bool EEPROM_config_read_write( const CANpacket & p, float & return_value)
{
  uint16_t command = p.data_h[0];

  if(( command < PARAMETER_OFFSET) || (command >= ( PARAMETER_OFFSET + PARAMETER_LIST_LENGTH)))
    return false; // nothing for us ...

  EEPROM_PARAMETER_ID id = parameter_list[ command - PARAMETER_OFFSET];

  switch( p.data_b[2])
  {
    case 0: // get value, return true on success
      {
	if( read_EEPROM_value( id, return_value)) // if error
	  return false;
	return true;
      }

    case 1: // set value
      {
	float value = p.data_f[1];

	bool success = write_EEPROM_value( id, value);
	signal_logger_event( EEPROM_CONFIGURATION_CHANGED | (success ? (id<<8) + 0x10000 : (id<<8)) );

	if( success)
	  // we need to reset the algorithms because of a significant change
	  {
	    switch( id)
	    {
	      case VARIO_TC:
	      case VARIO_P_TC:
	      case VARIO_INT_TC:
	      case WIND_TC:
		  communicator_command_queue.send( TIME_CONSTANT_CHANGED, 1);
		break;
	      case ANT_BASELENGTH:
	      case ANT_SLAVE_DOWN:
	      case ANT_SLAVE_RIGHT:
		  communicator_command_queue.send( GNSS_CONFIG_CHANGED, 1);
		break;
	      case PITOT_OFFSET:
	      case PITOT_SPAN:
	      case QNH_OFFSET:
		  communicator_command_queue.send( TUNE_PRESSURE_GAUGES, 1);
		break;
	      case HORIZON:
		  communicator_command_queue.send( HORIZON_LOCK_CHANGED, 1);
		break;
	      default:
		break;
	    }
	  }

	return false; // report "nothing read"
      }
      break;

    default:
      return false;// error, ignore request !
  }
}

#define XTRA_ACC_SCALE 2.39215e-3f
#define XTRA_GYRO_SCALE 0.000076358f
#define XTRA_MAG_SCALE 1.22e-4f;

inline float TEMP_CONVERSION( int16_t x)
{
  return ((float)x / 256.0f + 25.0f);
}

COMMON Queue<CANpacket> can_packet_q(10,"CAN_RX");

void CAN_listener_task_runnable (void*)
{
  TickType_t magnetometer_last_heard = 0;
  parameter_setting_message message;

  CAN_distributor_entry my_entry
    { 0x040F, 0x0402, &can_packet_q }; // Listen for "Set System Wide Config Item" on CAN
  subscribe_CAN_messages (my_entry);

  my_entry.ID_value = 0x070;
  my_entry.ID_mask = 0x0fff;
  subscribe_CAN_messages (my_entry);

  CANpacket p;
  while (true)
    {

      bool rx_ed = can_packet_q.receive (p, 100);
      if (rx_ed)
	{
	  if ((p.id == 0x070) && (p.dlc == 6))
	    {
	      external_magnetometer[0] = (float32_t) (p.data_sh[0])
		  * 0.01333333f; // 75LSB / uTesla
	      external_magnetometer[1] = (float32_t) (p.data_sh[1])
		  * 0.01333333f;
	      external_magnetometer[2] = (float32_t) (p.data_sh[2])
		  * 0.01333333f;
	      update_system_state_set (EXTERNAL_MAGNETOMETER_AVAILABLE);
	      magnetometer_last_heard = xTaskGetTickCount ();
	    }
	  else if ((p.id & 0x40F) == 0x402) // = "set system wide config item"
	    switch (p.data_h[0])
	      {
	      case SYSWIDECONFIG_ITEM_ID_MC:
		message.value = CLIP( p.data_f[1], 0.0f, 5.0f);
		message.type = MC_CREADY;
		parameter_setting_queue.send( message, 1);
		break;

	      case SYSWIDECONFIG_ITEM_ID_BUGS:
		message.value = p.data_f[1];
		message.type = BUGS;
		parameter_setting_queue.send( message, 1);
		break;

	      case SYSWIDECONFIG_ITEM_ID_QNH:
		message.value = CLIP( p.data_f[1], 87000.0f, 110000.0f);
		message.type = QNH;
		parameter_setting_queue.send( message, 1);
		break;

	      case SYSWIDECONFIG_ITEM_ID_VARIO_MODE:
		message.value = (float) (p.data_b[2]);
		message.type = VARIO_MODE;
		parameter_setting_queue.send( message, 1);
		break;

	      case SYSWIDECONFIG_ITEM_ID_BALLAST_FRACTION:
		message.value = (float) (p.data_f[1]);
		message.type = BALLAST;
		parameter_setting_queue.send( message, 1);
		break;

	      case CMD_MEASURE_LEFT:
		communicator_command_queue.send (MEASURE_CALIB_LEFT, 1);
		break;

	      case CMD_MEASURE_RIGHT:
		communicator_command_queue.send (MEASURE_CALIB_RIGHT, 1);
		break;

	      case CMD_MEASURE_LEVEL:
		communicator_command_queue.send (MEASURE_CALIB_LEVEL, 1);
		break;

	      case CMD_CALCULATE:
		communicator_command_queue.send (SET_SENSOR_ROTATION, 1);
		break;

	      case CMD_TUNE:
#if RUN_FLASH_WRITE_TESTER
extern Semaphore trigger_flash_fill;
		    trigger_flash_fill.signal();
#else
		communicator_command_queue.send (FINE_TUNE_CALIB, 1);
#endif
		break;
	      case CMD_RESET_SENSOR:
#if CRASFILE_ON_USER_RESET == 0
		    user_initiated_reset = true;
	#endif
		ASSERT(false); // trigger exception that way
//		perform_after_landing_actions.set(); // todo patch
		break;

	      default: // try to interpret the command as "set" or "get" value
		{
		  // config parameter range check
		  uint16_t command = p.data_h[0];
		  if(( command < PARAMETER_OFFSET) || (command >= ( PARAMETER_OFFSET + PARAMETER_LIST_LENGTH)))
		    break; // ignore invalid parameter ID

		  float value;
		  if (p.data_b[2] == 1) // if it is a "write"
		    {
		      (void) EEPROM_config_read_write (p, value);
		    }
		  else
		    {
		      bool read_successful = EEPROM_config_read_write (p,value);

		      if( not read_successful)
			return; // ignore request if no value in EEPROM

		      CANpacket txp ( CAN_Id_Send_Config_Value, 8);
		      txp.data_w[0] = p.data_h[0]; // the ID we have received
		      txp.data_f[1] = value;
		      (void)CAN_enqueue (txp, 1); // ignore CAN buffer full condition
		    }

		  break;
		}
	      }
	}
      else
	{
	  if ( xTaskGetTickCount () - magnetometer_last_heard > 100)
	    update_system_state_clear (EXTERNAL_MAGNETOMETER_AVAILABLE);
	}
    }
}

static ROM TaskParameters_t p =
  {
      CAN_listener_task_runnable,
      "CAN_RX",
      256,
      0,
      WATCHDOG_TASK_PRIORITY -1,
      0,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { (void *)0x080C0000, 0x00040000, portMPU_REGION_READ_WRITE}, // EEPROM
      { 0, 0, 0 }
    }
  };

COMMON RestrictedTask CAN_listener_task (p);





