/***********************************************************************//**
 * @file		NMEA_listener.cpp
 * @brief		NMEA listener for processing incoming NMEA Sentences
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
#include "communicator.h"
#include "NMEA_format.h"
#include "usart_1_driver.h"
#include "ascii_support.h"
#include "generic_CAN_driver.h"
#include "CAN_output.h"

#define MAX_LEN 40
COMMON char rxNMEASentence[MAX_LEN];
COMMON int PLARScnt = 0;
COMMON Queue <CANpacket> MC_et_al_queue(2);

bool CAN_gateway_poll( CANpacket &p, unsigned max_wait)
   {
      return MC_et_al_queue.receive( p, max_wait);
   }

void NMEA_listener_task_runnable( void *)
{
  delay(5000); // allow data acquisition setup
  char rxByte;
  int i = 0;
  int len = 0;
  float value = 0.0f;
  char *ptr = NULL;
  CANpacket can_packet;

  can_packet.id = 0x522;  // static id as the sensor does not use dynamic addressing.
  can_packet.dlc = 8;

  while( true)
    {
      if(true == UART1_Receive((uint8_t*)&rxByte, portMAX_DELAY))
	{
	  if ('$' == rxByte)
	    {
	      i=0;
	    }
	  if (i < MAX_LEN)
	    {
	      rxNMEASentence[i] = rxByte;
	      i++;

	      if ('*' == rxByte)
		{
		  len = i + 2;  // Two checksum bytes after *
		}

	      if (i == len)
		{
		  // NMEA sentence is complete add string termination.
		  rxNMEASentence[i] = 0;

		  if(true == NMEA_checksum(rxNMEASentence))
		    {
		      rxNMEASentence[len-2] = 0; // Cut the checksum from the sentence for ASCII parsing

		      if (strncmp(rxNMEASentence,"$PLARS,H,MC,",12) == 0)
			{
			  ptr = &rxNMEASentence[12];
			  value = my_atof(ptr);
			  can_packet.data_h[0] = SYSWIDECONFIG_ITEM_ID_MC;
			  can_packet.data_h[1] = 0;
			  can_packet.data_f[1] = value;
			  MC_et_al_queue.send( can_packet, portMAX_DELAY);
			}
		      else if (strncmp(rxNMEASentence,"$PLARS,H,BAL,",13) == 0)
			{
			  ptr = &rxNMEASentence[13];
			  value = my_atof(ptr);
			  can_packet.data_h[0] = SYSWIDECONFIG_ITEM_ID_BALLAST;
			  can_packet.data_h[1] = 0;
			  can_packet.data_f[1] = value;
			  MC_et_al_queue.send( can_packet, portMAX_DELAY);
			}
		      else if (strncmp(rxNMEASentence,"$PLARS,H,BUGS,",14) == 0)
			{
			  ptr = &rxNMEASentence[14];
			  value = my_atof(ptr);
			  can_packet.data_h[0] = SYSWIDECONFIG_ITEM_ID_BUGS;
			  can_packet.data_h[1] = 0;
			  can_packet.data_f[1] = value;
			  MC_et_al_queue.send( can_packet, portMAX_DELAY);
			}
		      else if (strncmp(rxNMEASentence,"$PLARS,H,QNH,",13) == 0)
			{
			  ptr = &rxNMEASentence[13];
			  value = my_atof(ptr);
			  can_packet.data_h[0] = SYSWIDECONFIG_ITEM_ID_QNH;
			  can_packet.data_h[1] = 0;
			  can_packet.data_f[1] = value;
			  MC_et_al_queue.send( can_packet, portMAX_DELAY);
			}
		    }
		  i = 0;
		  len = 0;
		}
	    }
	  else
	    {
	      i = 0;
	      len = 0;
	    }
	}
      else
	{
	  i = 0;
	  len = 0;
	  vTaskDelay(100);
	}
    }
}
Task NMEA_listener_task (NMEA_listener_task_runnable, "NMEA_IN");





