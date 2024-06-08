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


#define MAX_LEN 100
COMMON char rxNMEASentence[MAX_LEN];
COMMON int PLARScnt = 0;

void NMEA_listener_task_runnable( void *)
{
  delay(5000); // allow data acquisition setup
  char rxByte;
  int i = 0;
  int len = 0;

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
		  len = i + 2;
		}

	      if (i == len)
		{
		  rxNMEASentence[i] = 0;

		  if(true == NMEA_checksum(rxNMEASentence))
		    {
		      if (strncmp(rxNMEASentence,"$PLARS,H,MC,",12) == 0)
			{
			  PLARScnt++;
			  //$PLARS,H,MC,2.1*1B
			}
		      else if (strncmp(rxNMEASentence,"$PLARS,H,BAL,",13) == 0)
			{
			  PLARScnt++;
			  //$PLARS,H,BAL,1.000*68
			}
		      else if (strncmp(rxNMEASentence,"$PLARS,H,BUGS,",14) == 0)
			{
			  PLARScnt++;
			  //$PLARS,H,BUGS,0*0B
			}
		      else if (strncmp(rxNMEASentence,"$PLARS,H,QNH,",13) == 0)
			{
			  PLARScnt++;
			  //$PLARS,H,QNH,1031.4*76
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
Task NMEA_LISTENER_H_ (NMEA_listener_task_runnable, "NMEAIN");





