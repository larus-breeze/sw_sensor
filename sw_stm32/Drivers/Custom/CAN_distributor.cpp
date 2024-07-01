/** *****************************************************************************
 * @file    	CAN_distributor.cpp
 * @brief   	implements a Publisher/Subscriber pattern for CAN packets
 * @author  	Dr. Klaus Schaefer
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

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
#include "candriver.h"
#include "CAN_distributor.h"

#define CAN_LIST_SIZE 10

COMMON CAN_distributor_entry CAN_distributor_list[CAN_LIST_SIZE];

bool subscribe_CAN_messages( const CAN_distributor_entry &that)
{
  for( unsigned i=0; i<CAN_LIST_SIZE; ++i)
    {
      if( CAN_distributor_list[i].queue == 0) // queue == 0 means: entry = empty
	{
	  CAN_distributor_list[i]=that;
	  return true;
	}
    }
  return false; // list already full
}

static inline void distribute_CAN_packet(const CANpacket &p)
{
  for(unsigned i=0; i<CAN_LIST_SIZE; ++i)
    {
      if( CAN_distributor_list[i].queue ==0) // end of list
	return;
      if( (p.id & CAN_distributor_list[i].ID_mask) == CAN_distributor_list[i].ID_value)
	{
	bool ok = CAN_distributor_list[i].queue->send( p, NO_WAIT);
	ASSERT( ok);
	}
    }
}

void CAN_RX_task_code (void*)
{
  CANpacket p;
  while (1)
    {
      CAN_driver.receive( p);
      distribute_CAN_packet(p);
    }
}

Task CAN_RX_task (CAN_RX_task_code, "CAN_RX");


#if RUN_CAN_DISTRIBUTION_TEST

unsigned CAN_packet_counter;
Queue < CANpacket> packet_q(3,"DIST_TST_Q");

void CAN_distribution_test( void *)
{
  {
    CAN_distributor_entry my_entry{ 0xffff, 0x13+6, &packet_q};
    subscribe_CAN_messages( my_entry);
  }

  while( true)
    {
      CANpacket p;
      packet_q.receive(p);
      ++CAN_packet_counter;
    }
}

Task CAN_distribution_tester(
    CAN_distribution_test,
    "CAN_DIST",
    configMINIMAL_STACK_SIZE,
    0,
    STANDARD_TASK_PRIORITY + 1
    );

#endif


