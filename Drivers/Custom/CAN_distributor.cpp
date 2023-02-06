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
Queue < CAN_packet> packet_q(3,"DIST_TST_Q");

void CAN_distribution_test( void *)
{
  {
    CAN_distributor_entry my_entry{ 0xffff, 0x13+6, &packet_q};
    subscribe_CAN_messages( my_entry);
  }

  while( true)
    {
      CAN_packet p;
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


