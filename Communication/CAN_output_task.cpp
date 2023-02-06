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
