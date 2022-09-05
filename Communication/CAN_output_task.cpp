#include "system_configuration.h"
#include "common.h"
#include "CAN_output.h"

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
