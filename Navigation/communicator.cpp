/** ***********************************************************************
 * @file		communicator.cpp
 * @brief		talk to sensors and synchronize output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#include "system_configuration.h"

#if RUN_COMMUNICATOR == 1

#include <FreeRTOS_wrapper.h>
#include "navigator.h"
#include "flight_observer.h"
#include "serial_io.h"
#include "NMEA_format.h"
#include "common.h"
#include "CAN_output.h"

COMMON output_data_t __ALIGNED(1024) output_data =  { 0 };
COMMON GNSS_type GNSS (output_data.c);

void
communicator_runnable (void*)
{
  navigator_t navigator;

#if RUN_CAN_OUTPUT == 1
  uint8_t count_10Hz = 1; // de-synchronize CAN output by 1 cycle
#endif

  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

      navigator.update_pabs (output_data.m.static_pressure);
      navigator.update_pitot (output_data.m.pitot_pressure);

      if (GNSS_new_data_ready) // triggered at 10 Hz by GNSS
	{
	  GNSS_new_data_ready = false;
	  navigator.update_GNSS( GNSS.coordinates);
	}

      navigator.update_IMU (output_data.m.acc, output_data.m.mag, output_data.m.gyro);

      navigator.report_data (output_data);

#if RUN_CAN_OUTPUT == 1
      if (++count_10Hz >= 10)
	{
	  count_10Hz = 0;
	  trigger_CAN (); // todo alle abtastraten checken !
	}
#endif
    }
}

#define STACKSIZE 512 // in 32bit words
static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static TaskParameters_t p =
  {
      communicator_runnable,
      "COM",
      STACKSIZE,
      0,
      COMMUNICATOR_PRIORITY,
      stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 },
      { 0, 0, 0 }
    }
  };

COMMON RestrictedTask communicator_task (p);

void
sync_communicator (void)
{
  communicator_task.notify_give ();
}
#endif
