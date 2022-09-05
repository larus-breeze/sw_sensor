/** ***********************************************************************
 * @file		communicator.cpp
 * @brief		talk to sensors and synchronize output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#include "system_configuration.h"
#include "main.h"

#include <FreeRTOS_wrapper.h>
#include "navigator.h"
#include "flight_observer.h"
#include "serial_io.h"
#include "NMEA_format.h"
#include "common.h"
#include "organizer.h"
#include "CAN_output.h"
#include "D_GNSS_driver.h"
#include "GNSS_driver.h"
#include "CAN_distributor.h"

extern "C" void sync_logger (void);

COMMON Semaphore setup_file_handling_completed;

COMMON output_data_t __ALIGNED(1024) output_data = { 0 };
COMMON GNSS_type GNSS (output_data.c);

extern RestrictedTask NMEA_task;

static ROM bool TRUE=true;
static ROM bool FALSE=true;

void communicator_runnable (void*)
{
  organizer_t organizer;

  // wait until configuration file read
  setup_file_handling_completed.wait();

  organizer.initialize_before_measurement();

  uint16_t air_density_sensor_counter = 0;
  uint16_t GNSS_count = 0;

  Queue<CANpacket> air_density_sensor_Q (2);

    {
      CAN_distributor_entry cde =
	{ 0xffff, 0x120, &air_density_sensor_Q };
      bool result = subscribe_CAN_messages (cde);
      ASSERT(result);
    }

  GNSS_configration_t GNSS_configuration =
      (GNSS_configration_t) ROUND (configuration (GNSS_CONFIGURATION));

  uint8_t count_10Hz = 1; // de-synchronize CAN output by 1 cycle

  GNSS.coordinates.sat_fix_type = SAT_FIX_NONE; // just to be sure

  switch (GNSS_configuration)
    {
    case GNSS_NONE:
      break;
    case GNSS_M9N:
      {
	Task usart3_task (USART_3_runnable, "GNSS", 256, (void *)&FALSE, STANDARD_TASK_PRIORITY+1);

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);

	organizer.update_GNSS (GNSS.coordinates);
	GNSS_new_data_ready = false;
      }
      break;
    case GNSS_F9P_F9H: // extra task for 2nd GNSS module required
      {
	  {
	    Task usart3_task (USART_3_runnable, "GNSS", 256, (void *)&FALSE, STANDARD_TASK_PRIORITY+1);

	    Task usart4_task (USART_4_runnable, "D-GNSS", 256, 0, STANDARD_TASK_PRIORITY + 1);
	  }

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);

	organizer.update_GNSS (GNSS.coordinates);
	GNSS_new_data_ready = false;
      }
      break;
    case GNSS_F9P_F9P: // no extra task for 2nd GNSS module
      {
	Task usart3_task (USART_3_runnable, "GNSS", 256, (void *)&TRUE, STANDARD_TASK_PRIORITY+1);

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);

	organizer.update_GNSS (GNSS.coordinates);
	GNSS_new_data_ready = false;
      }
      break;
    default:
      ASSERT(false);
    }

  for( int i=0; i<100; ++i) // wait 1 s until measurement stable
    notify_take (true);

  organizer.initialize_after_first_measurement();

  NMEA_task.resume();

  // this is the MAIN data acquisition and processing loop
  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

      if (GNSS_new_data_ready) // triggered at 10 Hz by GNSS
	{
	  organizer.update_GNSS (GNSS.coordinates);
	  GNSS_new_data_ready = false;
	}

      organizer.on_new_pressure_data(); // todo check this update rate
      organizer.update_IMU();

      if(
	  (((GNSS_configuration == GNSS_F9P_F9H) || (GNSS_configuration == GNSS_F9P_F9P))
	      && (GNSS.fix_type & SAT_HEADING))
	  ||
	  ((GNSS_configuration == GNSS_M9N)
	      && (GNSS.fix_type & SAT_FIX))
	)
	{
	  ++GNSS_count;
	  HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
	      (GNSS_count & 0xff) > 127 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
      else
	HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);

      if (++count_10Hz >= 10) // resample 100Hz -> 10Hz
	{
	  count_10Hz = 0;
	  trigger_CAN ();

	  // take care of ambient air data if sensor reports any
	  CANpacket p;
	  if( air_density_sensor_Q.receive( p, 0) && p.dlc == 8)
	    {
	      air_density_sensor_counter = 0;
	      organizer.set_density_data(p.data_f[0], p.data_f[1]);
	      update_system_state_set( AIR_SENSOR_AVAILABLE);
	      output_data.m.outside_air_temperature = p.data_f[0];
	      output_data.m.outside_air_humidity = p.data_f[1];
	    }
	  else
	    {
	      if( air_density_sensor_counter < 10)
		  ++air_density_sensor_counter;
	      else
		{
		  organizer.disregard_density_data();
		  update_system_state_clear( AIR_SENSOR_AVAILABLE);
		  output_data.m.outside_air_humidity = -1.0f; // means: disregard humidity and temperature
		  output_data.m.outside_air_temperature = ZERO;
		}
	    }
	}

      organizer.report_data ( output_data);
      sync_logger (); // kick logger @ 100 Hz
    }
}

#define STACKSIZE 1024 // in 32bit words
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  { communicator_runnable, "COM",
  STACKSIZE, 0,
  COMMUNICATOR_PRIORITY, stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { (void*) 0x80f8000, 0x08000, portMPU_REGION_READ_WRITE }, // EEPROM access for MAG calib.
      { 0, 0, 0 } } };

COMMON RestrictedTask communicator_task (p);

void
sync_communicator (void) // global synchronization service function
{
  communicator_task.notify_give ();
}
