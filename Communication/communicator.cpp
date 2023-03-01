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
#include "CAN_output_task.h"
#include "D_GNSS_driver.h"
#include "GNSS_driver.h"
#include "CAN_distributor.h"
#include "uSD_handler.h"
#include "compass_ground_calibration.h"
#include "persistent_data.h"
#include "EEPROM_defaults.h"

extern "C" void sync_logger (void);

COMMON Semaphore setup_file_handling_completed(1,0,"SETUP");

COMMON output_data_t __ALIGNED(1024) output_data = { 0 };
COMMON GNSS_type GNSS (output_data.c);

extern RestrictedTask NMEA_task;
extern RestrictedTask communicator_task;

static ROM bool TRUE=true;
static ROM bool FALSE=false;

void communicator_runnable (void*)
{
  // wait until configuration file read
  setup_file_handling_completed.wait();

  if( ! all_EEPROM_parameters_existing())
      write_EEPROM_defaults();

  organizer_t organizer;
  organizer.initialize_before_measurement();

  uint16_t GNSS_count = 0;

#if WITH_DENSITY_DATA
  uint16_t air_density_sensor_counter = 0;

  Queue<CANpacket> air_density_sensor_Q (2);

    {
      CAN_distributor_entry cde =
	{ 0xffff, 0x120, &air_density_sensor_Q };
      bool result = subscribe_CAN_messages (cde);
      ASSERT(result);
    }
#endif

  GNSS_configration_t GNSS_configuration =
      (GNSS_configration_t) round(configuration (GNSS_CONFIGURATION));

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

	organizer.update_GNSS_data (output_data.c);
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

	organizer.update_GNSS_data (output_data.c);
	GNSS_new_data_ready = false;
      }
      break;
    case GNSS_F9P_F9P: // no extra task for 2nd GNSS module
      {
	Task usart3_task (USART_3_runnable, "GNSS", 256, (void *)&TRUE, STANDARD_TASK_PRIORITY+1);

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);

	organizer.update_GNSS_data (output_data.c);
	GNSS_new_data_ready = false;
      }
      break;
    default:
      ASSERT(false);
    }

  for( int i=0; i<100; ++i) // wait 1 s until measurement stable
    notify_take (true);

  organizer.initialize_after_first_measurement(output_data);

  NMEA_task.resume();

  unsigned synchronizer_10Hz = 10; // re-sampling 100Hz -> 10Hz

  communicator_task.set_priority( COMMUNICATOR_PRIORITY); // lift priority

  compass_ground_calibration_t compass_ground_calibration;
  unsigned magnetic_ground_calibrator_countdown = 100 * 60 * 2; // 2 minutes

  // this is the MAIN data acquisition and processing loop
  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

      // if we are in magnetic calibration mode:
      // do this here as a side-job and switch off it's flag at the end
      if( magnetic_gound_calibration && (magnetic_ground_calibrator_countdown > 0))
	{
	  --magnetic_ground_calibrator_countdown;
	  compass_ground_calibration.feed(output_data.m.mag);
	  if( 0 == magnetic_ground_calibrator_countdown)
	    {
	      compass_calibration_t new_calibration;
	      compass_ground_calibration.get_calibration_result(new_calibration.calibration);
	      new_calibration.calibration_done = true;
	      new_calibration.write_into_EEPROM();
	      magnetic_calibration_done.signal();
	      magnetic_gound_calibration = false;
	    }
	}

      if (GNSS_new_data_ready) // triggered after 100 or 150ms, GNSS-dependent
	{
	  organizer.update_GNSS_data (output_data.c);
	  GNSS_new_data_ready = false;
	}

      organizer.on_new_pressure_data(output_data); // todo check this update rate
      organizer.update_every_10ms(output_data);

      --synchronizer_10Hz;
      if( synchronizer_10Hz == 0)
	{
	  organizer.update_every_100ms (output_data);
	  synchronizer_10Hz = 10;
	}

      if(
	  (((GNSS_configuration == GNSS_F9P_F9H) || (GNSS_configuration == GNSS_F9P_F9P))
	      && (output_data.c.sat_fix_type == (SAT_HEADING | SAT_FIX)))
	  ||
	  ((GNSS_configuration == GNSS_M9N)
	      && (output_data.c.sat_fix_type == SAT_FIX))
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

#if WITH_DENSITY_DATA
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
#endif

	}

      organizer.report_data ( output_data);
      if( logger_is_enabled)
	sync_logger (); // kick logger @ 100 Hz
    }
}

#define STACKSIZE 1024 // in 32bit words
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  { communicator_runnable, "COM",
  STACKSIZE, 0,
  COMMUNICATOR_START_PRIORITY, stack_buffer,
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
