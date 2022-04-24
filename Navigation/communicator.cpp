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
#include "CAN_output.h"
#include "usart3_driver.h"
#include "usart4_driver.h"
#include "CAN_distributor.h"

void sync_logger (void);

COMMON Semaphore setup_file_handling_completed;

COMMON output_data_t __ALIGNED(1024) output_data =
  { 0 };
COMMON GNSS_type GNSS (output_data.c);

extern RestrictedTask NMEA_task;

static ROM bool TRUE=true;
static ROM bool FALSE=true;

void communicator_runnable (void*)
{
  navigator_t navigator;
  float3vector acc, mag, gyro;

  unsigned air_density_sensor_counter = 0;
  unsigned GNSS_count = 0;

  // if configuration file given:
  // wait until configuration file read
  setup_file_handling_completed.wait();

  Queue<CANpacket> air_density_sensor_Q (2);

    {
      CAN_distributor_entry cde =
	{ 0xffff, 0x120, &air_density_sensor_Q };
      bool result = subscribe_CAN_messages (cde);
      ASSERT(result);
    }

  GNSS_configration_t GNSS_configuration =
      (GNSS_configration_t) ROUND (configuration (GNSS_CONFIGURATION));

  float pitot_offset 	= configuration (PITOT_OFFSET);
  float pitot_span 	= configuration (PITOT_SPAN);
  float QNH_offset	= configuration (QNH_OFFSET);

  float3matrix sensor_mapping;
    {
      quaternion<float> q;
      q.from_euler (configuration (SENS_TILT_ROLL),
		    configuration (SENS_TILT_NICK),
		    configuration (SENS_TILT_YAW));
      q.get_rotation_matrix (sensor_mapping);
    }

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
	navigator.update_GNSS (GNSS.coordinates);
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
	navigator.update_GNSS (GNSS.coordinates);
	GNSS_new_data_ready = false;

	float present_heading = 0.0f;
	if (D_GNSS_new_data_ready && (GNSS.coordinates.sat_fix_type & SAT_HEADING))
	  present_heading = output_data.c.relPosHeading;
	navigator.set_attitude (0.0f, 0.0f, present_heading);
      }
      break;
    case GNSS_F9P_F9P: // no extra task for 2nd GNSS module
      {
	Task usart3_task (USART_3_runnable, "GNSS", 256, (void *)&TRUE, STANDARD_TASK_PRIORITY+1);

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);

	navigator.update_GNSS (GNSS.coordinates);
	GNSS_new_data_ready = false;

	float present_heading = 0.0f;
	if (GNSS.coordinates.sat_fix_type & SAT_HEADING)
	  present_heading = output_data.c.relPosHeading;
	navigator.set_attitude (0.0f, 0.0f, present_heading);
      }
      break;
    default:
      ASSERT(false);
    }

  navigator.update_pabs (output_data.m.static_pressure);
  navigator.reset_altitude ();

  for( int i=0; i<100; ++i) // wait 1 s until measurement stable
    notify_take (true);

  // setup initial attitude
  acc = sensor_mapping * output_data.m.acc;
  mag = sensor_mapping * output_data.m.mag;
  navigator.set_from_add_mag( acc, mag); // initialize attitude from acceleration + compass

  NMEA_task.resume();

  // this is the MAIN data acquisition and processing loop
  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

      navigator.update_pabs (output_data.m.static_pressure - QNH_offset);
      navigator.update_pitot (
	  (output_data.m.pitot_pressure - pitot_offset) * pitot_span);

      if (GNSS_new_data_ready) // triggered at 10 Hz by GNSS
	{
	  GNSS_new_data_ready = false;
	  navigator.update_GNSS (GNSS.coordinates);
	}

#if 1 // appears to be necessary (Flight 3.4.2022 D-KCOM)
      for( int i=0; i<3; ++i)
	if ( ! isnormal(output_data.m.gyro.e[i]) )
	  output_data.m.gyro.e[i]=0.0f;
#endif

      // rotate sensor coordinates into airframe coordinates
      acc = sensor_mapping * output_data.m.acc;
      mag = sensor_mapping * output_data.m.mag;
      gyro = sensor_mapping * output_data.m.gyro;
      navigator.update_IMU (acc, mag, gyro);
      navigator.report_data (output_data);

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
	      navigator.set_density_data(p.data_f[0], p.data_f[1]);
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
		navigator.disregard_density_data();
		output_data.m.outside_air_humidity = -1.0f; // means: disregard humidity and temperature
		update_system_state_clear( AIR_SENSOR_AVAILABLE);
		output_data.m.outside_air_temperature = ZERO;

		output_data.m.outside_air_humidity = ZERO;
		}
	    }
	}

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
