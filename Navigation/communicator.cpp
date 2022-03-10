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

void
sync_logger (void);

COMMON output_data_t __ALIGNED(1024) output_data =
  { 0 };
COMMON GNSS_type GNSS (output_data.c);

#if N_PROBES > 0
COMMON float * probe= output_data.probe; // debugging probes
#endif

extern RestrictedTask NMEA_task;

static ROM bool TRUE=true;
static ROM bool FALSE=true;

void communicator_runnable (void*)
{
  navigator_t navigator;
  float3vector acc, mag, gyro;

  unsigned GNSS_count = 0;

  GNSS_configration_t GNSS_configuration = (GNSS_configration_t) ROUND (
      configuration (GNSS_CONFIGURATION));

  float pitot_offset = configuration (PITOT_OFFSET);
  float pitot_span = configuration (PITOT_SPAN);
  float QNH_offset = configuration (QNH_OFFSET);

  float3matrix sensor_mapping;
    {
      quaternion<float> q;
      q.from_euler (configuration (SENS_TILT_ROLL),
		    configuration (SENS_TILT_NICK),
		    configuration (SENS_TILT_YAW));
      q.get_rotation_matrix (sensor_mapping);
    }

#if RUN_CAN_OUTPUT == 1
  uint8_t count_10Hz = 1; // de-synchronize CAN output by 1 cycle
#endif

#ifndef INFILE // only outside of the offline mode
  for (unsigned i = 0; i < 200; ++i) // wait 200 IMU loops
    notify_take (true);

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
#else
    double old_latitude; // used to trigger on new input
    float3vector old_velocity;
    float3vector old_acceleration;
    int32_t measurement_ticks;
    int32_t last_GNSS_update_at;
#endif

  navigator.update_pabs (output_data.m.static_pressure);
  navigator.reset_altitude ();

  notify_take (true); // wait for synchronization by IMU @ 100 Hz
  // setup initial attitude
  acc = sensor_mapping * output_data.m.acc;
  mag = sensor_mapping * output_data.m.mag;
  navigator.set_from_add_mag( acc, mag);

  NMEA_task.resume();

  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

      navigator.update_pabs (output_data.m.static_pressure - QNH_offset);
      navigator.update_pitot (
	  (output_data.m.pitot_pressure - pitot_offset) * pitot_span);

#ifdef INFILE // we presently run HIL/SIL
	  ++measurement_ticks;
	  if (GNSS.coordinates.latitude != old_latitude) // todo this is a dirty workaround
	    {
	      old_latitude = GNSS.coordinates.latitude;
	      GNSS.fix_type = FIX_3d; // has not been recorded ...

	      // todo remove me, bugfix for bad acceleration data 1.10.2021
	      float instant_sample_frequency = 100.0f / (float)( measurement_ticks - last_GNSS_update_at);
	      old_acceleration = (GNSS.coordinates.velocity - old_velocity) * instant_sample_frequency;
	      GNSS.coordinates.acceleration = old_acceleration;
	      old_velocity = GNSS.coordinates.velocity;

	      navigator.update_GNSS (GNSS.coordinates);

	      last_GNSS_update_at = measurement_ticks;
	    }
	  else
	      GNSS.coordinates.acceleration = old_acceleration; // keep OUR acceleration !

#else
      if (GNSS_new_data_ready) // triggered at 10 Hz by GNSS
	{
	  GNSS_new_data_ready = false;
	  navigator.update_GNSS (GNSS.coordinates);
	}
#endif

#if 1 // todo remove me some day...
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
	      && (output_data.c.sat_fix_type & SAT_HEADING))
	  ||
	  ((GNSS_configuration == GNSS_M9N)
	      && (output_data.c.sat_fix_type & SAT_FIX))
	)
	{
	  ++GNSS_count;
	  HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
	      (GNSS_count & 0xff) > 127 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
      else
	HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);

#if RUN_CAN_OUTPUT == 1
      if (++count_10Hz >= 10)
	{
	  count_10Hz = 0;
	  trigger_CAN ();
	}
#endif

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

