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

#ifdef DKCOM
ROM float SENSOR_MAPPING_MATRIX[] =
{
#if 0
	-0.9914f, +0.0f, -0.13f,
	+0.0f,  -1.0f,   +0.0f,
	-0.13f, +0.0f,   +.9914f
#else
	-1.0f, +0.0f, -0.0f,
	+0.0f, -1.0f, +0.0f,
	-0.0f, +0.0f, +1.0f
#endif
};
#else
ROM float SENSOR_MAPPING_MATRIX[] =
{
	+1.0f, +0.0f, +0.0f,
	+0.0f, -1.0f, +0.0f,
	+0.0f, +0.0f, -1.0f
    };
#endif

ROM float ACC_OFFSET_INIT[] = { 0.0f, 0.0f, 0.0f,};

ROM float MAG_OFFSET_INIT[] = { 0.0f, 0.0f, 0.0f,};


void communicator_runnable (void*)
{
  navigator_t navigator;
  float3matrix SENSOR_MAPPING = (float *)SENSOR_MAPPING_MATRIX;
  float3vector acc, mag, gyro;
  float3vector ACC_OFFSET = ACC_OFFSET_INIT;
  float3vector MAG_OFFSET = MAG_OFFSET_INIT;
#if RUN_CAN_OUTPUT == 1
  uint8_t count_10Hz = 1; // de-synchronize CAN output by 1 cycle
#endif

#ifndef INFILE // not in sim mode


  for( unsigned i=0; i < 200; ++i) // wait 200 IMU loops
    notify_take (true);

  while( ! GNSS_new_data_ready) // another lousy spinlock !
    delay(100);

  GNSS_new_data_ready = false;
  navigator.ins.set_from_euler(0.0f, 0.0f, 0.0f); // todo implement correct attitude setup
  navigator.ins_magnetic.set_from_euler(0.0f, 0.0f, 0.0f); // todo implement correct setup

  navigator.update_GNSS( GNSS.coordinates);

  navigator.update_pabs (output_data.m.static_pressure);
  navigator.reset_altitude();
#else
  uint32_t GNSS_sim=0;
#endif

  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

      navigator.update_pabs (output_data.m.static_pressure);
      navigator.update_pitot(output_data.m.pitot_pressure);

#ifndef INFILE // not in sim mode
      if (GNSS_new_data_ready) // triggered at 10 Hz by GNSS
	{
	  GNSS_new_data_ready = false;
#else
      if( ++GNSS_sim >=10)
	{
	  GNSS_sim=0;
	  GNSS.fix_type=FIX_3d; // not recorded
#endif
	  navigator.update_GNSS( GNSS.coordinates);
	}
      // rotate sensor coordinates into airframe coordinates
      acc  = SENSOR_MAPPING * ( output_data.m.acc - ACC_OFFSET);
      mag  = SENSOR_MAPPING * ( output_data.m.mag - MAG_OFFSET);
      gyro = SENSOR_MAPPING *   output_data.m.gyro; // offset auto-compensated by AHRS

      navigator.update_IMU ( acc, mag, gyro);

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

#define STACKSIZE 1024 // in 32bit words
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

void sync_communicator (void)
{
  communicator_task.notify_give ();
}
#endif
