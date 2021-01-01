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
COMMON GNSS_type GNSS( output_data.c);

void communicator_runnable( void *)
{
  navigator_t navigator;
  flight_observer_t & f_obs = navigator.get_flight_observer();

#if RUN_CAN_OUTPUT == 1
  uint8_t count_10Hz=1; // de-synchronize CAN output by 1 cycle
#endif

  while( true)
    {
      notify_take(true); // wait for synchronization by IMU

      navigator.update_pabs(output_data.m.static_pressure);
      navigator.update_pitot(output_data.m.pitot_pressure);
      output_data.TAS = navigator.get_TAS();
      output_data.IAS = navigator.get_IAS();

      if( GNSS_actualized) // triggered at 10 Hz
	{
	  GNSS_actualized = false;
	  navigator.update_GNSS(  GNSS.coordinates);
	}
      navigator.update_IMU( output_data.m.acc, output_data.m.mag, output_data.m.gyro);

      output_data.euler				= navigator.ins.get_euler();
      output_data.q					= navigator.ins.attitude;

      output_data.euler_magnetic	= navigator.ins_magnetic.get_euler();
      output_data.q_magnetic		= navigator.ins_magnetic.attitude;

      output_data.vario				= f_obs.get_vario_INS();
      output_data.integrator_vario	= navigator.get_vario_integrator();
      output_data.vario_uncompensated = f_obs.get_vario_uncompensated();

      output_data.wind				= f_obs.get_wind();

      output_data.speed_compensation_TAS = f_obs.get_speed_compensation_TAS();
      output_data.speed_compensation_INS = f_obs.get_speed_compensation_INS();
      output_data.effective_vertical_acceleration = f_obs.get_eff_vert_acc();

      output_data.circle_mode = navigator.ins.get_circling_state();

#if RUN_CAN_OUTPUT == 1
      if( ++count_10Hz >= 10)
	{
	count_10Hz=0;
	trigger_CAN();
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

COMMON RestrictedTask communicator_task( p);

void sync_communicator(void)
{
  communicator_task.notify_give();
}
#endif
