#include "system_configuration.h"

#if RUN_OFFLINE_CALCULATION == 1

#include <FreeRTOS_wrapper.h>
#include <differentiator.h>
#include "variointegrator.h"
#include "arm_math.h"
#include "vsqrtf.h"
#include "INS.h"
#include "GNSS.h"
#include "serial_io.h"
#include "NMEA_format.h"
#include "atmosphere.h"
#include "flight_observer.h"
#include "fatfs.h"
#include "sd_driver.h"

uint64_t getTime_usec_privileged(void);

#pragma pack(push, 1)

typedef struct // legacy data type
{
  float3vector acc;
  float3vector gyro;
  float3vector mag;
  float pitot_pressure, static_pressure;
} measurement_data_t;

typedef struct
{
  measurement_data_t m;
  coordinates_t c;
} input_data_t;

typedef struct
{
  measurement_data_t m;
  coordinates_t c;
  float vario_uncompensated;
  float vario;
  float speed_compensation_TAS;
  float speed_compensation_INS;
  float integrator_vario;
  float3vector wind;
  eulerangle<float> euler;
  float3vector nav_acceleration_ins;
  float3vector nav_acceleration_gnss;
  quaternion<float> q;
  float3vector nav_correction;
  float3vector gyro_correction;
} output_data_t;

FATFS fatfs;

output_data_t __ALIGNED(1024) x =  { 0 };

COMMON uint64_t duration;

void offline_runnable (void*)
{
  FRESULT fresult;
  fresult = f_mount (&fatfs, "", 0);
  ASSERT( fresult == FR_OK);
  FIL InFile; /* File object */

  uint64_t start_time;

  size_t bytes_transferred;

  INS_type ins (0.005);
  atmosphere_t atmosphere (101325.0f);
  flight_observer_t flight_observer;
  vario_integrator_t vario_integrator;
  float3vector GNSS_acceleration =
    { 0 };
  float3vector GNSS_velocity_previous =
    { 0 };

  unsigned integrator_sequencer = 0;

  acquire_privileges(); // sd-writing works only in privileged mode

/*
  BSP_SD_Init ();
  SD_driver_t drv;
  if (!drv.healthy ())
    asm("bkpt 0");
*/

  if (f_open (&InFile, "116_sim.FLG", FA_READ) != FR_OK)
    asm("bkpt 0");

  out_file output_file ( (char *)"116.FSM", true);
  if (!output_file.healthy ())
    asm("bkpt 0");

  //	unsigned samples=200*60*25;
  unsigned samples = 200 * 60;

  if ((FR_OK
      == f_read (&InFile, (void*) &x, sizeof(input_data_t), &bytes_transferred))
      && (bytes_transferred == sizeof(input_data_t)))
    {
      eulerangle<float> init;
      init.r = 0.0f;
      init.n = 0.0f;
      init.y = x.c.relPosHeading;
      ins.set_from_euler (init.r, init.n, init.y);
    }
  else
    asm("bkpt 0");

//  synchronous_timer t(5);

  while ((FR_OK
      == f_read (&InFile, (void*) &x, sizeof(input_data_t), &bytes_transferred))
      && (bytes_transferred == sizeof(input_data_t)))
    {
//      t.sync();
//      start_time = getTime_usec_privileged();
      drop_privileges();

      float3vector speed_now = x.c.velocity;
      if ((speed_now.e[NORTH] != GNSS_velocity_previous.e[NORTH])
	  || (speed_now.e[EAST] != GNSS_velocity_previous.e[EAST]))
	{
	  GNSS_acceleration = speed_now - GNSS_velocity_previous;
	  GNSS_velocity_previous = speed_now;
	  GNSS_acceleration *= 10.0f;
	}

      x.nav_correction = ins.update_diff_GNSS (x.m.gyro, x.m.acc, x.c.velocity,
					       GNSS_acceleration,
					       x.c.relPosHeading,
					       x.c.speed_motion);

      x.gyro_correction = ins.gyro_correction;

      // pressure stuff update ****************************************************************

      atmosphere.set_pressure (x.m.static_pressure); // absolute static pressure

      float TAS = atmosphere.get_velocity_from_dynamic_pressure (
	  x.m.pitot_pressure);

      float3vector true_airspeed;
      true_airspeed[NORTH] = ins.get_north () * TAS;
      true_airspeed[EAST] = ins.get_east () * TAS;
      true_airspeed[DOWN] = ins.get_down () * TAS; // todo: do we need this one ?

      // run flight observer ****************************************************************

      flight_observer.update (x.c.velocity, GNSS_acceleration, ins.get_acc (),
			      true_airspeed, x.c.position[DOWN], TAS);

      ++integrator_sequencer;
      if (integrator_sequencer >= 20) // save power by calling integrator @ 10Hz
	{
	  vario_integrator.update (flight_observer.get_vario_TAS (),
				   ins.get_euler ().y,
				   ins.get_circling_state ());
	  integrator_sequencer = 0;
	  x.integrator_vario = vario_integrator.get_value ();
	}

      x.vario_uncompensated = flight_observer.get_vario_uncompensated ();
      x.speed_compensation_TAS = flight_observer.get_speed_compensation_TAS ();
      x.speed_compensation_INS = flight_observer.get_speed_compensation_INS ();
      x.vario =
	  (ins.get_circling_state () == CIRCLING) ?
	      flight_observer.get_vario_TAS () :
	      flight_observer.get_vario_INS ();
      x.wind = flight_observer.get_wind ();
      x.euler = ins.get_euler ();

      x.nav_acceleration_gnss = GNSS_acceleration;
      x.nav_acceleration_ins = ins.get_acc ();

      x.q = ins.attitude;

      acquire_privileges();
//      duration = getTime_usec_privileged() - start_time;

      if (!output_file.write ((char*) &x, sizeof(output_data_t)))
	break;

      --samples;
      if (samples == 0)
	{
	  if( ! output_file.sync())
	    ASSERT(0);
	  //			BSP_LED_Toggle (LED2);
	  samples = 200 * 60;
	}
//			break;
    }
  output_file.close ();
  asm("bkpt 0");
}

#define STACKSIZE 1024
static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static TaskParameters_t p =
{
    offline_runnable,
    "CALC",
    STACKSIZE,
    0,
    STANDARD_TASK_PRIORITY-1 + portPRIVILEGE_BIT,
    stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { &x, 1024, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 }
    }
};

RestrictedTask offline_runnable_task( p);

#pragma pack(pop)

#endif
