#include "system_configuration.h"

#if RUN_OFFLINE_CALCULATION == 1

#include <FreeRTOS_wrapper.h>
#include <differentiator.h>
#include "variointegrator.h"
#include "arm_math.h"
#include "vsqrtf.h"
#include "navigator.h"
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
extern SD_HandleTypeDef hsd;

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
  navigator_t navigator;
  flight_observer_t &flight_observer=navigator.get_flight_observer();
  float3vector GNSS_acceleration = { 0 };
  float3vector GNSS_velocity_previous = { 0 };

  acquire_privileges(); // sd-writing works only in privileged mode

  while(f_open (&InFile, "116_sim.FLG", FA_READ) != FR_OK)
    {
      HAL_SD_DeInit (&hsd);
      delay (100);
      fresult = f_mount (&fatfs, "", 0);
      delay (100);
    }

  out_file output_file ( (char *)"116_gac.FSM", true);
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
      navigator.set_from_euler (init.r, init.n, init.y);
    }
  else
    asm("bkpt 0");

//  synchronous_timer t(5);

  bool toggler_200_to_100Hz = false;
  float3vector old_acc=0;
  float3vector old_mag=0;
  float3vector old_gyro=0;

  while ((FR_OK
      == f_read (&InFile, (void*) &x, sizeof(input_data_t), &bytes_transferred))
      && (bytes_transferred == sizeof(input_data_t)))
    {
//      t.sync();
//      start_time = getTime_usec_privileged();
      drop_privileges();

      float3vector speed_now = x.c.velocity;
      if (   (speed_now.e[NORTH] != GNSS_velocity_previous.e[NORTH])
	  || (speed_now.e[EAST]  != GNSS_velocity_previous.e[EAST])
	 )
	{
	  GNSS_acceleration = speed_now - GNSS_velocity_previous;
	  GNSS_velocity_previous = speed_now;
	  GNSS_acceleration *= 10.0f;
	  navigator.update_GNSS (x.c, GNSS_acceleration); // patch extra GNSS_acceleration
	  x.gyro_correction = navigator.ins.get_gyro_correction();
	}


      if( toggler_200_to_100Hz)
	{
	  navigator.update_pabs(x.m.static_pressure);
	  navigator.update_pitot(x.m.pitot_pressure);
	  navigator.update_IMU(
	      (x.m.acc+old_acc) * 0.5f,
	      (x.m.mag+old_mag) * 0.5f,
	      (x.m.gyro+old_gyro) * 0.5f
	      );
	}
      else
	{
	  old_acc=x.m.acc;
	  old_mag=x.m.mag;
	  old_gyro=x.m.gyro;
	}
      toggler_200_to_100Hz = ! toggler_200_to_100Hz;

      x.vario_uncompensated 	= flight_observer.get_vario_uncompensated ();
      x.speed_compensation_TAS 	= flight_observer.get_speed_compensation_TAS ();
      x.speed_compensation_INS 	= flight_observer.get_speed_compensation_INS ();
      x.vario 	 		= flight_observer.get_vario_INS ();
      x.integrator_vario	= navigator.get_vario_integrator ();
      x.wind  			= flight_observer.get_wind ();
      x.euler 			= navigator.get_euler ();

      x.nav_acceleration_gnss 	= GNSS_acceleration;
      x.nav_acceleration_ins 	= navigator.get_ins_acc ();

      x.q 			= navigator.get_attitude();

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
