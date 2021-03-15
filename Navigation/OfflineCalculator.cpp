/** ***********************************************************************
 * @file		OfflineCalculator.cpp
 * @brief		read logged sensor data and simulate output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
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

#include "data_structures.h"

output_data_t __ALIGNED(512) output_data =  { 0 };
COMMON GNSS_type GNSS (output_data.c);

FATFS fatfs;
extern SD_HandleTypeDef hsd;

COMMON uint64_t duration;

void offline_runnable (void*)
{
  HAL_SD_DeInit (&hsd);
  delay (100);

  FRESULT fresult;
  fresult = f_mount (&fatfs, "", 0);
  ASSERT( fresult == FR_OK);
  FIL InFile; /* File object */

//  uint64_t start_time;

  size_t bytes_transferred;
  navigator_t navigator;
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

  out_file output_file ( (char *)"mag_test.FSM", true);
  if (!output_file.healthy ())
    asm("bkpt 0");

  //	unsigned samples=200*60*25;
  unsigned samples = 200 * 60;

  if ((FR_OK
      == f_read (&InFile, (void*) &output_data, sizeof(input_data_t), &bytes_transferred))
      && (bytes_transferred == sizeof(input_data_t)))
    {
      eulerangle<float> init;
      init.r = 0.0f;
      init.n = 0.0f;
      init.y = output_data.c.relPosHeading;
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
      == f_read (&InFile, (void*) &output_data, sizeof(input_data_t), &bytes_transferred))
      && (bytes_transferred == sizeof(input_data_t)))
    {
//      t.sync();
//      start_time = getTime_usec_privileged();
      drop_privileges();

      float3vector speed_now = output_data.c.velocity;
      if (   (speed_now.e[NORTH] != GNSS_velocity_previous.e[NORTH])
	  || (speed_now.e[EAST]  != GNSS_velocity_previous.e[EAST])
	 )
	{
	  GNSS_acceleration = speed_now - GNSS_velocity_previous;
	  GNSS_velocity_previous = speed_now;
	  GNSS_acceleration *= 10.0f;
	  navigator.update_GNSS_old (output_data.c, GNSS_acceleration);
	}

      if( toggler_200_to_100Hz)
	{
	  navigator.update_pabs(output_data.m.static_pressure);
	  navigator.update_pitot(output_data.m.pitot_pressure);
	  navigator.update_IMU(
	      (output_data.m.acc+old_acc) * 0.5f,
	      (output_data.m.mag+old_mag) * 0.5f,
	      (output_data.m.gyro+old_gyro) * 0.5f
	      );
	}
      else
	{
	  old_acc=output_data.m.acc;
	  old_mag=output_data.m.mag;
	  old_gyro=output_data.m.gyro;
	}
      toggler_200_to_100Hz = ! toggler_200_to_100Hz;

      navigator.report_data( output_data);

      acquire_privileges();
//      duration = getTime_usec_privileged() - start_time;

      if (!output_file.write ((char*) &output_data, sizeof(output_data_t)))
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

#define STACKSIZE (2048)
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
      { &output_data, 512, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 }
    }
};

RestrictedTask offline_runnable_task( p);

#endif
