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

#pragma pack(push, 1)

typedef struct // legacy data type
{
	float3vector acc;
	float3vector gyro;
	float3vector mag;
	float 	pitot_pressure,
		static_pressure;
} measurement_data_t;

typedef struct {
	measurement_data_t m;
	coordinates_t c;
} input_data_t;

typedef struct {
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
	quaternion<float>q;
	float3vector nav_correction;
	float3vector gyro_correction;
} output_data_t;

volatile unsigned size1;
volatile unsigned size2;
volatile unsigned size3;
volatile unsigned size4;

void offline_runnable(void*) {
	FIL InFile; /* File object */

	size_t bytes_transferred;

	output_data_t x = { 0 };

	INS_type ins(0.005);
	atmosphere_t atmosphere(101325.0f);
	flight_observer_t flight_observer;
	vario_integrator_t vario_integrator;
	float3vector GNSS_acceleration = { 0 };
	float3vector GNSS_velocity_previous = { 0 };

	unsigned integrator_sequencer = 0;

	acquire_privileges(); // sd-writing works only in privileged mode

	BSP_SD_Init();
	SD_driver_t drv;
	if (!drv.healthy())
		asm("bkpt 0");

	if (f_open(&InFile, "116.FLG", FA_READ) != FR_OK)
		asm("bkpt 0");

	out_file output_file("116.FSIM", true);
	if( ! output_file.healthy())
		asm("bkpt 0");

	//	unsigned samples=200*60*25;
	unsigned samples=200*60;

	if ((FR_OK
			== f_read(&InFile, (void*) &x, sizeof(input_data_t),
					&bytes_transferred))
			&& (bytes_transferred == sizeof(input_data_t)))
	{
		eulerangle<float> init;
		init.r=0.0f;
		init.n=0.0f;
		init.y=x.c.relPosHeading;
		ins.set_from_euler(init.r, init.n, init.y);
	}
	else
		asm("bkpt 0");

//	BSP_LED_On (LED1);

	while ((FR_OK
			== f_read(&InFile, (void*) &x, sizeof(input_data_t),
					&bytes_transferred))
			&& (bytes_transferred == sizeof(input_data_t)))
	{
		drop_privileges();

		float3vector speed_now = x.c.velocity;
		if(
				( speed_now.e[NORTH] != GNSS_velocity_previous.e[NORTH]) ||
				( speed_now.e[EAST]  != GNSS_velocity_previous.e[EAST])
			)
		{
		    GNSS_acceleration = speed_now - GNSS_velocity_previous;
			GNSS_velocity_previous = speed_now;
			GNSS_acceleration *= 10.0f;
		}

		x.nav_correction =
		    ins.update_diff_GNSS( x.m.gyro, x.m.acc,
				x.c.velocity, GNSS_acceleration,
				x.c.relPosHeading, x.c.speed_motion);

		x.gyro_correction = ins.gyro_correction;

		// pressure stuff update ****************************************************************

		atmosphere.set_pressure(x.m.static_pressure); // absolute static pressure

		float TAS = atmosphere.get_velocity_from_dynamic_pressure( x.m.pitot_pressure);

		float3vector true_airspeed;
		true_airspeed[NORTH] = ins.get_north() * TAS;
		true_airspeed[EAST]  = ins.get_east()  * TAS;
		true_airspeed[DOWN]  = ins.get_down()  * TAS; // todo: do we need this one ?

		// run flight observer ****************************************************************

		flight_observer.update(
				x.c.velocity, GNSS_acceleration,
				ins.get_acc(), true_airspeed,
				x.c.position[DOWN],
				TAS);

		++integrator_sequencer;
		if( integrator_sequencer >=20) // save power by calling integrator @ 10Hz
		  {
		    vario_integrator.update(
			flight_observer.get_vario_TAS(),
			ins.get_euler().y,
			ins.get_circling_state()
			);
		    integrator_sequencer=0;
		    x.integrator_vario = vario_integrator.get_value();
		  }

		x.vario_uncompensated 	= flight_observer.get_vario_uncompensated();
		x.speed_compensation_TAS= flight_observer.get_speed_compensation_TAS();
		x.speed_compensation_INS= flight_observer.get_speed_compensation_INS();
		x.vario 		= (ins.get_circling_state() == CIRCLING)
					? flight_observer.get_vario_TAS()
					: flight_observer.get_vario_INS();
		x.wind 			= flight_observer.get_wind();
		x.euler 		= ins.get_euler();

		x.nav_acceleration_gnss	= GNSS_acceleration;
		x.nav_acceleration_ins	= ins.get_acc();

		x.q			= ins.attitude;

		acquire_privileges();

		if (!output_file.write((char*) &x, sizeof(output_data_t)))
			break;

		--samples;
		if(samples == 0)
		  {
//			BSP_LED_Toggle (LED2);
			samples=200*60;
		  }
//			break;
	}
	output_file.close();
	asm("bkpt 0");
}

RestrictedTask t(offline_runnable, "compute", 2048, 0, 2*2);

#pragma pack(pop)

#endif
