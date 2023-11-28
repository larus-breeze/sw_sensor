/** *****************************************************************************
 * @file    	GNSS.cpp
 * @brief   	Data acquisition using a uBlox GNSS receiver
 * @author  	Dr. Klaus Schaefer
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/
#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "GNSS.h"
#include "math.h"
#include "main.h"
#include "common.h"
#include "AHRS.h"
#include "system_state.h"

COMMON bool GNSS_new_data_ready;
COMMON bool D_GNSS_new_data_ready;
COMMON int64_t FAT_time; //!< DOS FAT time for file usage

#define SCALE_MM 0.001f
#define SCALE_MM_NEG -0.001f
#define SCALE_CM 0.01f
#define DEG_2_METER 111111.111e-7f // (10000 / 90) m / degree on great circle
#define ANGLE_SCALE (double)1e-7

GNSS_type::GNSS_type( coordinates_t & coo) :
		coordinates( coo),
		fix_type(FIX_none),
		num_SV(0),
		latitude_reference(0),
		longitude_reference(0),
		latitude_scale(	0.0f)
	{}

#if MEASURE_GNSS_REFRESH_TIME
COMMON float delta_t;
#endif

GNSS_Result GNSS_type::update(const uint8_t * data)
{
	if ((data[0] != 0xb5) || (data[1] != 'b') || (data[2] != 0x01)
			|| (data[3] != 0x07))
		return GNSS_ERROR;

	if (!checkSumCheck(data + 2, sizeof(uBlox_pvt)))
		return GNSS_ERROR;

	uBlox_pvt pvt;

	uint32_t * from = (uint32_t *)(data + 6);
	uint32_t * to   = (uint32_t *)&pvt;
	unsigned count = sizeof(uBlox_pvt)/sizeof(uint32_t);
	while( count --)
	  *to++ = *from++;

	// compute time since last sample has been recorded
	int32_t day_time_ms =
	    pvt.hour   * 3600000 +
	    pvt.minute * 60000 +
	    pvt.second * 1000  +
	    pvt.nano   / 1000000; // ns -> ms , see uBlox documentation. nano is SIGNED !

#if MEASURE_GNSS_REFRESH_TIME == 0
	float delta_t;
#endif

	delta_t = (float)(day_time_ms - old_timestamp_ms) * 0.001f;
	ASSERT( delta_t > 0.001f); // avoid dividing by zero below
	old_timestamp_ms = day_time_ms;

	/* Pack date and time into a DWORD variable */
	FAT_time = ((pvt.year - 1980) << 25) + (pvt.month << 21) + (pvt.day << 16)
			+ (pvt.hour << 11) + (pvt.minute << 5) + (pvt.second >> 1);

	coordinates.SATS_number = num_SV = pvt.num_SV;
	if( pvt.fix_type == 3) // 3 -> 3D-fix
	  coordinates.sat_fix_type |= SAT_FIX;
	else
	  coordinates.sat_fix_type &= ! SAT_FIX;

	if (latitude_reference == 0)
	{
		latitude_reference = pvt.latitude;
		longitude_reference = pvt.longitude;
		latitude_scale = COS((float) (pvt.latitude) * ANGLE_SCALE) * DEG_2_METER;
	}

	coordinates.latitude = (double) (pvt.latitude) * ANGLE_SCALE;
	coordinates.position[NORTH] = (double) (pvt.latitude - latitude_reference)
			* DEG_2_METER;

	coordinates.longitude = (double) (pvt.longitude) * ANGLE_SCALE;
	coordinates.position[EAST] = (double) (pvt.longitude - longitude_reference)
			* latitude_scale;

	coordinates.position[DOWN] = (double)(pvt.height) * SCALE_MM_NEG;
	coordinates.geo_sep_dm = (pvt.height_ellip - pvt.height) * SCALE_CM;

	// record new time
	coordinates.year   = pvt.year % 100;
	coordinates.month  = pvt.month;
	coordinates.day    = pvt.day;
	coordinates.hour   = pvt.hour;
	coordinates.minute = pvt.minute;
	coordinates.second = pvt.second;

#if OLD_COORD_FORMAT == 0
	coordinates.nano   = pvt.nano;
	coordinates.speed_acc = pvt.sAcc * SCALE_MM;
#endif

	float velocity_north = pvt.velocity[NORTH] * SCALE_MM;
	float velocity_east  = pvt.velocity[EAST] * SCALE_MM;

	coordinates.acceleration[NORTH]= (velocity_north - coordinates.velocity[NORTH]) / delta_t;
	coordinates.acceleration[EAST] = (velocity_east  - coordinates.velocity[EAST])  / delta_t;

	coordinates.velocity[NORTH] = velocity_north;
	coordinates.velocity[EAST]  = velocity_east;
	coordinates.velocity[DOWN]  = pvt.velocity[DOWN]  * SCALE_MM;

	coordinates.speed_motion    = pvt.gSpeed * SCALE_MM;
	coordinates.heading_motion  = pvt.gTrack * 1e-5f;

	GNSS_new_data_ready = true;

	fix_type = (FIX_TYPE) (pvt.fix_type);
	if( (pvt.fix_flags & 1) == 0)	// todo someday modify me for aerobatics support
	  {
	    coordinates.velocity[NORTH] 	= 0.0f;
	    coordinates.velocity[EAST] 		= 0.0f;
	    coordinates.velocity[DOWN] 		= 0.0f;
	    coordinates.acceleration[NORTH] 	= 0.0f;
	    coordinates.acceleration[EAST] 	= 0.0f;

	    return GNSS_NO_FIX;
	  }
	else
	    return GNSS_HAVE_FIX;
}

GNSS_Result GNSS_type::update_delta(const uint8_t * data)
{
	if ((data[0] != 0xb5) || (data[1] != 'b') || (data[2] != 0x01)
			|| (data[3] != 0x3c))
		return GNSS_ERROR;

	if (!checkSumCheck(data + 2, sizeof(uBlox_relpos_NED)))
		return GNSS_ERROR;

	uBlox_relpos_NED p;

	uint32_t * from = (uint32_t *)(data + 6);
	uint32_t * to   = (uint32_t *)&p;
	unsigned count = sizeof(uBlox_relpos_NED)/sizeof(uint32_t);
	while( count --)
	  *to++ = *from++;

	coordinates.relPosNED[NORTH]=0.01f*(float)(p.relPosN) + 0.0001f * (float)(p.relPosHP_N);
	coordinates.relPosNED[EAST] =0.01f*(float)(p.relPosE) + 0.0001f * (float)(p.relPosHP_E);
	coordinates.relPosNED[DOWN] =0.01f*(float)(p.relPosD) + 0.0001f * (float)(p.relPosHP_D);

	// 0x337 on f9pf9h if OK; 0x137 on f9p f9h if OK
	GNSS_Result res = ( (p.flags & 0x1ff) == 0x137) ? GNSS_HAVE_FIX : GNSS_NO_FIX;

	if( res == GNSS_HAVE_FIX) // patch
	  {
	    // 1e-5 deg -> rad
	    coordinates.relPosHeading = (float)(p.relPosheading) * 1.745329252e-7f;
	    coordinates.sat_fix_type |= SAT_HEADING;
	  }
	else
	  {
	    coordinates.relPosHeading = 0.0f;
	    coordinates.sat_fix_type &= ~SAT_HEADING;
	  }
	D_GNSS_new_data_ready = true;
	return res;
}

GNSS_Result
GNSS_type::update_combined (uint8_t *data)
{
  GNSS_Result res = GNSS.update(data);
  if( res != GNSS_HAVE_FIX)
      return res;

  res = GNSS.update_delta(data + sizeof( uBlox_pvt) + 8);

  return res;
}
