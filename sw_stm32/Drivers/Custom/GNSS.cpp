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

COMMON reminder_flag GNSS_new_data_ready;
COMMON bool D_GNSS_new_data_ready;
COMMON uint64_t FAT_time; //!< DOS FAT time for file usage
COMMON Mutex GNSS_data_guard;

#define SCALE_MM 0.001f
#define SCALE_MM_NEG -0.001f
#define SCALE_CM 0.01f
#define DEG_2_METER 111111.111e-7f // (10000 / 90) m / degree on great circle
#define ANGLE_SCALE (double)1e-7

void GNSS_data_lock( unsigned function)
{
  if( function)
    {
      bool success = GNSS_data_guard.lock(2);
      ASSERT( success);
    }
  else
    GNSS_data_guard.release();
}

#if SUPPORT_D_GNSS_ACCURACY
GNSS_type::GNSS_type( D_GNSS_coordinates_t & coo, D_GNSS_accuracy_t &acc) :
		coordinates( coo),
		accuracy( acc),
		fix_type(FIX_none),
		num_SV(0)
	{}
#else
GNSS_type::GNSS_type( D_GNSS_coordinates_t & coo) :
		coordinates( coo),
		fix_type(FIX_none),
		num_SV(0)
	{}
#endif

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

	/* Pack date and time into a DWORD variable */
	FAT_time = ((pvt.year - 1980) << 25) + (pvt.month << 21) + (pvt.day << 16)
			+ (pvt.hour << 11) + (pvt.minute << 5) + (pvt.second >> 1);

	coordinates.SATS_number = num_SV = pvt.num_SV;
	if( pvt.fix_type == 3) // 3 -> 3D-fix
	  coordinates.sat_fix_type |= SAT_FIX;
	else
	  coordinates.sat_fix_type &= ~SAT_FIX;

	GNSS_data_guard.lock();

	coordinates.latitude = (double) (pvt.latitude) * ANGLE_SCALE;
	coordinates.longitude = (double) (pvt.longitude) * ANGLE_SCALE;
	coordinates.GNSS_MSL_altitude = (double)(pvt.height) * SCALE_MM;
	coordinates.geo_sep_dm 	= (pvt.height_ellip - pvt.height) * SCALE_CM;
	coordinates.pDOP = pvt.pDOP;

	// record new time
	coordinates.year   = pvt.year % 100;
	coordinates.month  = pvt.month;
	coordinates.day    = pvt.day;
	coordinates.hour   = pvt.hour;
	coordinates.minute = pvt.minute;
	coordinates.second = pvt.second;

	coordinates.nano   = pvt.nano;
	coordinates.speed_acc = pvt.sAcc * SCALE_MM;

	GNSS_data_guard.release();

	coordinates.velocity[NORTH] = pvt.speed[NORTH] * SCALE_MM;
	coordinates.velocity[EAST]  = pvt.speed[EAST]  * SCALE_MM;
	coordinates.velocity[DOWN]  = pvt.speed[DOWN]  * SCALE_MM;

	fix_type = (FIX_TYPE) (pvt.fix_type);
	if( (pvt.fix_flags & 1) == 0)
	  {
	    report_dead_GNSS();
	    return GNSS_NO_FIX;
	  }
	else
	  {
	    GNSS_new_data_ready.set();
	    return GNSS_HAVE_FIX;
	  }
}

GNSS_Result GNSS_type::update_delta_X20D(const uint8_t * data)
{
	if ((data[0] != 0xb5) || (data[1] != 'b') || (data[2] != 0x01) || (data[3] != 0x45))
		return GNSS_ERROR;

	if (!checkSumCheck(data + 2, sizeof(uBlox_DAHEADING)))
		return GNSS_ERROR;

	uBlox_DAHEADING p;

	uint32_t * from = (uint32_t *)(data + 6);
	uint32_t * to   = (uint32_t *)&p;
	unsigned count = sizeof(uBlox_DAHEADING) / sizeof(uint32_t);
	while( count --)
	  *to++ = *from++;

	coordinates.relPosNED[NORTH]= SCALE_MM * (float)(p.relPosN);
	coordinates.relPosNED[EAST] = SCALE_MM * (float)(p.relPosE);
	coordinates.relPosNED[DOWN] = SCALE_MM * (float)(p.relPosD);

#if SUPPORT_D_GNSS_ACCURACY
	decimate( accuracy.relPosAccN, p.accN * SCALE_MM);
	decimate( accuracy.relPosAccE, p.accE * SCALE_MM);
	decimate( accuracy.relPosAccD, p.accD * SCALE_MM);
	decimate( accuracy.relPosAccLen, p.acc_len * SCALE_MM);
	decimate( accuracy.relPosHeadingAcc, p.acc_heading * 1e-5f);
	decimate( accuracy.relPosLength, p.relPoslength * SCALE_MM);
#endif

	if( (p.flags & 0x05) == 0x05)
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

	return GNSS_HAVE_FIX;
}

GNSS_Result GNSS_type::update_delta_F9x(const uint8_t * data)
{
	if ((data[0] != 0xb5) || (data[1] != 'b') || (data[2] != 0x01) || (data[3] != 0x3c))
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

#if SUPPORT_D_GNSS_ACCURACY
	decimate( accuracy.relPosAccN, p.accN * 0.0001f);
	decimate( accuracy.relPosAccE, p.accE * 0.0001f);
	decimate( accuracy.relPosAccD, p.accD * 0.0001f);
	decimate( accuracy.relPosAccLen, p.acc_len * 0.0001f);
	decimate( accuracy.relPosHeadingAcc, p.acc_heading * 1e-5f);
	decimate( accuracy.relPosLength, p.relPoslength * 0.01f);
#endif

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
	return res;
}

GNSS_Result GNSS_type::update_combined ( const uint8_t *data, GNSS_configration_t gnss)
{
  GNSS_Result res = GNSS.update(data);
  if( res != GNSS_HAVE_FIX)
      return res;

  switch( gnss)
  {
    default:
    case GNSS_TYPE_NOT_DEFINED:
      ASSERT( 0);
      break;
    case GNSS_F9P_F9P:
      res = GNSS.update_delta_F9x( data + sizeof( uBlox_pvt) + 8);
      break;
    case GNSS_X20D:
      res = GNSS.update_delta_X20D( data + sizeof( uBlox_pvt) + 8);
      break;
  }

  return res;
}

void GNSS_type::report_dead_GNSS (void)
{
  coordinates.velocity[NORTH] 	= ZERO;
  coordinates.velocity[EAST]	= ZERO;
  coordinates.velocity[DOWN]	= ZERO;
  coordinates.GNSS_MSL_altitude	= ZERO; // avoid reporting wrong GNSS altitude
  coordinates.speed_acc 	= ZERO;
  GNSS_new_data_ready.set();
}
