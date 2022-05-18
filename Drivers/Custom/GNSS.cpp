#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "GNSS.h"
#include "math.h"
#include "main.h"
#include "common.h"

COMMON bool GNSS_new_data_ready;
COMMON bool D_GNSS_new_data_ready;
COMMON int64_t FAT_time; //!< DOS FAT time for file usage

#define SCALE_MM 0.001f
#define SCALE_CM 0.01f
#define SCALE_MM_NEG -0.001f
#define DEG_2_METER 111111.111e-7f // (10000 / 90) m / degree on great circle
#define ANGLE_SCALE (double)1e-7

GNSS_type::GNSS_type( coordinates_t & coo) :
		fix_type(FIX_none),
		latitude_reference(0),
		longitude_reference(0),
		latitude_scale(	0.0f),
		coordinates( coo),
		num_SV(0)
	{}

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

	float delta_t = (float)(day_time_ms - old_timestamp_ms) * 0.001f;
	old_timestamp_ms = day_time_ms;

	/* Pack date and time into a DWORD variable */
	FAT_time = ((pvt.year - 1980) << 25) + (pvt.month << 21) + (pvt.day << 16)
			+ (pvt.hour << 11) + (pvt.minute << 5) + (pvt.second >> 1);

	fix_type = (FIX_TYPE) (pvt.fix_type);
	if (( (pvt.fix_flags & 1) == 0) || (pvt.sAcc > 250)) // todo modify me for M9N GNSS support
	  {
	  coordinates.velocity[NORTH] 		= NAN;
	  coordinates.velocity[EAST] 		= NAN;
	  coordinates.velocity[DOWN] 		= NAN;
	  coordinates.acceleration[NORTH] 	= NAN;
	  coordinates.acceleration[EAST] 	= NAN;

	  GNSS_new_data_ready = true;
	  update_system_state_clear( GNSS_AVAILABLE);
	  return GNSS_NO_FIX;
	  }

	coordinates.SATS_number=pvt.num_SV;
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

	unsigned lat_raw=pvt.latitude;
	double lat_double=lat_raw;
	coordinates.latitude = lat_double;
	coordinates.latitude *= ANGLE_SCALE;
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

	if( isnan( coordinates.velocity[NORTH])) // we had no GNSS no fix before
	  {
	    coordinates.acceleration[NORTH]= 0.0f;
	    coordinates.acceleration[EAST] = 0.0f;
	  }
	else
	  {
	    coordinates.acceleration[NORTH]= (velocity_north - coordinates.velocity[NORTH]) / delta_t;
	    coordinates.acceleration[EAST] = (velocity_east  - coordinates.velocity[EAST])  / delta_t;
	  }

	coordinates.velocity[NORTH] = velocity_north;
	coordinates.velocity[EAST]  = velocity_east;
	coordinates.velocity[DOWN]  = pvt.velocity[DOWN]  * SCALE_MM_NEG;

	coordinates.speed_motion    = pvt.gSpeed * SCALE_MM;
	coordinates.heading_motion  = pvt.gTrack * 1e-5f;

	GNSS_new_data_ready = true;

	update_system_state_set( GNSS_AVAILABLE);

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
	    update_system_state_set( D_GNSS_AVAILABLE);
	  }
	else
	  {
	    coordinates.relPosHeading = NAN;
	    update_system_state_clear( D_GNSS_AVAILABLE);
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
