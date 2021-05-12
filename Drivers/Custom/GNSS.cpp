#include "system_configuration.h"
#include "GNSS.h"
#include "math.h"
#include "common.h"

COMMON bool GNSS_new_data_ready;
COMMON bool D_GNSS_new_data_ready;

#define SCALE_MM 0.001
#define SCALE_MM_NEG -0.001
#define DEG_2_METER 111111.111e-7 // (10000 / 90) m / degree on great circle
#define ANGLE_SCALE (double)1e-7

GNSS_type::GNSS_type( coordinates_t & coo) :
		fix_type(FIX_none),
		latitude_reference(0),
		longitude_reference(0),
		latitude_scale(	0.0f),
		FAT_time(0),
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

	const uBlox_pvt * p = (uBlox_pvt *) (data + 6);
#if 0
	coordinates.time = p->hour * 3600000 + p->minute * 60000 + p->second * 1000
			+ p->nano / 1000000;
	coordinates.date = p->day + (p->month) * 100 + (p->year) * 10000;
#endif

	/* Pack date and time into a DWORD variable */
	FAT_time = ((p->year - 1980) << 25) + (p->month << 21) + (p->day << 16)
			+ (p->hour << 11) + (p->minute << 5) + (p->second >> 1);

	fix_type = (FIX_TYPE) (p->fix_type);
	if ((p->fix_flags & 1) == 0)
	  {
#if RUN_GNSS_UPDATE_WITHOUT_FIX
	    GNSS_new_data_ready = true;
#endif
	  return GNSS_NO_FIX;
	  }

	num_SV=p->num_SV;

	if (latitude_reference == 0)
	{
		latitude_reference = p->latitude;
		longitude_reference = p->longitude;
		latitude_scale = COS((float) (p->latitude) * ANGLE_SCALE) * DEG_2_METER;
	}

	unsigned lat_raw=p->latitude;
	double lat_double=lat_raw;
	coordinates.latitude = lat_double;
	coordinates.latitude *= ANGLE_SCALE;
//	coordinates.latitude = (double) (p->latitude) * ANGLE_SCALE;
	coordinates.position[NORTH] = (double) (p->latitude - latitude_reference)
			* DEG_2_METER;

	coordinates.longitude = (double) (p->longitude) * ANGLE_SCALE;
	coordinates.position[EAST] = (double) (p->longitude - longitude_reference)
			* latitude_scale;

	coordinates.position[DOWN] = (double)(p->height) * SCALE_MM_NEG;
	coordinates.geo_sep_dm = (p->height_ellip - p->height) / 100;
#if 0 // use precise sampling rate todo: BUGGY
	// use new - old timestamp to compute exact sample rate
	float sample_rate = 1e-9f / (
	    ((float)(p->nano) - (float)(coordinates.nano)) +
	    (p->second - coordinates.second) * 1e+9f);
#else
#define sample_rate 10.0f // depending on master GNSS RX configuration
#endif
	// record new time
	coordinates.year   = p->year % 100;
	coordinates.month  = p->month;
	coordinates.day    = p->day;
	coordinates.hour   = p->hour;
	coordinates.minute = p->minute;
	coordinates.second = p->second;
//	coordinates.nano   = p->nano;

	float velocity_north = p->velocity[NORTH] * SCALE_MM;
	float velocity_east  = p->velocity[EAST] * SCALE_MM;
#if OLD_FORMAT == 0
	coordinates.acceleration[NORTH]= (velocity_north - coordinates.velocity[NORTH]) * sample_rate;
	coordinates.acceleration[EAST] = (velocity_east  - coordinates.velocity[EAST])  * sample_rate;
#endif
	coordinates.velocity[NORTH] = velocity_north;
	coordinates.velocity[EAST]  = velocity_east;
	coordinates.velocity[DOWN]  = p->velocity[DOWN]  * SCALE_MM_NEG;

	coordinates.speed_motion    = p->gSpeed * SCALE_MM;
	coordinates.heading_motion  = p->gTrack * 1e-5f;

	GNSS_new_data_ready = true;

	return GNSS_HAVE_FIX;
}
#if USE_DIFF_GNSS == 1

GNSS_Result GNSS_type::update_delta(const uint8_t * data)
{
	if ((data[0] != 0xb5) || (data[1] != 'b') || (data[2] != 0x01)
			|| (data[3] != 0x3c))
		return GNSS_ERROR;

	if (!checkSumCheck(data + 2, sizeof(uBlox_relpos_NED)))
		return GNSS_ERROR;

	const uBlox_relpos_NED * p = (uBlox_relpos_NED *) (data + 6);

	coordinates.relPosNED[NORTH]=0.01f*(p->relPosN) + 0.0001f * p->relPosHP_N;
	coordinates.relPosNED[EAST] =0.01f*(p->relPosE) + 0.0001f * p->relPosHP_E;
	coordinates.relPosNED[DOWN] =0.01f*(p->relPosD) + 0.0001f * p->relPosHP_D;

	coordinates.relPosHeading = p->relPosheading * 1.745329252e-7f; // 1e-5 deg -> rad
//	coordinates.relPosLength  = 0.01f*(p->relPoslength) + 0.0001f * p->relPosHP_len;

//	return ( p->flags == 0b0100110111) ? GPS_HAVE_FIX : GPS_NO_FIX; // for two F9P receivers

	GNSS_Result res = ( p->flags == 0b1100110111) ? GNSS_HAVE_FIX : GNSS_NO_FIX; // for F9P + F9H receiver

#if RUN_GNSS_UPDATE_WITHOUT_FIX
	  D_GNSS_new_data_ready = true;
#else
	if( res == GNSS_HAVE_FIX) // patch
	  D_GNSS_new_data_ready = true;
#endif

	return res;
}

#endif
