#include "system_configuration.h"
#include "GNSS.h"
#include "math.h"
#include "common.h"

enum{ NORTH, EAST, DOWN};

COMMON GNSS_type GNSS;

#define SCALE_MM 0.001
#define SCALE_MM_NEG -0.001
#define DEG_2_METER 111111.111e-7 // (10000 / 90) m / degree on great circle
#define ANGLE_SCALE 1e-7d

GNSS_type::GNSS_type() :
		fix_type(FIX_none),
		latitude_reference(0),
		longitude_reference(0),
		latitude_scale(	0.0f),
		FAT_time(0),
		coordinates( {0}),
		num_SV(0)
	{}

GPS_Result GNSS_type::update(const uint8_t * data)
{
	if ((data[0] != 0xb5) || (data[1] != 'b') || (data[2] != 0x01)
			|| (data[3] != 0x07))
		return GPS_ERROR;

	if (!checkSumCheck(data + 2, sizeof(uBlox_pvt)))
		return GPS_ERROR;

	const uBlox_pvt * p = (uBlox_pvt *) (data + 6);
#if 0
	coordinates.time = p->hour * 3600000 + p->minute * 60000 + p->second * 1000
			+ p->nano / 1000000;
	coordinates.date = p->day + (p->month) * 100 + (p->year) * 10000;
#endif

	coordinates.year   = p->year % 100;
	coordinates.month  = p->month;
	coordinates.day    = p->day;
	coordinates.hour   = p->hour;
	coordinates.minute = p->minute;
	coordinates.second = p->second;

	/* Pack date and time into a DWORD variable */
	FAT_time = ((p->year - 1980) << 25) + (p->month << 21) + (p->day << 16)
			+ (p->hour << 11) + (p->minute << 5) + (p->second >> 1);

	fix_type = (FIX_TYPE) (p->fix_type);
	if ((p->fix_flags & 1) == 0)
		return GPS_NO_FIX;

	num_SV=p->num_SV;

	if (latitude_reference == 0)
	{
		latitude_reference = p->latitude;
		longitude_reference = p->longitude;
		latitude_scale =
				cosf((float) (p->latitude) * ANGLE_SCALE) * DEG_2_METER;
	}

	volatile unsigned lat_raw=p->latitude;
	volatile double lat_double=lat_raw;
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

	coordinates.velocity[NORTH] = p->velocity[NORTH] * SCALE_MM;
	coordinates.velocity[EAST]  = p->velocity[EAST]  * SCALE_MM;
	coordinates.velocity[DOWN]  = p->velocity[DOWN]  * SCALE_MM_NEG;

	coordinates.speed_motion    = p->gSpeed * SCALE_MM;
	coordinates.heading_motion  = p->gTrack * 1e-5f;

	return GPS_HAVE_FIX;
}

GPS_Result GNSS_type::update_delta(const uint8_t * data)
{
	if ((data[0] != 0xb5) || (data[1] != 'b') || (data[2] != 0x01)
			|| (data[3] != 0x3c))
		return GPS_ERROR;

	if (!checkSumCheck(data + 2, sizeof(uBlox_relpos_NED)))
		return GPS_ERROR;

	const uBlox_relpos_NED * p = (uBlox_relpos_NED *) (data + 6);

	coordinates.relPosNED[NORTH]=0.01f*(p->relPosN) + 0.0001f * p->relPosHP_N;
	coordinates.relPosNED[EAST] =0.01f*(p->relPosE) + 0.0001f * p->relPosHP_E;
	coordinates.relPosNED[DOWN] =0.01f*(p->relPosD) + 0.0001f * p->relPosHP_D;

	coordinates.relPosHeading = p->relPosheading * 1.745329252e-7f; // 1e-5 deg -> rad
	coordinates.relPosLength  = 0.01f*(p->relPoslength) + 0.0001f * p->relPosHP_len;
//	coordinates.relPosLength = (float)(p->flags); // patch

//	return ( p->flags == 0b0100110111) ? GPS_HAVE_FIX : GPS_NO_FIX; // for two F9P receivers
	return ( p->flags == 0b1100110111) ? GPS_HAVE_FIX : GPS_NO_FIX; // for F9P + F9H receiver
}
