#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "GNSS.h"
#include "math.h"
#include "main.h"
#include "common.h"

COMMON bool GNSS_new_data_ready;
COMMON bool D_GNSS_new_data_ready;

#define SCALE_MM 0.001f
#define SCALE_MM_NEG -0.001f
#define DEG_2_METER 111111.111e-7f // (10000 / 90) m / degree on great circle
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

	uBlox_pvt p;

	uint32_t * from = (uint32_t *)(data + 6);
	uint32_t * to   = (uint32_t *)&p;
	unsigned count = sizeof(uBlox_pvt)/sizeof(uint32_t);
	while( count --)
	  *to++ = *from++;

	/* Pack date and time into a DWORD variable */
	FAT_time = ((p.year - 1980) << 25) + (p.month << 21) + (p.day << 16)
			+ (p.hour << 11) + (p.minute << 5) + (p.second >> 1);

	fix_type = (FIX_TYPE) (p.fix_type);
	if ((p.fix_flags & 1) == 0)
	  return GNSS_NO_FIX;

	num_SV=p.num_SV;

	if (latitude_reference == 0)
	{
		latitude_reference = p.latitude;
		longitude_reference = p.longitude;
		latitude_scale = COS((float) (p.latitude) * ANGLE_SCALE) * DEG_2_METER;
	}

	unsigned lat_raw=p.latitude;
	double lat_double=lat_raw;
	coordinates.latitude = lat_double;
	coordinates.latitude *= ANGLE_SCALE;
//	coordinates.latitude = (double) (p.latitude) * ANGLE_SCALE;
	coordinates.position[NORTH] = (double) (p.latitude - latitude_reference)
			* DEG_2_METER;

	coordinates.longitude = (double) (p.longitude) * ANGLE_SCALE;
	coordinates.position[EAST] = (double) (p.longitude - longitude_reference)
			* latitude_scale;

	coordinates.position[DOWN] = (double)(p.height) * SCALE_MM_NEG;
	coordinates.geo_sep_dm = (p.height_ellip - p.height) / 100;

	// record new time
	coordinates.year   = p.year % 100;
	coordinates.month  = p.month;
	coordinates.day    = p.day;
	coordinates.hour   = p.hour;
	coordinates.minute = p.minute;
	coordinates.second = p.second;
//	coordinates.nano   = p.nano;

	float velocity_north = p.velocity[NORTH] * SCALE_MM;
	float velocity_east  = p.velocity[EAST] * SCALE_MM;

	coordinates.acceleration[NORTH]= (velocity_north - coordinates.velocity[NORTH]) * GNSS_SAMPLE_RATE;
	coordinates.acceleration[EAST] = (velocity_east  - coordinates.velocity[EAST])  * GNSS_SAMPLE_RATE;

	coordinates.velocity[NORTH] = velocity_north;
	coordinates.velocity[EAST]  = velocity_east;
	coordinates.velocity[DOWN]  = p.velocity[DOWN]  * SCALE_MM_NEG;

	coordinates.speed_motion    = p.gSpeed * SCALE_MM;
	coordinates.heading_motion  = p.gTrack * 1e-5f;

	GNSS_new_data_ready = true;

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

	GNSS_Result res = ( (p.flags & 0b0111111111) == 0b0100110111) ? GNSS_HAVE_FIX : GNSS_NO_FIX;

	res = GNSS_HAVE_FIX; // todo remove this patch

	if( res == GNSS_HAVE_FIX) // patch
	  coordinates.relPosHeading = (float)(p.relPosheading) * 1.745329252e-7f; // 1e-5 deg -> rad
	else
	  coordinates.relPosHeading = NAN_F; // report missing D-GNSS heading

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

  if(  res == GNSS_HAVE_FIX)
    update_system_state_set( D_GNSS_AVAILABLE);
  return res;
}
