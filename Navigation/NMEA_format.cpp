#include "NMEA_format.h"

#include "vsqrtf.h"
#include "asin_atan.h"

#define ANGLE_SCALE 1e-7
#define MPS_TO_NMPH 1.944 // 90 * 60 NM / 10000km * 3600 s/h
#define RAD_TO_DEGREE_10 572.958
#define METER_TO_FEET 3.2808

//#pragma GCC optimize ("O1") // todo patch funny BUG workaround

char *
angle_format ( double angle, char * p, char posc, char negc)
{
  bool pos = angle > 0.0f;
  if (!pos)
    angle = -angle;

  int degree = (int) angle;

  *p++ = degree / 10 + '0';
  *p++ = degree % 10 + '0';

  double minutes = (angle - (double) degree) * 60.0;
  int min = (int) minutes;
  *p++ = min / 10 + '0';
  *p++ = min % 10 + '0';

  *p++ = '.';

  minutes -= min;
  minutes *= 100000;
  min = (int) (minutes + 0.5f);

  p[4] = min % 10 + '0';
  min /= 10;
  p[3] = min % 10 + '0';
  min /= 10;
  p[2] = min % 10 + '0';
  min /= 10;
  p[1] = min % 10 + '0';
  min /= 10;
  p[0] = min % 10 + '0';

  p += 5;

  *p++ = ',';
  *p++ = pos ? posc : negc;
  return p;
}

inline float
sqr (float a)
{
  return a * a;
}

char *format_RMC (const GNSS_type &gnss, char *p)
{
  strcpy (p, (const char *) "$GPRMC,");

  while (*p)
    ++p;

  *p++ = (gnss.coordinates.hour) / 10 + '0';
  *p++ = (gnss.coordinates.hour) % 10 + '0';
  *p++ = (gnss.coordinates.minute) / 10 + '0';
  *p++ = (gnss.coordinates.minute) % 10 + '0';
  *p++ = (gnss.coordinates.second) / 10 + '0';
  *p++ = (gnss.coordinates.second) % 10 + '0';
  *p++ = '.';
  *p++ = '0';
  *p++ = '0';
  *p++ = ',';
  *p++ = gnss.fix_type >= FIX_2d ? 'A' : 'V';
  *p++ = ',';

  p = angle_format (gnss.coordinates.latitude, p, 'N', 'S');
  *p++ = ',';

  p = angle_format (gnss.coordinates.longitude, p, 'E', 'W');
  *p++ = ',';

#if 0
  float value = VSQRTF
      (
	  sqr( gnss.coordinates.velocity.e[NORTH]) +
	  sqr( gnss.coordinates.velocity.e[EAST])
      ) * MPS_TO_NMPH;
#else
  float value = gnss.coordinates.speed_motion * MPS_TO_NMPH;
#endif

  unsigned knots = (unsigned)(value * 10.0f + 0.5f);
  *p++ = knots / 1000 + '0';
  knots %= 1000;
  *p++ = knots / 100 + '0';
  knots %= 100;
  *p++ = knots / 10 + '0';
  *p++ = '.';
  *p++ = knots % 10 + '0';
  *p++ = ',';

#if 0
  float true_track =
      my_atan2f( gnss.coordinates.velocity.e[EAST], gnss.coordinates.velocity.e[NORTH]);
#else
  float true_track = gnss.coordinates.heading_motion;
#endif
  int angle_10 = true_track * 10.0 + 0.5;
  if( angle_10 < 0)
    angle_10 += 3600;

  *p++ = angle_10 / 1000 + '0';
  angle_10 %= 1000;
  *p++ = angle_10 / 100 + '0';
  angle_10 %= 100;
  *p++ = angle_10 / 10 + '0';
  *p++ = '.';
  *p++ = angle_10 % 10 + '0';

  *p++ = ',';

  *p++ = (gnss.coordinates.day) / 10 + '0';
  *p++ = (gnss.coordinates.day) % 10 + '0';
  *p++ = (gnss.coordinates.month) / 10 + '0';
  *p++ = (gnss.coordinates.month) % 10 + '0';
  *p++ = ((gnss.coordinates.year)%100) / 10 + '0';
  *p++ = ((gnss.coordinates.year)%100) % 10 + '0';

  *p++ = ',';
  *p++ = ',';
  *p++ = ',';
  *p++ = 'A';
  *p++ = 0;

  return p;
}

char *format_GGA(const GNSS_type &gnss, char *p)
{
  strcpy (p, (const char *) "$GPGGA,");

  while (*p)
    ++p;

  *p++ = (gnss.coordinates.hour)   / 10 + '0';
  *p++ = (gnss.coordinates.hour)   % 10 + '0';
  *p++ = (gnss.coordinates.minute) / 10 + '0';
  *p++ = (gnss.coordinates.minute) % 10 + '0';
  *p++ = (gnss.coordinates.second) / 10 + '0';
  *p++ = (gnss.coordinates.second) % 10 + '0';
  *p++ = '.';
  *p++ = '0';
  *p++ = '0';
  *p++ = ',';

  p = angle_format (gnss.coordinates.latitude, p, 'N', 'S');
  *p++ = ',';

  p = angle_format (gnss.coordinates.longitude, p, 'E', 'W');
  *p++ = ',';

  *p++ = gnss.fix_type >= FIX_2d ? '1' : '0';
  *p++ = ',';

  *p++ = (gnss.num_SV) / 10 + '0';
  *p++ = (gnss.num_SV) % 10 + '0';
  *p++ = ',';

  *p++ = '0'; // fake HDOP
  *p++ = '.';
  *p++ = '0';
  *p++ = ',';

  uint32_t altitude_msl = gnss.coordinates.position.e[DOWN] * -10.0;
  *p++ = altitude_msl / 1000 + '0';
  altitude_msl %= 1000;
  *p++ = altitude_msl / 100 + '0';
  altitude_msl %= 100;
  *p++ = altitude_msl / 10 + '0';
  *p++ = '.';
  *p++ = altitude_msl % 10 + '0';
  *p++ = ',';
  *p++ = 'M';
  *p++ = ',';

  uint32_t geo_sep = gnss.coordinates.geo_sep_dm;
  *p++ = geo_sep / 1000 + '0';
  geo_sep %= 1000;
  *p++ = geo_sep / 100 + '0';
  geo_sep %= 100;
  *p++ = geo_sep / 10 + '0';
  geo_sep %= 10;
  *p++ = '.';
  *p++ = geo_sep + '0';
  *p++ = ',';
  *p++ = 'm';
  *p++ = ','; // no DGPS
  *p++ = ',';
  *p++ = 0;

  return p;
}

char *format_MWV ( float wind_north, float wind_east, char *p)
{
  strcpy (p, (const char *) "$GPMWV,");

//  wind_north = 3.0; // this setting reports 18km/h from 53 degrees
//  wind_east = 4.0;

  while (*p)
    ++p;

  float direction =
      my_atan2f( -wind_east, -wind_north);

  int angle_10 = direction * RAD_TO_DEGREE_10 + 0.5;
  if( angle_10 < 0)
    angle_10 += 3600;

  *p++ = angle_10 / 1000 + '0';
  angle_10 %= 1000;
  *p++ = angle_10 / 100 + '0';
  angle_10 %= 100;
  *p++ = angle_10 / 10 + '0';
  *p++ = '.';
  *p++ = angle_10 % 10 + '0';
  *p++ = ',';
  *p++ = 'T'; // true direction
  *p++ = ',';

  float value = VSQRTF(sqr( wind_north) + sqr( wind_east));

  unsigned wind = value * 10.0f;
  *p++ = wind / 1000 + '0';
  wind %= 1000;
  *p++ = wind / 100 + '0';
  wind %= 100;
  *p++ = wind / 10 + '0';
  *p++ = '.';
  *p++ = wind % 10 + '0';

  *p++ = ',';
  *p++ = 'M'; // m/s
  *p++ = ',';
  *p++ = 'A'; // valid
  *p++ = 0;

  return p;
}

char *format_PTAS1 ( float vario, float avg_vario, float altitude, float TAS, char *p)
{
  uint16_t i_vario = vario * MPS_TO_NMPH * 10 + 200.5;
  uint16_t i_avg_vario = avg_vario * MPS_TO_NMPH * 10 + 200.5;
  uint16_t i_altitude = altitude * METER_TO_FEET + 2000.5;
  uint16_t i_TAS = TAS * MPS_TO_NMPH + 0.5;

  strcpy (p, (const char *) "$PTAS1,");

  while (*p)
    ++p;

  *p++ = i_vario / 100 + '0';
  i_vario %= 100;
  *p++ = i_vario / 10 + '0';
  *p++ = i_vario % 10 + '0';
  *p++ = ',';

  *p++ = i_avg_vario / 100 + '0';
  i_avg_vario %= 100;
  *p++ = i_avg_vario / 10 + '0';
  *p++ = i_avg_vario % 10 + '0';
  *p++ = ',';

  *p++ = i_altitude / 10000 + '0';
  i_altitude %= 10000;
  *p++ = i_altitude / 1000 + '0';
  i_altitude %= 1000;
  *p++ = i_altitude / 100 + '0';
  i_altitude %= 100;
  *p++ = i_altitude / 10 + '0';
  *p++ = i_altitude % 10 + '0';
  *p++ = ',';

  *p++ = i_TAS / 100 + '0';
  i_TAS %= 100;
  *p++ = i_TAS / 10 + '0';
  *p++ = i_TAS % 10 + '0';

  *p++ = 0;

  return p;
}

inline char hex4( uint8_t data)
{
  return "0123456789ABCDEF"[data];
}

bool NMEA_checksum( const char *line)
 {
 	uint8_t checksum = 0;
 	if( line[0]!='$')
 		return false;
 	const char * p;
 	for( p=line+1; *p && *p !='*'; ++p)
 		checksum ^= *p;
 	return ( (p[0] == '*') && hex4( checksum >> 4) == p[1]) && ( hex4( checksum & 0x0f) == p[2]) && (p[3] == 0);
 }

char * NMEA_append_tail( char *p)
 {
 	uint8_t checksum = 0;
 	if( p[0]!='$')
 		return 0;
 	for( p=p+1; *p && *p !='*'; ++p)
 		checksum ^= *p;
 	p[0] = '*';
 	p[1] = hex4(checksum >> 4);
 	p[2] = hex4(checksum & 0x0f);
 	p[3] = '\r';
 	p[4] = '\n';
 	p[5] = 0;
 	return p+5;
 }

