/** ***********************************************************************
 * @file		NMEA_format.h
 * @brief		converters for NMEA string output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef APPLICATION_NMEA_FORMAT_H_
#define APPLICATION_NMEA_FORMAT_H_

#include "GNSS.h"

char *format_RMC (const GNSS_type &gps, char *p);
char *format_GGA (const GNSS_type &gps, char *p);
char *format_MWV ( float wind_north, float wind_east, char *p);
char *format_PTAS1 ( float vario, float avg_vario, float altitude, float TAS, char *p);
char *format_POV( float TAS, float pabs, float pitot, float TEK_vario, char *p);

char * NMEA_append_tail( char *p);
bool NMEA_checksum( const char *line);

#endif /* APPLICATION_NMEA_FORMAT_H_ */
