#include "sensor_dump.h"
#include "ascii_support.h"

float pabs_mean, pabs_squared_mean;

ROM float BETA = 0.04;
ROM float ALFA = 1 - BETA;

ROM float RAD_2_DEGREES_100 = 5729.6f;

void decimate_sensor_dump( const output_data_t &output_data)
{
  pabs_mean = pabs_mean * ALFA + output_data.m.static_pressure * BETA;
  pabs_squared_mean = pabs_squared_mean * ALFA + SQR( output_data.m.static_pressure) * BETA;
}

void format_sensor_dump( const output_data_t &output_data, string_buffer_t &NMEA_buf)
{
  char *s = NMEA_buf.string;

  s=append_string( s, "acc   ");
  for( unsigned i=0; i<3; ++i)
    {
      s = integer_to_ascii_2_decimals( 100.0f * output_data.m.acc.e[i] , s);
      *s++ = '\t';
    }
  s=append_string( s, "\r\n");

  s=append_string( s, "gyro  ");
  for( unsigned i=0; i<3; ++i)
    {
      s = integer_to_ascii_2_decimals( RAD_2_DEGREES_100 * output_data.m.gyro.e[i] , s);
      *s++ = '\t';
    }
  s=append_string( s, "\r\n");

  s=append_string( s, "mag   ");
  for( unsigned i=0; i<3; ++i)
    {
      s = integer_to_ascii_2_decimals( 100.0f * output_data.m.mag.e[i] , s);
      *s++ = '\t';
    }
  s=append_string( s, "\r\n");

  s=append_string( s, "pitot / Pa ");
  s = integer_to_ascii_2_decimals( 100.0f * output_data.m.pitot_pressure , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "pabs / hPa ");
  s = integer_to_ascii_2_decimals( pabs_mean , s);
  s=append_string( s, "\r\n");

#if 1
  s=append_string( s, "pabs RMS / Pa ");
  s = integer_to_ascii_2_decimals( 100.0f * SQRT( pabs_squared_mean - SQR( pabs_mean)) , s);
  s=append_string( s, "\r\n");
#endif

  s=append_string( s, "temp  ");
  s = integer_to_ascii_2_decimals( 100.0f * output_data.m.static_sensor_temperature , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "Ubatt  ");
  s = integer_to_ascii_2_decimals( 100.0f * output_data.m.supply_voltage , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "GNSS time ");
  s = format_integer( output_data.c.hour , s);
  *s ++ = ':';
  s = format_integer( output_data.c.minute , s);
  *s ++ = ':';
  s = format_integer( output_data.c.second , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "\r\n");

  *s = 0;
  NMEA_buf.length = s - NMEA_buf.string;
}




