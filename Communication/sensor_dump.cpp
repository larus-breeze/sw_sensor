#include "sensor_dump.h"
#include "ascii_support.h"

uint64_t pabs_sum, samples, noise_energy;

#define RAD_2_DEGREES_100 5729.6f

struct statistics
{
  float mean;
  float rms;
  uint64_t samples;
};

statistics stat;

void decimate_sensor_observations( const output_data_t &output_data)
{
  pabs_sum += (uint64_t)(output_data.m.static_pressure);
  ++samples;
  noise_energy += SQR( (uint64_t)(output_data.m.static_pressure + 0.5f) - pabs_sum / samples);
}

statistics get_sensor_data( void)
{
  statistics retv;
  retv.mean = round( pabs_sum / samples);
  retv.rms = SQRT( (float)noise_energy / (float)samples);
  retv.samples=samples;
  return retv;
}
void reset_sensor_data( void)
{
  samples = pabs_sum = noise_energy = 0;
}

extern uint32_t UNIQUE_ID[4];

void format_sensor_dump( const output_data_t &output_data, string_buffer_t &NMEA_buf)
{
  char *s = NMEA_buf.string;

  s=append_string( s, "Sensor ID = ");
  s=utox( UNIQUE_ID[0], s);
  s=append_string( s, "\r\n");

  float squaresum;

  s=append_string( s, "acc   ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      squaresum += SQR( output_data.m.acc.e[i]);
      s = integer_to_ascii_2_decimals( 100.0f * output_data.m.acc.e[i] , s);
      *s++ = '\t';
    }
  s = integer_to_ascii_2_decimals( 100.0f * SQRT( squaresum) , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "gyro  ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      s = integer_to_ascii_2_decimals( RAD_2_DEGREES_100 * output_data.m.gyro.e[i] , s);
      *s++ = '\t';
    }
  s=append_string( s, "\r\n");

  s=append_string( s, "mag   ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      squaresum += SQR( output_data.m.mag.e[i]);
      s = integer_to_ascii_2_decimals( 100.0f * output_data.m.mag.e[i] , s);
      *s++ = '\t';
    }
  s = integer_to_ascii_2_decimals( 100.0f * SQRT( squaresum) , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "pitot / Pa ");
  s = integer_to_ascii_2_decimals( 100.0f * output_data.m.pitot_pressure , s);
  s=append_string( s, "\r\n");

  statistics present_stat = get_sensor_data();
  if( present_stat.samples >= 100)
    {
      stat = present_stat;
      reset_sensor_data();
    }

  s=append_string( s, "pabs / hPa ");
  s = integer_to_ascii_2_decimals( stat.mean , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "pabs RMS / Pa ");
  s = integer_to_ascii_2_decimals( stat.rms * 100.0f , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "temp  ");
  s = integer_to_ascii_2_decimals( 100.0f * output_data.m.static_sensor_temperature , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "Ubatt  ");
  s = integer_to_ascii_2_decimals( 100.0f * output_data.m.supply_voltage , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "GNSS time ");
  s = format_2_digits( s, output_data.c.hour);
  *s ++ = ':';
  s = format_2_digits( s, output_data.c.minute);
  *s ++ = ':';
  s = format_2_digits( s, output_data.c.second);
  s=append_string( s, "\r\n");

  s=append_string( s, "\r\n");

  *s = 0;
  NMEA_buf.length = s - NMEA_buf.string;
}




