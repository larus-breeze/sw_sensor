#include "sensor_dump.h"
#include "ascii_support.h"
#include "NMEA_format.h"
#include "embedded_math.h"
#include "uSD_handler.h"

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
  s=utox( s, UNIQUE_ID[0]);
  s=append_string( s, "\r\n");

  float squaresum;

  s=append_string( s, "acc   ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      squaresum += SQR( output_data.m.acc.e[i]);
      s = integer_to_ascii_2_decimals( 100.0f * output_data.m.acc.e[i] , s);
      *s++ = ' ';
    }
  s = integer_to_ascii_2_decimals( 100.0f * SQRT( squaresum) , s);
  s=append_string( s, "\r\n");

  s=append_string( s, "gyro  ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      s = integer_to_ascii_2_decimals( RAD_2_DEGREES_100 * output_data.m.gyro.e[i] , s);
      *s++ = ' ';
    }
  s=append_string( s, "\r\n");

  s=append_string( s, "mag   ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      squaresum += SQR( output_data.m.mag.e[i]);
      s = integer_to_ascii_2_decimals( 100.0f * output_data.m.mag.e[i] , s);
      *s++ = ' ';
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

  s=append_string( s, "NAV Induction: ");
  for( unsigned i=0; i<3; ++i)
    {
      s = integer_to_ascii_2_decimals( 100.0f * output_data.nav_induction_gnss.e[i], s);
      *s++=' ';
    }

  s=append_string( s, "-> ");
  s = integer_to_ascii_2_decimals( 100.0f * output_data.nav_induction_gnss.abs(), s);
  s=append_string( s, "\r\n");

  s=append_string( s, "Mag. Inclination: ");
  float inclination = ATAN2( output_data.nav_induction_gnss.e[DOWN], output_data.nav_induction_gnss.e[NORTH]);
  s = integer_to_ascii_2_decimals( RAD_2_DEGREES_100 * inclination, s);

  s=append_string( s, "\r\n");

  // here we report a fake vario value indicating the maximum magnetic field strength
  if( magnetic_gound_calibration)
    {
      unsigned max_acc_value_axis = 0;
      float max_abs_acceleration = -1.0f;
      for( unsigned axis = 0; axis < 3; ++axis)
        if( abs( output_data.m.acc.e[axis]) > max_abs_acceleration)
          {
    	max_abs_acceleration = fabs( output_data.m.acc.e[axis]);
    	max_acc_value_axis = axis;
          }

      float vario = (fabs(output_data.m.mag.e[max_acc_value_axis]) - 0.5f) * 5.0f;
      format_PLARV ( vario, 0.0f, 0.0f, 0.0f, s);
      s = NMEA_append_tail( s);
    }
  // here we report a fake vario value indicating the magnetic error
  else
    {
      float vario = output_data.magnetic_disturbance * -5.0f;
      format_PLARV ( vario, 0.0f, 0.0f, 0.0f, s);
      s = NMEA_append_tail( s);
    }
  s=append_string( s, "\r\n");

  *s = 0;
  NMEA_buf.length = s - NMEA_buf.string;
}




