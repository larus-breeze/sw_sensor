/***********************************************************************//**
 * @file		sensor_dump.cpp
 * @brief		Output routines to help the user calibrating the sensor
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

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
#include "sensor_dump.h"
#include "ascii_support.h"
#include "NMEA_format.h"
#include "embedded_math.h"
#include "uSD_handler.h"
#include "pt2.h"

COMMON uint64_t pabs_sum, samples, noise_energy;
COMMON pt2 <float, float> heading_decimator( 0.01);
COMMON pt2 <float, float> inclination_decimator( 0.01);
COMMON pt2 <float, float> voltage_decimator( 0.01);

#define RAD_2_DEGREES 57.296f

struct statistics
{
  float mean;
  float rms;
  uint64_t samples;
};

statistics stat;

void decimate_sensor_observations( const measurement_data_t &m, const state_vector_t &x)
{
  pabs_sum += (uint64_t)( m.static_pressure);
  ++samples;
  noise_energy += SQR( (uint64_t)( m.static_pressure + 0.5f) - pabs_sum / samples);
  heading_decimator.respond( x.euler.yaw);
  voltage_decimator.respond( m.supply_voltage);
  inclination_decimator.respond( ATAN2( x.nav_induction[DOWN], x.nav_induction[NORTH]));
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

void format_sensor_dump( const measurement_data_t &m, const D_GNSS_coordinates_t c, const state_vector_t x, string_buffer_t &NMEA_buf)
{
  char *s = NMEA_buf.string;

  append_string( s, "Sensor ID: ");
  utox( s, UNIQUE_ID[0]);
  append_string( s, (char*)" Firmware: ");
  append_string( s, GIT_TAG_INFO);
  newline( s);

  float squaresum;

  append_string( s, "Acc m/s^2 ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      squaresum += SQR( m.acc[i]);
      to_ascii_n_decimals( m.acc[i], 2, s);
      *s++ = ' ';
    }
  to_ascii_n_decimals( SQRT( squaresum), 2, s);
  append_string( s, "\r\n");

  append_string( s, "Gyro deg/s  ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      to_ascii_n_decimals( RAD_2_DEGREES * m.gyro[i], 1, s);
      *s++ = ' ';
    }
  newline( s);

  append_string( s, "Magn.Ind.Sens ");
  squaresum=0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      squaresum += SQR( m.mag[i]);
      to_ascii_n_decimals( m.mag[i], 2, s);
      *s++ = ' ';
    }
  to_ascii_n_decimals( SQRT( squaresum), 2, s);
  newline( s);

  append_string( s, "P_pitot / Pa ");
  to_ascii_n_decimals( m.pitot_pressure, 2, s);
  newline( s);

  statistics present_stat = get_sensor_data();
  if( present_stat.samples >= 100)
    {
      stat = present_stat;
      reset_sensor_data();
    }

  append_string( s, "Pabs / hPa ");
  to_ascii_n_decimals( stat.mean * 0.01f, 2, s);
  newline( s);

  append_string( s, "Pabs noise RMS / Pa ");
  to_ascii_n_decimals( stat.rms, 2, s);
  newline( s);

  append_string( s, "Sensor Temp = ");
  to_ascii_n_decimals( m.static_sensor_temperature, 2, s);
  newline( s);

  append_string( s, "U_batt = ");
  to_ascii_n_decimals( voltage_decimator.get_output(), 2, s);
  newline( s);

  append_string( s, "Sats: ");
  format_integer( s, c.SATS_number);

  append_string( s, " Speed-Accuracy = ");
  to_ascii_n_decimals( c.speed_acc, 2, s);

  append_string( s, "m/s, GNSS time: ");
  format_2_digits( s, c.hour);
  *s ++ = ':';
  format_2_digits( s, c.minute);
  *s ++ = ':';
  format_2_digits( s, c.second);
  newline( s);

  append_string( s, "Induction NED: ");
  for( unsigned i=0; i<3; ++i)
    {
      to_ascii_n_decimals( x.nav_induction[i], 2, s);
      *s++=' ';
    }

  append_string( s, " Strength = ");
  to_ascii_n_decimals( x.nav_induction.abs(), 2, s);
  newline( s);

  float heading = heading_decimator.get_output();
  if( heading < 0.0f)
    heading += 2.0f * M_PI_F;
  append_string( s, "AHRS-Heading = ");
  to_ascii_n_decimals( RAD_2_DEGREES * heading, 1, s);

  append_string( s, " Inclination = ");
  to_ascii_n_decimals( RAD_2_DEGREES * inclination_decimator.get_output(), 1, s);

  append_string( s, " MagAnomaly = ");
  to_ascii_n_decimals( x.magnetic_disturbance * 100.0f, 2, s);
  *s++ = '%';
  newline( s);

  if( (c.sat_fix_type & 0x03) == (SAT_FIX | SAT_HEADING))
    {
      float baselength = c.relPosNED.abs();
      append_string( s, "D-GNSS: BaseLength: ");
      to_ascii_n_decimals( baselength, 2, s);
      append_string( s, "m  SlaveDown = ");
      to_ascii_n_decimals( c.relPosNED[DOWN], 2, s);
      append_string( s, "m D-GNSS-Heading= ");
      to_ascii_n_decimals( RAD_2_DEGREES * c.relPosHeading, 1, s);
    }
  else
    append_string( s, "No D-GNSS-fix");

  newline( s);
  NMEA_buf.length = s - NMEA_buf.string;
}




