/**
 * @file 	read_configuration_file.cpp
 * @brief 	individual sensor configuration read routines
 * @author: 	Klaus Schaefer
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

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

#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "fatfs.h"
#include "common.h"
#include "ascii_support.h"
#include "embedded_memory.h"
#include "embedded_math.h"
#include "read_configuration_file.h"
#include "persistent_data.h"
#include "stdlib.h"

#define TEST_MODULE 0

#define LINELEN 50

class ASCII_file_reader
{
  enum { BUFLEN = 512 *2};
public:
  ASCII_file_reader( char * filename)
    : current(0),
      end(0),
      eof(true)
  {
    FRESULT fresult;
    fresult = f_open(&infile, filename, FA_READ);
    if( fresult != FR_OK)
      return;

    UINT bytesread;
    fresult = f_read(&infile, file_buffer, BUFLEN, &bytesread);
    if( (fresult != FR_OK) || (bytesread == 0))
      return;

    f_close( &infile);

    end = file_buffer + bytesread;
    current = file_buffer;
    eof=false;
  }

  // return true if next line read, false if EOF
  bool read_line( char * &target)
  {
    if( eof)
      return false;
    target = current;
    while( (current < end) && (*current != '\n'))
      ++current;
    while( (current < end) && (*current <= ' '))
      ++current;
    eof = current >= end;
    return true;
  }
  bool is_eof( void)
  {
    return eof;
  }
private:
  FIL infile;
  char file_buffer[BUFLEN];
  char * current;
  char * end;
  bool eof;
};

#if TEST_MODULE
unsigned write_EEPROM_value_dummy( EEPROM_PARAMETER_ID identifier, float value)
{
  return 0;
}
#endif

void read_configuration_file(void)
{
  ASCII_file_reader file_reader((char *)"sensor_config.txt");
  if( file_reader.is_eof())
    return;

  char *linebuffer;
  unsigned status;

#if ! TEST_MODULE
  lock_EEPROM( false);
#endif

  // get all readable configuration lines and program data into EEPROM
  while( file_reader.read_line( linebuffer))
    {
      if( linebuffer[2] != ' ')
        continue; // bad format

      EEPROM_PARAMETER_ID identifier = (EEPROM_PARAMETER_ID)atoi( linebuffer);
      const persistent_data_t *this_persistent_parameter = find_parameter_from_ID( identifier);

      if( this_persistent_parameter == 0)
	continue; // we did not find this one

      unsigned name_len = strlen( this_persistent_parameter->mnemonic);

      if( 0 != strncmp( this_persistent_parameter->mnemonic, linebuffer+3, name_len))
	continue; // parameter name does not match

      if( linebuffer[name_len+4] != '=')
	continue; // bad format again

      float value = atof( linebuffer + name_len + 6);

      if( this_persistent_parameter->is_an_angle)
	{
	  value *= M_PI_F / 180.0;
	  // map angle range
	  while( value > M_PI_F)
	    value -= M_PI_F * 2.0f;
	  while( value < -M_PI_F)
	    value += M_PI_F * 2.0f;
	}

#if TEST_MODULE
      status = write_EEPROM_value_dummy( identifier, value);
#else
      status = write_EEPROM_value( identifier, value);
#endif
      ASSERT( ! status);
    }

#if ! TEST_MODULE
  lock_EEPROM( true);
#endif
}
