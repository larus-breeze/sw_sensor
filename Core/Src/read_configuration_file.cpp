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

int read_identifier( char *s)
  {
    if( s[2] != ' ')
      return -1;
    int identifier = atoi( s);
    if( ( identifier > 0) && ( identifier < EEPROM_PARAMETER_ID_END))
	return identifier;
    return EEPROM_PARAMETER_ID_END; // error
  }

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
  status = EEPROM_initialize();
  ASSERT( ! status);
#endif

  // get and program all readable configuration lines
  while( file_reader.read_line( linebuffer))
    {
      EEPROM_PARAMETER_ID identifier = (EEPROM_PARAMETER_ID)read_identifier( linebuffer);
      if( identifier == EEPROM_PARAMETER_ID_END)
	continue;
      const persistent_data_t *param = find_parameter_from_ID(identifier);
      unsigned name_len = strlen( param->mnemonic);
      if( 0 != strncmp( param->mnemonic,linebuffer+3, name_len))
	continue;
      if( linebuffer[name_len+4] != '=')
	continue;
      float value = string2float( linebuffer + name_len + 6);

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
