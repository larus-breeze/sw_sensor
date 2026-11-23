/** *****************************************************************************
 * @file    	uSD_helpers.cpp
 * @brief   	uSD File r/w helper functions
 * @author  	Dr. Klaus Schaefer
 * @copyright 	Copyright 2026 Dr. Klaus Schaefer. All rights reserved.
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
#include "SHA256.h"
#include "GNSS.h"
#include "persistent_data_file.h"
#include "flexible_log_file_implementation.h"
#include "EEPROM_data_file_implementation.h"
#include "communicator.h"
#include "system_state.h"
#include "uSD_helpers.h"
#include "fatfs_access.h"
#include "uSD_handler.h"
#include "emergency.h"
#include "watchdog_handler.h"
#include "git-commit-version.h"
#include <stddef.h>
#include <string.h>

COMMON char *crashfile;
COMMON unsigned crashline;

ROM uint8_t SHA_INITIALIZATION[] = "presently a well-known string";

void sync_logger(void);
extern  uint32_t UNIQUE_ID[4];
extern SD_HandleTypeDef hsd;

#define MEM_BUFSIZE 16384 // bytes
COMMON uint8_t __ALIGNED(32) mem_buffer[MEM_BUFSIZE];
COMMON flexible_log_file_implementation_t flex_file(
    (uint32_t *)mem_buffer,
    MEM_BUFSIZE / sizeof( uint32_t),
    sync_logger
    );

bool write_block( uint32_t * begin, uint32_t size_words)
{
  return flex_file.write_block ( begin, size_words);
}

//!< format date and time from sat fix data
char * format_date_time( char * target, const D_GNSS_coordinates_t &coordinates)
{
  format_2_digits( target, coordinates.year);
  format_2_digits( target, coordinates.month);
  format_2_digits( target, coordinates.day);
  *target ++ = '_';
  format_2_digits( target, coordinates.hour);
  format_2_digits( target, coordinates.minute);
  format_2_digits( target, coordinates.second);
  *target=0;
  return target;
}

extern RestrictedTask uSD_handler_task; // will come downwards ...

//!< Crash information that survives the reset: it is written to the SD
//!< card after the restart, see documentation/wlan_link.md, "Crash dumps".
//!< Byte arrays, not the original types: a constructor run at start-up
//!< would overwrite the saved data.
typedef struct
{
  uint32_t magic;
  uint32_t firmware_id;
  uint32_t user_initiated; // intentional reset: *.REBOOT, else *.CRASHDUMP
  char reason[40];         // of an intentional reset
  uint32_t line;
  char file[48];
  char task[configMAX_TASK_NAME_LEN];
  register_dump_t registers;
  uint32_t fpu[32];
  uint32_t system_state;
  uint32_t coordinates[(sizeof(D_GNSS_coordinates_t) + 3) / 4];
  uint32_t observations[(sizeof(measurement_data_t) + 3) / 4];
  uint32_t state_vector[(sizeof(state_vector_t) + 3) / 4];
  uint32_t time;        // seconds since 1970, names the files
  uint32_t time_source; // crash_time_source_t
  uint32_t checksum; // last
} crash_report_t;

typedef enum { TIME_UNKNOWN, TIME_GNSS, TIME_ESTIMATED } crash_time_source_t;

//!< Time of the last report, to date a crash that follows it before the
//!< next GNSS fix (e.g. while the report of the first crash is written)
typedef struct
{
  uint32_t magic;
  uint32_t time;
  uint32_t reset_count;
  uint32_t checksum; // last
} last_crash_time_t;

#define CRASH_REPORT_MAGIC 0x43524153u

// NoInit (STM32F407VGTX_FLASH.ld): not zeroed by the start-up code
static crash_report_t crash_report __attribute__((section("noinit_crash_report")));
static last_crash_time_t last_crash_time __attribute__((section("noinit_crash_report")));

static uint32_t fnv1a (const void *data, size_t size, uint32_t hash = 2166136261u)
{
  const uint8_t *p = (const uint8_t *) data;
  while (size--)
    hash = (hash ^ *p++) * 16777619u;
  return hash;
}

// a report from another firmware build has a different layout
static uint32_t firmware_id (void)
{
  return GIT_TAG_DEC ^ fnv1a (GIT_COMMIT_HASH, sizeof(GIT_COMMIT_HASH) - 1);
}

static uint32_t crash_report_checksum (void)
{
  return fnv1a (&crash_report, offsetof(crash_report_t, checksum));
}

// days since 1970-01-01 (H. Hinnant, "chrono-Compatible Low-Level Date Algorithms")
static int32_t days_from_civil (int32_t y, uint32_t m, uint32_t d)
{
  y -= m <= 2;
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const uint32_t yoe = (uint32_t) (y - era * 400);
  const uint32_t doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int32_t) doe - 719468;
}

static void civil_from_days (int32_t z, int32_t &y, uint32_t &m, uint32_t &d)
{
  z += 719468;
  const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  const uint32_t doe = (uint32_t) (z - era * 146097);
  const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const uint32_t mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y = (int32_t) yoe + era * 400 + (m <= 2);
}

//!< YYMMDD_HHMMSS like format_date_time(), from seconds since 1970
static char * format_time (char * target, uint32_t time)
{
  int32_t y;
  uint32_t m, d;
  civil_from_days ((int32_t) (time / 86400), y, m, d);
  uint32_t s = time % 86400;
  format_2_digits( target, (uint8_t) (y % 100));
  format_2_digits( target, (uint8_t) m);
  format_2_digits( target, (uint8_t) d);
  *target ++ = '_';
  format_2_digits( target, (uint8_t) (s / 3600));
  format_2_digits( target, (uint8_t) (s / 60 % 60));
  format_2_digits( target, (uint8_t) (s % 60));
  *target=0;
  return target;
}

//!< Crash time from GNSS. Without a fix (shortly after a reset) it is
//!< estimated from the previous report plus the uptime, if that report was
//!< made in the boot right before this one.
static void date_crash (void)
{
  crash_report.time_source = TIME_UNKNOWN;
  crash_report.time = 0;

  if ((coordinates.sat_fix_type & SAT_FIX) && (coordinates.year != 0))
    {
      crash_report.time = (uint32_t) days_from_civil (2000 + coordinates.year, coordinates.month, coordinates.day) * 86400
	  + coordinates.hour * 3600 + coordinates.minute * 60 + coordinates.second;
      crash_report.time_source = TIME_GNSS;
    }
  else if ((last_crash_time.magic == CRASH_REPORT_MAGIC)
      && (last_crash_time.checksum == fnv1a (&last_crash_time, offsetof(last_crash_time_t, checksum)))
      && (reset_count - last_crash_time.reset_count <= 1))
    {
      // +1 s: never the same file name as the previous report
      crash_report.time = last_crash_time.time + 1 + xTaskGetTickCount () / configTICK_RATE_HZ;
      crash_report.time_source = TIME_ESTIMATED;
    }

  if (crash_report.time_source != TIME_UNKNOWN)
    {
      last_crash_time.magic = CRASH_REPORT_MAGIC;
      last_crash_time.time = crash_report.time;
      last_crash_time.reset_count = reset_count;
      last_crash_time.checksum = fnv1a (&last_crash_time, offsetof(last_crash_time_t, checksum));
    }
}

bool crash_report_pending (void)
{
  return (crash_report.magic == CRASH_REPORT_MAGIC)
      && (crash_report.firmware_id == firmware_id ())
      && (crash_report.checksum == crash_report_checksum ());
}

static void save_crash_report (bool user_initiated)
{
  crash_report.magic = 0;
  crash_report.firmware_id = firmware_id ();
  crash_report.user_initiated = user_initiated;
  crash_report.line = crashline;

  const char *reason = (user_initiated && user_reset_reason) ? user_reset_reason : "";
  unsigned i = 0;
  for (; (i < sizeof(crash_report.reason) - 1) && reason[i]; ++i)
    crash_report.reason[i] = reason[i];
  crash_report.reason[i] = 0;

  const char *file = crashfile ? crashfile : "";
  for (i = 0; (i < sizeof(crash_report.file) - 1) && file[i]; ++i)
    crash_report.file[i] = file[i];
  crash_report.file[i] = 0;

  const char *task = register_dump.active_TCB ? pcTaskGetName ((TaskHandle_t) register_dump.active_TCB) : "";
  for (i = 0; (i < sizeof(crash_report.task) - 1) && task[i]; ++i)
    crash_report.task[i] = task[i];
  crash_report.task[i] = 0;

  crash_report.registers = register_dump;
  memcpy (crash_report.fpu, FPU_register_dump, sizeof(crash_report.fpu));
  crash_report.system_state = system_state;
  memcpy (crash_report.coordinates, &coordinates, sizeof(D_GNSS_coordinates_t));
  memcpy (crash_report.observations, &observations, sizeof(measurement_data_t));
  memcpy (crash_report.state_vector, &state_vector, sizeof(state_vector_t));
  date_crash ();

  crash_report.magic = CRASH_REPORT_MAGIC;
  crash_report.checksum = crash_report_checksum ();
}

//!< Saves the crash information and resets. Nothing is written to the SD
//!< card here: a WLAN session may hold it, see write_pending_crash_report().
void write_crash_dump( bool user_initiated)
{
  uSD_handler_task.set_priority(configMAX_PRIORITIES - 1); // set it to highest priority

  acquire_privileges(); // ... need to access trace data etc

#if configUSE_TRACE_FACILITY
  vTraceStop(); // keep the events leading up to the crash
#endif

  save_crash_report (user_initiated);

  delay( 100); // let a running SD transfer finish
  HAL_SD_DeInit (&hsd);
  NVIC_SystemReset ();
}

static void write_line (FIL &fp, char *buffer, const char *label, uint32_t value)
{
  char *next = buffer;
  UINT writtenBytes = 0;
  append_string( next, (char *) label);
  utox( next, value);
  newline( next);
  f_write (&fp, buffer, next - buffer, &writtenBytes);
}

//!< Writes the report saved by write_crash_dump() before the reset.
//!< Caller holds fatfs_lock().
void write_pending_crash_report (void)
{
  if ( ! crash_report_pending ())
    return;

  acquire_privileges(); // the report and the trace buffer are in protected RAM

  FIL fp;
  char *buffer = (char *)mem_buffer; // use global buffer here
  char *next = buffer;
  UINT writtenBytes = 0;
  const char *extension = crash_report.user_initiated ? ".REBOOT" : ".CRASHDUMP";

  // all files of one report share the base name; a suffix keeps earlier
  // reports with the same time (or without one) from being overwritten
  char base_name[24];
  FILINFO filinfo;
  for (unsigned n = 1; n < 100; ++n)
    {
      next = format_time( base_name, crash_report.time);
      if (n > 1)
	{
	  *next++ = '_';
	  next = my_itoa( next, n);
	  *next = 0;
	}
      next = buffer;
      append_string( next, base_name);
      append_string( next, extension);
      if (f_stat (buffer, &filinfo) == FR_NO_FILE)
	break;
    }

  next = buffer;
  append_string( next, base_name);
  append_string( next, (char *) extension);

  if (f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      next = buffer;
      append_string( next, (char*)"Firmware: ");
      append_string( next, GIT_TAG_INFO);
      newline( next);

      static const char * const time_text[] = { "unknown", "GNSS", "estimated (previous crash + uptime)" };
      append_string( next, (char*)"Time:     ");
      append_string( next, time_text[crash_report.time_source <= TIME_ESTIMATED ? crash_report.time_source : TIME_UNKNOWN]);
      newline( next);

      if (crash_report.user_initiated)
	{
	  append_string( next, (char*)"Reboot:   ");
	  append_string( next, crash_report.reason[0] ? crash_report.reason : (char*)"unknown");
	  newline( next);
	}

      append_string( next, (char*)"Hardware: ");
      utox( next, UNIQUE_ID[0], 8);
      newline( next);

      append_string( next, crash_report.file);
      append_string( next, (char*)" Line: ");
      next = my_itoa( next, crash_report.line);
      newline( next);

      append_string( next, (char*)"Task:     ");
      append_string( next, crash_report.task);
      newline( next);

      f_write (&fp, buffer, next-buffer, &writtenBytes);

      const register_dump_t &r = crash_report.registers;
      write_line (fp, buffer, "IPSR:     ", r.IPSR);
      write_line (fp, buffer, "PC:       ", r.stacked_pc);
      write_line (fp, buffer, "LR:       ", r.stacked_lr);
      write_line (fp, buffer, "BusFA:    ", r.Bus_Fault_Address);
      write_line (fp, buffer, "MemA:     ", r.Bad_Memory_Address);
      write_line (fp, buffer, "MemFS:    ", r.Memory_Fault_status);
      write_line (fp, buffer, "FPU_S:    ", r.FPU_StatusControlRegister);
      write_line (fp, buffer, "UsgFS:    ", r.Usage_Fault_Status_Register);
      write_line (fp, buffer, "HardFS:   ", r.Hard_Fault_Status);

      for( unsigned i=0; i<32; ++i)
	if( crash_report.fpu[i] != 0x00) // only if the dump is populated
	  {
	    next = buffer;
	    append_string( next, (char*)"FPU dump:");
	    newline( next);
	    for( unsigned k=0; k<32; ++k)
	      {
		utox( next, crash_report.fpu[k]);
		newline( next);
	      }
	    f_write (&fp, buffer, next-buffer, &writtenBytes);
	    break;
	  }
      f_close(&fp);
    }

#if configUSE_TRACE_FACILITY // ************************************************
  extern RecorderDataType myTraceBuffer;

  next = buffer;
  append_string( next, base_name);
  append_string (next, ".bin");
  if (f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      for( uint8_t *ptr=(uint8_t *)&myTraceBuffer; ptr < (uint8_t *)&myTraceBuffer + sizeof(RecorderDataType); ptr += MEM_BUFSIZE)
	{
	  unsigned blocksize = (uint8_t *)&myTraceBuffer + sizeof(RecorderDataType) - ptr;
	  if( blocksize > MEM_BUFSIZE)
	    blocksize = MEM_BUFSIZE;
	  // data needs to be copied out of CCM RAM
	  memcpy( mem_buffer, ptr, blocksize);
	  f_write (&fp, mem_buffer, blocksize, &writtenBytes);
	  if( writtenBytes < blocksize)
	    break;
	}
      f_close(&fp);
    }
#endif // ************************************************************************

  // one complete set of output data independent of data logging status
  next = buffer;
  append_string( next, base_name);
  *next++ = '.';
  *next++  = 'f';
  format_2_digits( next, sizeof( state_vector_t) / sizeof(float));

  if (f_open ( &fp, buffer, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      f_write (&fp, (uint8_t*) crash_report.observations, sizeof( measurement_data_t), &writtenBytes);
      f_write (&fp, (uint8_t*) crash_report.coordinates,  sizeof( D_GNSS_coordinates_t), &writtenBytes);
      f_write (&fp, (uint8_t*) &crash_report.system_state, sizeof( uint32_t), &writtenBytes);
      f_write (&fp, (uint8_t*) crash_report.state_vector,  sizeof( state_vector_t), &writtenBytes);
      f_close( &fp);
    }

  crash_report.magic = 0; // written once, even if the SD card failed

#if configUSE_TRACE_FACILITY
  // tracing was held back since the start-up, see initialize_RTOS_memory()
  prvTraceInitCortexM ();
  vTraceClear ();
  vTraceStart ();
#endif
}

COMMON uint8_t firmware_SHA256_digest[32];

void make_firmware_digest( void)
{
  SHA256 sha;
  sha.update( SHA_INITIALIZATION, sizeof( SHA_INITIALIZATION));

  extern uint8_t * __fini_array_end;
  unsigned block_size = 1024;
  for( uint8_t * block_start = (uint8_t *)0x08000000;  block_start < __fini_array_end; block_start += block_size)
    {
      uint8_t * block_end = block_start + block_size;
      if( block_end > __fini_array_end)
	block_end = __fini_array_end;
      sha.update( block_start, block_end - block_start);
      delay(1); // beware of our watchdog !
    }
  sha.make_digest(firmware_SHA256_digest);
}

bool write_EEPROM_dump( const char * file_path)
{
  FRESULT fresult;
  FIL fp;
  char buffer[128];
  char *next = buffer;
  int32_t writtenBytes = 0;
  SHA256 sha;

  append_string (next, file_path);
  append_string (next, ".EEPROM");

  fresult = f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    return fresult;

  sha.reset();
  sha.update( SHA_INITIALIZATION, sizeof( SHA_INITIALIZATION));

  newline(next); // first line = my filename (incl. time)
  append_string( next, "SHA256(Flash Program) = \r\n");

  for( unsigned i=0; i<16; ++i)
      utox( next, (uint32_t)(firmware_SHA256_digest[i]), 2);
  newline(next);
  for( unsigned i=0; i<16; ++i)
      utox( next, (uint32_t)(firmware_SHA256_digest[i+16]), 2);
  newline(next);

  (void)f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
  sha.update( (uint8_t *)buffer, next-buffer);

  next = buffer;
  append_string( next, "Fw = ");
  append_string( next, GIT_TAG_INFO);
  newline(next);
  append_string( next, "Hw = ");
  utox( next, UNIQUE_ID[0], 8);
  utox( next, UNIQUE_ID[1], 8);
  utox( next, UNIQUE_ID[2], 8);
  utox( next, UNIQUE_ID[3], 8);
  newline(next);

  fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
  if( (fresult != FR_OK) || (writtenBytes != (next-buffer)))
    {
      f_close(&fp);
      return fresult; // give up ...
    }
  sha.update( (uint8_t *)buffer, next-buffer);

  for (unsigned index = 0; index < PERSISTENT_DATA_ENTRIES; ++index)
    {
      float value;
      bool result = read_EEPROM_value (PERSISTENT_DATA[index].id, value);
      if (result == HAL_OK)
	{
	  if (PERSISTENT_DATA[index].is_an_angle)
	    value *= 180.0 / M_PI_F; // format it human readable

	  next = buffer;
	  append_string (next, PERSISTENT_DATA[index].mnemonic);
	  append_string (next, " = ");

	  if (PERSISTENT_DATA[index].id == HORIZON)
	    {
	      uint32_t date = *(uint32_t*) &value;
	      next = my_itoa (next, date >> 16);
	      *next++ = '-';
	      next = my_itoa (next, (date & 0x0f00) >> 8);
	      *next++ = '-';
	      next = my_itoa (next, date & 0x1f);
	      newline (next);
	    }
	  else
	    {
	      next = my_ftoa (next, value);
	      newline (next);
	    }

	  fresult = f_write (&fp, buffer, next - buffer, (UINT*) &writtenBytes);
	  sha.update ((uint8_t*) buffer, next - buffer);
	  if ((fresult != FR_OK) || (writtenBytes != (next - buffer)))
	    {
	      f_close (&fp);
	      return fresult; // give up ...
	    }
	}
    }

      {
      float32_t mag_calib_param[4*3];

      if( permanent_data_file.retrieve_data( MAG_SENSOR_XFER_MATRIX, 4*3, (uint32_t *)mag_calib_param))
	{
	  for( unsigned i=0; i< 4*3; ++i)
	    {
	      next = buffer;
	      append_string( next, "Mag_");
	      utox( next, i, 1);
	      append_string( next, " = ");
	      next = my_ftoa (next, mag_calib_param[i]);
	      newline( next);
	      sha.update( (uint8_t *)buffer, next-buffer);
	      fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
	      if( (fresult != FR_OK) || (writtenBytes != (next-buffer)))
		{
		  f_close(&fp);
		  return fresult; // give up ...
		}
	    }
	}

      if( permanent_data_file.retrieve_data( EXT_MAG_SENSOR_XFER_MATRIX, 4*3, (uint32_t *)mag_calib_param))
	{
	  for( unsigned i=0; i< 4*3; ++i)
	    {
	      next = buffer;
	      append_string( next, "XMag_");
	      utox( next, i, 1);
	      append_string( next, " = ");
	      next = my_ftoa (next, mag_calib_param[i]);
	      newline( next);
	      sha.update( (uint8_t *)buffer, next-buffer);
	      fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
	      if( (fresult != FR_OK) || (writtenBytes != (next-buffer)))
		{
		  f_close(&fp);
		  return fresult; // give up ...
		}
	    }
	}

      {
      float32_t acc_calib_param[6];

      if( permanent_data_file.retrieve_data( ACCELEROMETER_CALIBRATION, 6, (uint32_t *)acc_calib_param))
	{
	  for( unsigned i=0; i < 6; ++i)
	    {
	      next = buffer;
	      append_string( next, "Acc_");
	      utox( next, i, 1);
	      append_string( next, " = ");
	      next = my_ftoa (next, acc_calib_param[i]);
	      newline( next);
	      sha.update( (uint8_t *)buffer, next-buffer);
	      fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
	      if( (fresult != FR_OK) || (writtenBytes != (next-buffer)))
		{
		  f_close(&fp);
		  return fresult; // give up ...
		}
	    }
	}
      }
    }

  uint16_t option = *(uint16_t *) 0x1fffc000;
  next = buffer;
  append_string( next, "Option bytes = ");
  utox( next, option >> 8, 2);
  newline(next);
  sha.update( (uint8_t *)buffer, next-buffer);
  fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
  if( (fresult != FR_OK) || (writtenBytes != (next-buffer)))
    {
      f_close(&fp);
      return fresult; // give up ...
    }

  uint8_t digest[32];
  sha.make_digest(digest);
  next = buffer;
  append_string( next, "SHA256(text above) = \r\n");

  for( unsigned i=0; i<16; ++i)
      utox( next, (uint32_t)(digest[i]), 2);
  newline(next);

  for( unsigned i=0; i<16; ++i)
      utox( next, (uint32_t)(digest[i+16]), 2);

  newline(next);

  fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
  f_close(&fp);
  return fresult;
}

//!< true if the header block holds a firmware image for this sensor
//!< (pack.py magic number + hardware version)
static bool is_sensor_firmware_image (const uint8_t *header)
{
  uint32_t file_hw_version = header[23] | (header[22] << 8)
      | (header[21] << 16) | (header[20] << 24);
  uint64_t file_magic_number = 0;
  for (int i = 7; i >= 0; i--)
    {
      file_magic_number <<= 8;
      file_magic_number |= (uint64_t) header[i];
    }
  return (file_hw_version == 0x01010100)
      && (file_magic_number == 0x1c8073ab20853579);
}

static uint32_t firmware_image_sw_version (const uint8_t *header)
{
  return header[27] | (header[26] << 8) | (header[25] << 16) | (header[24] << 24);
}

//!< Caller must hold fatfs_lock(). Deletes all firmware images for this
//!< sensor that are not newer than the running firmware. The version is
//!< re-checked per file: a WLAN upload may have stored a newer image since
//!< read_software_update() scanned the card.
static void delete_installed_firmware_images_locked (void)
{
  FIL the_file;
  FILINFO fno;
  DIR dj;
  UINT bytes_read;

  // Restart the scan after each delete instead of unlinking while the
  // directory is being iterated. Non-images are skipped by the scan, so
  // this terminates once no image is left (or a delete fails).
  for (;;)
    {
      bool image_found = false;
      FRESULT fresult = f_findfirst (&dj, &fno, "", "????*.bin");
      while ((fresult == FR_OK) && (fno.fname[0] != 0))
	{
	  if (f_open (&the_file, (char*) &fno.fname[0], FA_READ) == FR_OK)
	    {
	      fresult = f_read (&the_file, mem_buffer, 32, &bytes_read);
	      f_close (&the_file);
	      if ((fresult == FR_OK) && (bytes_read == 32)
		  && is_sensor_firmware_image (mem_buffer)
		  && (firmware_image_sw_version (mem_buffer) <= GIT_TAG_DEC))
		{
		  image_found = true;
		  break;
		}
	    }
	  fresult = f_findnext (&dj, &fno);
	}
      f_closedir (&dj);

      if (! image_found)
	return;
      if (f_unlink ((char*) &fno.fname[0]) != FR_OK)
	return; // e.g. read-only file - give up instead of looping forever
    }
}

//!< create-git-info-header.py's GIT_TAG_DEC when no version tag was found
#define GIT_TAG_DEC_UNKNOWN 0xffffffffu

//!< Only called after a complete scan found nothing newer, i.e. after the
//!< update has actually been installed - deleting right after staging
//!< would lose the image if the copy routine fails, as the staging area is
//!< erased on every boot. See documentation/wlan_link.md.
static void delete_installed_firmware_images (void)
{
  // Unknown running version: every image compares as "not newer".
  if (GIT_TAG_DEC == GIT_TAG_DEC_UNKNOWN)
    return;

  // A WLAN upload holds the lock for its whole session. Skip instead of
  // waiting - the next boot repeats the cleanup.
  if (! fatfs_lock (pdMS_TO_TICKS (FATFS_ACCESS_WLAN_TIMEOUT_MS)))
    return;
  delete_installed_firmware_images_locked ();
  fatfs_unlock ();
}

//!< find software image file and load it if applicable
bool read_software_update (void)
{
  FIL the_file;
  FILINFO fno;
  FRESULT fresult;
  UINT bytes_read;
  DIR dj;

  uint32_t highest_sw_version_found = 0;
  bool scan_complete = false;
  char highest_sw_version_fname[_MAX_LFN + 1];

  uint32_t flash_address = 0x60000;
  unsigned status;
  bool last_block_read = false;

  // find all *.bin files which could be software update images
  fresult = f_findfirst (&dj, &fno, "", "????*.bin");
  if (fresult != FR_OK)
    return false;

  while (fresult == FR_OK)
    {
      // try to open the next image file
      fresult = f_open (&the_file, (char*) &fno.fname[0], FA_READ);
      if (fresult != FR_OK)
	return false;

      // read first block to check hardware and firmware version
      fresult = f_read (&the_file, mem_buffer, MEM_BUFSIZE, &bytes_read);
      if (fresult != FR_OK)
	{
	  // _FS_LOCK (ffconf.h): an early return without closing leaves this
	  // slot open forever, later rejecting a same-named WLAN upload.
	  f_close (&the_file);
	  return false;
	}
      f_close (&the_file);

      if (is_sensor_firmware_image (mem_buffer))
	{
	  uint32_t file_sw_version = firmware_image_sw_version (mem_buffer);
	  if (file_sw_version > highest_sw_version_found)
	    {

	      // Search for the highest version and copy filename
	      highest_sw_version_found = file_sw_version;
	      for (int i = 0; i < _MAX_LFN; i++)
		{
		  highest_sw_version_fname[i] = fno.fname[i];
		}
	    }
	}

      fresult = f_findnext (&dj, &fno);
      if (fresult != FR_OK)
	break; // enumeration error - highest_sw_version_found may be incomplete
      if (fno.fname[0] == 0)
	{
	  scan_complete = true;
	  break; // no more files found
	}
    }

#if DISALLOW_DOWNGRADE
  if (highest_sw_version_found <= GIT_TAG_DEC)
    {
      // Nothing new: the update (if any) is installed, the images are obsolete.
      // After an enumeration error a newer image may have gone unseen - keep all.
      if (scan_complete && (highest_sw_version_found != 0))
	delete_installed_firmware_images ();
      return false;
    }

#endif

  // try to open new software image file
  fresult = f_open (&the_file, highest_sw_version_fname, FA_READ);
  if (fresult != FR_OK)
    return false;

  // read first block
  fresult = f_read (&the_file, mem_buffer, MEM_BUFSIZE, &bytes_read);
  if ((fresult != FR_OK) || (bytes_read < MEM_BUFSIZE))
    {
      f_close (&the_file);
      return false;
    }

  // Compare the whole image, not just the first block - an interrupted
  // earlier write could leave the first block(s) correct but the rest
  // stale.
  uint32_t *mem_ptr;
  uint32_t *flash_ptr;
  bool image_is_equal = true;
  uint32_t compare_address = flash_address;
  UINT compare_bytes_read = bytes_read;

  for (;;)
    {
      for (mem_ptr = (uint32_t*) mem_buffer, flash_ptr = (uint32_t*) compare_address;
	  mem_ptr < (uint32_t*) (mem_buffer + compare_bytes_read); ++mem_ptr, ++flash_ptr)
	if (*mem_ptr != *flash_ptr)
	  {
	    image_is_equal = false;
	    break;
	  }

      if ((! image_is_equal) || (compare_bytes_read < MEM_BUFSIZE))
	break; // mismatch found, or that was the last (short) block - done either way

      compare_address += compare_bytes_read;
      fresult = f_read (&the_file, mem_buffer, MEM_BUFSIZE, &compare_bytes_read);
      if (fresult != FR_OK)
	{
	  image_is_equal = false; // treat a read error conservatively as "not equal"
	  break;
	}
      if (compare_bytes_read == 0)
	break; // exact multiple of MEM_BUFSIZE - true end of file, all blocks matched
    }

  // Close the_file on every return path - _FS_LOCK (ffconf.h) else keeps
  // its slot open, rejecting a later same-named WLAN upload.
  if (image_is_equal)
    {
      f_close (&the_file);
      return false;
    }

  // Comparison consumed the file - rewind and re-read the first block so
  // the program loop below starts as it always has.
  fresult = f_lseek (&the_file, 0);
  if (fresult != FR_OK)
    {
      f_close (&the_file);
      return false;
    }
  fresult = f_read (&the_file, mem_buffer, MEM_BUFSIZE, &bytes_read);
  if ((fresult != FR_OK) || (bytes_read < MEM_BUFSIZE))
    {
      f_close (&the_file);
      return false;
    }

  status = HAL_FLASH_Unlock ();
  if (status != HAL_OK)
    {
      f_close (&the_file);
      return false;
    }

  // Stale flags surviving a warm reset would otherwise abort programming
  // below - see jump_to_pending_flash_update_if_any(), uSD_handler.cpp.
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);

  // No erase here (deliberately) - erase_flash_staging_area_if_needed()
  // (uSD_handler.cpp) already erased this range before the watchdog was
  // armed. Erasing here, with the watchdog live, risks a mid-erase reset
  // leaving a corrupt-but-valid-looking header staged.

  // 4096 back-to-back HAL_FLASH_Program() calls can exceed the WWDG
  // window on their own (see documentation/wlan_link.md). Dropping below
  // WATCHDOG_TASK_PRIORITY lets the scheduler preempt this loop; the
  // explicit delay(1) every 256 words guarantees the watchdog gets fed
  // regardless.
  uSD_handler_task.set_priority (WATCHDOG_TASK_PRIORITY - 1);

  for (;;)
    {
      unsigned words_since_yield = 0;
      for (uint32_t *data_pointer = (uint32_t*) mem_buffer;
	  data_pointer < (uint32_t*) (mem_buffer + bytes_read); ++data_pointer)
	{
	  status = HAL_FLASH_Program ( TYPEPROGRAM_WORD, flash_address,
				      (uint64_t) *data_pointer);
	  if (status != HAL_OK)
	    break;
	  flash_address += sizeof(uint32_t);

	  if (++words_since_yield >= 256)
	    {
	      words_since_yield = 0;
	      delay (1); // beware of our watchdog !
	    }
	}

      if (last_block_read)
	{
	  HAL_FLASH_Lock ();
	  uSD_handler_task.set_priority (LOGGER_PRIORITY);
	  f_close (&the_file);
	  delay (100); // wait until uSD operations are finished
	  fresult = f_mount (0, "", 0); // unmount file system
	  delay (100); // wait until uSD operations are finished
	  // One jump attempt on the next boot - see firmware_update_retry_marker,
	  // uSD_handler.cpp.
	  firmware_update_retry_marker = FIRMWARE_UPDATE_JUST_STAGED;
	  return true;
	}

      fresult = f_read (&the_file, mem_buffer, MEM_BUFSIZE, &bytes_read);
      if (fresult != FR_OK)
	{
	  f_close (&the_file);
	  break;
	}

      if (bytes_read < MEM_BUFSIZE)
	{
	  f_close (&the_file);
	  last_block_read = true;
	}
    }

  HAL_FLASH_Lock ();
  uSD_handler_task.set_priority (LOGGER_PRIORITY);
  return false;
}


