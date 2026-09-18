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

//!< write crash dump file and force MPU reset via watchdog
void write_crash_dump( bool is_crash)
{
  uSD_handler_task.set_priority(configMAX_PRIORITIES - 1); // set it to highest priority

  FRESULT fresult;
  FIL fp;
  char *buffer = (char *)mem_buffer; // use global buffer here
  char *next = buffer;
  UINT writtenBytes = 0;

  acquire_privileges(); // ... need to access trace data etc

#if configUSE_TRACE_FACILITY // ************************************************
#include "trcConfig.h"
  vTraceStop(); // don't trace ourselves ...
#endif

  // Best-effort only - unbounded could deadlock if the crashed task
  // itself held the lock.
  (void) fatfs_lock_best_effort ();

  next = format_date_time( buffer, coordinates);
  append_string (next, is_crash ? ".CRASHDUMP" : ".RESET");

  fresult = f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    goto emergency_exit;

  next = buffer;
  append_string( next, (char*)"Firmware: ");
  append_string( next, GIT_TAG_INFO);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, (char*)"Hardware: ");
  utox( next, UNIQUE_ID[0], 8);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, crashfile);
  append_string( next, (char*)" Line: ");
  next = my_itoa( next, crashline);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, (char*)"Task:     ");
  append_string( next, pcTaskGetName( (TaskHandle_t)(register_dump.active_TCB)));
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, (char*)"IPSR:     ");
  utox( next, register_dump.IPSR);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, (char*)"PC:       ");
  utox( next, register_dump.stacked_pc);
  newline( next);
  append_string( next, (char*)"LR:       ");
  utox( next, register_dump.stacked_lr);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, (char*)"BusFA:    ");
  utox(  next, register_dump.Bus_Fault_Address);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, (char*)"MemA:     ");
  utox( next, register_dump.Bad_Memory_Address);
  newline( next);

  append_string( next, (char*)"MemFS:    ");
  utox( next, register_dump.Memory_Fault_status);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, (char*)"FPU_S:    ");
  utox( next, register_dump.FPU_StatusControlRegister);
  newline( next);

  append_string( next, (char*)"UsgFS:    ");
  utox( next, register_dump.Usage_Fault_Status_Register);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next = buffer;
  append_string( next, (char*)"HardFS:   ");
  utox( next, register_dump.Hard_Fault_Status);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  for( unsigned i=0; i<32; ++i)
    {

    // only if the dump is populated
      if( FPU_register_dump[i] != 0x00)
	{
	  next = buffer;
	  append_string( next, (char*)"FPU dump:");
	  newline( next);

	  for( unsigned i=0; i<32; ++i)
	    {
	      utox( next, FPU_register_dump[i]);
	      newline( next);
	    }
	  f_write (&fp, buffer, next-buffer, &writtenBytes);
	  break;
	}
    }
  fresult = f_close(&fp);
  if (fresult != FR_OK)
    goto emergency_exit;

#if configUSE_TRACE_FACILITY // ************************************************

extern RecorderDataType myTraceBuffer;

  next = format_date_time( buffer, coordinates);
  append_string (next, ".bin");
  fresult = f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    goto emergency_exit;

  for( uint8_t *ptr=(uint8_t *)&myTraceBuffer; ptr < (uint8_t *)&myTraceBuffer + sizeof(RecorderDataType); ptr += MEM_BUFSIZE)
    {
      unsigned blocksize = (uint8_t *)&myTraceBuffer + sizeof(RecorderDataType) - ptr;
      if( blocksize > MEM_BUFSIZE)
	blocksize = MEM_BUFSIZE;
      // data needs to be copied out of CCM RAM
      memcpy( mem_buffer, ptr, blocksize);
      fresult = f_write (&fp, mem_buffer, blocksize, &writtenBytes);
      if( writtenBytes < blocksize)
	break;
    }
  fresult = f_close(&fp);
  if (fresult != FR_OK)
    goto emergency_exit;

#endif // ************************************************************************

  // log one complete set of output data independent of data logging status
  next = format_date_time( buffer, coordinates);
  *next++ = '.';
  *next++  = 'f';
  format_2_digits( next, sizeof( state_vector_t) / sizeof(float));

  fresult = f_open ( &fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    goto emergency_exit;

  fresult = f_write (&fp, (uint8_t*) &observations, sizeof( observations), &writtenBytes);
  fresult = f_write (&fp, (uint8_t*) &coordinates,  sizeof( coordinates), &writtenBytes);
  fresult = f_write (&fp, (uint8_t*) &system_state, sizeof( system_state), &writtenBytes);
  fresult = f_write (&fp, (uint8_t*) &state_vector,  sizeof( state_vector), &writtenBytes);
  f_close( &fp);

emergency_exit:
  f_mount ( 0, "", 0); // unmount uSD
  delay( 100);
  HAL_SD_DeInit (&hsd);
  delay( 100);

  while( true)
    /* wake watchdog */;
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

//!< find software image file and load it if applicable
bool read_software_update (void)
{
  FIL the_file;
  FILINFO fno;
  FRESULT fresult;
  UINT bytes_read;
  DIR dj;

  uint32_t highest_sw_version_found = 0;
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

      uint32_t file_hw_version = mem_buffer[23] | (mem_buffer[22] << 8)
	  | (mem_buffer[21] << 16) | (mem_buffer[20] << 24);
      uint64_t file_magic_number = 0;
      for (int i = 7; i >= 0; i--)
	{
	  file_magic_number <<= 8;
	  file_magic_number |= (uint64_t) mem_buffer[i];
	}

      if ((file_hw_version == 0x01010100)
	  && (file_magic_number == 0x1c8073ab20853579))
	{
	  uint32_t file_sw_version = mem_buffer[27] | (mem_buffer[26] << 8)
	      | (mem_buffer[25] << 16) | (mem_buffer[24] << 24);
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
      if ((fresult != FR_OK) || (fno.fname[0] == 0))
	break; // now more files found, break loop-
    }

#if DISALLOW_DOWNGRADE
  if (highest_sw_version_found <= GIT_TAG_DEC)
      return false; //The firmware image with the highest version is it nothing new. Finishing here.

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


