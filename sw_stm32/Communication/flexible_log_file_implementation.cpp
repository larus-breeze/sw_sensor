#include "flexible_file_format.h"
#include <flexible_log_file_implementation.h>
#include "CRC16.h"
#include "my_assert.h"
#include "common.h"
#include "system_configuration.h"
#include "signal_flight_event.h"
#include "system_support.h"

bool flexible_log_file_implementation_t::open (char *file_name)
{
  FRESULT fresult;
  fresult = f_open (&out_file, (const TCHAR*)file_name, FA_CREATE_ALWAYS | FA_WRITE);
  if( fresult == FR_OK)
    {
      file_is_open = true;
      write_pointer = buffer;
      status = FILLING_LOW;
      return true;
    }
  return false;
}

bool flexible_log_file_implementation_t::close( void)
{
  file_is_open = false;
  UINT writtenBytes = 0;
  if( status & FILLING_LOW)
    f_write( &out_file, (const char *)flexible_log_file_t::buffer, (flexible_log_file_t::write_pointer - flexible_log_file_t::buffer) * sizeof( uint32_t), &writtenBytes);
  else
    f_write( &out_file, (const char *)second_part, (flexible_log_file_t::write_pointer - second_part) * sizeof( uint32_t), &writtenBytes);

  f_close ( &out_file);

  status = 0;
  return true;
}

bool flexible_log_file_implementation_t::sync_file (void)
{
  FRESULT fresult;
  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
  fresult = f_sync (&out_file);
  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);
  return (fresult == FR_OK);
}

bool flexible_log_file_implementation_t::flush_buffer( void)
{
  UINT written_bytes = 0;
  FRESULT fresult;

#if MEASURE_WRITE_TIME
  uint64_t time;
#endif

  unsigned size_bytes = (second_part - buffer) * sizeof( uint32_t);

  if( status & WRITING_LOW)
    {
      ASSERT( not( status & FILLING_LOW));

      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);

#if MEASURE_WRITE_TIME
      time = getTime_usec();
#endif

      fresult = f_write( &out_file, (const char *)buffer, size_bytes, &written_bytes);

#if MEASURE_WRITE_TIME
      time = getTime_usec() - time;
      signal_logger_event( DEBUGGER_DATA | time);
#endif
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);

#if 0 // debug version
      ASSERT(( fresult == FR_OK) && (written_bytes == size_bytes));
#else
      if( not (( fresult == FR_OK) && (written_bytes == size_bytes)))
	block_input();
#endif

#if ANALYZE_WRITE_PERFORMANCE // using debugger
      if( (write_pointer - second_part) > used_size)
	{
	  used_size = write_pointer - second_part;
	  signal_logger_event( DEBUGGER_DATA | (used_size << 8));
	}

#endif
    }
  else if( ( status & WRITING_HIGH))
    {
      ASSERT( not( status & FILLING_HIGH));

      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
      fresult = f_write( &out_file, (const char *)second_part, size_bytes, &written_bytes);
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);

#if 0 // debug version
      ASSERT(( fresult == FR_OK) && (written_bytes == size_bytes));
#else
      if( not (( fresult == FR_OK) && (written_bytes == size_bytes)))
	block_input();
#endif

#if ANALYZE_WRITE_PERFORMANCE
      if( (write_pointer - buffer) > used_size)
	{
	  used_size = write_pointer - buffer;
	  signal_logger_event( DEBUGGER_DATA | (used_size << 8));
	}
#endif
    }
  else
    {
      status &= ~(WRITING_LOW | WRITING_HIGH);
    }

  status &= ~(WRITING_LOW | WRITING_HIGH);

  return ( size_bytes == written_bytes);
}

bool flexible_log_file_implementation_t::write_block (uint32_t *p_data,
						 uint32_t size_words)
{
  bool need_to_signal = false;

  while (size_words--)
    {

      *write_pointer++ = *p_data++;

      if (write_pointer >= buffer_end)
	{
	  ASSERT(not (status & WRITING_LOW))

	  write_pointer = buffer;

	  status &= ~FILLING_HIGH;
	  status |= FILLING_LOW;
	  status |= WRITING_HIGH;

	  need_to_signal = true;
	}
      else if (write_pointer == second_part)
	{
	  ASSERT(not (status & WRITING_HIGH));

	  status &= ~FILLING_LOW;
	  status |= FILLING_HIGH;
	  status |= WRITING_LOW;

	  need_to_signal = true;

	}
    }

  if (need_to_signal)
    signal ();

  return true;
}
