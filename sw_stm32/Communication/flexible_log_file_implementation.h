#ifndef FLEXIBLE_LOG_FILE_IMPLEMENTATION_H_
#define FLEXIBLE_LOG_FILE_IMPLEMENTATION_H_

#include "stdint.h"
#include "fatfs.h"
#include "flexible_log_file.h"
#include "FreeRTOS_wrapper.h"

typedef void ( *FPTR)( void); // declare void -> void function pointer

class flexible_log_file_implementation_t : public flexible_log_file_t
{
public:

  flexible_log_file_implementation_t ( uint32_t * buf, unsigned size_words, FPTR _signal)
  : flexible_log_file_t( buf, size_words),
    file_is_open( false),
    status( FILLING_LOW),
    second_part ( buffer + size_words / 2),
    signal( _signal)
  {
  }

  virtual ~flexible_log_file_implementation_t()
  {
    close();
  }

  bool is_open( void)
  {
    return file_is_open;
  }

  bool append_record ( flexible_log_file_record_type type, uint32_t *data, uint32_t data_size_words)
  {
    if( not file_is_open)
      return true; // silently give up

    // delegate to base class
    return flexible_log_file_t::append_record(type, data, data_size_words, FLEXIBLE_LOG_FILE_FORMAT_VERSION);
  }

  bool open( char * file_name) override;
  bool flush_buffer( void);
  bool sync_file( void);
  bool close( void) override;
  void block_input( void)
  {
    file_is_open = false;
  }

  bool write_block( uint32_t * begin, uint32_t size_words);

private:
  enum {
    FILLING_LOW=1,
    FILLING_HIGH=2,
    WRITING_LOW=4,
    WRITING_HIGH=8
  };

  void wrap_around( void);
  FIL out_file;
  bool file_is_open;
  uint32_t status;
  uint32_t *second_part;
  FPTR signal;
#if ANALYZE_WRITE_PERFORMANCE
  int used_size;
#endif

};

#endif
