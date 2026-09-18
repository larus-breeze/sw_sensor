#include "embedded_memory.h"
#include "embedded_math.h"
#include "common.h"
#include "FreeRTOS_wrapper.h"
#include "system_configuration.h"
#include "my_assert.h"
#include "EEPROM_data_file_implementation.h"
#include "stm32f4xx_hal.h"
#include "uSD_handler.h"

#define PAGE_0_HEAD ((uint32_t *)0x080C0000)
#define PAGE_1_HEAD ((uint32_t *)0x080E0000)
#define PAGE_SIZE_BYTES 0x20000
#define PAGE_SIZE_WORDS (PAGE_SIZE_BYTES / 4)
#define PAGE_SIZE_LONG_WORDS (PAGE_SIZE_WORDS / 2)

COMMON Queue <flash_write_order> flash_command_queue( 3, "FLASH_CMD");
COMMON Semaphore flash_isr_to_task( 1, 0, (char *)"FLASH_ISR");
COMMON Mutex_Wrapper_Type my_mutex( (char *)"MTX_WRAPPER");

COMMON EEPROM_file_system <LOWEST_UNUSED_EEPROM_ID> permanent_data_file;
extern Queue <flash_write_order> flash_command_queue;

void FLASH_write (uint32_t *dest, uint32_t *source, unsigned n_words, bool synchronized)
{
  if (not synchronized)
    {
      while (n_words--)
	  *dest++ = *source++;
    }
  else // synchronous write, interrupt-synchronized
    {
      HAL_StatusTypeDef status;
      status = HAL_FLASH_Unlock ();
      ASSERT(HAL_OK == status);

      while (n_words--)
	{
	  status = HAL_FLASH_Program_IT ( TYPEPROGRAM_WORD, (uint32_t) dest++,
					 *source++);
	  ASSERT(status == HAL_OK);
	  bool no_timeout = flash_isr_to_task.wait ( FLASH_ACCESS_TIMEOUT);
	  ASSERT(no_timeout);
	}

      status = HAL_FLASH_Lock ();
      ASSERT(HAL_OK == status);
    }
}

//!< test interface for reading
bool read_blob( EEPROM_file_system_node::ID_t id, unsigned length_in_words, void * data)
{
  if( not permanent_data_file.is_consistent())
    return false;

  return permanent_data_file.retrieve_data ( id, length_in_words, (uint32_t *)data);
}
//!< test interface for writing
bool write_blob( EEPROM_file_system_node::ID_t id, unsigned length_in_words, const void * data)
{
  if( not permanent_data_file.is_consistent())
    return false;

  return permanent_data_file.store_data ( id, length_in_words, (uint32_t *)data);
}

//!< order sector erase
bool erase_sector( unsigned sector)
{
  LOCK_SECTION();
  if( (sector != 0) && (sector != 1))
    return false;

  uint64_t * location = sector == 0 ? (uint64_t *)PAGE_0_HEAD : (uint64_t *)PAGE_1_HEAD;
  unsigned size = PAGE_SIZE_LONG_WORDS;

  bool is_erased = true;
  while( size --)
    {
    if( *location++ != 0xffffffffffffffff)
      {
	is_erased = false;
	break;
      }
    }
  if( is_erased)
    return true;

  flash_write_order cmd;
  cmd.dest = (uint32_t *)sector;
  bool result = flash_command_queue.send( cmd, FLASH_ERASE_TIMEOUT);
  ASSERT( result);

  delay( MAXIMUM_PAGE_ERASE_TIME); // wait longest possible time until ERASE done

  return true;
}

//!< execute sector erase
void erase_sector_operation( unsigned sector)
{
  ASSERT( sector < 2);
  HAL_StatusTypeDef status;

  FLASH_EraseInitTypeDef EraseInit;
  EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInit.Sector = sector == 0 ? FLASH_SECTOR_10 : FLASH_SECTOR_11;
  EraseInit.NbSectors = 1;

  status = HAL_FLASHEx_Erase_IT( &EraseInit);
  ++flash_erase_count;
  ASSERT(status == HAL_OK);
}

float configuration (EEPROM_PARAMETER_ID id)
{
  if ( permanent_data_file.in_use())
    {
      float value;
      // try to read float object size one
      bool result = permanent_data_file.retrieve_data (id, 1, (uint32_t *)&value);
      if (result)
	return value;
      else // read direct data ( 8 bit) object
	{
	  uint8_t value;
	  result = permanent_data_file.retrieve_data (id, value);
	  ASSERT( result);
	  return (float)value;
	}
    }
  else // using old EEPROM data
    {
      float value = 0.0f;
      read_EEPROM_value( id, value);
      return value;
    }
}

bool file_system_page_swap( void)
{
  if( permanent_data_file.get_head() == (void *)PAGE_1_HEAD)
    {
      bool result = erase_sector( 0);
      ASSERT( result);
      EEPROM_file_system <LOWEST_UNUSED_EEPROM_ID> new_data_file( (EEPROM_file_system_node *)PAGE_0_HEAD, (EEPROM_file_system_node *)(PAGE_0_HEAD+PAGE_SIZE_BYTES));
      new_data_file.import_all_data(permanent_data_file);
      result = erase_sector( 1);
      ASSERT( result);
      return permanent_data_file.set_memory_to_existing_data( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_WORDS);
     }
  else
    {
      bool result = erase_sector( 1);
      ASSERT( result);
      EEPROM_file_system <LOWEST_UNUSED_EEPROM_ID> new_data_file( (EEPROM_file_system_node *)PAGE_1_HEAD, (EEPROM_file_system_node *)(PAGE_1_HEAD+PAGE_SIZE_BYTES));
      new_data_file.import_all_data(permanent_data_file);
      result = erase_sector( 0);
      ASSERT( result);
      return permanent_data_file.set_memory_to_existing_data( PAGE_1_HEAD, PAGE_1_HEAD+PAGE_SIZE_WORDS);
    }
}

bool write_EEPROM_value (EEPROM_PARAMETER_ID id, float value)
{
  ASSERT (permanent_data_file.in_use());
  bool success = permanent_data_file.store_data( id, 1, &value);
  return success;
}

//!< interface to legacy read function
bool read_EEPROM_value (EEPROM_PARAMETER_ID id, float &value)
{
  return not permanent_data_file.retrieve_data( id, 1, &value);
}

bool import_raw_EEPROM_data( EEPROM_PARAMETER_ID id, uint32_t * flash_address, unsigned size_words, uint16_t &datum)
{
  if( *(uint16_t *)flash_address == 0xEEEE) // dirty flash segment
    return false;

  uint16_t candidate;
  bool found = false;
  while( size_words --)
    {
      if( *flash_address == 0xffffffff) // erased flash, end of data range
	break;

      if( (*flash_address >> 16) == id)
	{
	candidate = (uint16_t)(*flash_address);
	found = true;
	}
      ++flash_address; // continue to search for newer data
    }
  if( found)
    {
      datum = (uint16_t)candidate; // strip ID
      return true;
    }
  else
    return false;
}

bool import_single_EEPROM_value( EEPROM_PARAMETER_ID id, uint32_t * flash_address, unsigned size_words, float &value)
{
  EEPROM_data_t raw_EEPROM_value;

  if( import_raw_EEPROM_data( id, flash_address, size_words, raw_EEPROM_value.u16))
    {
      if( not EEPROM_convert( id, raw_EEPROM_value, value , true))
	return true;
    }
  return false;
}

static bool import_legacy_EEPROM_data( uint32_t * flash_address, unsigned size_words)
{
  bool result = true;
  float value;

  for( 	const persistent_data_t * parameter = PERSISTENT_DATA;
	parameter < PERSISTENT_DATA + PERSISTENT_DATA_ENTRIES;
	++parameter)
    {
	if( not import_single_EEPROM_value( parameter->id, flash_address, size_words, value))
	  value = parameter->default_value;
	else
	  result = false;

	if( parameter->id != HORIZON)
	  result &= (0 != permanent_data_file.store_data ( parameter->id, 1, &value));
    }
  return result;
}

void recover_and_initialize_flash( void)
{
  if( *(uint16_t *)0x080F8000 == 0 && *(int64_t *)0x080E0000 == -1) // old flash layout
    {
      // prepare page 0 for data import
      erase_sector( 0);
      permanent_data_file.set_memory_virgin( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_WORDS);
      (void) import_legacy_EEPROM_data( (uint32_t *)0x080F8000, 0x2000);
      erase_sector( 1); // now we clean the upper sector removing the old data
      return; // job done
    }
  else if( *(uint16_t *)PAGE_0_HEAD == 0) // new flash layout, import from page 0
    {
      erase_sector( 1);
      permanent_data_file.set_memory_virgin( PAGE_1_HEAD, PAGE_1_HEAD+PAGE_SIZE_WORDS);
      (void) import_legacy_EEPROM_data( PAGE_0_HEAD, PAGE_SIZE_BYTES / sizeof( uint32_t));
      erase_sector( 0); // now we clean the upper sector from the old data
      return; // job done
    }
  else if( *(uint16_t *)PAGE_1_HEAD == 0) // new flash layout, using page 1
    {
      erase_sector( 0);
      permanent_data_file.set_memory_virgin( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_WORDS);
      (void) import_legacy_EEPROM_data( PAGE_1_HEAD, PAGE_SIZE_BYTES / sizeof( uint32_t));
      erase_sector( 1); // now we clean the upper sector from the old data
      return; // job done
    }
  else if( *(int32_t *)PAGE_1_HEAD != -1) // check for file system on page 1
    {
      bool success = permanent_data_file.set_memory_to_existing_data( PAGE_1_HEAD, PAGE_1_HEAD+PAGE_SIZE_WORDS);
      if( success)
	{
	  // finally: if the file system is almost full: do a page swap right now
	  if( permanent_data_file.get_remaining_space_words() < (PAGE_SIZE_WORDS - (PAGE_SIZE_WORDS >> 2)))
	    file_system_page_swap();
	  return;
	}

      // make a page swap and copy all clean records
      erase_sector( 0);
      EEPROM_file_system <LOWEST_UNUSED_EEPROM_ID> new_data_copy( (EEPROM_file_system_node *)PAGE_0_HEAD, (EEPROM_file_system_node *)PAGE_0_HEAD+PAGE_SIZE_WORDS);

      // try data recovery
      new_data_copy.import_all_data( permanent_data_file);

      // ... and change over
      success = permanent_data_file.set_memory_to_existing_data( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_WORDS);
      ASSERT( success); // now it must be OK !
      erase_sector(1); // the old sector is obsolete now !
    }
  else if( *(int32_t *)PAGE_0_HEAD != -1) // check for file system on page 0
    {
      bool success = permanent_data_file.set_memory_to_existing_data( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_WORDS);
      if( success)
	{
	  // finally: if the file system is almost full: do a page swap right now
	  if( permanent_data_file.get_remaining_space_words() < (PAGE_SIZE_WORDS - (PAGE_SIZE_WORDS >> 2)))
	    file_system_page_swap();
	  return;
	}

      // make a page swap and copy all clean records
      erase_sector( 1);
      EEPROM_file_system <LOWEST_UNUSED_EEPROM_ID> new_data_copy( (EEPROM_file_system_node *)PAGE_1_HEAD, (EEPROM_file_system_node *)PAGE_1_HEAD+PAGE_SIZE_WORDS);

      // try data recovery
      new_data_copy.import_all_data(permanent_data_file);

      // ... and change over
      success = permanent_data_file.set_memory_to_existing_data( PAGE_1_HEAD, PAGE_1_HEAD+PAGE_SIZE_WORDS);
      ASSERT( success); // now it must be OK !
      erase_sector(0); // the old sector is obsolete now !
    }
  else // virgin start, there appears to be nothing useful within any EEPROM emulation section
    {
      erase_sector( 0);
      erase_sector( 1);
      (void) permanent_data_file.set_memory_virgin( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_WORDS);
    }
}

static void EEPROM_writing_runnable( void *)
{
  uint32_t prioritygroup = NVIC_GetPriorityGrouping ();

  NVIC_SetPriority ((IRQn_Type) FLASH_IRQn,
		    NVIC_EncodePriority (prioritygroup, STANDARD_ISR_PRIORITY, 0));
  NVIC_EnableIRQ ((IRQn_Type) FLASH_IRQn);

  flash_write_order order;
  bool no_timeout;
  while( true)
    {
      flash_command_queue.receive( order, INFINITE_WAIT);

      HAL_StatusTypeDef status;
      status = HAL_FLASH_Unlock();
      ASSERT(HAL_OK == status);

      if( order.dest == 0) // erase commmand
	{
	  erase_sector_operation( 0);
	  no_timeout = flash_isr_to_task.wait( FLASH_ERASE_TIMEOUT);
	  ASSERT( no_timeout);
	}
      else if( order.dest == (uint32_t *)1)
	{
	  erase_sector_operation( 1);
	  no_timeout = flash_isr_to_task.wait( FLASH_ERASE_TIMEOUT);
	  ASSERT( no_timeout);
	}
      else
	{
	  status = HAL_FLASH_Program_IT( TYPEPROGRAM_WORD, (uint32_t)(order.dest), order.value);
	  ASSERT( status == HAL_OK);
	  no_timeout = flash_isr_to_task.wait( FLASH_ERASE_TIMEOUT);
	  ASSERT( no_timeout);
	}

      status = HAL_FLASH_Lock();
      ASSERT(HAL_OK == status);
    }
}

#define STACKSIZE 128
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  {
    EEPROM_writing_runnable,
    "EEPROM",
    STACKSIZE,
    0,
    EEPROM_WRITER_PRIORITY | portPRIVILEGE_BIT,
    stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { PAGE_0_HEAD, PAGE_SIZE_BYTES * 2, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 }
    }
  };

static RestrictedTask EEPROM_accessor( p);

extern "C" void FLASH_IRQHandler( void)
{
  HAL_FLASH_IRQHandler();
}

extern "C" void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
  flash_isr_to_task.signal_from_ISR();
}
