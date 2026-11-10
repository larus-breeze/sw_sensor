#include "CRC32.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS_wrapper.h"
#include "my_assert.h"

Mutex CRC32_lock;

CRC32::CRC32( void)
{
  bool success = CRC32_lock.lock(2);
  ASSERT( success); // be sure to have the lock
  CRC->CR = CRC_CR_RESET;
}

CRC32::~CRC32( void)
{
  CRC32_lock.release();
}

//! match STM32F407 hardware CRC unit
uint32_t CRC32::update( uint32_t input)
{
  CRC->DR = input;
  return CRC->DR;
}

uint32_t CRC32::update( const uint32_t *data, uint32_t len)
{
  while( len--)
      CRC->DR = *data++;

  return CRC->DR;
}

void CRC32::reset( void)
{
  CRC->CR = CRC_CR_RESET;
}

uint32_t CRC32::get_value( void)
{
  return CRC->DR;
}

uint16_t CRC32::get_value_16bits( void)
{
  crc = CRC->DR;
  uint16_t retv = (uint16_t)(crc ^ (crc >> 16));
  return retv;
}

