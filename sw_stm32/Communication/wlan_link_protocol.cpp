/** *****************************************************************************
 * @file    	wlan_link_protocol.cpp
 * @brief   	CRC helpers for the ESP32 <-> STM32 SPI2 request/response link
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

#include "wlan_link_protocol.h"
#include "stm32f4xx_hal.h" // CRC->, CRC_CR_RESET, __HAL_RCC_CRC_CLK_ENABLE() - see wlan_link_crc32_hw_compute()
#include <string.h> // memcpy, wlan_link_crc32_hw_compute()'s remainder padding

uint16_t wlan_link_crc16_ccitt (const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i)
    {
      crc ^= (uint16_t) ((uint16_t) data[i] << 8);
      for (int bit = 0; bit < 8; ++bit)
	crc = (crc & 0x8000u) ? (uint16_t) ((crc << 1) ^ 0x1021u) : (uint16_t) (crc << 1);
    }
  return crc;
}

uint32_t wlan_link_crc32_init (void)
{
  return 0xFFFFFFFFu;
}

// Cross-checked against pack.py's stm32_crc() for several test vectors before
// this file was written - see the commit message for this file for details.
// 'len' must be a multiple of 4; callers must zero-pad the last partial word
// of a buffer before calling this (matching how pack.py's ReadApp.get_binary()
// pads the flash image to a word boundary).
uint32_t wlan_link_crc32_update (uint32_t crc, const uint8_t *data, size_t len)
{
  for (size_t i = 0; i < len; i += 4)
    {
      for (int k = 3; k >= 0; --k)
	{
	  crc ^= ((uint32_t) data[i + (size_t) k]) << 24;
	  for (int bit = 0; bit < 8; ++bit)
	    crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
	}
    }
  return crc;
}

// The loop above was deliberately written byte-for-byte equivalent to the
// STM32's own CRC peripheral: it feeds each 4-byte group MSB-first
// (data[i+3] first, down to data[i+0]) into the same init-0xFFFFFFFF,
// poly-0x04C11DB7, no-reflect/no-final-xor algorithm the hardware unit
// implements - which is exactly what happens when a *native* (little-
// endian) 32-bit load of those same 4 bytes is written straight to the
// peripheral's CRC->DR: bits 31-24 of that load are data[i+3], bits 23-16
// are data[i+2], and so on. No byte-swap needed here as a result.
//
// Unlike wlan_link_crc32_update() above, this can't be hardware-
// accelerated one call at a time with an arbitrary 'crc' handed back in -
// the peripheral itself has no way to be seeded with anything other than
// its fixed 0xFFFFFFFF reset value, only ever reset-then-accumulate. That
// chained-accumulator signature exists so several *software* calls can
// share one computation across other, unrelated work in between; this
// function does the whole computation itself in one uninterrupted
// reset-to-result sequence instead, which is also why it takes the full
// data range up front rather than being called repeatedly.
//
// It is also a single, global, stateful peripheral shared by the whole
// MCU (the vendored crypto library, Middlewares/ST/Crypto_Lib, enables
// and reserves it too, though nothing in this firmware actually calls
// into that library yet) - safe without a lock only because its two
// callers can never actually run concurrently, not because there's only
// one: download_prefetch_runnable() (wlan_link_handler.cpp) needs an
// active WLAN download session, which can't exist before the WLAN link
// is even initialized; staged_image_crc_is_valid() (uSD_handler.cpp)
// only ever runs at the very top of uSD_handler_runnable(), before that
// initialization happens. If a future caller could run concurrently with
// either of these, it needs real mutual exclusion (e.g. a Mutex,
// mutex_implementation.h) around the whole reset-to-result sequence, not
// just this function in isolation.
uint32_t wlan_link_crc32_hw_compute (const uint8_t *data, size_t len)
{
  __HAL_RCC_CRC_CLK_ENABLE ();
  CRC->CR = CRC_CR_RESET;

  size_t word_aligned_len = len & ~(size_t) 3u;
  for (size_t i = 0; i < word_aligned_len; i += 4)
    CRC->DR = *(const uint32_t *) (data + i);

  if (word_aligned_len != len)
    {
      uint32_t last_word = 0; // zero-padded, matching every software caller's own padding
      memcpy (&last_word, data + word_aligned_len, len - word_aligned_len);
      CRC->DR = last_word;
    }

  return CRC->DR;
}
