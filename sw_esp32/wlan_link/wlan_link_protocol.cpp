/** *****************************************************************************
 * @file    wlan_link_protocol.cpp
 * @brief   CRC helpers for the ESP32 <-> STM32 SPI2 request/response link
 *
 * Ported from sw_stm32/Communication/wlan_link_protocol.cpp - must stay
 * byte-for-byte identical in behavior. The CRC32 algorithm was cross-checked
 * against pack.py's stm32_crc() reference implementation on a host machine
 * before being used on the STM32 side (see that commit's message in the
 * sw_sensor repository for the test vectors); this is the same algorithm,
 * unchanged.
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#include "wlan_link_protocol.h"

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
