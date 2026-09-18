/** *****************************************************************************
 * @file    	wlan_link_protocol.h
 * @brief   	Wire format for the ESP32 <-> STM32 SPI2 request/response link
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

#ifndef COMMUNICATION_WLAN_LINK_PROTOCOL_H_
#define COMMUNICATION_WLAN_LINK_PROTOCOL_H_

#include <stdint.h>
#include <stddef.h>

// See documentation/wlan_link.md, section "SPI2 protocol". ESP32 is always
// SPI master, STM32 is SPI2 slave; every exchange is initiated by an
// ESP32 request, even for STM32 "sender" operations (log file download).

#define WLAN_LINK_CHUNK_SIZE      4096u  //!< bytes, both directions
#define WLAN_LINK_MAX_FILENAME    48u    //!< bytes, NUL-padded - fits *.lrsx log names and the
                                          //!< longest firmware image names (e.g.
                                          //!< "larus_sensor_stm32_v255.255.255-255.bin", 39 chars)
                                          //!< with room to spare

//!< frame header, exactly 16 bytes on the wire
#pragma pack(push, 1)
typedef struct
{
  uint8_t  frame_type;
  uint8_t  reserved0[3];
  uint32_t sequence_no;
  uint32_t payload_length;
  uint16_t header_crc16;
  uint16_t reserved1;
} wlan_link_frame_header_t;
#pragma pack(pop)

static_assert (sizeof(wlan_link_frame_header_t) == 16, "frame header must be exactly 16 bytes on the wire");

typedef enum
{
  WLAN_FRAME_UPLOAD_BEGIN            = 0x01, //!< ESP32->STM32, payload: total length (4B) + filename (48B)
  WLAN_FRAME_UPLOAD_CHUNK            = 0x02, //!< ESP32->STM32, payload: up to CHUNK_SIZE bytes + CRC32
  WLAN_FRAME_UPLOAD_END              = 0x03, //!< ESP32->STM32, payload: whole-file CRC32 (4B)
  WLAN_FRAME_UPLOAD_ABORT            = 0x04, //!< ESP32->STM32, no payload

  WLAN_FRAME_STATUS_REQUEST          = 0x10, //!< ESP32->STM32, no payload
  WLAN_FRAME_LIST_REQUEST            = 0x11, //!< ESP32->STM32, payload: entry index (4B)
  WLAN_FRAME_DOWNLOAD_BEGIN          = 0x12, //!< ESP32->STM32, payload: filename (48B)
  WLAN_FRAME_DOWNLOAD_CHUNK_REQUEST  = 0x13, //!< ESP32->STM32, payload: chunk index (4B)
  WLAN_FRAME_DOWNLOAD_END            = 0x14, //!< ESP32->STM32, no payload
  WLAN_FRAME_DELETE_REQUEST          = 0x15, //!< ESP32->STM32, payload: filename (48B)
  WLAN_FRAME_FORMAT_REQUEST          = 0x16, //!< ESP32->STM32, payload: confirmation token (4B)
  WLAN_FRAME_STOP_LOGGING_REQUEST    = 0x17, //!< ESP32->STM32, no payload - see "Logging pause"
  WLAN_FRAME_START_LOGGING_REQUEST   = 0x18, //!< ESP32->STM32, no payload - see "Logging pause"

  WLAN_FRAME_ACK                     = 0x81, //!< STM32->ESP32, payload: sequence number acknowledged (4B)
  WLAN_FRAME_NACK                    = 0x82, //!< STM32->ESP32, payload: sequence number (4B) + reason (1B)
  WLAN_FRAME_DATA                    = 0x83, //!< STM32->ESP32, payload depends on the request being answered
} wlan_link_frame_type_t;

typedef enum
{
  WLAN_NACK_HEADER_CRC        = 0x01,
  WLAN_NACK_PAYLOAD_CRC       = 0x02,
  WLAN_NACK_OUT_OF_SEQUENCE   = 0x03,
  WLAN_NACK_NO_SD_CARD        = 0x04,
  WLAN_NACK_VERSION_REJECTED  = 0x05,
  WLAN_NACK_LOGGING_ACTIVE    = 0x06, //!< logging is currently active (holds the SD card) - see STOP_LOGGING_REQUEST
  WLAN_NACK_FILE_NOT_FOUND    = 0x07,
  WLAN_NACK_BAD_FORMAT_TOKEN  = 0x08,
  WLAN_NACK_NOT_IMPLEMENTED   = 0x09,
  WLAN_NACK_INTERNAL_ERROR    = 0x0A,
  WLAN_NACK_BAD_FILENAME      = 0x0B,
  WLAN_NACK_SD_CARD_BUSY      = 0x0C, //!< uSD_handler_task currently holds the SD card (landing/takeoff transition race), retry shortly
} wlan_link_nack_reason_t;

//!< confirmation token FORMAT_REQUEST must carry - not a security mechanism,
//!< just a guard against a stray/replayed/malformed frame formatting the card
#define WLAN_LINK_FORMAT_CONFIRMATION_TOKEN  0x4C617246u  // "FraL" (Larus Format), arbitrary

#pragma pack(push, 1)
typedef struct
{
  uint32_t total_length;
  char     filename[WLAN_LINK_MAX_FILENAME];
} wlan_link_upload_begin_payload_t;

typedef struct
{
  uint32_t whole_file_crc32;
} wlan_link_upload_end_payload_t;

typedef struct
{
  char     filename[WLAN_LINK_MAX_FILENAME];
} wlan_link_filename_payload_t;

typedef struct
{
  uint32_t confirmation_token;
} wlan_link_format_payload_t;

typedef struct
{
  uint32_t sequence_no;
} wlan_link_ack_payload_t;

typedef struct
{
  uint32_t sequence_no;
  uint8_t  reason;
} wlan_link_nack_payload_t;

//!< payload of the DATA frame answering a STATUS_REQUEST
typedef struct
{
  uint8_t  sd_card_present;   //!< 0/1
  uint64_t sd_free_bytes;     //!< free space on the SD card, 0 if not mounted or logging_active (see "Logging pause" - the FatFs lock is held for the whole logging session, so this can't be queried then)
  uint64_t sd_total_bytes;    //!< total SD card capacity, 0 under the same conditions as sd_free_bytes
  uint8_t  airborne;          //!< 0/1, informational only - see documentation "Flight state gating"
  uint8_t  logging_active;    //!< 0/1 - the actual list/download/upload/delete/format gate, see "Logging pause"
  uint8_t  logging_paused_by_user; //!< 0/1 - true while a WLAN-requested pause is in effect
  uint32_t current_sw_version; //!< GIT_TAG_DEC of the firmware currently running
  char     stm32_git_tag_info[32]; //!< NUL-padded GIT_TAG_INFO, e.g. "0.7.6-150-ge720e17"
  uint8_t  gnss_year;   //!< 2-digit (e.g. 24 for 2024), matches D_GNSS_coordinates_t::year; all-zero fields mean no GNSS fix yet
  uint8_t  gnss_month;
  uint8_t  gnss_day;
  uint8_t  gnss_hour;
  uint8_t  gnss_minute;
  uint8_t  gnss_second;
} wlan_link_status_payload_t;

typedef struct
{
  uint32_t entry_index;
} wlan_link_list_request_payload_t;

//!< one entry per request/response round trip - directory listings aren't
//!< the throughput-critical path, simplicity was chosen over packing
//!< several entries into one frame. has_entry=0 means entry_index was past
//!< the end of the (filtered to *.lrsx) directory listing.
typedef struct
{
  uint8_t  has_entry;
  uint8_t  reserved[3];
  uint32_t file_size;
  uint16_t fat_date;   //!< FatFs FILINFO.fdate encoding
  uint16_t fat_time;   //!< FatFs FILINFO.ftime encoding
  char     filename[WLAN_LINK_MAX_FILENAME];
} wlan_link_list_response_payload_t;

typedef struct
{
  char     filename[WLAN_LINK_MAX_FILENAME];
} wlan_link_download_begin_payload_t;

//!< DATA response to DOWNLOAD_BEGIN
typedef struct
{
  uint32_t file_size;
} wlan_link_download_begin_response_payload_t;

typedef struct
{
  uint32_t chunk_index;
} wlan_link_download_chunk_request_payload_t;

//!< DATA response to DOWNLOAD_CHUNK_REQUEST: this fixed part, then
//!< 'valid_bytes' bytes of file data, then a trailing CRC32 over that data
//!< (word-padded the same way UPLOAD_CHUNK's trailer is) - payload_length
//!< in the frame header covers this fixed part plus the data, not the CRC32.
typedef struct
{
  uint32_t chunk_index;
  uint32_t valid_bytes;
} wlan_link_download_chunk_response_header_t;
#pragma pack(pop)

//!< CRC16/CCITT-FALSE (init 0xFFFF, poly 0x1021, no reflect, no final xor) - protects the frame header only
uint16_t wlan_link_crc16_ccitt (const uint8_t *data, size_t len);

//!< matches pack.py's stm32_crc(): CRC-32/MPEG-2 style (init 0xFFFFFFFF, poly 0x04C11DB7,
//!< no reflect, no final xor), fed 4-byte words in reversed byte order to match the STM32
//!< hardware CRC unit's behavior. 'len' must be a multiple of 4. Chainable: pass the
//!< previous call's return value as 'crc' (start with wlan_link_crc32_init()).
uint32_t wlan_link_crc32_init (void);
uint32_t wlan_link_crc32_update (uint32_t crc, const uint8_t *data, size_t len);

//!< Same result via the STM32's CRC hardware peripheral instead of the
//!< bit-banged loop - see wlan_link_crc32_hw_compute()'s doc comment
//!< (wlan_link_protocol.cpp) for why this needs a single non-reentrant
//!< caller. 'len' need not be a multiple of 4; any trailing partial word
//!< is zero-padded internally.
uint32_t wlan_link_crc32_hw_compute (const uint8_t *data, size_t len);

#endif /* COMMUNICATION_WLAN_LINK_PROTOCOL_H_ */
