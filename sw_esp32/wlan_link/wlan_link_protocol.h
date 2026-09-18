/** *****************************************************************************
 * @file    wlan_link_protocol.h
 * @brief   Wire format for the ESP32 <-> STM32 SPI2 request/response link
 *
 * MUST MATCH sw_stm32/Communication/wlan_link_protocol.h byte-for-byte -
 * this is the STM32-side header, ported. See
 * documentation/wlan_link.md, "SPI2 protocol", for the design rationale.
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#ifndef WLAN_LINK_PROTOCOL_H_
#define WLAN_LINK_PROTOCOL_H_

#include <stdint.h>
#include <stddef.h>

#define WLAN_LINK_CHUNK_SIZE      4096u  //!< bytes, both directions
#define WLAN_LINK_MAX_FILENAME    48u    //!< bytes, NUL-padded - fits *.lrsx log names and the
                                          //!< longest firmware image names (e.g.
                                          //!< "larus_sensor_stm32_v255.255.255-255.bin", 39 chars)
                                          //!< with room to spare

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

enum wlan_link_frame_type_t : uint8_t
{
  WLAN_FRAME_UPLOAD_BEGIN            = 0x01,
  WLAN_FRAME_UPLOAD_CHUNK            = 0x02,
  WLAN_FRAME_UPLOAD_END              = 0x03,
  WLAN_FRAME_UPLOAD_ABORT            = 0x04,

  WLAN_FRAME_STATUS_REQUEST          = 0x10,
  WLAN_FRAME_LIST_REQUEST            = 0x11,
  WLAN_FRAME_DOWNLOAD_BEGIN          = 0x12,
  WLAN_FRAME_DOWNLOAD_CHUNK_REQUEST  = 0x13,
  WLAN_FRAME_DOWNLOAD_END            = 0x14,
  WLAN_FRAME_DELETE_REQUEST          = 0x15,
  WLAN_FRAME_FORMAT_REQUEST          = 0x16,
  WLAN_FRAME_STOP_LOGGING_REQUEST    = 0x17,
  WLAN_FRAME_START_LOGGING_REQUEST   = 0x18,

  WLAN_FRAME_ACK                     = 0x81,
  WLAN_FRAME_NACK                    = 0x82,
  WLAN_FRAME_DATA                    = 0x83,
};

enum wlan_link_nack_reason_t : uint8_t
{
  WLAN_NACK_HEADER_CRC        = 0x01,
  WLAN_NACK_PAYLOAD_CRC       = 0x02,
  WLAN_NACK_OUT_OF_SEQUENCE   = 0x03,
  WLAN_NACK_NO_SD_CARD        = 0x04,
  WLAN_NACK_VERSION_REJECTED  = 0x05,
  WLAN_NACK_LOGGING_ACTIVE    = 0x06, // logging is currently active - see STOP_LOGGING_REQUEST
  WLAN_NACK_FILE_NOT_FOUND    = 0x07,
  WLAN_NACK_BAD_FORMAT_TOKEN  = 0x08,
  WLAN_NACK_NOT_IMPLEMENTED   = 0x09,
  WLAN_NACK_INTERNAL_ERROR    = 0x0A,
  WLAN_NACK_BAD_FILENAME      = 0x0B,
  WLAN_NACK_SD_CARD_BUSY      = 0x0C,
};

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

typedef struct
{
  uint8_t  sd_card_present;
  uint64_t sd_free_bytes;  // free space on the SD card, 0 if not mounted or logging_active (see "Logging pause")
  uint64_t sd_total_bytes; // total SD card capacity, 0 under the same conditions as sd_free_bytes
  uint8_t  airborne; // informational only, not a gate - see "Logging pause"
  uint8_t  logging_active; // the actual list/download/upload/delete/format gate
  uint8_t  logging_paused_by_user;
  uint32_t current_sw_version;
  char     stm32_git_tag_info[32]; // NUL-padded GIT_TAG_INFO, e.g. "0.7.6-150-ge720e17"
  uint8_t  gnss_year;   // 2-digit (e.g. 24 for 2024); all-zero fields mean no GNSS fix yet
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

typedef struct
{
  uint8_t  has_entry;
  uint8_t  reserved[3];
  uint32_t file_size;
  uint16_t fat_date;
  uint16_t fat_time;
  char     filename[WLAN_LINK_MAX_FILENAME];
} wlan_link_list_response_payload_t;

typedef struct
{
  char     filename[WLAN_LINK_MAX_FILENAME];
} wlan_link_download_begin_payload_t;

typedef struct
{
  uint32_t file_size;
} wlan_link_download_begin_response_payload_t;

typedef struct
{
  uint32_t chunk_index;
} wlan_link_download_chunk_request_payload_t;

typedef struct
{
  uint32_t chunk_index;
  uint32_t valid_bytes;
} wlan_link_download_chunk_response_header_t;
#pragma pack(pop)

//!< CRC16/CCITT-FALSE (init 0xFFFF, poly 0x1021, no reflect, no final xor) - protects the frame header only
uint16_t wlan_link_crc16_ccitt (const uint8_t *data, size_t len);

//!< matches the STM32 firmware's pack.py-compatible CRC32 exactly - see
//!< wlan_link_protocol.cpp for the cross-check notes. 'len' must be a
//!< multiple of 4. Chainable: wlan_link_crc32_init() then repeated
//!< wlan_link_crc32_update() calls to accumulate a running CRC.
uint32_t wlan_link_crc32_init (void);
uint32_t wlan_link_crc32_update (uint32_t crc, const uint8_t *data, size_t len);

#endif /* WLAN_LINK_PROTOCOL_H_ */
