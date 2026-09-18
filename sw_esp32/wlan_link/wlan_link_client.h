/** *****************************************************************************
 * @file    wlan_link_client.h
 * @brief   SPI master client for the ESP32 <-> STM32 WLAN link
 *
 * ESP32 is always SPI master, STM32 is SPI2 slave with no hardware NSS -
 * framing is by convention. IO5/PB1 is a software chip-select instead
 * (see assertChipSelect()/deassertChipSelect(), wlan_link_client.cpp).
 * See documentation/wlan_link.md.
 *
 * Pin mapping: IO18/19/23 (SCK/MISO/MOSI) - VSPI (SPI3_HOST), via the
 * ESP-IDF spi_master driver directly, not Arduino's SPIClass (see
 * spiTransfer()). IO5 - software chip-select to the STM32 (PB1). IO27 -
 * ready(high)/busy(low) input from the STM32 (PA8).
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#ifndef WLAN_LINK_CLIENT_H_
#define WLAN_LINK_CLIENT_H_

#include <Arduino.h>
#include "driver/spi_master.h"
#include "wlan_link_protocol.h"

#define WLAN_LINK_PIN_SCK          18
#define WLAN_LINK_PIN_MISO         19
#define WLAN_LINK_PIN_MOSI         23
#define WLAN_LINK_PIN_FRAME_START   5  // IO5 -> STM32 PB1, software chip-select (low = asserted, per SPI phase)
#define WLAN_LINK_PIN_READY        27  // IO27 <- STM32 PA8

// Classic ESP32's VSPI peripheral - IO18/19/23 are its default pins.
#define WLAN_LINK_SPI_HOST         SPI3_HOST

// Measured on real hardware: reliable up to 30 MHz, CRC errors at 40 MHz,
// and above 20 MHz an unrelated STM32 task starts asserting (CPU
// starvation under SPI2's DMA/ISR load). See documentation/wlan_link.md.
#define WLAN_LINK_SPI_CLOCK_HZ   15000000u

// Mode 1 (CPOL=0, CPHA=1), not mode 0 - gives the STM32 slave (no hardware
// NSS) a half-clock-cycle of margin before the first sample edge. Must
// match hspi2.Init.CLKPhase on the STM32 side exactly.
#define WLAN_LINK_SPI_MODE       1u

// Settle delay before the first spiTransfer() of each phase. Currently 0 -
// real hardware showed READY going high well before this delay started,
// i.e. the STM32 was never waiting on it. Kept as a one-line revert.
#define WLAN_LINK_SPI_SETTLE_US             0u

// spiTransfer() splits any request over this into multiple back-to-back
// DMA transactions (driver's actual cap is 4092 bytes).
#define WLAN_LINK_SPI_MAX_DMA_LEN       4000u

#define WLAN_LINK_READY_TIMEOUT_MS      3000u // waiting for the STM32 to signal ready for the next transaction
#define WLAN_LINK_RETRY_COUNT              3u // per logical request, on NACK/timeout/CRC mismatch

class WlanLinkClient
{
public:
  void begin (void);

  //!< true once IO27 is observed high - confirms the STM32-side module is
  //!< present and idle, without sending it a request
  bool waitForLinkReady (uint32_t timeoutMs = WLAN_LINK_READY_TIMEOUT_MS);

  bool statusRequest (wlan_link_status_payload_t &out);
  bool listRequest (uint32_t entryIndex, wlan_link_list_response_payload_t &out);

  bool uploadBegin (const char *filename, uint32_t totalLength, wlan_link_nack_reason_t &nackReasonOut);
  bool uploadChunk (uint32_t chunkIndex, const uint8_t *data, uint32_t len, wlan_link_nack_reason_t &nackReasonOut);
  bool uploadEnd (uint32_t wholeFileCrc32, wlan_link_nack_reason_t &nackReasonOut);
  void uploadAbort (void);

  bool downloadBegin (const char *filename, uint32_t &fileSizeOut, wlan_link_nack_reason_t &nackReasonOut);
  //!< dataOut must have room for WLAN_LINK_CHUNK_SIZE bytes
  bool downloadChunk (uint32_t chunkIndex, uint8_t *dataOut, uint32_t &validBytesOut, wlan_link_nack_reason_t &nackReasonOut);
  void downloadEnd (void);

  bool deleteFile (const char *filename, wlan_link_nack_reason_t &nackReasonOut);
  bool formatSdCard (wlan_link_nack_reason_t &nackReasonOut);

  bool stopLogging (void);  //!< always ACKs on the STM32 side - see "Logging pause"
  bool startLogging (void);

private:
  spi_device_handle_t spiDevice;
  uint32_t nextSequenceNo;

  //!< shared bus/device setup, used by begin() and resetSpiBus()
  void initSpiBus (void);

  //!< Tears down and reinitializes the SPI bus/device - an actual
  //!< peripheral power/clock-gate cycle, the ESP32-side equivalent of the
  //!< STM32's RCC-level SPI2 reset. Called once exchangeWithRetry() gives
  //!< up after WLAN_LINK_RETRY_COUNT attempts. See documentation/wlan_link.md.
  void resetSpiBus (void);

  void assertChipSelect (void);
  void deassertChipSelect (void);
  bool waitReadyBeforeTransaction (void);

  //!< Full-duplex transfer of 'len' bytes, split into WLAN_LINK_SPI_MAX_DMA_LEN
  //!< chunks as needed. txBuf/rxBuf may be nullptr when that direction
  //!< doesn't matter. Returns false on a driver-level failure (distinct
  //!< from a garbled-but-completed transfer, caught by CRC checks instead).
  bool spiTransfer (const uint8_t *txBuf, uint8_t *rxBuf, uint32_t len);

  //!< reads and discards 'count' bytes on the still-open SPI transaction -
  //!< needed whenever exchangeOnce() abandons a response read partway
  //!< through, since the STM32 has already armed a single DMA transmit
  //!< covering the whole response (see wlan_link_client.cpp)
  void drainResponseBytes (uint32_t count);

  //!< checks a trailing CRC32 (the last 4 bytes of 'buf', beyond
  //!< 'payloadLen') against payloadLen bytes of actual payload
  bool checkPayloadTrailerCrc32 (const uint8_t *buf, uint32_t payloadLen);

  //!< one attempt, no retry: send a request frame, read back its response
  //!< header + payload (+ extraTrailerBytes, e.g. a CRC32 the caller
  //!< checks itself). responseTimeoutMs bounds only the wait for the
  //!< response to be staged. verifyPayloadCrc32: check the trailing 4
  //!< bytes as a CRC32 over the payload - used by everything except
  //!< DOWNLOAD_CHUNK_REQUEST, which verifies its own differently-scoped
  //!< trailer itself in downloadChunk().
  bool exchangeOnce (uint8_t frameType, uint32_t sequenceNo,
                      const void *reqPayload, uint16_t reqPayloadLen,
                      const uint8_t *reqTrailerCrcBytes, // 4 bytes, or nullptr
                      uint8_t *respPayloadBuf, size_t respPayloadCapacity, uint32_t extraTrailerBytes,
                      wlan_link_frame_header_t &respHeaderOut,
                      uint32_t responseTimeoutMs = WLAN_LINK_READY_TIMEOUT_MS,
                      bool verifyPayloadCrc32 = false);

  //!< retries exchangeOnce() up to WLAN_LINK_RETRY_COUNT times on timeout
  //!< or a transport-level problem; a received NACK is returned as-is on
  //!< the first attempt, not retried here - callers decide whether a given
  //!< NACK reason is worth retrying
  bool exchangeWithRetry (uint8_t frameType, uint32_t sequenceNo,
                           const void *reqPayload, uint16_t reqPayloadLen,
                           const uint8_t *reqTrailerCrcBytes,
                           uint8_t *respPayloadBuf, size_t respPayloadCapacity, uint32_t extraTrailerBytes,
                           wlan_link_frame_header_t &respHeaderOut,
                           uint32_t responseTimeoutMs = WLAN_LINK_READY_TIMEOUT_MS,
                           bool verifyPayloadCrc32 = false);
};

// f_mkfs() on a large SD card can take tens of seconds.
#define WLAN_LINK_FORMAT_RESPONSE_TIMEOUT_MS  60000u

// Shorter than the STM32 side's own WLAN_LINK_TRANSACTION_TIMEOUT_MS
// (5000ms) for the analogous stall-tolerance wait - not the same phase on
// both sides, but no reason to cut this side's margin shorter.
#define WLAN_LINK_DOWNLOAD_CHUNK_RESPONSE_TIMEOUT_MS  6000u

extern WlanLinkClient wlanLink;

#endif /* WLAN_LINK_CLIENT_H_ */
