/** *****************************************************************************
 * @file    wlan_link_client.cpp
 * @brief   SPI master client for the ESP32 <-> STM32 WLAN link
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#include "wlan_link_client.h"
#include <string.h>

// Every [WLAN] log line gets a "[seconds.milliseconds]" prefix, millis()-
// based - lets log lines be correlated against each other to see actual
// elapsed time, not just message order. Wraps every ~49.7 days like
// millis() itself - fine for this project's use (short debug sessions),
// not meant as a wall-clock timestamp. handleFilesList()'s own listing-
// duration log line (web_server.cpp) uses the same prefix format, kept in
// sync by hand since it isn't routed through this macro.
#define WLAN_LOG(fmt, ...) \
  do \
    { \
      uint32_t wlan_log_now_ms = millis (); \
      Serial.printf ("[%lu.%03lu] " fmt, \
                      (unsigned long) (wlan_log_now_ms / 1000), (unsigned long) (wlan_log_now_ms % 1000), \
                      ##__VA_ARGS__); \
    } \
  while (0)

WlanLinkClient wlanLink;

void WlanLinkClient::initSpiBus (void)
{
  // ESP-IDF spi_master driver directly, not Arduino's SPIClass - see
  // spiTransfer()'s doc comment for why (DMA-sized transactions instead of
  // SPIClass's 64-byte hardware-FIFO chunking).
  spi_bus_config_t busConfig = {};
  busConfig.mosi_io_num = WLAN_LINK_PIN_MOSI;
  busConfig.miso_io_num = WLAN_LINK_PIN_MISO;
  busConfig.sclk_io_num = WLAN_LINK_PIN_SCK;
  busConfig.quadwp_io_num = -1;
  busConfig.quadhd_io_num = -1;
  busConfig.max_transfer_sz = WLAN_LINK_SPI_MAX_DMA_LEN;
  ESP_ERROR_CHECK (spi_bus_initialize (WLAN_LINK_SPI_HOST, &busConfig, SPI_DMA_CH_AUTO));

  spi_device_interface_config_t devConfig = {};
  devConfig.clock_speed_hz = WLAN_LINK_SPI_CLOCK_HZ;
  devConfig.mode = WLAN_LINK_SPI_MODE;
  devConfig.spics_io_num = -1; // no hardware-managed CS pin - see header comment, PB1 is a plain GPIO (assertChipSelect()/deassertChipSelect())
  devConfig.queue_size = 1;
  ESP_ERROR_CHECK (spi_bus_add_device (WLAN_LINK_SPI_HOST, &devConfig, &spiDevice));
}

void WlanLinkClient::begin (void)
{
  pinMode (WLAN_LINK_PIN_FRAME_START, OUTPUT);
  digitalWrite (WLAN_LINK_PIN_FRAME_START, HIGH); // idle (deasserted) high - see assertChipSelect()/deassertChipSelect()
  // Pulldown, not plain INPUT: this pin floats until the STM32's
  // configure_wlan_link_gpio_and_spi() has actually run and driven it -
  // spanning all of STM32 boot/reset before that point. A floating input
  // can read HIGH by chance right when this side is deciding whether the
  // STM32 is ready, which would read as "go ahead" while the STM32 hasn't
  // done anything yet. See wlan_link_handler.cpp's wlan_link_cs_edge() for
  // the matching STM32-side half of this (PA8/READY forced low the
  // instant CS is asserted, not just when a phase is actively armed).
  pinMode (WLAN_LINK_PIN_READY, INPUT_PULLDOWN);

  initSpiBus ();

  nextSequenceNo = 0;

  WLAN_LOG ("[WLAN] begin(): READY (IO%d) reads %s at startup\n",
                  WLAN_LINK_PIN_READY, (digitalRead (WLAN_LINK_PIN_READY) == HIGH) ? "HIGH" : "LOW");
}

//!< See the header's doc comment. spi_bus_remove_device()/spi_bus_free()
//!< tear down the driver's own bookkeeping *and* release the underlying
//!< peripheral (periph_module_disable()) before initSpiBus() re-enables
//!< it (periph_module_enable(), inside spi_bus_initialize()) - not merely
//!< a driver-state reset, an actual hardware peripheral power/clock-gate
//!< cycle, the ESP32-side equivalent of the STM32's RCC-level SPI2 reset.
//!< CS (IO5) and READY (IO27) are plain GPIOs, untouched by this and not
//!< reconfigured here - deliberately: they're already idle/high-Z-safe at
//!< this point (the failed exchange's own deassertChipSelect() already
//!< ran), and this function's whole purpose is to reset the SPI
//!< peripheral's own hardware state, not the framing GPIOs.
void WlanLinkClient::resetSpiBus (void)
{
  ESP_ERROR_CHECK (spi_bus_remove_device (spiDevice));
  ESP_ERROR_CHECK (spi_bus_free (WLAN_LINK_SPI_HOST));
  initSpiBus ();
}

bool WlanLinkClient::waitForLinkReady (uint32_t timeoutMs)
{
  // History (see documentation/wlan_link.md, "Download
  // throughput" sections, for the full story): was delay(2), tightened to
  // delayMicroseconds(50)+yield() which caused STM32-side CAN RX queue
  // overruns (higher overall exchange rate -> more EXTI/DMA ISRs/sec ->
  // less headroom for other STM32 tasks), stepped back to delay(1) as a
  // cautious middle ground while the actual overrun contributors got
  // fixed instead: the download-chunk CRC32 moved to the STM32 hardware
  // CRC unit (wlan_link_crc32_hw_compute(), wlan_link_handler.cpp).
  // Real hardware then ran stable at delay(1) with no CAN overruns even
  // at the higher throughput that fix unlocked (660KB/s) - re-attempting
  // the original delayMicroseconds(50)+yield() tightening now that the
  // CPU-load contributor is gone, as an isolated test. The remaining,
  // still-unaddressed suspect (WLAN_LINK_CS_ISR_PRIORITY sitting above
  // CAN's interrupts - see the doc) may still bite here since tightening
  // this poll raises the exchange rate the same way it did before; watch
  // for CAN overruns again specifically, not just link throughput/
  // reliability.
  uint32_t start = millis ();
  while ((millis () - start) < timeoutMs)
    {
      if (digitalRead (WLAN_LINK_PIN_READY) == HIGH)
        return true;
      delayMicroseconds (50);
      yield ();
    }
  return false;
}

// PB1 software chip-select - not a literal SPI CS/NSS (no SPI2_NSS
// alternate function on this pin), a plain GPIO the STM32 reads via EXTI.
// See documentation/wlan_link.md, "Framing without a hardware NSS pin".
void WlanLinkClient::assertChipSelect (void)
{
  digitalWrite (WLAN_LINK_PIN_FRAME_START, LOW);
}

void WlanLinkClient::deassertChipSelect (void)
{
  // A settle delay was tried here before releasing CS - real hardware
  // showed it made things measurably worse ("stuck shift register"
  // repeated-byte corruption), so it was reverted.
  digitalWrite (WLAN_LINK_PIN_FRAME_START, HIGH);
}

bool WlanLinkClient::waitReadyBeforeTransaction (void)
{
  return waitForLinkReady (WLAN_LINK_READY_TIMEOUT_MS);
}

//!< See the header's doc comment. Splits into WLAN_LINK_SPI_MAX_DMA_LEN-
//!< sized chunks as needed - each spi_device_transmit() call is itself one
//!< continuous DMA transaction (no CPU-mediated gap within it), so this is
//!< still a handful of large transfers rather than the old SPIClass
//!< driver's ~64-byte hardware-FIFO-limited chunking.
bool WlanLinkClient::spiTransfer (const uint8_t *txBuf, uint8_t *rxBuf, uint32_t len)
{
  while (len > 0)
    {
      uint32_t chunk = (len < WLAN_LINK_SPI_MAX_DMA_LEN) ? len : WLAN_LINK_SPI_MAX_DMA_LEN;

      spi_transaction_t t = {};
      t.length = chunk * 8u; // bits, not bytes
      t.tx_buffer = txBuf;
      t.rx_buffer = rxBuf;

      if (ESP_OK != spi_device_transmit (spiDevice, &t))
        return false;

      if (txBuf != nullptr)
        txBuf += chunk;
      if (rxBuf != nullptr)
        rxBuf += chunk;
      len -= chunk;
    }
  return true;
}

//!< Reads and discards 'count' bytes on the still-open SPI transaction.
//!< Needed whenever exchangeOnce() must abandon reading a response partway
//!< through: arm_ready_and_transmit() on the STM32 side arms a *single* DMA
//!< transmit covering the whole response (header + payload together)
//!< *before* announcing it via PA8/READY - if this side stops clocking
//!< after only the header, the STM32's DMA is left mid-transfer, still
//!< expecting the remaining bytes to be clocked out. There is no hardware
//!< CS to tell it the master gave up, so it just keeps waiting - the
//!< *next* transaction's header bytes then get shifted into that still-
//!< active DMA instead of being received fresh by the STM32, corrupting
//!< every following exchange in a fully deterministic (not random-noise)
//!< way, until the STM32 eventually asserts on an arm_ready_and_transmit()
//!< timeout. See documentation/wlan_link.md.
void WlanLinkClient::drainResponseBytes (uint32_t count)
{
  spiTransfer (nullptr, nullptr, count); // neither side's content matters here, only the byte count
}

//!< see the header's doc comment on exchangeOnce()'s verifyPayloadCrc32 param
bool WlanLinkClient::checkPayloadTrailerCrc32 (const uint8_t *buf, uint32_t payloadLen)
{
  uint32_t wordAlignedLen = payloadLen & ~3u;
  uint32_t crc = wlan_link_crc32_update (wlan_link_crc32_init (), buf, wordAlignedLen);
  if (wordAlignedLen != payloadLen)
    {
      uint8_t padded[4] = { 0, 0, 0, 0 };
      memcpy (padded, buf + wordAlignedLen, payloadLen - wordAlignedLen);
      crc = wlan_link_crc32_update (crc, padded, 4);
    }
  uint32_t trailerCrc;
  memcpy (&trailerCrc, buf + payloadLen, sizeof(trailerCrc));
  return crc == trailerCrc;
}

// UPLOAD_CHUNK, DOWNLOAD_CHUNK_REQUEST and LIST_REQUEST are the frame
// types that repeat many times in a row (hundreds to thousands for a
// chunked transfer or a long directory listing - see
// handle_list_request(), wlan_link_handler.cpp), versus at most a
// handful of times for every other frame type. Their
// success-path logging (raw byte dumps, the "OK" line) is suppressed
// below - not failures/retries, which stay logged unconditionally for
// every frame type, since they're rare for these and are exactly what's
// needed to diagnose a stalled/corrupted transfer. Reason: this logging
// goes out via Serial.printf(), and Serial is also COM[0] in
// uart_bridge.cpp - one of the three real UART<->WiFi TCP bridges, fixed
// at 115200 baud for that - so a ~110-character dump line still costs a
// few ms to actually leave the UART (was ~25-30ms at the 38400 baud this
// was originally measured with), which Serial.printf() blocks on once its
// TX buffer fills. Measured on real hardware for the chunk-transfer case
// at that original baud: with 2-3 such lines per exchange and no idle
// time left to drain them in the background, this was costing tens of
// ms per exchange - the
// dominant share of an observed gap between the logged request and the
// logged response that plain SPI bit time didn't explain. A file listing
// (LIST_REQUEST) hit the exact same cost, just not caught here initially
// since it wasn't recognized as "high volume" at the time - each entry is
// its own separate exchange, so a listing of a few hundred files was
// paying this 3-lines-per-entry cost on every single one.
static bool isHighVolumeFrame (uint8_t frameType)
{
  return (frameType == WLAN_FRAME_UPLOAD_CHUNK) || (frameType == WLAN_FRAME_DOWNLOAD_CHUNK_REQUEST)
      || (frameType == WLAN_FRAME_LIST_REQUEST);
}

bool WlanLinkClient::exchangeOnce (uint8_t frameType, uint32_t sequenceNo,
                                    const void *reqPayload, uint16_t reqPayloadLen,
                                    const uint8_t *reqTrailerCrcBytes,
                                    uint8_t *respPayloadBuf, size_t respPayloadCapacity, uint32_t extraTrailerBytes,
                                    wlan_link_frame_header_t &respHeaderOut,
                                    uint32_t responseTimeoutMs,
                                    bool verifyPayloadCrc32)
{
  if (! waitReadyBeforeTransaction ())
    {
      WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: READY never went high before sending (STM32 not responding / idle?)\n",
                      frameType, (unsigned long) sequenceNo);
      return false;
    }

  wlan_link_frame_header_t reqHeader;
  reqHeader.frame_type = frameType;
  reqHeader.reserved0[0] = reqHeader.reserved0[1] = reqHeader.reserved0[2] = 0;
  reqHeader.sequence_no = sequenceNo;
  reqHeader.payload_length = reqPayloadLen;
  reqHeader.reserved1 = 0;
  reqHeader.header_crc16 = wlan_link_crc16_ccitt ((const uint8_t *) &reqHeader, 12);

  assertChipSelect ();
  delayMicroseconds (WLAN_LINK_SPI_SETTLE_US); // let the peripheral settle before the first spiTransfer() - see the doc comment on this constant

  // aligned(4): an unaligned rx/tx buffer forces the ESP-IDF driver through
  // an internal DMA-capable bounce buffer - on real hardware that path was
  // observed to only copy back the first 12 of 16 bytes on the response
  // read below, silently leaving the tail (header_crc16+reserved1) at
  // whatever the buffer held before (see the memset() poison fill there).
  // Aligning both header buffers lets the driver DMA straight to/from them,
  // avoiding the bounce copy entirely.
  uint8_t __attribute__((aligned(4))) headerBuf[sizeof(wlan_link_frame_header_t)];
  memcpy (headerBuf, &reqHeader, sizeof(headerBuf));

  // rxBuf nullptr: received bytes here are ignored - STM32 is only receiving in this phase
  bool headerSendOk = spiTransfer (headerBuf, nullptr, sizeof(headerBuf));

  deassertChipSelect ();

  if (! headerSendOk)
    {
      WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: SPI driver error sending request header\n",
                      frameType, (unsigned long) sequenceNo);
      return false;
    }

  // Logged *after* deassertChipSelect(), deliberately - this is what we
  // intended to send, independent of anything that might go wrong clocking
  // it out, same as before, but Serial.printf() over UART (115200 baud by
  // the time this runs - see wlan_link.ino's Serial.begin() call) blocks
  // for several ms on a line this long - logging it *before* spiTransfer()
  // used to stretch the gap between the settle delay above and the actual
  // transfer by that much, on every single request.
  // See documentation/wlan_link.md. Reads
  // from headerBuf itself - unlike the old SPIClass driver, spiTransfer()
  // with rxBuf=nullptr never overwrites it, so it still holds exactly what
  // was sent.
  if (! isHighVolumeFrame (frameType))
    {
      const uint8_t *sentBytes = headerBuf;
      WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: sending request header: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                      frameType, (unsigned long) sequenceNo,
                      sentBytes[0], sentBytes[1], sentBytes[2], sentBytes[3],
                      sentBytes[4], sentBytes[5], sentBytes[6], sentBytes[7],
                      sentBytes[8], sentBytes[9], sentBytes[10], sentBytes[11],
                      sentBytes[12], sentBytes[13], sentBytes[14], sentBytes[15]);
    }

  if ((reqPayloadLen > 0) || (reqTrailerCrcBytes != nullptr))
    {
      // The STM32 only arms its second (payload) DMA receive *after*
      // parsing the header we just sent - clocking the payload immediately,
      // back-to-back with the header and with no pause of its own, raced
      // that re-arm on real hardware and lost the first payload byte(s),
      // stalling the STM32 side forever waiting for bytes that had already
      // gone by. Wait for the STM32 to explicitly announce "payload DMA
      // armed, go ahead" (see arm_ready_and_receive() in
      // wlan_link_handler.cpp), the same way the response phase below
      // already waits for "response staged".
      if (! waitForLinkReady (WLAN_LINK_READY_TIMEOUT_MS))
        {
          WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: READY timeout waiting for payload DMA to be armed (payloadLen=%u)\n",
                          frameType, (unsigned long) sequenceNo, reqPayloadLen);
          return false;
        }

      assertChipSelect ();
      delayMicroseconds (WLAN_LINK_SPI_SETTLE_US); // let the peripheral settle before the first spiTransfer() - see the doc comment on this constant

      bool payloadSendOk = true;
      if (reqPayloadLen > 0)
        // rxBuf nullptr: received bytes here are irrelevant. Passed
        // straight through from the caller's const payload pointer -
        // unlike the old SPIClass driver, spiTransfer() never writes
        // through txBuf, so no scratch copy is needed to protect it.
        payloadSendOk = spiTransfer ((const uint8_t *) reqPayload, nullptr, reqPayloadLen);

      if (payloadSendOk && (reqTrailerCrcBytes != nullptr))
        {
          // Same per-phase settle gap as elsewhere (see
          // WLAN_LINK_SPI_SETTLE_US's doc comment) - only needed here if a
          // payload transfer already happened first in this same phase;
          // otherwise this trailer *is* the first transfer and already got
          // its settle delay above.
          if (reqPayloadLen > 0)
            delayMicroseconds (WLAN_LINK_SPI_SETTLE_US);
          payloadSendOk = spiTransfer (reqTrailerCrcBytes, nullptr, 4);
        }

      deassertChipSelect ();

      if (! payloadSendOk)
        {
          WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: SPI driver error sending request payload\n",
                          frameType, (unsigned long) sequenceNo);
          return false;
        }
    }

  // Now wait for the STM32 to actually have the response staged - see
  // arm_ready_and_transmit() on the STM32 side: PA8 only goes high once the
  // response DMA is armed, so there is no race here no matter how fast we
  // react to it.
  if (! waitForLinkReady (responseTimeoutMs))
    {
      WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: READY timeout waiting for the response (STM32 never staged one within %lums)\n",
                      frameType, (unsigned long) sequenceNo, (unsigned long) responseTimeoutMs);
      return false;
    }

  assertChipSelect ();
  delayMicroseconds (WLAN_LINK_SPI_SETTLE_US); // let the peripheral settle before the first spiTransfer() - see the doc comment on this constant

  // aligned(4): see headerBuf's doc comment above - this is the buffer
  // where the truncated-copy-back symptom was actually observed.
  uint8_t __attribute__((aligned(4))) respHeaderBuf[sizeof(wlan_link_frame_header_t)];
  memset (respHeaderBuf, 0xAA, sizeof(respHeaderBuf)); // distinctive fill, not 0x00/0xFF - see the raw dump below
  if (! spiTransfer (nullptr, respHeaderBuf, sizeof(respHeaderBuf)))
    {
      // Same reasoning as the header-CRC-mismatch drain just below: we
      // don't know how many of the header's 16 bytes the failed
      // spiTransfer() actually got through, so payload_length can't be
      // trusted either - drain up to the caller's best-effort upper bound
      // so the STM32's single-DMA response transmit (see
      // drainResponseBytes()'s doc comment) gets fully consumed either
      // way. Missing here until real hardware showed 50MB downloads
      // sometimes wedging the link badly enough to need a manual reset of
      // both sides - see documentation/wlan_link.md.
      drainResponseBytes ((uint32_t) respPayloadCapacity);
      deassertChipSelect ();
      WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: SPI driver error reading response header\n",
                      frameType, (unsigned long) sequenceNo);
      return false;
    }
  memcpy (&respHeaderOut, respHeaderBuf, sizeof(respHeaderOut));

  // Raw dump logged further below, *after* whichever deassertChipSelect()
  // call actually applies - deliberately not here. This used to log right
  // here, between this transfer and the payload transfer further down,
  // both still inside the same open phase - but Serial.printf() over UART
  // (115200 baud by the time this runs - see wlan_link.ino's Serial.begin()
  // call) blocks for several ms on a line this long, turning the "same
  // per-transfer settle gap" the payload read below is documented to need
  // into an unplanned multi-millisecond one instead of
  // the intended tightly-bounded WLAN_LINK_SPI_SETTLE_US. See
  // documentation/wlan_link.md.
#define WLAN_LOG_RAW_RESPONSE_HEADER() \
  WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: raw response header: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", \
                  frameType, (unsigned long) sequenceNo, \
                  respHeaderBuf[0], respHeaderBuf[1], respHeaderBuf[2], respHeaderBuf[3], \
                  respHeaderBuf[4], respHeaderBuf[5], respHeaderBuf[6], respHeaderBuf[7], \
                  respHeaderBuf[8], respHeaderBuf[9], respHeaderBuf[10], respHeaderBuf[11], \
                  respHeaderBuf[12], respHeaderBuf[13], respHeaderBuf[14], respHeaderBuf[15])

  uint16_t computedCrc = wlan_link_crc16_ccitt (respHeaderBuf, 12);
  if (computedCrc != respHeaderOut.header_crc16)
    {
      // The header itself is garbled, so its payload_length can't be
      // trusted - drain up to what the caller told us it could ever
      // receive as a best-effort upper bound, so the STM32's single-DMA
      // response transmit (see drainResponseBytes()'s doc comment) gets
      // fully consumed either way.
      drainResponseBytes ((uint32_t) respPayloadCapacity);
      deassertChipSelect ();
      WLAN_LOG_RAW_RESPONSE_HEADER ();
      WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: response header CRC mismatch (got 0x%04X, computed 0x%04X) - garbled response\n",
                      frameType, (unsigned long) sequenceNo, respHeaderOut.header_crc16, computedCrc);
      return false; // garbled header - exchangeWithRetry() will retry the whole exchange
    }

  uint32_t bytesToRead = respHeaderOut.payload_length + extraTrailerBytes;
  if (bytesToRead > respPayloadCapacity)
    {
      // Header CRC passed, so payload_length itself is trustworthy here -
      // drain exactly what the STM32 actually armed instead of just the
      // (smaller) capacity we can't fit it in.
      drainResponseBytes (bytesToRead);
      deassertChipSelect ();
      WLAN_LOG_RAW_RESPONSE_HEADER ();
      WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: response payload too big (%lu bytes, capacity %u) - treating as transport failure\n",
                      frameType, (unsigned long) sequenceNo, (unsigned long) bytesToRead, (unsigned) respPayloadCapacity);
      return false; // response bigger than the caller told us to expect - treat as a transport failure
    }

  if (bytesToRead > 0)
    {
      // Same settle gap as before the *first* transfer of this phase (see
      // WLAN_LINK_SPI_SETTLE_US's doc comment) - real hardware (with the
      // old SPIClass-based driver) showed this payload read itself
      // occasionally picking up one stale byte even after the header CRC
      // had already passed (current_sw_version misread as 0x40040000 -
      // 0x40 being the same stale-FIFO byte value seen throughout this
      // investigation). See documentation/wlan_link.md,
      // "Response payload still corrupting despite a valid header CRC".
      delayMicroseconds (WLAN_LINK_SPI_SETTLE_US);
      memset (respPayloadBuf, 0, bytesToRead);
      if (! spiTransfer (nullptr, respPayloadBuf, bytesToRead)) // internally split into WLAN_LINK_SPI_MAX_DMA_LEN-sized transactions if needed
        {
          // spiTransfer() doesn't report how far it got before failing -
          // draining another full bytesToRead is deliberately generous
          // rather than trying to guess the actual shortfall: clocking a
          // few extra dummy bytes past what the STM32 was still expecting
          // is harmless (it just stops feeding real data once its own DMA
          // completes), but draining too little leaves its single-DMA
          // response transmit mid-flight - see drainResponseBytes()'s doc
          // comment and, same as the header-read failure above, the real-
          // hardware 50MB-download wedge that was missing this.
          drainResponseBytes (bytesToRead);
          deassertChipSelect ();
          WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: SPI driver error reading response payload\n",
                          frameType, (unsigned long) sequenceNo);
          return false;
        }
    }

  deassertChipSelect ();
  if (! isHighVolumeFrame (frameType))
    WLAN_LOG_RAW_RESPONSE_HEADER ();
#undef WLAN_LOG_RAW_RESPONSE_HEADER

  if (verifyPayloadCrc32 && (! checkPayloadTrailerCrc32 (respPayloadBuf, respHeaderOut.payload_length)))
    {
      // header_crc16 only ever covered the 12-byte header, never the
      // payload - this is what catches the corruption that used to slip
      // through as a plausible-looking but wrong value (current_sw_version
      // read as 0x40040000, airborne read as true while grounded). See
      // documentation/wlan_link.md, "Payload CRC protection for
      // all responses".
      WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: response payload CRC32 mismatch - garbled response\n",
                      frameType, (unsigned long) sequenceNo);
      return false; // exchangeWithRetry() will retry the whole exchange
    }

  if (! isHighVolumeFrame (frameType))
    WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: OK, response 0x%02X payloadLen=%lu\n",
                    frameType, (unsigned long) sequenceNo, respHeaderOut.frame_type, (unsigned long) respHeaderOut.payload_length);

  // NACK reason byte, decoded here rather than left to each caller - this
  // is the one piece of information that actually distinguishes *why* the
  // STM32 rejected a request (header CRC vs. SD card busy vs. airborne
  // vs. ...), which the earlier investigation into frequent NACKs on
  // STATUS_REQUEST was missing. wlan_link_nack_payload_t is
  // {sequence_no(4), reason(1)} - respPayloadBuf[4] is the reason byte,
  // valid whenever payload_length is at least 5 (true for every real NACK).
  if ((respHeaderOut.frame_type == WLAN_FRAME_NACK) && (respHeaderOut.payload_length >= 5))
    WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: NACK reason=0x%02X\n",
                    frameType, (unsigned long) sequenceNo, respPayloadBuf[4]);

  return true;
}

bool WlanLinkClient::exchangeWithRetry (uint8_t frameType, uint32_t sequenceNo,
                                         const void *reqPayload, uint16_t reqPayloadLen,
                                         const uint8_t *reqTrailerCrcBytes,
                                         uint8_t *respPayloadBuf, size_t respPayloadCapacity, uint32_t extraTrailerBytes,
                                         wlan_link_frame_header_t &respHeaderOut,
                                         uint32_t responseTimeoutMs,
                                         bool verifyPayloadCrc32)
{
  for (unsigned attempt = 0; attempt < WLAN_LINK_RETRY_COUNT; ++attempt)
    {
      if (exchangeOnce (frameType, sequenceNo, reqPayload, reqPayloadLen, reqTrailerCrcBytes,
                         respPayloadBuf, respPayloadCapacity, extraTrailerBytes, respHeaderOut, responseTimeoutMs,
                         verifyPayloadCrc32))
        return true;
      if ((attempt + 1) < WLAN_LINK_RETRY_COUNT)
        WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: attempt %u/%u failed, retrying\n",
                        frameType, (unsigned long) sequenceNo, attempt + 1, WLAN_LINK_RETRY_COUNT);
      delay (20 * (attempt + 1)); // small backoff
    }
  WLAN_LOG ("[WLAN] frame 0x%02X seq=%lu: giving up after %u attempts\n",
                  frameType, (unsigned long) sequenceNo, WLAN_LINK_RETRY_COUNT);
  // see resetSpiBus()'s doc comment - every WLAN_LINK_RETRY_COUNT attempts
  // already failed on the wire, so this side's own SPI hardware is at
  // least a plausible contributor; resetting it here costs nothing on the
  // (overwhelmingly common) path where it wasn't actually the cause, and
  // the caller already treats this whole exchange as failed regardless.
  resetSpiBus ();
  return false;
}

bool WlanLinkClient::statusRequest (wlan_link_status_payload_t &out)
{
  // +4: trailing CRC32 every response now carries - see checkPayloadTrailerCrc32()'s
  // doc comment and exchangeOnce()'s verifyPayloadCrc32 param. Read into a
  // scratch buffer, not straight into 'out': 'out' is sized to exactly the
  // real payload, with no room for those extra 4 bytes.
  // aligned(4): see headerBuf's doc comment in exchangeOnce() - an
  // unaligned rx buffer here can silently lose the tail of the response.
  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_status_payload_t) + 4];
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_STATUS_REQUEST, nextSequenceNo++, nullptr, 0, nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false;
  if (! ((respHeader.frame_type == WLAN_FRAME_DATA) && (respHeader.payload_length == sizeof(out))))
    return false;
  memcpy (&out, respBuf, sizeof(out));
  return true;
}

bool WlanLinkClient::listRequest (uint32_t entryIndex, wlan_link_list_response_payload_t &out)
{
  wlan_link_list_request_payload_t req;
  req.entry_index = entryIndex;

  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_list_response_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_LIST_REQUEST, nextSequenceNo++, &req, sizeof(req), nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false;
  if (! ((respHeader.frame_type == WLAN_FRAME_DATA) && (respHeader.payload_length == sizeof(out))))
    return false;
  memcpy (&out, respBuf, sizeof(out));
  return true;
}

bool WlanLinkClient::uploadBegin (const char *filename, uint32_t totalLength, wlan_link_nack_reason_t &nackReasonOut)
{
  wlan_link_upload_begin_payload_t req;
  memset (&req, 0, sizeof(req));
  req.total_length = totalLength;
  strncpy (req.filename, filename, sizeof(req.filename) - 1);

  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_nack_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_UPLOAD_BEGIN, nextSequenceNo++, &req, sizeof(req), nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false;

  if (respHeader.frame_type == WLAN_FRAME_ACK)
    return true;

  if (respHeader.frame_type == WLAN_FRAME_NACK)
    {
      wlan_link_nack_payload_t nack;
      memcpy (&nack, respBuf, sizeof(nack));
      nackReasonOut = (wlan_link_nack_reason_t) nack.reason;
    }
  return false;
}

bool WlanLinkClient::uploadChunk (uint32_t chunkIndex, const uint8_t *data, uint32_t len, wlan_link_nack_reason_t &nackReasonOut)
{
  // sequence_no doubles as the chunk index here - the STM32 side uses it
  // directly to seek within the upload file (f_lseek), not just as a
  // generic per-transaction counter. See handle_upload_chunk() in
  // wlan_link_handler.cpp.
  uint32_t wordAlignedLen = len & ~3u;
  uint32_t remainder = len - wordAlignedLen;
  uint32_t crc = wlan_link_crc32_update (wlan_link_crc32_init (), data, wordAlignedLen);
  if (remainder != 0)
    {
      uint8_t padded[4] = { 0, 0, 0, 0 };
      memcpy (padded, data + wordAlignedLen, remainder);
      crc = wlan_link_crc32_update (crc, padded, 4);
    }

  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_nack_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_UPLOAD_CHUNK, chunkIndex, data, (uint16_t) len, (const uint8_t *) &crc,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false;

  if (respHeader.frame_type == WLAN_FRAME_ACK)
    return true;

  if (respHeader.frame_type == WLAN_FRAME_NACK)
    {
      wlan_link_nack_payload_t nack;
      memcpy (&nack, respBuf, sizeof(nack));
      nackReasonOut = (wlan_link_nack_reason_t) nack.reason;
    }
  return false;
}

bool WlanLinkClient::uploadEnd (uint32_t wholeFileCrc32, wlan_link_nack_reason_t &nackReasonOut)
{
  wlan_link_upload_end_payload_t req;
  req.whole_file_crc32 = wholeFileCrc32;

  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_nack_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_UPLOAD_END, nextSequenceNo++, &req, sizeof(req), nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false; // note: the STM32 reboots right after ACK'ing this, so a transport failure here is ambiguous - see web_server.cpp's handling

  if (respHeader.frame_type == WLAN_FRAME_ACK)
    return true;

  if (respHeader.frame_type == WLAN_FRAME_NACK)
    {
      wlan_link_nack_payload_t nack;
      memcpy (&nack, respBuf, sizeof(nack));
      nackReasonOut = (wlan_link_nack_reason_t) nack.reason;
    }
  return false;
}

void WlanLinkClient::uploadAbort (void)
{
  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_ack_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  exchangeWithRetry (WLAN_FRAME_UPLOAD_ABORT, nextSequenceNo++, nullptr, 0, nullptr,
                      respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true);
  // best-effort: nothing sensible to do if this itself fails, the STM32-side
  // idle-session-timeout (30s, see wlan_link_handler.cpp) cleans up eventually either way
}

bool WlanLinkClient::downloadBegin (const char *filename, uint32_t &fileSizeOut, wlan_link_nack_reason_t &nackReasonOut)
{
  wlan_link_download_begin_payload_t req;
  memset (&req, 0, sizeof(req));
  strncpy (req.filename, filename, sizeof(req.filename) - 1);

  // aligned(4): see headerBuf's doc comment in exchangeOnce()
  uint8_t __attribute__((aligned(4))) respBuf[(sizeof(wlan_link_download_begin_response_payload_t) > sizeof(wlan_link_nack_payload_t)
                  ? sizeof(wlan_link_download_begin_response_payload_t) : sizeof(wlan_link_nack_payload_t)) + 4]; // +4: see statusRequest()
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_DOWNLOAD_BEGIN, nextSequenceNo++, &req, sizeof(req), nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false;

  if (respHeader.frame_type == WLAN_FRAME_DATA)
    {
      wlan_link_download_begin_response_payload_t resp;
      memcpy (&resp, respBuf, sizeof(resp));
      fileSizeOut = resp.file_size;
      return true;
    }

  if (respHeader.frame_type == WLAN_FRAME_NACK)
    {
      wlan_link_nack_payload_t nack;
      memcpy (&nack, respBuf, sizeof(nack));
      nackReasonOut = (wlan_link_nack_reason_t) nack.reason;
    }
  return false;
}

bool WlanLinkClient::downloadChunk (uint32_t chunkIndex, uint8_t *dataOut, uint32_t &validBytesOut, wlan_link_nack_reason_t &nackReasonOut)
{
  wlan_link_download_chunk_request_payload_t req;
  req.chunk_index = chunkIndex;

  // response is: wlan_link_download_chunk_response_header_t (8B) + up to
  // WLAN_LINK_CHUNK_SIZE bytes of data, with payload_length covering both;
  // a trailing 4B CRC32 follows on top of that (extraTrailerBytes=4).
  // aligned(4): word-aligned so spiTransfer()'s DMA reads land straight in
  // this buffer instead of the ESP-IDF driver bouncing through an internal
  // copy - see spiTransfer()'s doc comment.
  static uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_download_chunk_response_header_t) + WLAN_LINK_CHUNK_SIZE + 4];
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_DOWNLOAD_CHUNK_REQUEST, nextSequenceNo++, &req, sizeof(req), nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_DOWNLOAD_CHUNK_RESPONSE_TIMEOUT_MS))
    return false;

  if (respHeader.frame_type == WLAN_FRAME_NACK)
    {
      wlan_link_nack_payload_t nack;
      memcpy (&nack, respBuf, sizeof(nack));
      nackReasonOut = (wlan_link_nack_reason_t) nack.reason;
      return false;
    }

  if (respHeader.frame_type != WLAN_FRAME_DATA)
    return false;

  wlan_link_download_chunk_response_header_t chunkHeader;
  memcpy (&chunkHeader, respBuf, sizeof(chunkHeader));

  uint32_t dataLen = chunkHeader.valid_bytes;
  if (dataLen > WLAN_LINK_CHUNK_SIZE)
    return false; // malformed - shouldn't happen, guard against a bad read anyway

  const uint8_t *data = respBuf + sizeof(chunkHeader);
  const uint32_t *trailer = (const uint32_t *) (data + dataLen);

  uint32_t wordAlignedLen = dataLen & ~3u;
  uint32_t crc = wlan_link_crc32_update (wlan_link_crc32_init (), data, wordAlignedLen);
  if (wordAlignedLen != dataLen)
    {
      uint8_t padded[4] = { 0, 0, 0, 0 };
      memcpy (padded, data + wordAlignedLen, dataLen - wordAlignedLen);
      crc = wlan_link_crc32_update (crc, padded, 4);
    }

  if (crc != *trailer)
    return false; // caller (web_server.cpp) is expected to retry the same chunk_index on failure

  memcpy (dataOut, data, dataLen);
  validBytesOut = dataLen;
  return true;
}

void WlanLinkClient::downloadEnd (void)
{
  // response is always a plain ACK (handle_download_end() never NACKs) -
  // +4: see statusRequest(). Was previously passed a nullptr/0-capacity
  // buffer, which made every attempt fail the "response too big" check in
  // exchangeOnce() before even reading the ACK; harmless since this is
  // best-effort and the result was never checked, but fixed anyway while
  // touching this function for the trailer CRC32 change.
  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_ack_payload_t) + 4]; // aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  exchangeWithRetry (WLAN_FRAME_DOWNLOAD_END, nextSequenceNo++, nullptr, 0, nullptr,
                      respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true); // best-effort, see uploadAbort()'s comment
}

bool WlanLinkClient::deleteFile (const char *filename, wlan_link_nack_reason_t &nackReasonOut)
{
  wlan_link_filename_payload_t req;
  memset (&req, 0, sizeof(req));
  strncpy (req.filename, filename, sizeof(req.filename) - 1);

  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_nack_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_DELETE_REQUEST, nextSequenceNo++, &req, sizeof(req), nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false;

  if (respHeader.frame_type == WLAN_FRAME_ACK)
    return true;

  if (respHeader.frame_type == WLAN_FRAME_NACK)
    {
      wlan_link_nack_payload_t nack;
      memcpy (&nack, respBuf, sizeof(nack));
      nackReasonOut = (wlan_link_nack_reason_t) nack.reason;
    }
  return false;
}

bool WlanLinkClient::formatSdCard (wlan_link_nack_reason_t &nackReasonOut)
{
  wlan_link_format_payload_t req;
  req.confirmation_token = WLAN_LINK_FORMAT_CONFIRMATION_TOKEN;

  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_nack_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  // Deliberately a single exchangeOnce(), not exchangeWithRetry(): format
  // can take tens of seconds (WLAN_LINK_FORMAT_RESPONSE_TIMEOUT_MS), and
  // blindly retrying a still-processing request could send a second
  // FORMAT_REQUEST while the first is still running. If this returns
  // false, the caller (web_server.cpp) should not treat it as certain
  // failure - the format may still be in progress or may have already
  // succeeded without the response making it back.
  if (! exchangeOnce (WLAN_FRAME_FORMAT_REQUEST, nextSequenceNo++, &req, sizeof(req), nullptr,
                       respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_FORMAT_RESPONSE_TIMEOUT_MS, true))
    return false;

  if (respHeader.frame_type == WLAN_FRAME_ACK)
    return true;

  if (respHeader.frame_type == WLAN_FRAME_NACK)
    {
      wlan_link_nack_payload_t nack;
      memcpy (&nack, respBuf, sizeof(nack));
      nackReasonOut = (wlan_link_nack_reason_t) nack.reason;
    }
  return false;
}

bool WlanLinkClient::stopLogging (void)
{
  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_ack_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_STOP_LOGGING_REQUEST, nextSequenceNo++, nullptr, 0, nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false;
  return respHeader.frame_type == WLAN_FRAME_ACK;
}

bool WlanLinkClient::startLogging (void)
{
  uint8_t __attribute__((aligned(4))) respBuf[sizeof(wlan_link_ack_payload_t) + 4]; // +4: see statusRequest(); aligned(4): see headerBuf's doc comment in exchangeOnce()
  wlan_link_frame_header_t respHeader;
  if (! exchangeWithRetry (WLAN_FRAME_START_LOGGING_REQUEST, nextSequenceNo++, nullptr, 0, nullptr,
                            respBuf, sizeof(respBuf), 4, respHeader, WLAN_LINK_READY_TIMEOUT_MS, true))
    return false;
  return respHeader.frame_type == WLAN_FRAME_ACK;
}
