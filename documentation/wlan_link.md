# WLAN Link (STM32 <-> ESP32) — Design Notes

The ESP32's WiFi access point gives access to the STM32's SD card without a
physical card swap: firmware update, `.lrsx` flight-log download/delete,
and SD card formatting.

## Requirements

1. **Firmware update** for the ESP32 itself (Arduino OTA, `/update_esp32`)
   and for the STM32F407 main sensor board. Updates for other devices
   reachable via CAN, with the STM32 as gateway, are deferred.
2. **Log file access**: list, download and delete the STM32's `*.lrsx`
   flight log files, and format the SD card — all without removing the
   card from the sensor.

Both share the same STM32 <-> ESP32 SPI2 link and the same logging-active
gating (don't touch the SD card while it's being actively logged to,
see "Logging pause").

## Hardware link ESP32 <-> STM32

* **UART**: already in use for NMEA/telemetry, not used for this link.
* **SPI2**:
  * IO5 (ESP32) <-> PB1 (STM32) — software chip-select/framing signal, not
    a hardware NSS (see "Framing without a hardware NSS pin")
  * IO18 (ESP32) <-> PB13 (STM32) — SCK
  * IO19 (ESP32) <-> PB14 (STM32) — MISO
  * IO23 (ESP32) <-> PB15 (STM32) — MOSI
  * IO27 (ESP32) <-> PA8 (STM32) — Ready/Busy handshake, STM32 -> ESP32

SPI2 runs with `SPI_NSS_SOFT` (PB1 is not the STM32F407's SPI2 hardware NSS
pin — that alternate function only exists on PB9/PB12).

## STM32 firmware update mechanism

Reuses the existing SD-card update mechanism, predating the WLAN feature —
the WLAN feature only gets the `.bin` file onto the SD card over SPI2
instead of a manual card swap.

`sw_stm32/scripts/pack.py` builds a `.bin` with a metadata header (magic
number, CRC32, hardware/software version, a pointer to a prebuilt copy
routine) in front of the app image. `uSD_helpers.cpp::read_software_update()`
runs on every SD mount: validates magic/hardware version/software version
against the running firmware, writes the file to the staging area
(`0x08060000`) if newer, then resets so `jump_to_pending_flash_update_if_any()`
(`uSD_handler.cpp`) can pick it up on the next boot, before the watchdog is
armed, and jump into the copy routine (source in the sibling
[`larus-breeze/sw_tools`](https://github.com/larus-breeze/sw_tools) repo).
The copy routine re-checks magic + CRC32 (via the STM32's hardware CRC
peripheral) and only then installs the app at `0x08000000` and resets —
**fail-closed**: a corrupted/truncated transfer can't result in a broken
app being installed. The SPI protocol below adds its own end-to-end CRC
check too, so a bad transfer is caught immediately rather than only on the
next boot.

## Design decisions

* **Reboot after a WLAN-delivered update**: the codebase's existing idiom
  (`user_initiated_reset = true;` then `while(true);`, letting the window
  watchdog reset) is reused — no `NVIC_SystemReset()`.
* **Airborne accessor**: `state_vector.flight_mode != ON_GROUND` (already
  refreshed at 100Hz in `communicator.cpp`, sourced from
  `sw_algorithms_lib`'s hysteresis-debounced `airborne_detector_t`).
* **CAN device updates**: deferred — would relay chunks through the STM32
  as a CAN gateway.
* Reuses `Drivers/Custom/spi.cpp`'s existing DMA-driven,
  FreeRTOS-notification-based SPI transfer layer, configuring `hspi2` for
  slave mode (`SPI_NSS_SOFT`) — no new SPI driver.
* Firmware update writes to the SD card and lets the existing
  `read_software_update()`/copy-routine mechanism apply it, rather than
  writing directly into flash over SPI (would duplicate its erase/program
  logic, more safety-critical code on an avionics-adjacent device).

## SPI2 protocol

ESP32 is always SPI master; STM32 is SPI2 slave — the STM32 can never push
data unsolicited, every exchange is initiated by an ESP32 request, even
when the STM32 is logically "the sender" (e.g. a log file download).

### Framing

All multi-byte integers are little-endian. Every frame from the master
starts with a fixed 16-byte header, optionally followed by a payload:

```
Offset  Field           Size    Note
0x00    Frame Type      1 B     see below
0x01    Reserved        3 B     0x00, padding for alignment
0x04    Sequence No.    4 B     monotonically increasing, starts at 0 per session
0x08    Payload Length  4 B     0 for control frames
0x0C    Header CRC16    2 B     CCITT, over bytes [0x00:0x0C)
0x0E    Reserved        2 B     0x00
0x10    Payload         N B     present where noted below
...     Payload CRC32   4 B     stm32_crc() as used by pack.py,
                                 present on all responses
```

| Value | Name                  | Direction     | Payload |
|-------|-----------------------|---------------|---------|
| 0x01  | UPLOAD_BEGIN          | ESP32 -> STM32| total file length (4 B) + filename (up to 32 B, NUL-padded) |
| 0x02  | UPLOAD_CHUNK          | ESP32 -> STM32| up to `CHUNK_SIZE` bytes of file data |
| 0x03  | UPLOAD_END            | ESP32 -> STM32| CRC32 of the whole file (4 B) |
| 0x04  | UPLOAD_ABORT          | ESP32 -> STM32| none |
| 0x10  | STATUS_REQUEST        | ESP32 -> STM32| none |
| 0x11  | LIST_REQUEST          | ESP32 -> STM32| continuation index (4 B, 0 for first request) |
| 0x12  | DOWNLOAD_BEGIN        | ESP32 -> STM32| filename (up to 32 B, NUL-padded) |
| 0x13  | DOWNLOAD_CHUNK_REQUEST| ESP32 -> STM32| chunk index (4 B) |
| 0x14  | DOWNLOAD_END          | ESP32 -> STM32| none (releases the STM32-side file handle) |
| 0x15  | DELETE_REQUEST        | ESP32 -> STM32| filename (up to 32 B, NUL-padded) |
| 0x16  | FORMAT_REQUEST        | ESP32 -> STM32| confirmation token (4 B) |
| 0x81  | ACK                   | STM32 -> ESP32| sequence number being acknowledged (4 B) |
| 0x82  | NACK                  | STM32 -> ESP32| sequence number + reason code (5 B) |
| 0x83  | DATA                  | STM32 -> ESP32| payload depends on the request being answered |

`CHUNK_SIZE` (both directions): **4096 bytes**.

Because SPI cannot be "paused" mid-transaction, the STM32 must have a
`DATA` frame's payload fully staged before the ESP32 starts clocking it
out — this is what makes the PA8/IO27 handshake load-bearing for every
direction, not just upload.

NACK reason codes: `0x01` header CRC mismatch, `0x02` payload CRC
mismatch, `0x03` out-of-sequence, `0x04` SD card not available, `0x05`
file already exists with a version >= incoming, `0x06` rejected —
logging active (see "Logging pause"), `0x07` file not found, `0x08`
format confirmation token mismatch.

Every frame carries a `Sequence No.`; upload chunks are idempotent
(STM32 writes at `f_lseek`'d offsets derived from `Sequence No. * CHUNK_SIZE`),
download chunk requests are idempotent by construction (pure reads, keyed
by chunk index). `Header CRC16` protects the header; every response also
carries a trailing payload `CRC32` (the header CRC alone doesn't cover the
payload). `UPLOAD_END` additionally carries a whole-file CRC32, checked
before ACKing — independent of, and in addition to, the copy routine's own
fail-closed check.

### Handshake (PA8/IO27)

STM32 drives PA8 **high** = ready for the next transaction, **low** =
busy. Every phase of an exchange (request-header receive, request-payload
receive if any, response send) gets its own explicit low-then-high
transition — there is no hardware NSS to mark a transaction boundary, so
PA8 is the only signal telling the ESP32 "the STM32 has actually armed its
DMA for the next phase". The ESP32 (`WlanLinkClient::waitForLinkReady()`)
polls IO27, does not clock while it reads low, and treats a timeout as a
transport failure to retry. IO27 is `INPUT_PULLDOWN`, not plain `INPUT`:
it floats during STM32 boot/reset before
`configure_wlan_link_gpio_and_spi()` has run, and the pulldown guarantees
it reads low (not a stray "go ahead") until the STM32 actively drives it.

### Framing without a hardware NSS pin

PB1 isn't wired to SPI2's hardware NSS input, so the slave peripheral has
no hardware notion of "a new transaction just started" — framing is done
entirely by convention:

1. The STM32 always first arms a DMA receive for exactly the 16-byte
   header (`arm_ready_for_header()`).
2. Once that completes, it parses `Frame Type`/`Payload Length`, then (if
   nonzero) arms a second DMA receive for exactly that many more bytes
   (`arm_ready_and_receive()`) — PA8/READY is announced only once that
   second DMA is armed.
3. Only after both phases complete does it validate `Header CRC16` and
   dispatch the frame, then re-arms the next header receive.

PB1 also carries a second, independent role: a **software chip-select**.
The ESP32 (`assertChipSelect()`/`deassertChipSelect()`) drives it low for
the duration of each SPI phase and releases it right after — this carries
no framing meaning of its own (the STM32 still frames purely by the
header-then-payload convention above), it only tells the STM32 "the
master has genuinely stopped clocking this phase", independent of the
STM32's own DMA byte count. The STM32 watches both edges of this line via
an EXTI interrupt, which is also how a stuck SPI2/DMA transfer is detected
and recovered without a multi-second timeout.

### SPI clock

STM32 SPI2 slave, clocked by the ESP32 master, at **15MHz**. Mode 1
(CPHA=1) gives the slave a half-clock-cycle of margin before the first
sample edge, instead of mode 0's zero-margin requirement.

### ESP32-side transport

`wlan_link_client.cpp` talks to the SPI bus via `driver/spi_master.h`
directly (not Arduino's `SPIClass`, whose non-DMA `transfer()` is limited
to the hardware FIFO's 64 bytes per transaction). `spiTransfer()` issues
DMA-backed transactions up to `WLAN_LINK_SPI_MAX_DMA_LEN` (4000 bytes)
each. PB1/PA8 are unaffected — both are plain GPIO (`spics_io_num = -1`,
no hardware-managed CS).

### Firmware upload

Filename pattern must match `????*.bin`. Written as received, no
transformation. Once `UPLOAD_END` is acknowledged, the STM32 gives the SD
card a moment to flush, then reboots.

## Logging pause

The STM32 writes a `.lrsx` log file (and holds the FatFs lock,
`uSD_handler.cpp`) whenever `is_airborne()` (`state_vector.flight_mode !=
ON_GROUND`) is true. `logging_active` (`uSD_handler.cpp`) is the actual
"is the SD card busy with logging right now" signal — true exactly while
a file is open and the lock held — and is what `UPLOAD_BEGIN`,
`DOWNLOAD_BEGIN`, `DELETE_REQUEST` and `FORMAT_REQUEST` gate on (NACK
reason `0x06`, named `WLAN_NACK_LOGGING_ACTIVE` on the wire), instead of
`is_airborne()` directly.

`STOP_LOGGING_REQUEST`/`START_LOGGING_REQUEST` (`wlan_link_handler.cpp`)
set/clear `logging_paused_by_user`, which the "wait for airborne" loop
treats the same as being grounded — it closes the current file (if any)
and stays out of `logging_active` regardless of `is_airborne()`. This
lets a user resume logging deliberately while still airborne. To guard
against a pause simply being forgotten, `logging_paused_by_user`
auto-clears after `LOGGING_PAUSE_TIMEOUT_ITERATIONS` (~5 minutes) with no
explicit resume, also setting `logging_force_start` (a one-shot flag that
exits the "wait for airborne" loop the same way `is_airborne()` does) so
the timeout firing while genuinely grounded doesn't silently leave
logging off.

`status.airborne` still reports raw flight state for the web UI's info
display, but it's informational only — every actual gate uses
`logging_active`.

## Log file download (STM32 -> ESP32)

`*.lrsx` log files are written hourly, roughly 25MB/hour — a 10-hour
flight is a plausible ~250MB file. The ESP32 cannot buffer a whole file:
downloads stream through RAM only, straight from SPI into the outgoing
HTTP response.

**Double buffering, both sides**: STM32 side, a dedicated `RestrictedTask`
(`download_prefetch_runnable`) owns the download `FIL` exclusively and
reads ahead into a two-slot double buffer — the SD read for chunk N+1
overlaps with the SPI send of chunk N, and each slot's CRC32 is
precomputed off the critical path. Only strictly sequential chunk
requests (plus an immediate retry of the just-delivered chunk, served
from a one-chunk cache) are served — a request for anything else gets
`WLAN_NACK_OUT_OF_SEQUENCE`. ESP32 side mirrors the pattern:
`downloadChunk()` requests chunks, `handleFileDownload()` streams each
into the chunked HTTP response as it arrives.

`web_server.cpp`'s `resyncDownloadSession()` handles an out-of-sequence
NACK by starting a fresh STM32-side session and re-requesting every chunk
already delivered to walk the STM32's cursor back up to the failed one,
bounded to `WLAN_LINK_DOWNLOAD_MAX_RESYNCS` (3) resyncs per download.

## Log file management (list, delete)

* `LIST_REQUEST`/`DATA` enumerates the `logger/` directory
  (`f_opendir`/`f_readdir`, filtering `*.lrsx`), one entry per round trip;
  the continuation index pages a large directory across multiple round
  trips. `handle_list_request()` caches the open `DIR` across requests
  when one continues sequentially right after the previous one — O(1)
  FatFs work per request. `WLAN_LINK_MAX_LISTED_FILES` (10000) bounds the
  loop as a safety net; if hit, the `/files` response carries an
  `X-Files-Truncated: true` header and the web UI shows a "more files
  than can be listed" banner.
* `DELETE_REQUEST` calls `f_unlink()`. Gated on flight state.

## SD card format

* `FORMAT_REQUEST` carries a confirmation token the ESP32 only sends after
  the web UI's own "are you sure" dialog. NACK `0x08` if the token doesn't
  match.
* After a successful format, the handler immediately `f_mkdir("logger")`
  — without it the logger task never starts writing `.lrsx` files.
* Gated on flight state, same as delete.

## SD card presence

`uSD_handler_runnable()` tracks whether a card is mounted, surfaced in
`STATUS_REQUEST` so the web UI can show "please insert an SD card"
instead of every request failing individually. Removing the card is
detected by polling `BSP_PlatformIsDetected()` in the "wait for airborne"
loop (grounded state, the only time `wlan_link_handler_task` can reach
the card anyway); detecting removal there resets the flag, unmounts, and
restarts the mount state machine so a later re-insertion is picked up
cleanly.

`STATUS_REQUEST`'s response also carries `sd_free_bytes`/`sd_total_bytes`
(both `uint64_t`, so a card past 4GB doesn't overflow) via `f_getfree()`,
attempted only when a card is present **and** `logging_active` is false
(that lock is held for the whole logging session, so trying while it's
true would just block and fail). Both fields are 0 whenever they weren't
queried — the web UI shows a plain "present" then, rather than a
misleading "0% free".

## Firmware version reporting

`STATUS_REQUEST`'s response carries `stm32_git_tag_info` - the STM32's
`GIT_TAG_INFO` (`git describe --always --dirty --tags`, e.g.
`"0.7.6-150-ge720e17"`), generated at commit time by the STM32 project's
git hook into `Core/Inc/git-commit-version.h`. The ESP32 has its own,
analogous `GIT_TAG_INFO` (same format, own repo-wide `git describe`),
generated at *build* time instead: `sw_esp32/scripts/create-git-info-header.py`
writes `wlan_link/git-commit-version.h` (gitignored, like the STM32 one),
called automatically by `scripts/build_firmware.py` before compiling.
Both versions are shown in the web UI's "Info" section.

## ESP32 web UI

Single-page layout: an Info section (STM32/ESP32 firmware versions,
airborne state — informational only); status banner (SD-card/
logging-active, from periodic `STATUS_REQUEST` polling); a WiFi client
mode section (see below); a Logging section (Stop/Resume buttons, see
"Logging pause"); firmware update section (STM32 `.bin`, gated on
`logging_active`, and a separate ESP32-WiFi-module update section,
`/update_esp32`, Arduino `Update` library, no relaying to the STM32,
gated only on "not airborne" since it never touches the SD card), both
grouped under one "Larus Sensor" device group in anticipation of other
CAN-connected Larus devices eventually joining the same update mechanism;
log files section (list sorted by modification date descending, with
Download/Delete/Download-all, "Format SD card" with its own
confirmation).

## WiFi client mode

`wifi_config.{h,cpp}`: lets the sensor join an existing WiFi network as a
client instead of only running its own access point - persisted in NVS
(`Preferences`), configured via the web UI's "WiFi client mode" section
(`POST /wifi/config`, `ssid`+`password` form fields; an empty `ssid`
disables it - "Forget network" in the UI). Both `setupWifi()` and the
periodic background retry in `handleWifiLoop()` bring the access point up
*first* and only hand over to the client connection once it actually
succeeds - `WiFi.mode(WIFI_AP_STA)` during the connection attempt keeps
serving any already-connected access point clients throughout, rather
than dropping them for the ultimately uncertain duration of a join
attempt. Falls back to (or stays on) the access point if the configured
network can't be reached at boot, is lost later for more than
`STA_LOST_GRACE_MS` (10s), or a background retry attempt
(`STA_RETRY_INTERVAL_MS`, 60s) times out (`STA_RETRY_CONNECT_TIMEOUT_MS`,
8s). Once connected as a client, the web UI and the NMEA TCP bridge move
to whatever IP that network assigns (`/status`'s `wifiIp`) - no longer
the fixed `192.168.4.1`.

The web UI is also reachable as `http://larus.local`, on either the
access point or a joined client network - an mDNS responder
(`restartMdns()`, `wifi_config.cpp`) is (re)started on every AP<->STA
transition, since the underlying network interface changes and a
responder bound to the old one stops working. On a network shared by
several sensors, only one can hold that name at a time - client OS
support for mDNS (`.local` resolution) varies.

## AP identity: SSID and password

* SSID: `"Larus_<uid>"`, `<uid>` from the low 16 bits of
  `ESP.getEfuseMac()`.
* Password: a random 80-bit value from the ESP32's hardware RNG
  (`esp_random()`), Crockford base32-encoded, generated once on first boot
  and persisted in NVS (`ap_identity.cpp`, namespace `apident`) — printed
  over serial on every boot from then on. Deliberately not derived from
  the chip's MAC, since the AP's BSSID (a fixed, publicly documented
  offset from the same MAC) is broadcast in the clear in every beacon
  frame — a per-device random secret with no relationship to anything
  broadcast over the air avoids leaking anything through the SSID/password
  themselves. `sw_esp32/scripts/generate_ap_label.py` renders a printable
  QR-code label from the SSID/password the firmware itself printed over
  serial at boot.

## FatFs thread-safety

`_FS_REENTRANT` is `0` (`ffconf.h`) — not safe for concurrent access, and
both `uSD_handler_task` (flight logging) and `wlan_link_handler_task`
(upload/download/delete/format) need it. Resolved with two measures:

* `uSD_handler_task` only actively logs while airborne and not paused
  (see "Logging pause"), freeing the SD card for `wlan_link_handler_task`
  the rest of the time. `fatfs_lock()`ed for the whole logging session,
  not just per-write.
* A shared FatFs access layer (`Communication/fatfs_access.{h,cpp}`) owns
  the lock privately and is the only way to reach FatFs:
  `fatfs_lock()`/`fatfs_unlock()` (unbounded wait for `uSD_handler_task`;
  500ms timeout -> `WLAN_NACK_SD_CARD_BUSY` for `wlan_link_handler_task`,
  held for a whole session, not per-chunk), and `fatfs_lock_best_effort()`
  (bounded 20ms, used only by `write_crash_dump()`, which can't wait
  unboundedly if the lock's holder is the task that just crashed).

**Simplifying assumption**: the SD card stays inserted for the whole
session — `wlan_link_handler_task` degrades reasonably if not (checks
`sd_card_mounted`, `WLAN_NACK_NO_SD_CARD`), but a mid-session unmount
during an in-progress upload/format isn't specifically designed or tested.
