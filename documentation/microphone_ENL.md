# Microphone and ENL (engine noise level)

Prototype: a TDK ICS-43434 I2S MEMS microphone on the existing sensor
hardware yields the IGC Environmental Noise Level (ENL, IGC GNSS FR
Technical Specification, chapter 5) plus noise band levels for tuning.
Built in by `RUN_MICROPHONE` in `system_configuration.h`; the same
firmware runs on hardware with and without a microphone (see
"Microphone detection").

Not yet calibrated or tested in a cockpit: the ENL numbers below are
design values, not GFAC-tested ones.

## Wiring

| ICS-43434 | STM32 | Header signal | Note |
|-----------|-------|---------------|------|
| WS  | PA15 (I2S3_WS, AF6) | SPI1_NSS_ALT | |
| SCK | PB3 (I2S3_CK, AF6)  | SWO          | SWD still works, SWV trace does not |
| SD  | PB5 (I2S3_SD, AF6)  | CAN2_RX      | internal pull-down: SD is tri-stated in the other slot |
| SEL | GND or VDD          |              | either slot works, the driver uses the active one |

None of these pins is used by the firmware otherwise (CAN2 is never
initialized). PA15 and PB3 are JTAG pins after reset; selecting AF6
releases them.

## Clock and low-power mode

PLLI2S = 2 MHz (16 MHz HSE / PLLM 8) × 128 = 256 MHz VCO, / 5 =
51.2 MHz; I2SDIV = 50 gives fs = 8 kHz with 32-bit slots, so SCK =
512 kHz. In the SCK range 460–870 kHz the ICS-43434 runs in its
low-power mode and delivers already decimated and filtered data at
fs = SCK / 64. The driver checks the resulting I2S clock at start-up:
the first version assumed an 8 MHz HSE and ran at 16 kHz (VCO 512 MHz,
out of spec), so every tone showed up at half its frequency.

## Data flow

- I2S3 master receive, Philips, 24 bit in 32-bit slots.
- DMA1 stream 0 channel 0 (SPI3_RX), circular, 2 × 128 frames
  (16 ms per half). The buffer is in RAM, as the DMA cannot reach the CCM.
- Task `MIC` (`Communication/microphone_driver.cpp`) feeds the samples to
  `microphone_analysis_t` (`lib/Generic_Algorithms/microphone_analysis.cpp`,
  built `-O3`; `Communication/` is built `-O0`). On DMA errors, overrun
  or a 100 ms timeout it restarts I2S and DMA.
- `MIC` runs above the watchdog task, so its load must stay low: the
  window watchdog needs a refresh every 24–50 ms (task period 40 ms). A
  first third-octave version with all bands at 8 kHz and `-O0` took
  about half of the CPU and caused resets. Now, measured in QEMU
  (`-icount`) including the driver's `-O0` block loop: 4.9 M
  instructions/s, 2.9 % of 168 MHz at one instruction per cycle; real
  cycles are more (FPU loads/stores, flash wait states), estimated below
  6 %. One 16 ms block runs for well under 1 ms.
- Every 250 ms one record goes to `microphone_record_queue`; the
  communicator writes it into the LRSX log file. `microphone_data` holds
  the latest record for the debugger's live view.

- Web UI test page `/audio` (ESP32): live ENL, levels and third-octave
  spectrum with peak hold, and a 60 s history, via `MICROPHONE_REQUEST` (documentation/wlan_link.md,
  "Microphone levels").

## Signal processing

- DC blocker (~2.5 Hz, keeps the 25 Hz band unattenuated).
- ENL filter: two identical RBJ band-pass stages, 150 Hz, Q = 1.2. This
  is the "bell curve" with peak sensitivity between 100 and 200 Hz that
  IGC asks for (relative response: 100 Hz −6 dB, 200 Hz −3.5 dB,
  50 Hz −21 dB, 1 kHz −37 dB).
- ENL = 10 + gain × RMS of the filter output over the last 1 s,
  clamped to 999. Linear in amplitude like the analog ENL circuits:
  default gain gives ENL 900 at −26 dBFS in band (94 dB SPL), ENL ~190
  at 80 dB SPL, ~40 at 65 dB SPL.
- 010 is the no-noise baseline required by IGC. 000 (unserviceable) is
  reported when the samples are constant over a record (no microphone,
  stuck line).
- 21 third-octave bands 25 Hz … 2.5 kHz (centers 1000 Hz × 2^(k/3)),
  broadband level and peak sample, 250 ms each: to choose center
  frequency, bandwidth and gain from real flights (engine firing and
  propeller blade frequencies, "organ pipe" noise of open vents) and to
  see electric or jet engines (MOP, IGC 5.4). The band-passes are 6th
  order Butterworth (3 biquads each, designed at start-up): flat within
  the band, adjacent bands −16…−24 dB. Not verified against IEC 61260
  classes.
- Multirate filter bank: each octave of three bands runs at half the
  rate of the octave above, behind an 8th order Butterworth decimation
  low-pass (corner 0.205 × input rate, aliasing into the bands below
  −30 dB). Measured in QEMU (Cortex-M4, `-icount`): 464 instructions per
  sample, 3.7 M instructions/s, about 2 % of the CPU at one instruction
  per cycle (all bands at 8 kHz: 939 with `-O3`, 6345 with `-O0`).

The center frequency and gain are constructor parameters, as IGC 5.3.3
recommends adjustable values for GFAC tests. No AGC (IGC 5.3.2) yet.

## Microphone detection

After start-up the `MIC` task checks up to 8 records (about 2.75 s
including settling) for a signal, i.e. samples that are not constant.

- Signal found: the microphone counts as present until the next
  start-up. All records are logged; a microphone failing later shows up
  as ENL 000 with status bit 0 set, as IGC requires for an unserviceable
  ENL system.
- No signal: I2S, its DMA and PLLI2S are switched off, PA15/PB3/PB5 go
  back to analog (high impedance), the task suspends. No MICROPHONE_DATA
  records are logged and the web UI reports "no microphone detected".

`microphone_get_state()` reports detecting / present / absent.

## LRSX record MICROPHONE_DATA (ID 35, 27 words, 4 Hz)

Not yet in `doc_larus/documentation/LRSX_log_file_format.md`.

| Word | Type | Content |
|------|------|---------|
| 0    | uint32 | record counter, +1 per record since start (shows gaps) |
| 1    | uint32 | ENL 0..999, 1 s window |
| 2    | uint32 | status: bit 0 no signal, bits 16..31 lost DMA blocks (from 10 s after start, saturating) |
| 3    | float  | ENL filter output level, dBFS |
| 4    | float  | broadband level (DC removed), dBFS |
| 5    | float  | peak: largest raw sample, dBFS (0 = clipping) |
| 6..26| float  | third-octave bands, centers 1000 Hz × 2^(k/3), k = −16…4 (nominal 25, 31.5, 40 … 2000, 2500 Hz), dBFS |

Levels are 250 ms RMS values relative to a full-scale sine (0 dBFS =
sine with full-scale peak; for the ICS-43434, 94 dB SPL = −26 dBFS).
The record time is given by its position between the 100 Hz records.

## Open points

- Calibrate the gain in gliders with and without engines (IGC 1.4.2,
  5.3.1, 5.5).
- IGC 5.2 (immediate fix when ENL first exceeds 500) is for the logger
  writing the IGC file, not for the sensor. ENL is not yet sent on CAN or
  NMEA; both are interface changes that need a spec in `doc_larus`.
