# Timestamps of the observations

(see #263) Every 100 Hz cycle the logger writes a `TIMESTAMPS` record
(`flexible_log_file_record_type` 16, layout `timestamps_t`, both in
`sw_algorithms_lib`) right after `BASIC_SENSOR_DATA`.

## Counter

`Drivers/Custom/timestamps.cpp`: TIM4 (16 bit) counts the 84 MHz APB1 timer
clock, TIM5 (32 bit) counts the TIM4 overflows in hardware (slave mode
"external clock", trigger ITR2 = TIM4 TRGO). Together a 48 bit counter with
11.9 ns resolution (`TIMESTAMP_TICKS_PER_SECOND`), wrapping after 38 days.
It needs no overflow interrupt, so it keeps counting while interrupts are
blocked, e.g. during FLASH programming.

## Fields

| Field | Source | Captured by |
|---|---|---|
| `IMU_sample` | data-ready edge of the MTi-1 sample in this cycle's `BASIC_SENSOR_DATA` | TIM4_CH1 input capture on PD12 (the same pin also raises the EXTI interrupt that wakes the IMU task) |
| `GNSS_reception` | end of the latest GNSS message (NAV-PVT, with D-GNSS also RELPOSNED) | USART3 RX DMA transfer-complete interrupt |
| `GNSS_timepulse` | latest GNSS time pulse edge, i.e. the start of a GNSS second | TIM4_CH3 input capture on PD14 - not connected on the current board (hw_sensor#31), so 0 |

The values are the low 32 bits of the counter. They wrap every 51 s; a
reader unwraps them with the help of the record sequence (one record per
10 ms).

A capture register holds only the low 16 bits of the counter. They are
extended with the high part when the capture is read, which has to happen
within one TIM4 period (780 us) after the edge - in the interrupt the edge
raises. If the IMU capture flag is not set when the data-ready interrupt
runs, the interrupt entry time is used instead.

## Using it

- IMU sample time: `IMU_sample`. The MTi-1 itself samples a little earlier
  (its internal filter and output delay); that offset is constant for a
  given configuration.
- GNSS: `GNSS_reception` minus the receiver's output latency gives an
  estimate of the measurement epoch in counter time, with the jitter of the
  receiver's output. With `GNSS_timepulse` connected, every GNSS second is
  known in counter time to within the capture resolution, and every
  observation can be expressed in GNSS time.
