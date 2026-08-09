# External RTC Time Synchronization Through GPS and ESP32 BLE

## Purpose

This design uses the external PCF85263A as the telemetry board's primary RTC.
GPS time comes from a checksum-valid, active NMEA RMC sentence and is used to
set and periodically verify the external calendar. App time travels through
the ESP32 BLE/SPI bridge and can be used when GPS is unavailable.

## Important behavior

- The PCF85263A retains time across main-board power loss only when its backup
  supply and 32.768 kHz crystal are present and working.
- The first valid GPS RMC after startup automatically sets the RTC. The app can
  send `SET_RTC` when a GPS fix is unavailable or a manual override is wanted.
- GPS checks the RTC once per hour. A write occurs only when the clock is
  unreadable or differs from GPS by more than two seconds.
- Use UTC for app commands because GPS time is UTC and the command contains no
  time-zone field.

## GPS synchronization behavior

The STM32 accepts time only from an RMC sentence that passes its NMEA checksum,
has status `A` (active), and contains valid `hhmmss` and `ddmmyy` fields.

- The first accepted RMC after boot sets the PCF85263A immediately.
- Further valid RMC sentences trigger a comparison every 3,600,000 ms (one
  hour). Drift of -2 through +2 seconds is accepted without rewriting the RTC.
- GPS provides whole UTC seconds. The fractional field is ignored because an
  NMEA sentence arrives after the precise second boundary. A future GPS PPS
  interrupt can provide sub-second alignment if needed.
- A successful app `SET_RTC` applies immediately. If GPS has already supplied
  time, the next GPS correction is postponed for one hour. If GPS has not yet
  supplied time, the first later fix becomes authoritative UTC.

## BLE command

Write this ASCII command to the ESP32 control characteristic:

```text
SET_RTC,YYYY-MM-DD,HH:MM:SS
```

Example:

```text
SET_RTC,2026-07-12,18:45:30
```

The desktop app accepts `RTC,now` (and the older `SET_RTC,now` spelling) in
its command box. `now` is expanded by the app before BLE transmission, so the
firmware receives the fully specified `SET_RTC` command above. A bare `now`
must not be forwarded to the boards because neither offline MCU can infer the
phone's wall-clock time.

Accepted range:

- Years 2000 through 2099
- Months 1 through 12
- Valid day for the selected month, including leap-year February
- Hours 0 through 23
- Minutes and seconds 0 through 59

The format must contain all leading fields and no trailing characters.

## End-to-end message flow

```text
Phone application
    -> BLE: SET_RTC,2026-07-12,18:45:30

ESP32
    -> BLE immediate response: ACK,SET_RTC,queued=1
    -> SPI MISO command:
       ESP_CMD,seq=<n>,cmd="SET_RTC,2026-07-12,18:45:30",...

STM32
    -> validates the full calendar
    -> writes the external PCF85263A
    -> reads the RTC back
    -> queues one of:
       $RSP,<n>,OK,RTC_SET_PCF85263A
       $RSP,<n>,ERR,<reason>

ESP32
    -> forwards the $RSP line to the BLE control response characteristic
```

The first BLE acknowledgement only means the ESP32 queued the command. The
`$RSP` message is the authoritative result from the STM32. Because SPI runs
once per second, the final result can take approximately two seconds.

## STM32 telemetry confirmation

Normal STM32-to-ESP32 status frames are fixed 768-byte, zero-padded ASCII
frames. A typical frame begins:

```text
$TEL,seq=42,ms=73117,rtc=2026-07-12T18:45:31,rtc_valid=1,rtc_source=GPS_UTC,...,rtc_sync_valid=1,rtc_drift_s=0,...
```

RS232/SWV also reports the active external RTC:

```text
TL_TIM,2026-07-12T18:45:31,RTC_SOURCE=GPS_UTC,RTC_SYNC_VALID=1,RTC_DRIFT_S=0,UPTIME_MS=73117
```

`RTC_SOURCE`/`rtc_source` is `UNSYNCED`, `APP`, `GPS_UTC`, or `PRESERVED` when
the PCF85263A retained a valid calendar. `RTC_SYNC_VALID=1` means the most
recent scheduled comparison with GPS succeeded. `RTC_DRIFT_S` is external RTC
time minus GPS time, in whole seconds, at that comparison. SWV also reports
the number of GPS checks and actual RTC writes:

```text
[RTC] 2026-07-12T18:45:31 device=PCF85263A time_source=GPS_UTC gps_sync_valid=1 drift_s=0 gps_checks=1 gps_sets=1 uptime=0:00:01:13.117
```

The STM32 SWV console reports a successful set/readback as:

```text
[ESP32 RTC] SET_RTC applied and read back: 2026-07-12T18:45:30
```

## SPI link contract

- STM32 SPI4 is master; ESP32-C3 SPI2 is slave.
- SPI mode 0: idle-low clock, sample on the first edge.
- Frame length is exactly 768 bytes in both directions.
- Frames contain printable ASCII followed by zero padding.
- ESP32 READY goes high only after the slave transaction is queued.
- STM32 asserts CS only while READY is high.
- STM32 currently initiates one transaction every 1000 ms.
- ESP32 repeats commands for reliability; STM32 deduplicates identical command
  IDs so the RTC is written once.

Both projects must use the same frame size. Changing only one side will break
the link.

## External PCF85263A configuration

The active RTC driver is written specifically for the PCF85263A. It
uses the 7-bit I2C address `0x51`, selects RTC/24-hour mode, and follows the
device's required coherent set sequence: STOP, clear prescaler, write registers
`0x00` through `0x07`, then release STOP.

The current STM32 `Core/Src/main.c` selection is:

```c
#define ENABLE_EXTERNAL_PCF85263A_RTC 1U
#define ENABLE_STM32_INTERNAL_RTC 0U
```

## Application implementation recommendation

The application should generate the command from UTC. For Python:

```python
now = datetime.now(timezone.utc).strftime("%Y-%m-%d,%H:%M:%S")
self._send_command(f"SET_RTC,{now}")
```

Using local wall-clock time here would disagree with GPS UTC and cause the
calendar to jump when the next GPS correction occurs.

On each BLE connection after board power-up:

1. Obtain the phone's current UTC calendar time.
2. Format it exactly as `SET_RTC,YYYY-MM-DD,HH:MM:SS`.
3. Write it to the control characteristic.
4. Treat `ACK,SET_RTC,queued=1` as pending, not complete.
5. Wait for `$RSP,...,OK,RTC_SET_PCF85263A`.
6. Optionally read status and confirm `rtc=` or monitor `TL_TIM`.

Do not restore an old timestamp from ESP32 NVS after power-up. Without an
independent running clock, a stored timestamp becomes stale while power is off.

## Error responses

Possible responses include:

```text
ERR,BAD_RTC
$RSP,<n>,ERR,BAD_RTC_FORMAT
$RSP,<n>,ERR,RTC_RANGE_INVALID
$RSP,<n>,ERR,RTC_SET_FAILED
```

`RTC_SET_FAILED` means the active RTC could not be written. Check I2C1, address
`0x51`, device power, common ground, and the PCF85263A oscillator components.

## Build commands and outputs

STM32 firmware output:

```text
STM32CubeIDE/Debug/telemetry_adv_v1.elf
```

ESP32 PlatformIO Windows environment:

```powershell
platformio run -e esp32-c3-devkitm-1-windows
```

ESP32 firmware output:

```text
.pio/build/esp32-c3-devkitm-1-windows/firmware.bin
```

Flash both updated firmwares. Flashing only one side leaves the old 32-byte
binary STM32 protocol or the new 768-byte ESP32 ASCII protocol unmatched.

## Bench verification checklist

1. Power both boards and connect the BLE app.
2. Confirm ESP32 READY and STM32 yellow LED 9 show SPI activity.
3. Send a known time with `SET_RTC`.
4. Confirm immediate `queued=1` and later `$RSP,...,OK,...` responses.
5. Confirm SWV `SET_RTC applied and read back` output.
6. Confirm `TL_TIM` advances once per second and reports `RTC_SOURCE=APP`.
7. Allow a valid GPS RMC and confirm `[GPS RTC] ... UTC` plus
   `RTC_SOURCE=GPS_UTC` after the initial GPS synchronization.
8. Leave GPS running until a later scheduled comparison and confirm
   `RTC_SYNC_VALID=1`; `RTC_DRIFT_S` should be within +/-2 unless a correction
   was required, in which case the RTC is rewritten and drift reports zero.
9. Remove main-board power while maintaining the RTC backup supply, restore
   power, and confirm `RTC_SOURCE=PRESERVED` before the first GPS update.
