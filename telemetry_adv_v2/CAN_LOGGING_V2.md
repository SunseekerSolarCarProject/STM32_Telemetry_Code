# Telemetry Advanced V2 CAN logging

## Coherent frame capture

The CAN receive interrupt first builds one complete local record containing:

- HAL receive tick and monotonic sequence number
- standard or extended identifier
- bxCAN timestamp and filter-match index
- IDE, RTR, and DLC
- all eight payload-byte positions
- cumulative software-queue overflow count

The completed record is then published to `can_latest` and to a 512-entry
single-producer/single-consumer ring buffer. Main-loop readers copy or pop a
whole record while interrupts are masked only for the short memory transfer.
No filesystem operation runs with interrupts disabled.

## SD-card files

`CANV2.CSV` is the compact per-frame bus log. The SD task drains as many as 64
queued frames per pass and synchronizes the open file at least once per second.
Sequence gaps or an increase in `queue_dropped_total` identify frames lost
because the software queue filled.

`TELV2.CSV` is the one-second combined GPS, IMU, RTC, speed, and newest-CAN
status file. Every CAN column in a row comes from one protected local snapshot.
This prevents a payload from one frame being written beside another frame's
identifier or DLC.

The ring buffer has 511 usable entries. If sustained bus traffic exceeds SD
write throughput, V2 reports the loss rather than silently relabeling data.

## Preserved board configuration

- CAN1: 250 kbit/s, prescaler 8, BS1 13 TQ, BS2 2 TQ, SJW 1 TQ
- I2C1: 100 kbit/s, 7-bit addressing
- SPI1: 4 MHz for BMI270/BME280
- SPI2: 2 MHz for GPS
- SPI3: 4 MHz for SD
- SPI4: 4 MHz for ESP32
- USART1: 115200 baud, 8 data bits, no parity, 1 stop bit
- CAN RX/TX interrupt priority: 0

`telemetry_adv_v2.ioc` matches the working V1 peripheral and pin configuration
except for the V2 project name.
