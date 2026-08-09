# Telemetry ADV Board Debug Codes and Status LEDs

This reference matches the current `telemetry_adv_v1` firmware. Values from
different tables are not interchangeable. For example, BMI result `-2`, HAL
status `2`, and red-LED error mask `0x02` have different meanings.

## Quick debugging sequence

1. Confirm green LED 11 continues its heartbeat.
2. If red LED 10 is on, read the SWV `[ERROR FLAGS]` line.
3. Use the mask table below to identify the subsystem.
4. Find that subsystem's detailed code in the later tables.
5. Clear only historical flags with `CLEAR_ERRORS`; if the fault remains, the
   subsystem will set its bit again.

## LED electrical behavior

All status LEDs are active-low. The STM32 turns an LED on by driving its GPIO
low and sinking current. A low output is therefore intentional and does not
mean the output logic is inverted incorrectly.

| LED | Color | GPIO | Meaning | Normal behavior |
|---|---|---:|---|---|
| LED5 | Yellow | PD14 | Fresh checksum-valid GPS navigation traffic | On while an RMC, GGA, VTG, or GSV sentence has arrived within 3 seconds. This does not by itself guarantee a position fix. |
| LED6 | Yellow | PD13 | SD-card activity/status | Pulses for about 120 ms after a successful SD row write. It stays off when the card is absent, unmounted, logging is stopped, or a write fails. |
| LED7 | Yellow | PD12 | CAN receive activity | Pulses for about 120 ms after a CAN frame is received. Solid/frequent light can be normal on a busy CAN bus. |
| LED8 | Yellow | PD11 | RS-232 transmit activity | Pulses after the one-second RS-232 telemetry block completes. At a one-second interval, it should flash briefly once per second. |
| LED9 | Yellow | PD10 | ESP32 link success | Stays on while an ESP32 exchange has succeeded within the last 2 seconds. It normally appears solid because exchanges occur once per second. |
| LED10 | Red | PD9 | Combined error indicator | On whenever any `error_flags` bit is set. Use `[ERROR FLAGS]` to identify the cause. |
| LED11 | Green | PD8 | Heartbeat/calibration progress | Normally 1 second on and 1 second off. It blinks faster while Bosch IMU calibration is running. |

If green LED 11 stops permanently, suspect a hard fault, `Error_Handler()`, lost
power/clock, or firmware that is no longer reaching the main loop.

## Red LED 10 error mask

SWV prints the combined state in this form:

```text
[ERROR FLAGS] mask=0x00 SD=0 CAN=0 RS232=0 ESP32=0 IMU=0 ...
```

| Mask bit | Mask | Printed field | Meaning and first checks |
|---:|---:|---|---|
| 0 | `0x01` | `SD=1` | SD mount, initialization, synchronization, close, or write failed. Check card detect, FAT format, PC8 chip-select, SPI3 wiring, and SD supply. |
| 1 | `0x02` | `CAN=1` | CAN start, receive, FIFO overrun, bus-off, or other HAL CAN error. Check 250 kbit/s, termination, CANH/CANL, transceiver power, and ground. |
| 2 | `0x04` | `RS232=1` | USART1 interrupt transmit start or completion failed. Check UART state/error, 115200 8-N-1 configuration, transceiver power, and wiring. |
| 3 | `0x08` | `ESP32=1` | ESP32 READY/SPI exchange failed. Check SPI4, PE3 chip-select, PE4 READY, power, common ground, and matching 768-byte protocol. |
| 4 | `0x10` | `IMU=1` | BMI270 is absent, communication failed, sampling failed, calibration failed, or mount orientation is invalid. Check the BMI tables and IMU boot messages. |

Masks combine by addition/bitwise OR. Examples:

| Mask | Active errors |
|---:|---|
| `0x00` | None; red LED should be off |
| `0x03` | SD and CAN |
| `0x0C` | RS-232 and ESP32 |
| `0x10` | IMU only |
| `0x1F` | All five monitored subsystems |

`CLEAR_ERRORS` clears the current mask and selected counters, but it does not
repair hardware. Persistent faults set their bits again.

GPS does not currently have its own red-error bit; use LED5 and the GPS status
fields instead. The external PCF85263A RTC is enabled but does not currently
have a red-error bit; use the RTC status fields. The BME280 is compiled off.

## General STM32 HAL status values

These values appear as `HAL=`, function return values, or UART/CAN diagnostics.

| Value | Name | Meaning |
|---:|---|---|
| `0` | `HAL_OK` | Operation succeeded |
| `1` | `HAL_ERROR` | Peripheral or operation error |
| `2` | `HAL_BUSY` | Peripheral/HAL state machine is already busy |
| `3` | `HAL_TIMEOUT` | Operation did not finish before its timeout |

## BMI270 boot and status fields

A completely successful installed-board startup is:

```text
[IMU PREFLIGHT] valid=5/5 chip=0x24 expected=0x24 HAL=0 ...
[IMU INIT] stage=bmi270-init result=0 chip=0x24 load=0x01
[IMU INIT] stage=set-config result=0
[IMU INIT] stage=enable-accel-gyro result=0
[IMU BOOT] ready=1 calibrated=1 mount_valid=1 result=0
```

| Field | Good value | Meaning |
|---|---:|---|
| `chip` | `0x24` | Correct BMI270 chip ID |
| `load` | `0x01` | Bosch configuration loaded successfully |
| `ready` | `1` | Communication, configuration, and sensor enable completed |
| `calibrated` | `1` | Both stationary gravity windows and gyro FOC succeeded |
| `mount_valid` | `1` | Board was vertical with left/pin-1 edge facing forward during calibration |
| `VALID` | `1` | A current IMU sample was read successfully |
| `AGE_MS` | Small | Milliseconds since the most recent successful sample |

The mounting check expects gravity mainly on X: `abs(X) >= 800 mg`,
`abs(Y) <= 350 mg`, and `abs(Z) <= 350 mg`. The vehicle must also be still.

### Bosch BMI270 result codes

| Result | Name | Meaning / likely action |
|---:|---|---|
| `0` | `BMI2_OK` | Success |
| `-1` | `BMI2_E_NULL_PTR` | Driver received a null pointer; firmware/API error |
| `-2` | `BMI2_E_COM_FAIL` | SPI communication failed; check HAL, chip-select, wiring, power, and timeout |
| `-3` | `BMI2_E_DEV_NOT_FOUND` | Chip ID was not `0x24`; check the same SPI/power items |
| `-4` | `BMI2_E_OUT_OF_RANGE` | API parameter or requested value is outside its allowed range |
| `-5` | `BMI2_E_ACC_INVALID_CFG` | Invalid accelerometer configuration |
| `-6` | `BMI2_E_GYRO_INVALID_CFG` | Invalid gyro configuration |
| `-7` | `BMI2_E_ACC_GYR_INVALID_CFG` | Both accelerometer and gyro configurations are invalid |
| `-8` | `BMI2_E_INVALID_SENSOR` | Unsupported/invalid sensor type requested |
| `-9` | `BMI2_E_CONFIG_LOAD` | Bosch BMI270 configuration upload failed |
| `-10` | `BMI2_E_INVALID_PAGE` | Invalid feature/configuration page |
| `-11` | `BMI2_E_INVALID_FEAT_BIT` | Invalid feature-selection bit |
| `-12` | `BMI2_E_INVALID_INT_PIN` | Invalid interrupt pin selection |
| `-13` | `BMI2_E_SET_APS_FAIL` | Advanced power-save setting failed |
| `-14` | `BMI2_E_AUX_INVALID_CFG` | Invalid auxiliary sensor configuration |
| `-15` | `BMI2_E_AUX_BUSY` | Auxiliary interface is busy |
| `-16` | `BMI2_E_SELF_TEST_FAIL` | Accelerometer/self-test failure |
| `-17` | `BMI2_E_REMAP_ERROR` | Axis remapping configuration failed |
| `-18` | `BMI2_E_GYR_USER_GAIN_UPD_FAIL` | Gyro user-gain update failed |
| `-19` | `BMI2_E_SELF_TEST_NOT_DONE` | Self-test did not complete |
| `-20` | `BMI2_E_INVALID_INPUT` | Invalid API input |
| `-21` | `BMI2_E_INVALID_STATUS` | Device returned an invalid/unexpected status |
| `-22` | `BMI2_E_CRT_ERROR` | Component-retrim error |
| `-23` | `BMI2_E_ST_ALREADY_RUNNING` | Self-test/retrim is already running |
| `-24` | `BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT` | Retrim download-ready step failed/aborted |
| `-25` | `BMI2_E_DL_ERROR` | Configuration/retrim download error |
| `-26` | `BMI2_E_PRECON_ERROR` | Calibration prerequisite failed: vehicle moved or board mount orientation is wrong |
| `-27` | `BMI2_E_ABORT_ERROR` | Sensor operation abort failed |
| `-28` | `BMI2_E_GYRO_SELF_TEST_ERROR` | Gyro self-test reported an error |
| `-29` | `BMI2_E_GYRO_SELF_TEST_TIMEOUT` | Gyro self-test timed out |
| `-30` | `BMI2_E_WRITE_CYCLE_ONGOING` | Nonvolatile write cycle is still running |
| `-31` | `BMI2_E_WRITE_CYCLE_TIMEOUT` | Nonvolatile write cycle timed out |
| `-32` | `BMI2_E_ST_NOT_RUNING` | Requested self-test operation was not running |
| `-33` | `BMI2_E_DATA_RDY_INT_FAILED` | Data-ready interrupt validation failed |
| `-34` | `BMI2_E_INVALID_FOC_POSITION` | Fast-offset calibration position is invalid |

For `result=-2`, use the accompanying values:

- `HAL=0..3`: general HAL result from the table above.
- `HAL_ERR`: SPI error bitmask from the next table.
- `SR`: raw STM32 SPI status register, mainly useful with the reference manual.
- `stage`: `raw-chip-id`, `bmi270-init`, `set-config`,
  `enable-accel-gyro`, or `sample` identifies where the failure occurred.

### SPI `HAL_ERR` bitmask

| Bit | Mask | Name | Meaning |
|---:|---:|---|---|
| 0 | `0x01` | `HAL_SPI_ERROR_MODF` | SPI mode fault |
| 1 | `0x02` | `HAL_SPI_ERROR_CRC` | SPI CRC error |
| 2 | `0x04` | `HAL_SPI_ERROR_OVR` | Receive overrun |
| 3 | `0x08` | `HAL_SPI_ERROR_FRE` | Frame-format error |
| 4 | `0x10` | `HAL_SPI_ERROR_DMA` | DMA error |
| 5 | `0x20` | `HAL_SPI_ERROR_FLAG` | RXNE/TXE/BSY flag error |
| 6 | `0x40` | `HAL_SPI_ERROR_ABORT` | Abort procedure failed |
| 7 | `0x80` | `HAL_SPI_ERROR_INVALID_CALLBACK` | Invalid callback configuration |

## ESP32 link result codes

The `[ERROR FLAGS]` line includes `esp_result`:

| Result | Meaning | First checks |
|---:|---|---|
| `0` | SPI4 exchange succeeded | Normal |
| `-1` | ESP32 READY did not go high before timeout | ESP32 firmware, READY PE4, power, boot state |
| `-2` | 768-byte SPI4 transmit/receive returned a HAL error | SPI4 pins/configuration, PE3 chip-select, protocol length |
| `-3` | READY stayed high after chip-select/exchange | ESP32 transaction completion/handshake logic |

`esp_pass` and `esp_fail` are cumulative success/failure counters.

### App/ESP32 response strings

| Response message | Meaning |
|---|---|
| `LOG_STARTED` | SD logging enabled |
| `SD_NOT_READY` | `START_LOG` requested but the SD card is not mounted |
| `LOG_STOPPED` | SD logging disabled |
| `IMU_NOT_READY` | IMU initialization/communication is not ready |
| `IMU_CALIBRATED_AT_REST` | IMU recalibration succeeded |
| `IMU_NOT_STATIONARY` | Calibration detected movement or non-1-g magnitude |
| `IMU_WRONG_MOUNT_ORIENTATION` | Board was not vertical in the required vehicle orientation |
| `IMU_CALIBRATION_FAILED` | Other Bosch calibration error; inspect the numeric result |
| `IMU_DISABLED` | IMU feature switch is compiled off |
| `ERRORS_CLEARED` | Firmware error mask and selected counters were cleared |
| `BAD_RTC_FORMAT` | `SET_RTC` syntax was not `YYYY-MM-DD,hh:mm:ss` |
| `RTC_RANGE_INVALID` | Date/time values or calendar date are invalid |
| `RTC_SET_FAILED` | Active RTC could not be written; check PCF85263A/I2C1 |
| `RTC_SET_PCF85263A` | External PCF85263A set successfully |
| `RTC_SET_INTERNAL` | Internal STM32 RTC set successfully |
| `RTC_SET_INTERNAL_AND_PCF85263A` | Both RTC devices set successfully |
| `UNKNOWN_CMD` | Command verb is unsupported |
| `ESP_CMD_MISSING_CMD` | ESP command wrapper did not contain `cmd="..."` |

## CAN debugging

A healthy periodic line with no bus traffic can look like:

```text
[CAN STATUS] rx=0 state=2 hal_error=0x00000000 ESR=0x00000000
```

CAN state values are `0=RESET`, `1=READY`, `2=LISTENING`, `3=SLEEP_PENDING`,
`4=SLEEP_ACTIVE`, and `5=ERROR`. Normal receive operation is state `2`.

### CAN `hal_error` bitmask

| Mask | Name | Meaning |
|---:|---|---|
| `0x00000001` | `EWG` | Protocol error warning |
| `0x00000002` | `EPV` | Error-passive state |
| `0x00000004` | `BOF` | Bus off |
| `0x00000008` | `STF` | Stuff error |
| `0x00000010` | `FOR` | Form error |
| `0x00000020` | `ACK` | No acknowledgment; commonly no other active node, wrong bitrate, wiring, or termination |
| `0x00000040` | `BR` | Bit recessive error |
| `0x00000080` | `BD` | Bit dominant error |
| `0x00000100` | `CRC` | CAN CRC error |
| `0x00000200` | `RX_FOV0` | Receive FIFO0 overrun |
| `0x00000400` | `RX_FOV1` | Receive FIFO1 overrun |
| `0x00000800` | `TX_ALST0` | Mailbox 0 lost arbitration |
| `0x00001000` | `TX_TERR0` | Mailbox 0 transmit error |
| `0x00002000` | `TX_ALST1` | Mailbox 1 lost arbitration |
| `0x00004000` | `TX_TERR1` | Mailbox 1 transmit error |
| `0x00008000` | `TX_ALST2` | Mailbox 2 lost arbitration |
| `0x00010000` | `TX_TERR2` | Mailbox 2 transmit error |
| `0x00020000` | `TIMEOUT` | HAL CAN timeout |
| `0x00040000` | `NOT_INITIALIZED` | CAN peripheral not initialized |
| `0x00080000` | `NOT_READY` | CAN peripheral not ready |
| `0x00100000` | `NOT_STARTED` | CAN was not started |
| `0x00200000` | `PARAM` | Invalid HAL CAN parameter |
| `0x00400000` | `INVALID_CALLBACK` | Invalid HAL callback configuration |
| `0x00800000` | `INTERNAL` | Internal HAL CAN error |

CAN error masks can contain several bits simultaneously. `ESR` is the raw bxCAN
Error Status Register and is useful for checking warning/passive/bus-off state
and the last error code.

## RS-232/UART debugging

USART1 is configured for `115200 baud, 8 data bits, no parity, 1 stop bit`, with
no hardware flow control. A successful block produces `[RS232 TX START]` and a
short LED8 pulse when transmission completes.

UART state values commonly printed on failure are:

| Value | State |
|---:|---|
| `0x00` | Reset/not initialized |
| `0x20` | Ready |
| `0x21` | Busy transmitting |
| `0x22` | Busy receiving |
| `0x23` | Busy TX and RX |
| `0x24` | Busy |
| `0xA0` | Timeout |
| `0xE0` | Error |

UART `uart_error` bits are:

| Mask | Meaning |
|---:|---|
| `0x00` | No UART error |
| `0x01` | Parity error |
| `0x02` | Noise error |
| `0x04` | Framing error; check baud rate/levels |
| `0x08` | Receive overrun |
| `0x10` | DMA error |

## SD/FatFS codes

### Low-level disk status and results

`DSTATUS` bits: `0x01=STA_NOINIT`, `0x02=STA_NODISK`, and
`0x04=STA_PROTECT`. Multiple status bits can be set together.

| `DRESULT` | Name | Meaning |
|---:|---|---|
| `0` | `RES_OK` | Success |
| `1` | `RES_ERROR` | Read/write error |
| `2` | `RES_WRPRT` | Write protected |
| `3` | `RES_NOTRDY` | Card/drive not ready |
| `4` | `RES_PARERR` | Invalid parameter |

### FatFS `FRESULT`

| Value | Name | Meaning |
|---:|---|---|
| `0` | `FR_OK` | Success |
| `1` | `FR_DISK_ERR` | Low-level disk I/O hard error |
| `2` | `FR_INT_ERR` | FatFS internal assertion/error |
| `3` | `FR_NOT_READY` | Physical drive cannot operate |
| `4` | `FR_NO_FILE` | File not found |
| `5` | `FR_NO_PATH` | Path not found |
| `6` | `FR_INVALID_NAME` | Invalid path/name format |
| `7` | `FR_DENIED` | Access denied or directory full |
| `8` | `FR_EXIST` | Object already exists |
| `9` | `FR_INVALID_OBJECT` | Invalid file/directory object |
| `10` | `FR_WRITE_PROTECTED` | Media is write protected |
| `11` | `FR_INVALID_DRIVE` | Invalid logical drive number |
| `12` | `FR_NOT_ENABLED` | Volume has no work area/mount |
| `13` | `FR_NO_FILESYSTEM` | No valid FAT filesystem |
| `14` | `FR_MKFS_ABORTED` | Filesystem creation aborted |
| `15` | `FR_TIMEOUT` | FatFS access/grant timeout |
| `16` | `FR_LOCKED` | File-sharing policy rejected operation |
| `17` | `FR_NOT_ENOUGH_CORE` | Not enough memory for LFN work buffer |
| `18` | `FR_TOO_MANY_OPEN_FILES` | Open-file limit exceeded |
| `19` | `FR_INVALID_PARAMETER` | Invalid FatFS parameter |

## GPS, RTC, and telemetry status fields

These are status indicators rather than red-LED error bits:

| Field | Meaning |
|---|---|
| `GPS_VALID=1` | Fresh valid GPS speed is available |
| `FIX=1` | Fresh valid latitude/longitude fix is available |
| `ELEV_VALID=1` | Fresh GGA mean-sea-level elevation is available |
| `AGE_MS` / `ELEV_AGE_MS` | Age of the corresponding GPS solution |
| `SATS_VISIBLE` | GPS satellites reported in view by the latest GP/GN GSV sentence; useful while searching for a fix |
| `SATS_VISIBLE_VALID=1` | The visible-satellite count is no more than 5 seconds old |
| `SATS_VISIBLE_AGE_MS` | Age of the most recent visible-satellite count |
| `SATS_USED` | Satellites reported by GGA as participating in the navigation solution |
| `SATS_USED_VALID=1` | The satellites-used count is no more than 5 seconds old |
| `SATS_USED_AGE_MS` | Age of the most recent satellites-used count |
| `checksum_err` | Count of failed NMEA checksums; intermittent growth suggests SPI/noise/data corruption |
| `RTC_SOURCE=UNSYNCED` | External RTC has not supplied a valid calendar yet |
| `RTC_SOURCE=PRESERVED` | PCF85263A retained a valid calendar across reset/power loss |
| `RTC_SOURCE=GPS_UTC` | GPS RMC set or verified the external RTC |
| `RTC_SOURCE=APP` | ESP32/Bluetooth app set the RTC |
| `RTC_SYNC_VALID=1` | Most recent scheduled comparison with valid GPS UTC succeeded |
| `RTC_DRIFT_S` | External RTC minus GPS UTC at the most recent check; correction occurs outside +/-2 seconds |
| `IMU_G VALID=1` | Latest BMI270 sample is valid |
| `CALIBRATED=1` | Gravity baseline and gyro calibration succeeded |
| `MOUNT_VALID=1` | Calibration used the required installed orientation |
| `SOURCE=CAN_MC1/CAN_MC2` | Vehicle speed comes from fresh motor-controller CAN data |
| `SOURCE=GPS` | CAN speed is stale/unavailable and GPS speed is used |
| `SOURCE=IMU_INTEGRATED` | CAN/GPS are unavailable and short-term IMU speed is used |
| `SOURCE=NONE` | No valid vehicle-speed source is available |

## Fatal `Error_Handler()` behavior

If Cube/HAL initialization calls `Error_Handler()`, interrupts are disabled,
red LED 10 is forced on, green LED 11 is forced off, and execution stays in an
infinite loop. This differs from a normal red subsystem error where the green
heartbeat continues. Check clock configuration and the peripheral initializer
that ran immediately before the halt.
