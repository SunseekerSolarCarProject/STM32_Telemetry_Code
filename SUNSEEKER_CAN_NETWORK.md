# Sunseeker CAN Network Reference

This file documents the CAN network format currently used by the
`Telemetry_Reference` STM32 firmware. It is based on the CAN constants and
decode paths in `Telemetry_Reference/Core/Src/main.c`.

## Current Firmware Role

The STM32 telemetry board is currently acting mostly as a CAN listener and
logger:

- It initializes `CAN1` in normal CAN mode.
- It accepts all CAN IDs into `CAN_RX_FIFO0`.
- It stores the latest received CAN frame for CSV logging.
- It updates the Sunseeker RS232 `raw_data` table when a known Sunseeker CAN ID
  arrives.
- It decodes motor-controller velocity packets for the selected vehicle speed.
- It can transmit a local test frame for bus bring-up.

The STM32 CAN pins are:

| Signal | STM32 pin | Function |
| --- | --- | --- |
| CAN RX | `PA11` | `CAN1_RX` |
| CAN TX | `PA12` | `CAN1_TX` |

The STM32 pins must connect through a CAN transceiver. Do not connect `PA11` and
`PA12` directly to `CANH` and `CANL`.

## Bus Setup

Current CAN timing in the firmware:

| Setting | Value |
| --- | --- |
| Peripheral | `CAN1` |
| Mode | Normal |
| Prescaler | `8` |
| Sync jump width | `1 tq` |
| Time segment 1 | `13 tq` |
| Time segment 2 | `2 tq` |
| Total time quanta | `16 tq` |
| APB1 clock in current `.ioc` | `32 MHz` |
| Resulting bitrate | `250 kbit/s` |

Physical network checklist:

- Use a CAN transceiver between the STM32 and the vehicle CAN bus.
- Connect `CANH` to `CANH`, `CANL` to `CANL`, and share signal ground.
- Use 120 ohm termination at the two physical ends of the bus only.
- With power off, a correctly terminated bus usually measures about 60 ohms
  between `CANH` and `CANL`.
- Keep stubs short and use twisted pair wiring for longer runs.
- If the STM32 is only listening, it still needs the same bitrate as the bus.

## Frame Format

The Sunseeker rows used by the firmware expect standard 11-bit CAN data frames
with 8 data bytes.

The payload is split into two little-endian 32-bit words:

| Bytes | Firmware name | Meaning in RS232 output |
| --- | --- | --- |
| `data[0..3]` | `low_word` | Printed second |
| `data[4..7]` | `high_word` | Printed first |

The legacy RS232 format prints rows as:

```text
NAME,0xHIGHWORD,0xLOWWORD
```

Example:

```text
MC1VEL,0x41200000,0x00000FA0
```

In memory that came from:

```text
data[0..3] = low word, little-endian
data[4..7] = high word, little-endian
```

## Motor Controller Messages

Motor controller 1 uses base ID `0x400`.
Motor controller 2 uses base ID `0x420`.

| Row name | CAN ID | Source | High word | Low word | Expected period |
| --- | ---: | --- | --- | --- | --- |
| `MC1LIM` | `0x401` | MC1 | CAN error / active motor | Error and limit flags | 200 ms |
| `MC1BUS` | `0x402` | MC1 | Bus current | Bus voltage | 200 ms |
| `MC1VEL` | `0x403` | MC1 | Velocity, m/s | Velocity, rpm | 200 ms |
| `MC1PHA` | `0x404` | MC1 | Phase C current | Phase B current | 200 ms |
| `MC1VVC` | `0x405` | MC1 | `Vd` vector | `Vq` vector | 200 ms |
| `MC1IVC` | `0x406` | MC1 | `Id` vector | `Iq` vector | 200 ms |
| `MC1BEM` | `0x407` | MC1 | `BEMFd` vector | `BEMFq` vector | 200 ms |
| `MC1TP1` | `0x40B` | MC1 | Heatsink/case temp | Motor internal temp | 1 s |
| `MC1TP2` | `0x40C` | MC1 | Reserved | DSP temp | 1 s |
| `MC1CUM` | `0x40E` | MC1 | DC bus amp-hours | Odometer, m | 1 s |
| `MC2LIM` | `0x421` | MC2 | CAN error / active motor | Error and limit flags | 200 ms |
| `MC2BUS` | `0x422` | MC2 | Bus current | Bus voltage | 200 ms |
| `MC2VEL` | `0x423` | MC2 | Velocity, m/s | Velocity, rpm | 200 ms |
| `MC2PHA` | `0x424` | MC2 | Phase C current | Phase B current | 200 ms |
| `MC2VVC` | `0x425` | MC2 | `Vd` vector | `Vq` vector | 200 ms |
| `MC2IVC` | `0x426` | MC2 | `Id` vector | `Iq` vector | 200 ms |
| `MC2BEM` | `0x427` | MC2 | `BEMFd` vector | `BEMFq` vector | 200 ms |
| `MC2TP1` | `0x42B` | MC2 | Heatsink/case temp | Motor internal temp | 1 s |
| `MC2TP2` | `0x42C` | MC2 | Reserved | DSP temp | 1 s |
| `MC2CUM` | `0x42E` | MC2 | DC bus amp-hours | Odometer, m | 1 s |

The firmware also defines these motor-controller offsets, but they are not in
the current RS232 Sunseeker row table:

| Offset | MC1 ID | MC2 ID | High word | Low word | Expected period |
| ---: | ---: | ---: | --- | --- | --- |
| `0x08` | `0x408` | `0x428` | 15 V rail | Reserved | 1 s |
| `0x09` | `0x409` | `0x429` | 3.3 V rail | 1.9 V rail | 1 s |
| `0x17` | `0x417` | `0x437` | Slip speed, Hz | Reserved | 200 ms |

## Driver Controls Messages

Driver controls use base ID `0x500`.

| Row name | CAN ID | High word | Low word | Expected period |
| --- | ---: | --- | --- | --- |
| `DC_DRV` | `0x501` | Motor current setpoint | Motor velocity setpoint | 100 ms |
| `DC_SWC` | `0x504` | Switch position | Switch state change | 100 ms |

The firmware also defines these driver-control offsets:

| Offset | CAN ID | High word | Low word |
| ---: | ---: | --- | --- |
| `0x02` | `0x502` | Bus current setpoint | Unused |
| `0x03` | `0x503` | Unused | Unused |

## Steering Wheel Messages

The steering wheel base ID is `0x540`.

The firmware defines:

| Name | CAN ID | High word | Low word | Expected period |
| --- | ---: | --- | --- | --- |
| `STW_SWITCH` | `0x541` | Switch position | Switch state change | 100 ms |

Known switch bit masks:

| Bit mask | Name |
| ---: | --- |
| `0x0001` | Horn |
| `0x0002` | Left indicator |
| `0x0004` | Right indicator |
| `0x0008` | Regen |
| `0x0010` | Cruise |

`STW_SWITCH` is defined in the firmware but is not currently included in the
RS232 Sunseeker row table.

## Battery Protection Messages

Battery protection uses base ID `0x580`.

| Row name | CAN ID | High word | Low word | Expected period |
| --- | ---: | --- | --- | --- |
| `BP_VMX` | `0x581` | Max voltage value | Max voltage cell number | 10 s |
| `BP_VMN` | `0x582` | Min voltage value | Min voltage cell number | 10 s |
| `BP_TMX` | `0x583` | Max temperature | Max temperature cell | 10 s |
| `BP_ISH` | `0x585` | Shunt current | Battery SOC | 1 s |
| `BP_PVS` | `0x586` | Pack voltage | Shunt sum | 1 s |

The firmware also defines:

| Offset | CAN ID | High word | Low word | Expected period |
| ---: | ---: | --- | --- | --- |
| `0x04` | `0x584` | `"BPV2"` or `"0000"` string | CAN serial number | When ready |
| `0x07` | `0x587` | Unused | Unused | Not specified |

## MPPT Messages

The firmware defines MPPT-related CAN constants, but the current telemetry code
does not yet decode MPPT messages into the RS232 Sunseeker row table.

| Name | Value | Notes |
| --- | ---: | --- |
| `MPPT_CAN_BASE` | `0x600` | Base address for MPPT RTR requests |
| `MPPT_CAN_ONOFF` | `0x10` | Offset for MPPT on/off messages |
| `MPPT_CAN_ADDRESS1` | `0x00` | MPPT 1 address |
| `MPPT_CAN_ADDRESS2` | `0x01` | MPPT 2 address |

## Vehicle Speed From CAN

The firmware uses CAN speed before GPS or IMU speed when a fresh motor velocity
message is available.

Current speed decode path:

1. Receive `MC1VEL` (`0x403`) or `MC2VEL` (`0x423`).
2. Read bytes `4..7` as the high 32-bit word.
3. Interpret that high word as an IEEE-754 floating point value in meters per
   second.
4. Convert meters per second to miles per hour.
5. Reject the value if it is not finite or above the configured maximum.
6. Mark the speed source as `CAN_MC1` or `CAN_MC2`.

Important assumption:

The current firmware assumes the velocity high word is a float. If the motor
controller actually sends scaled integers, then `CAN_U32ToFloat()` and
`VehicleSpeed_UpdateFromCANVelocity()` need to be changed.

## RS232 Sunseeker Output

The RS232 output sends the known CAN rows once per second in this format:

```text
raw_data
ABCDEF
MC1BUS,0xHHHHHHHH,0xHHHHHHHH
MC1VEL,0xHHHHHHHH,0xHHHHHHHH
...
BP_PVS,0xHHHHHHHH,0xHHHHHHHH
MC1LIM,0xHHHHHHHH,0xHHHHHHHH
MC2LIM,0xHHHHHHHH,0xHHHHHHHH
BME,T=23.65,P=98110.73,H=52.43
NAV,IMU_MPH=0.00,GPS_MPH=0.00,GPS_VALID=0,VEHICLE_MPH=0.00,SOURCE=NONE,LAT=0.000000,LON=0.000000,FIX=0,AGE_MS=0
TL_TIM,2004-01-10T00:13:54_INVALID
VWXYZ
```

`0xHHHHHHHH` means that the STM32 has not received a valid 8-byte frame for that
row yet.

## CSV Logging Fields

The SD card and ESP32/Bluetooth CSV output keep a latest CAN snapshot in these
fields:

| Field | Meaning |
| --- | --- |
| `can_rx_count` | Count of frames received by the STM32 |
| `can_id` | Latest received CAN ID |
| `can_ext` | `0` for standard ID, `1` for extended ID |
| `can_dlc` | Latest received frame length |
| `can_data` | Latest received data bytes as hex text |
| `can_speed_mph` | Decoded CAN vehicle speed |
| `can_speed_valid` | Whether decoded CAN speed is currently fresh |
| `can_speed_source` | `CAN_MC1`, `CAN_MC2`, or `NONE` |

## Bring-Up Debugging

Useful firmware prints:

```text
CAN1 listen-all initialized.
CAN1: listen-all mode ready.
CAN RX #12 | ID=0x403 | STD | DLC=8 | DATA=...
CAN DBG: state=... hal_err=... ESR=... TEC=... REC=... LEC=...
```

The `SD_Card_test_nd_Can_test` firmware has a more detailed debug line:

```text
[CAN] DBG t=61045 ready=0 state=2 bitrate=250000 rx=0 delta=0 fifo0=0 irq=0 txq=16 txdone=0 txabort=0 txfree=3 txdone_tick=0 PA11=1 err=0x00001084 ESR=0x00F80007 TEC=248 REC=0 LEC=0 MSR=0x00000C08 TSR=0x1C000000 RF0R=0x00000000 IER=0x00008C03 BTR=0x001C0007
```

That pattern means the CAN controller is not on a healthy bus:

| Field | What it means |
| --- | --- |
| `ready=0` | The test firmware marked CAN not ready after an error. |
| `bitrate=250000` | Firmware is configured for 250 kbit/s. |
| `rx=0` / `delta=0` | No frames have been received. |
| `txq=16` | The firmware queued 16 test transmissions. |
| `txdone=0` | None of the queued test frames completed successfully. |
| `txfree=3` | All three transmit mailboxes are free now; this does not mean the frames were acknowledged. |
| `PA11=1` | RX pin is recessive/high at the sampled instant. That is normal for an idle CAN bus. |
| `err=0x00001084` | HAL saw bus-off, bit-dominant error, and mailbox 0 transmit error. |
| `ESR=0x00F80007` | Error warning, error passive, and bus-off are set. |
| `TEC=248` | Transmit error counter is very high, near the bus-off threshold. |
| `REC=0` | Receive error counter is zero because the board is not seeing receive traffic. |

Most likely causes for this exact pattern:

- No other active CAN node is connected to acknowledge transmitted frames.
- CANH/CANL are swapped.
- The transceiver is missing, unpowered, in standby, or wired incorrectly.
- The bus bitrate is not actually 250 kbit/s.
- The bus does not have correct termination.
- Ground is not shared between the STM32/transceiver and the rest of the CAN
  network.

CAN transmit requires another active node at the same bitrate to send the ACK
bit. If the STM32 is alone on the bus, it can queue a frame, but it will not
complete successfully and the transmit error counter will climb.

For receive-only bring-up, disable periodic test transmission until the bus is
known good:

```c
#define CAN_TX_TEST_PERIODIC_ENABLE 0U
```

Then confirm that `rx` increases when the vehicle network is active. After the
physical bus is fixed, reset/restart the CAN peripheral or reboot the board
because the test firmware disables `CanReady` after bus-off. Automatic bus-off
recovery is currently disabled by `AutoBusOff = DISABLE`.

What to check if no CAN frames appear:

- Confirm the bus bitrate is `250 kbit/s`.
- Confirm the CAN transceiver has power and ground.
- Confirm `CANH` and `CANL` are not swapped.
- Confirm there are exactly two 120 ohm bus terminations.
- Confirm the STM32 and vehicle bus share a signal ground.
- Confirm the CAN bus is active; a listener will not receive anything if no
  other node is transmitting.
- Check `TEC`, `REC`, and `LEC` in the debug print for bus errors.

## Firmware Locations

Main code locations:

| Purpose | File / function |
| --- | --- |
| CAN constants | `Telemetry_Reference/Core/Src/main.c` |
| CAN initialization | `MX_CAN1_Init()` |
| Listen-all filter setup | `STM32_CAN_ListenAllInit()` |
| RX interrupt callback | `HAL_CAN_RxFifo0MsgPendingCallback()` |
| RX polling and storage | `STM32_CAN_PollRx()`, `STM32_CAN_StoreRxFrame()` |
| Sunseeker row update | `SunRaw_UpdateFromCAN()` |
| RS232 block build | `SunRaw_BuildBlock()` |
| CAN speed decode | `VehicleSpeed_UpdateFromCANVelocity()` |
