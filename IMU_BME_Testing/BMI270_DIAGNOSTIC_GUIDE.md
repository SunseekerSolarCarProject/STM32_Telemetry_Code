# BMI270 SPI diagnostic

This firmware tests the BMI270 in increasing levels of confidence. Output is
sent through SWV ITM stimulus port 0.

## What the test proves

1. `PREFLIGHT` reads `CHIP_ID` five times before changing the sensor. All five
   reads must return `0x24`.
2. `bmi270-init` performs the Bosch soft reset and configuration-file upload.
   A successful upload reports `result=0` and `load_status=0x01`.
3. `set-config` selects 100 Hz, +/-4 g acceleration and 100 Hz, +/-2000 dps
   angular rate. `enable-accel-gyro` then powers both sensing blocks.
4. `WRITE TEST` saves register `ACC_RANGE` (`0x41`), writes a different valid
   range, reads it back, restores the original value, and verifies the restore.
   `PASS` proves that MOSI, chip select, and register writes work while the IMU
   is active. The temporary setting is immediately restored.
5. `SAMPLE` prints the status, sensor time, raw axes, acceleration in mg, and
   angular rate in mdps every 250 ms.

## Important output fields

- `HAL`: `0=OK`, `1=ERROR`, `2=BUSY`, `3=TIMEOUT`.
- `phase`: `1` means the register-address transfer failed; `2` means the data
  phase failed.
- `HAL_ERR`: STM32 HAL SPI error bits.
- `SPI_SR`: the SPI1 status register captured at the end of the transaction.
- `op` and `reg`: the final read/write operation and BMI270 register involved.
- `INTERNAL[21]=01`: the BMI270 configuration file loaded successfully.
- `STATUS[03]`: bits `0x80` and `0x40` are accelerometer and gyro data-ready.
- `TIME`: the BMI270 sensor-time counter. It must continue changing even when
  the board is motionless. `time_stuck` should normally remain zero.

At rest, one accelerometer axis should be near +1000 mg or -1000 mg depending
on board orientation, the other axes should be near zero, and gyro values
should be near zero with normal noise. Moving or rotating the board should
produce obvious changes.

## How to distinguish common failures

- `HAL=0` with `CHIP_ID=00` or `FF`: the STM32 generated clocks, but valid MISO
  data did not return. Check BMI270 power, PA6/MISO, and PA3/chip select.
- `HAL=3`: the STM32 SPI operation timed out. Inspect `phase`, `HAL_ERR`, and
  `SPI_SR` and verify the SPI peripheral state.
- Correct `CHIP_ID`, but failure during `bmi270-init`: inspect the reported
  `failed_stage`, `reg`, `op`, and `load_status`. A failure around registers
  `0x5B`, `0x5C`, or `0x5E` points to the configuration upload.
- Initialization succeeds but `WRITE TEST` fails: reads may work while MOSI or
  write timing does not.
- Register tests pass but `TIME` is stuck or raw axes never respond to motion:
  inspect `STATUS`, `ACC[40:41]`, `GYR[42:43]`, and `PWR[7C:7D]`.

This diagnostic uses blocking/polling HAL SPI calls. It does not require
`SPI1_IRQHandler`; that interrupt matters only for interrupt-driven HAL calls
such as `HAL_SPI_TransmitReceive_IT`.

The BMI270 Bosch sources (`bmi2.c` and `bmi270.c`) must be included in the
STM32CubeIDE build. If they were copied into the project while it was open,
refresh the project and perform a clean build so CubeIDE regenerates its source
list.
