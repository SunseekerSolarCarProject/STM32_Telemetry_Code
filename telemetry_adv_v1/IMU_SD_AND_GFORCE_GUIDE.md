# BMI270 Boot Calibration, SD Logging, and G-Force Processing

## Boot sequence

The BMI270 is enabled with a 100 Hz output data rate, a +/-4 g accelerometer
range, and a +/-2000 degrees/second gyroscope range. Normal firmware startup
does the following:

1. Waits for BMI270 power-up, switches it into SPI mode, and requires five
   consecutive `0x24` chip-ID reads using the SPI dummy byte.
2. Uploads the Bosch configuration over proven polling-mode 4 MHz SPI1
   transfers while holding the shared BME chip-select high.
3. Collects 32 samples over approximately 320 ms and rejects calibration if
   rotation is detected or acceleration magnitude is not close to 1 g.
4. Runs Bosch gyroscope fast-offset compensation. This collects 128 samples at
   25 Hz internally and takes approximately 6.4 seconds.
5. Collects another stationary window and saves the installed board's X/Y/Z
   gravity vector.
6. Starts normal 100 Hz sampling. A failed or disconnected IMU is retried every
   five seconds without requiring an STM32 reset.

The board must already be mounted vertically with its left/pin-1 edge facing
the front of the vehicle, and the vehicle must remain stationary for roughly
seven seconds after power-up. Calibration requires gravity primarily on sensor
X (at least 0.8 g), with no more than 0.35 g on forward Y or lateral Z. This
prevents accidentally calibrating the board while it is lying flat.
Green LED 11 blinks during this calibration so the board does not appear
stalled; it returns to the normal heartbeat pattern afterward.
If initialization or calibration fails, red LED 10 remains on through the IMU
error flag. The rest of the board still runs, and the SD fields show the IMU
status instead of silently treating bad data as calibrated.

The `CALIBRATE_IMU` ESP32/app command repeats the complete stationary
calibration. The final response is delayed by roughly seven seconds while the
calibration runs. A successful response is:

```text
$RSP,<sequence>,OK,IMU_CALIBRATED_AT_REST
```

Movement during the initial stationary check returns `IMU_NOT_STATIONARY`.
An incorrect board orientation returns `IMU_WRONG_MOUNT_ORIENTATION`.

## G-force calculations

All acceleration values use milli-g (`mg`), where 1000 mg equals 1 g.

- `imu_total_g_mg` is the magnitude of the measured accelerometer vector. A
  stationary board should report approximately 1000 mg because it includes
  gravity.
- `imu_linear_x_mg`, `imu_linear_y_mg`, and `imu_linear_z_mg` subtract the
  gravity vector measured during boot calibration.
- `imu_dynamic_g_mg` is the magnitude of the gravity-subtracted axes after a
  35 mg per-axis noise deadband. This estimates movement-related acceleration.
- `imu_peak_window_g_mg` is the highest dynamic value sampled between SD rows,
  so a brief event is retained even though the CSV is written once per second.
- `imu_peak_boot_g_mg` is the highest dynamic value since calibration.

For example, `imu_dynamic_g_mg=425` means approximately 0.425 g of dynamic
acceleration. Axis signs and meanings are board coordinates until the physical
mounting orientation is mapped to vehicle longitudinal, lateral, and vertical
directions.

Gravity subtraction uses the fixed vector captured while stationary. This is
appropriate for a rigidly mounted telemetry board and avoids learning steady
braking or cornering as gravity. Large chassis pitch/roll can still mix some
gravity into the dynamic axes; a future accelerometer/gyro attitude filter can
improve that behavior.

The +/-4 g setting matches the proven BMI270 test configuration and provides
headroom for vehicle events. An impact above 4 g on any one sensor axis clips
that axis; use a wider range if harsh-impact measurement becomes the priority.

## Short-term acceleration speed

The IMU speed is a short-term forward-speed estimate, not an odometer or a
replacement for CAN/GPS speed:

1. The configured forward acceleration axis is gravity-subtracted and passed
   through the 35 mg noise deadband.
2. That signed acceleration is integrated at the 100 Hz sample rate.
3. Every new CAN speed sample re-anchors the estimate. If CAN is unavailable,
   every new valid GPS speed sample re-anchors it instead.
4. With no new anchor and no measured forward acceleration, a gentle retention
   factor limits long-term drift without immediately erasing coasting speed.
5. CAN remains the first-choice vehicle speed, GPS the second, and the IMU
   estimate is used only while the other sources are unavailable.

For reference, a sustained 0.5 g forward acceleration for one second produces
an ideal speed increase of approximately 10.97 mph. Sensor bias, road pitch,
mounting flex, and missed samples make long-duration inertial speed and
distance inaccurate, which is why CAN/GPS correction remains essential.

For the specified installation, the board is vertical and the BMI270 pin-1/dot
side faces the front of the car. Bosch's top-view sensing diagram makes that
direction sensor +Y, so `Core/Src/main.c` uses:

```c
#define IMU_FORWARD_AXIS  IMU_AXIS_Y
#define IMU_FORWARD_SIGN  1L
```

This makes forward acceleration positive and braking negative. With the PCB
image's upper edge physically upward, the expected stationary readings are
approximately +1000 mg on X and 0 mg on Y/Z. The exact gravity vector is learned
during boot, but this is a useful installation check in `TELIMU2.CSV`. If X is
near -1000 mg instead, the board's vertical direction is inverted; the forward
Y mapping remains correct as long as the dot side still faces forward.

Sensor Z is normal to the PCB, so on a vertical wall it measures the car's
side-to-side direction; its sign depends on which wall and which way the
component side faces. An incorrect forward-axis setting makes braking look like
acceleration or integrates a lateral/vertical event into forward speed.

RS-232 reports `MOUNT_VALID=1` only after this orientation check and both
stationary calibration windows succeed. The ESP32 `$TEL` frame reports the same
state as `imu_mount_valid=1`.

## SD file

Expanded telemetry is written once per second to:

```text
0:/TELIMU2.CSV
```

The new filename prevents older `TELMSPD.CSV` or `TELIMU.CSV` rows from sharing
an incompatible header. The file includes existing RTC, CAN, GPS, and
vehicle-speed fields plus:

- IMU ready, calibrated, valid, sample time/age, counts, and read errors
- Short-term `imu_speed_mph` and signed `imu_forward_accel_mg`
- Raw accelerometer and gyroscope readings
- Acceleration converted to mg
- Angular rate converted to milli-degrees/second (`mdps`)
- Gravity-subtracted X/Y/Z acceleration
- Current total/dynamic g and peak dynamic g values

`imu_valid=1` means a sensor sample was read successfully. It does not replace
`imu_calibrated`; processing should require both fields when calibrated motion
data is needed.

## ESP32 status

The one-second `$TEL` frame now also contains:

```text
imu_ready=1,imu_calibrated=1,imu_valid=1,
imu_speed_mph=24.31,imu_forward_accel_mg=184,
imu_dynamic_g_mg=125,imu_peak_g_mg=438
```

This lets the app show calibration state and current/maximum movement g without
reading the SD card.

## RS-232 GPS elevation and g-force

The one-second `raw_data` block includes GPS elevation on the existing `NAV`
row. Elevation comes from checksum-valid GGA field 9 and is the receiver's
height above mean sea level in metres:

```text
NAV,...,ELEV_M=214.372,ELEV_VALID=1,ELEV_AGE_MS=84
```

`ELEV_VALID=1` means a valid-fix GGA elevation arrived within the last five
seconds. The value may remain printed after that, but consumers must ignore it
when `ELEV_VALID=0`.

The following row carries current processed BMI270 acceleration in units of g:

```text
IMU_G,VALID=1,CALIBRATED=1,FORWARD_G=0.184,LINEAR_X_G=0.021,LINEAR_Y_G=0.184,LINEAR_Z_G=-0.012,TOTAL_G=1.018,DYNAMIC_G=0.186,PEAK_BOOT_G=0.438,AGE_MS=4
```

- `FORWARD_G` is signed: positive is acceleration toward the pin-1/dot side
  (the front of the car), and negative is braking.
- `LINEAR_X_G`, `LINEAR_Y_G`, and `LINEAR_Z_G` have the boot-calibrated gravity
  vector removed.
- `TOTAL_G` is the accelerometer-vector magnitude including gravity, so it is
  normally close to 1 g while the car is stationary.
- `DYNAMIC_G` is the magnitude after gravity removal and the noise deadband, so
  it is normally close to 0 g while stationary.
- `PEAK_BOOT_G` is the greatest dynamic magnitude measured since calibration.
- Process the g-force fields only when both `VALID=1` and `CALIBRATED=1`.
