# STM32 Telemetry Firmware Code

This repository contains the main Sunseeker STM32 telemetry firmware plus
small hardware-test projects. Every firmware directory is a self-contained
STM32CubeMX/CubeIDE project.

## Quick start: GitHub to STM32CubeIDE

### 1. Get the code

On this repository's GitHub page, select **Code > Download ZIP**. Extract the
entire ZIP to a normal working directory before opening STM32CubeIDE.

Because this is a public repository, no GitHub account, username, or sign-in is
required. If you use Git, select **Code**, copy the HTTPS URL, and run:

```powershell
git clone <repository-url>
```

Do not move an individual project or open files directly from inside the ZIP.
Whether you download or clone the repository, keep its directory structure
intact because each CubeIDE project links to source files one directory above
its `STM32CubeIDE` folder.

### 2. Choose a project

| Project directory | Target | Purpose |
| --- | --- | --- |
| `telemetry_adv_v2` | STM32F437VG/VI | Newest integrated telemetry revision; the normal starting point for current F437 hardware |
| `telemetry_adv_v1` | STM32F437VG/VI | Previous integrated telemetry revision |
| `Telemetry_Reference` | STM32F429ZI | Full reference implementation for the older F429 target |
| `Blink_LED` | STM32F437VG/VI | LED bring-up test |
| `esp32_testing` | STM32F437VG/VI | ESP32/SPI test |
| `gps_testing` | STM32F437VG/VI | GPS test |
| `IMU_BME_Testing` | STM32F437VG/VI | BMI270 and BME sensor test |
| `RS232_testing` | STM32F437VG/VI | RS232 test |
| `RTC_testing` | STM32F437VG/VI | Real-time-clock test |
| `SD_Card_test_nd_Can_test` | STM32F437VG/VI | SD card and CAN test |

The MCU target matters: do not flash the F429 `Telemetry_Reference` build to
the F437 board.

### 3. Import it into STM32CubeIDE

1. Install STM32CubeIDE and open it with any workspace directory **outside this
   repository**.
2. Select **File > Import...**.
3. Select **General > Existing Projects into Workspace**, then select **Next**.
4. For **Select root directory**, browse to the chosen project's nested
   `STM32CubeIDE` directory. For example:
   `STM32_Telemetry_Code/telemetry_adv_v2/STM32CubeIDE`.
5. Make sure the project is checked. Leave **Copy projects into workspace**
   unchecked, then select **Finish**.

Do not import the repository root or only the `.ioc` file. CubeIDE detects a
project by the hidden `.project` and `.cproject` files inside each
`STM32CubeIDE` directory.

### 4. Build and flash

1. Right-click the imported project and select **Clean Project**.
2. Select **Project > Build Project** (or press `Ctrl+B`). CubeIDE recreates
   the ignored `Debug` build directory locally.
3. Connect the correct target through ST-LINK and supply target power.
4. Select **Run > Debug As > STM32 C/C++ Application** to program and debug,
   or **Run As > STM32 C/C++ Application** to program and run.

If CubeIDE asks to migrate the project, allow it, then clean and rebuild. If
linked files appear missing, remove the imported project from the workspace
without deleting its contents and repeat the import from the nested
`STM32CubeIDE` directory.

### Editing the hardware configuration

The `.ioc` file is stored at the top of each project directory, one level above
`STM32CubeIDE`. It appears as a linked file after a correct import. Open it in
CubeIDE to change pins or peripherals and regenerate code. Keep application
changes inside CubeMX `USER CODE BEGIN` / `USER CODE END` sections so code
generation does not overwrite them.

## Repository layout

```text
STM32_Telemetry_Code/
|-- README.md
|-- SUNSEEKER_CAN_NETWORK.md
|-- telemetry_adv_v2/
|   |-- telemetry_adv_v2.ioc       # CubeMX hardware configuration
|   |-- Core/                      # Application source and headers
|   |-- Drivers/                   # STM32 HAL and CMSIS
|   |-- FATFS/ and Middlewares/    # Filesystem support
|   `-- STM32CubeIDE/              # Import this directory into CubeIDE
|       |-- .project and .cproject # CubeIDE project metadata
|       `-- Debug/                 # Local generated output; not committed
|-- telemetry_adv_v1/
|-- Telemetry_Reference/
`-- *_testing/                     # Focused hardware-test projects
```

Generated `Debug` and `Release` directories are intentionally excluded from
Git. A fresh GitHub download contains the source, libraries, linker scripts,
launch configuration, `.ioc`, and CubeIDE metadata needed to rebuild them.

## Firmware overview

This workspace contains different versions of telemetry firmware for the STM32.
The `Telemetry_Reference` project is the full reference firmware and is meant to
work with the SD card, real time clock, BME environmental sensor, BMI270 IMU,
ESP32-C3, GPS, CAN bus, RS232 output, and status LEDs.

The STM32 sends telemetry out in two main ways:

1. To the ESP32-C3 over SPI, so the ESP32 can forward data to the Bluetooth app.
2. To the RS232 port as a readable raw telemetry block for external displays,
   logging tools, or debugging.

See [`SUNSEEKER_CAN_NETWORK.md`](SUNSEEKER_CAN_NETWORK.md) for the CAN network
IDs, payload layout, speed decode notes, and Sunseeker RS232 row mapping.

## ESP32 / Bluetooth Telemetry

The ESP32-C3 is connected to the STM32 over SPI. The STM32 sends framed text
messages to the ESP32 when the ESP32 ready signal is active. The ESP32 side can
then forward those messages to the Bluetooth application.

The STM32 sends three main frame types:

```text
$HDR,<csv header>
$LOG,<sequence number>,<csv telemetry row>
$RSP,<command id>,<OK or ERR>,<message>
```

### Header Frame

The header frame tells the ESP32/app what fields are present in each telemetry
row. It is sent before the normal log stream when a header is pending.

```text
$HDR,time_ms,adalogger_rtc,adalogger_rtc_valid,acc_x_mg,...
```

### Log Frame

The log frame is the normal telemetry data stream. It uses the same CSV row
format that is written to the SD card.

```text
$LOG,123,time_ms,adalogger_rtc,adalogger_rtc_valid,...
```

Current CSV fields include:

```text
time_ms
adalogger_rtc
adalogger_rtc_valid
acc_x_mg, acc_y_mg, acc_z_mg
gyr_x_mdps, gyr_y_mdps, gyr_z_mdps
imu_speed_mph
gps_speed_mph
gps_speed_valid
can_speed_mph
can_speed_valid
can_speed_source
gps_lat_deg
gps_lon_deg
gps_fix_valid
gps_fix_age_ms
vehicle_speed_mph
vehicle_speed_source
bme_temp_c
bme_pressure_pa
bme_humidity_pct
can_rx_count
can_id
can_ext
can_dlc
can_data
```

### Response Frame

The response frame is sent when the ESP32 sends a command to the STM32. Response
frames are sent before normal log frames so the app can quickly see whether a
command succeeded or failed.

```text
$RSP,42,OK,LOG_STARTED
$RSP,43,ERR,SD_NOT_READY
```

Supported ESP32 commands include:

```text
CMD,<id>,START_LOG
CMD,<id>,STOP_LOG
CMD,<id>,SET_RTC,YYYY-MM-DD,HH:MM:SS
```

The firmware also accepts direct command text such as `START_LOG`, `STOP_LOG`,
and `SET_RTC,...`.

## RS232 Telemetry

The RS232 output is a human-readable telemetry block. It is intended for the
Sunseeker telemetry application display. The UART is configured
for 115200 baud.

The block is sent about once per second and uses fixed start/end markers:

```text
raw_data
ABCDEF
...
VWXYZ
```

Inside the block, the firmware sends:

- Fixed CAN table rows such as `MC1BUS`, `MC1VEL`, `MC2VEL`, `DC_DRV`,
  `BP_VMX`, and others.
- BME temperature, pressure, and humidity.
- A single `NAV` line containing processed IMU speed, GPS speed, selected
  vehicle speed, GPS position, fix state, and GPS age.
- A `TL_TIM` line containing the current telemetry timestamp.

Example RS232 block:

```text
raw_data
ABCDEF
MC1BUS,0xHHHHHHHH,0xHHHHHHHH
MC1VEL,0xHHHHHHHH,0xHHHHHHHH
MC2VEL,0xHHHHHHHH,0xHHHHHHHH
DC_DRV,0xHHHHHHHH,0xHHHHHHHH
BP_VMX,0xHHHHHHHH,0xHHHHHHHH
BME,T=23.65,P=98110.73,H=52.43
NAV,IMU_MPH=0.00,GPS_MPH=0.00,GPS_VALID=0,VEHICLE_MPH=0.00,SOURCE=NONE,LAT=0.000000,LON=0.000000,FIX=0,AGE_MS=4294967295
TL_TIM,2004-01-10T00:13:54_INVALID
VWXYZ
```

Missing or invalid CAN values are shown with the placeholder:

```text
0xHHHHHHHH
```

Valid CAN rows are sent as two 32-bit hexadecimal values:

```text
MC1VEL,0x12345678,0x9ABCDEF0
```

## Speed Data

The firmware keeps separate speed values for the different sources:

- `imu_speed_mph`: speed estimated from the IMU.
- `gps_speed_mph`: speed reported by the GPS module.
- `can_speed_mph`: speed decoded from CAN data.
- `vehicle_speed_mph`: the selected vehicle speed after choosing the best
  available source.

For the ESP32/Bluetooth CSV stream, all of these speed fields are included.

For the RS232 stream, CAN speed is not repeated in the `NAV` line because the
raw CAN speed messages are already present in the fixed hexadecimal CAN rows,
such as `MC1VEL` and `MC2VEL`. The `NAV` line keeps the display cleaner by
showing IMU speed, GPS speed, selected vehicle speed, GPS position, and GPS fix
status in one place.
