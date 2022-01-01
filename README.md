[![Pipeline](https://gitlab.com/bolderflight/software/ublox/badges/main/pipeline.svg)](https://gitlab.com/bolderflight/software/ublox/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Ubx
This library communicates with uBlox GNSS receivers using the UBX protocol.  This library is compatible with Arduino ARM and CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
uBlox produces standard and high precision GPS receivers. These receivers feature high sensitivity, minimal acquisition times, and small form factors. UBX is a uBlox binary format for efficiently retrieving data from the receiver.

# Installation

## Arduino
Use the Arduino Library Manager to install this library or clone to your Arduino/libraries folder. In addition, the [Bolder Flight Systems Units library](https://github.com/bolderflight/units) and the [Bolder Flight Systems Eigen library](https://github.com/bolderflight/eigen) must be installed. This library is added as:

```C++
#include "ubx.h"
```

An example Arduino executable is located in: *examples/arduino/ublox_example/ublox_example.ino*. Teensy 3.x, 4.x, and LC devices are used for testing under Arduino and this library should be compatible with other Arduino ARM devices. This library is **not** expected to work with AVR devices.

## CMake
CMake is used to build this library, which is exported as a library target called *ubx*. The header is added as:

```C++
#include "ubx.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and an example executable called *ublox_example*. The example executable source files are located at *examples/cmake/ublox_example.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The example target creates an executable for communicating with the GNSS receiver using the UBX protocol. Each target also has a *_hex*, for creating the hex file to upload to the microcontroller, and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that the CMake build tooling is expected to be run under Linux or WSL, instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools).

# Namespace
This library is within the namespace *bfs*.

# Usage

## Receiver Setup
This library parses data from the following messages:
   * UBX-NAV-DOP
   * UBX-NAV-EOE
   * UBX-NAV-POSECEF
   * UBX-NAV-PVT
   * UBX-NAV-VELECEF

Additionally, if high accuracy position data is available, it is also used.
   * UBX-NAV-HPPOSECEF
   * UBX-NAV-HPPOSLLH

Methods are provided in this library for automatically configuring the receiver to output the correct packets at the expected rate. Alternatively, the receiver can be configured in U-Center and this library used to parse the data for the microcontroller.

# Ubx

## Methods

**Ubx(HardwareSerial* bus)** Creates a Ubx object. This constructor is used for the serial communication interface and a pointer to the serial bus object is passed to the constructor.

```C++
bfs::Ubx ubx(&Serial1);
```
### Automatic setup
The following method automatically establishes communication with the GNSS reciever and configures it correctly. It is recommended for most situations.

**bool AutoBegin()** Automatically determines the GNSS receiver baudrate and configures the receiver:
   * To output the expected UBX-NAV packets at 10 Hz
   * To use a baudrate of 460800
   * To disable NMEA outputs
   * To use an airborne, 4G dynamic model

This method returns true on success or false on failure.

```C++
ubx.AutoBegin();
```

### Manual setup
The following methods are available for manually setting up the receiver.

**bool Begin(const int32_t baud)** Establishes communication with the GNSS receiver. Returns true on successfully receiving data, otherwise, returns false.

```C++
bool status = ubx.Begin(921600);
```
**void SetFactoryDefaults()** Clears the current receiver configuration, loading the factory defaults.

**bool SetRate(const uint16_t period_ms)** Sets the navigation solution rate, returns true on success or false on failure.

**bool SetDynModel(const DynMdl mdl)** Sets the dynamic model, returns true on success or false on failure. The available dynamic models are:

| Enum | Description |
| --- | --- |
| DYN_MDL_PORTABLE | Portable |
| DYN_MDL_STATIONARY | Stationary |
| DYN_MDL_PEDESTRIAN | Pedestrian |
| DYN_MDL_AUTOMOTIVE | Automotive |
| DYN_MDL_SEA | Sea |
| DYN_MDL_AIRBORNE_1G | Airborne with < 1G accel |
| DYN_MDL_AIRBORNE_2G | Airborne with < 2G accel |
| DYN_MDL_AIRBORNE_4G | Airborne with < 4G accel |
| DYN_MDL_WRIST | Writst worn watch |
| DYN_MDL_BIKE | Bike |

```C++
ubx.SetDynModel(bfs::Ubx::DYN_MDL_AIRBORNE_4G);
```

**bool TestComms()** Tests for communication with the GNSS receiver, returns true if communication is successful, otherwise, returns false.

**int32_t AutoBaud()** Determines the baudrate of the GNSS receiver, returns the baudrate on success or -1 on failure.

**bool SetBaud(const uint8_t port, const uint32_t baud)** Sets the receiver baudrate given a port and a baudrate. Returns true on success or false on failure. Available ports are: **UBX_COM_PORT_UART1_** and **UBX_COM_PORT_UART2_**.

**bool ConfigPort(const uint8_t port, const uint8_t in_prot, const uint8_t out_prot)** Configures the input and output protocols for a given port, returning true on success or false on failure. Available ports are: **UBX_COM_PORT_UART1_** and **UBX_COM_PORT_UART2_**, available protocols are: **UBX_COM_PROT_UBX_**, **UBX_COM_PROT_NMEA_**, **UBX_COM_PROT_RTCM_**, **UBX_COM_PROT_RTCM3_**. Protocols can be OR'd together to enable multiple.


### Data Collection
In all cases, the following method reads and parses the serial data. True is returned on receiving a full epoch of new data.

**bool Read()** Reads and parses data from the serial port. Returns true on receiving the end of epoch frame, which indicates that all data should be updated and available to use.

```C++
if (ubx.Read()) {
  // use the GNSS data
}
```

### Data Retrieval
The most recent valid packet is stored in the Ubx object. Data fields can be retrieved using the following functions.

**Fix fix()** Returns the GNSS fix status.

| Enum | Description |
| --- | --- |
| FIX_NONE | No fix |
| FIX_2D | 2D fix |
| FIX_3D | 3D fix |
| FIX_DGNSS | 3D fix with differential corrections applied |
| FIX_RTK_FLOAT | 3D fix, RTK corrections with floating ambiguities |
| FIX_RTK_FIXED | 3D fix, RTK corrections with fixed ambiguities |

**int8_t num_sv()** Number of satellite vehicles used in the navigation solution.

**int16_t utc_year()** UTC year.

**int8_t utc_month()** UTC month.

**int8_t utc_day()** UTC day.

**int8_t utc_hour()** UTC hour.

**int8_t utc_min()** UTC minute.

**int8_t utc_sec()** UTC second.

**int32_t utc_nano()** UTC nanoseconds.

**int32_t gps_tow_ms()** GPS time of week, ms.

**uint32_t time_acc_ns()** Estimated time accuracy, ns.

**Eigen::Vector3f ned_vel_mps()** North, East, Down (NED) velocity, m/s

**float north_vel_mps()** North velocity, m/s

**float east_vel_mps()** East velocity, m/s

**float down_vel_mps()** Down velocity, m/s

**float gnd_spd_mps()** Ground speed (2D), m/s

**Eigen::Vector3f ecef_vel_mps()** ECEF velocity, m/s

**float ecef_vel_x_mps()** ECEF x velocity, m/s

**float ecef_vel_y_mps()** ECEF y velocity, m/s

**float ecef_vel_z_mps()** ECEF z velocity, m/s

**float spd_acc_mps()** Estimated speed accuracy, m/s

**float track_deg()** Estimated ground track (2D heading of motion), deg

**float track_rad()** Estimated ground track (2D heading of motion), rad

**float track_acc_deg()** Estimated ground track (2D heading of motion) accuracy, deg

**float track_acc_rad()** Estimated ground track (2D heading of motion) accuracy, rad

**Eigen::Vector3d llh_deg_m()** Latitude, longitude, and height above the WGS84 ellipsoid in degrees and meters.

**Eigen::Vector3d llh_rad_m()** Latitude, longitude, and height above the WGS84 ellipsoid in radians and meters.

**double lat_deg()** Latitude, deg

**double lat_rad()** Latitude, rad

**double lon_deg()** Longitude, deg

**double lon_rad()** Longitude, rad

**float alt_wgs84_m()** Altitude above the WGS84 ellipsoid, m

**float alt_msl_m()** Altitude above Mean Sea Level, m

**float horz_acc_m()** Estimated horizontal position accuracy, m

**float vert_acc_m()** Estimated vertical position accuracy, m

**Eigen::Vector3d ecef_pos_m()** ECEF position, m

**double ecef_pos_x_m()** ECEF x position, m

**double ecef_pos_y_m()** ECEF y position, m

**double ecef_pos_z_m()** ECEF z position, m

**float ecef_pos_acc_m()** Estimated ECEF position accuracy, m

**float gdop()** geometric dilution of precision.

**float pdop()** position dilution of precision.

**float tdop()** time dilution of precision.

**float vdop()** vertical dilution of precision.

**float hdop()** horizontal dilution of precision.

**float ndop()** northing dilution of precision.

**float edop()** easting dilution of precision.
