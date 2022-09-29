[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Ubx
This library communicates with uBlox GNSS receivers using the UBX protocol.  This library is compatible with Arduino and CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
uBlox produces standard and high precision GPS receivers. These receivers feature high sensitivity, minimal acquisition times, and small form factors. UBX is a uBlox binary format for efficiently retrieving data from the receiver.

# Installation

## Arduino
Use the Arduino Library Manager to install this library or clone to your Arduino/libraries folder. This library is added as:

```C++
#include "ubx.h"
```

An example Arduino executable is located in: *examples/arduino/ublox_example/ublox_example.ino*. Teensy 3.x, 4.x, and LC devices are used for testing under Arduino and this library should be compatible with other Arduino devices.

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
   * IMXRT1062_MMOD

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The example target creates an executable for communicating with the GNSS receiver using the UBX protocol. Each target also has a *_hex*, for creating the hex file to upload to the microcontroller, and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools).

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
   * UBX-NAV-TIMEGPS

These messages should be enabled using the [u-center software](https://www.u-blox.com/en/product/u-center).

If high accuracy position data is available, the following messages should be enabled and will be used by this library.
   * UBX-NAV-HPPOSECEF
   * UBX-NAV-HPPOSLLH

If relative position data is available, such as from a stationary or moving reference, the following message should be enabled and will be used by this library.
   * UBX-NAV-RELPOSNED

Finally, if you are connected to a fixed-baseline and conducting a survey-in, the following message should be enabled and will be used by this library to provide information regarding the survey-in status.
   * UBX-NAV-SVIN

# Ubx

## Methods

**Ubx()** Default constructor, requires calling the Config method to setup the serial port.

**Ubx(HardwareSerial&ast; bus)** Creates a Ubx object. This constructor is used for the serial communication interface and a pointer to the serial bus object is passed to the constructor.

```C++
bfs::Ubx ubx(&Serial1);
```

**void Config(HardwareSerial&ast; bus)** Sets up the serial port to use for communication. Required if the default constructor is used.

**bool Begin(const int32_t baud)** Establishes communication with the GNSS receiver. Returns true on successfully receiving data, otherwise, returns false.

```C++
bool status = ubx.Begin(921600);
```

### Data Collection
The following method reads and parses the serial data. True is returned on receiving a full epoch of new data.

**bool Read()** Reads and parses data from the serial port. Returns true on receiving the end of epoch frame, which indicates that all data should be updated and available to use.

```C++
if (ubx.Read()) {
  // use the GNSS data
}
```

### Data Retrieval
The most recent valid packet is stored in the Ubx object. Data fields can be retrieved using the following functions.

#### Common Data

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

**double gps_tow_s()** GPS time of week, s.

**int16_t week()** GPS week number.

**int8_t leap_s()** Leap seconds (GPS-UTC).

**uint32_t time_acc_ns()** Estimated time accuracy, ns.

**float north_vel_mps()** North velocity, m/s

**float east_vel_mps()** East velocity, m/s

**float down_vel_mps()** Down velocity, m/s

**float gnd_spd_mps()** Ground speed (2D), m/s

**float ecef_vel_x_mps()** ECEF x velocity, m/s

**float ecef_vel_y_mps()** ECEF y velocity, m/s

**float ecef_vel_z_mps()** ECEF z velocity, m/s

**float spd_acc_mps()** Estimated speed accuracy, m/s

**float track_deg()** Estimated ground track (2D heading of motion), deg

**float track_rad()** Estimated ground track (2D heading of motion), rad

**float track_acc_deg()** Estimated ground track (2D heading of motion) accuracy, deg

**float track_acc_rad()** Estimated ground track (2D heading of motion) accuracy, rad

**double lat_deg()** Latitude, deg

**double lat_rad()** Latitude, rad

**double lon_deg()** Longitude, deg

**double lon_rad()** Longitude, rad

**float alt_wgs84_m()** Altitude above the WGS84 ellipsoid, m

**float alt_msl_m()** Altitude above Mean Sea Level, m

**float horz_acc_m()** Estimated horizontal position accuracy, m

**float vert_acc_m()** Estimated vertical position accuracy, m

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

#### Relative Position Data

**bool rel_pos_avail()** Whether relative position data is available.

**bool rel_pos_moving_baseline()** Whether the receiver is operating in moving base mode.

**bool rel_pos_ref_pos_miss()** Whether extrapolated reference position was used to compute moving base solution this epoch.

**bool rel_pos_ref_obs_miss()** Whether extrapolated reference observations were used to compute moving base solution this epoch.

**bool rel_pos_heading_valid()** Whether heading of the relative position vector is valid.

**bool rel_pos_normalized()** Whether the components of the relative position vector
(including the high-precision parts) are normalized.

**double rel_pos_north_m()** North component of relative position vector, m.

**double rel_pos_east_m()** East component of relative position vector, m.

**double rel_pos_down_m()** Down component of relative position vector, m.

**float rel_pos_acc_north_m()** Accuracy of relative position North component, m.

**float rel_pos_acc_east_m()** Accuracy of relative position East component, m.

**float rel_pos_acc_down_m()** Accuracy of relative position Down component, m.

**double rel_pos_len_m()** Length of the relative position vector, m.

**float rel_pos_len_acc_m()**  Accuracy of length of the relative position vector, m.

**float rel_pos_heading_deg()** Heading of the relative position vector, deg.

**float rel_pos_heading_rad()** Heading of the relative position vector, rad.

**float rel_pos_heading_acc_deg()** Accuracy of the heading of the relative position vector, deg.

**float rel_pos_heading_acc_rad()** Accuracy of the heading of the relative position vector, rad.

#### Survey In Data

**bool svin_valid()** Survey-in position validity flag, true = valid, otherwise false.

**bool svin_in_progress()** Survey-in in progress flag, true = in-progress, otherwise false.

**uint32_t svin_dur_s()** Passed survey-in observation time, s.

**double svin_ecef_pos_x_m()** Current survey-in mean position ECEF X coordinate, m.

**double svin_ecef_pos_y_m()** Current survey-in mean position ECEF Y coordinate, m.

**double svin_ecef_pos_z_m()** Current survey-in mean position ECEF Z coordinate, m.

**float svin_ecef_pos_acc_m()** Current survey-in mean position accuracy, m.

**uint32_t svin_num_obs()** Number of position observations used during survey-in.
