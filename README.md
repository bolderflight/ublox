# ublox
This library communicates with uBlox GNSS receivers using the UBX protocol and is built for use with the Arduino IDE.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)

# Description
uBlox produces standard and high precision GPS receivers. These receivers feature high sensitivity, minimal acquisition times, and small form factors. UBX is a uBlox binary format for efficiently retrieving data from the receiver.

## Installation
Simply clone or download and extract the zipped library into your Arduino/libraries folder. The library is added as:

```C++
#include "ublox.h"
```

# Usage

## Receiver Setup
The GNSS receiver must be configured in the uBlox u-center application prior to use. Currently, this library only receives and parses data, it does not support configuration. The following packets must be enabled on the receiver:
   * UBX-NAV-PVT: this packet transmits standard accuracy position, velocity, and timing data.
   * UBX-NAV-EOE: this packet marks the end of the navigation epoch.

Optionally, if it's available, the following packet should also be enabled to use higher precision navigation:
   * UBX-NAV-HPPOSLLH: this packet transmits high accuracy position data.

You can also use u-center to configure the navigation solution and transmission rate, the baud rate for the serial interface, expected operation environment (i.e. stationary, airborne, etc), and any corrections (RTK or CORS network).

## Methods

**Ublox(HardwareSerial&ast; bus)** Creates a Ublox object. This constructor is used for the serial communication interface and a pointer to the serial bus object is passed to the constructor.

```C++
Ublox ubx(&Serial1);
```

**bool Begin(uint32_t baud)** Establishes communication with the GNSS receiver. Returns true on successfully receiving data, otherwise, returns false.

```C++
bool status = ubx.Begin(921600);
```

**bool Read()** Checks for GNSS packets and returns true on receiving data.

```C++
if (ubx.Read()) {
  // good GNSS data received
}
```

### Data Retrieval

The most recent valid packet is stored in the UBLOX object. Data fields can be retrieved using the following functions.

**uint32_t tow_ms()** GNSS time of week of the navigation solution, ms

**uint16_t year()** UTC year

**uint8_t month()** UTC month

**uint8_t day()** UTC day

**uint8_t hour()** UTC hour

**uint8_t min()** UTC minute

**uint8_t sec()** UTC second

**int32_t nano_sec()** UTC nano second

**FixType fix()** The current GNSS receiver fix. The following enum describes the fix type:

| Fix Type | Enum Value |
| --- | --- |
| No fix | FIX_NONE |
| Dead reckoning only | FIX_DEAD_RECKONING_ONLY |
| 2D | FIX_2D |
| 3D | FIX_3D |
| GNSS and dead reckoning | FIX_GNSS_DEAD_RECKONING |
| Time only | FIX_TIME_ONLY |

Note that dead reckoning is only supported on dead-reckoning capable receivers.

**uint8_t num_satellites()** The number of satellites used in the navigation solution

**Eigen::Vector3d lla_wgs84_rad_m()** Latitude (rad), longitude (rad), and altitude above the WGS-84 ellipsoid (m) returned as an Eigen::Vector object

**Eigen::Vector3d lla_msl_rad_m()** Latitude (rad), longitude (rad), and altitude above MSL (m) returned as an Eigen::Vector object

**double lat_rad()** Latitude, rad

**double lon_rad()** Longitude, rad

**float alt_wgs84_m()** Altitude above the WGS-84 ellipsoid, m

**float alt_msl_m()** Altitude above MSL, m

**Eigen::Vector3f ned_velocity_mps()** North, east, down velocity returned as an Eigen::Vector object, m/s

**float north_velocity_mps()** North  velocity, m/s

**float east_velocity_mps()** East velocity, m/s

**float down_velocity_mps()** Down velocity, m/s

**float ground_speed_mps()** Ground speed, m/s

**float ground_track_rad()** Ground track, rad

**uint32_t time_accuracy_ns()** Accuracy of the time solution, ns

**float horizontal_accuracy_m()** Horizontal position accuracy, m

**float vertical_accuracy_m()** Vertical position accuracy, m

**float velocity_accuracy_mps()** Velocity accuracy, m/s

**float track_accuracy_rad()** Ground track accuracy, rad

**bool valid_time_and_date()** Whether the date and time data is valid

**bool valid_gnss_fix()** Whether the GNSS data is valid
