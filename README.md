# ublox
This library communicates with uBlox GNSS receivers using the UBX protocol.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
uBlox produces standard and high precision GPS receivers. These receivers feature high sensitivity, minimal acquisition times, and small form factors. UBX is a uBlox binary format for efficiently retrieving data from the receiver.

## Installation
CMake is used to build this library, which is exported as a library target called *ublox*. The header is added as:

```
#include "ublox/ublox.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and an example executables called *uart_example*. The example executable source files are located at *examples/uart_example.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *uart_example* target creates an executables for communicating with the sensor using serial communication. The target also has a *_hex* for creating the hex to upload to the microcontroller. 

# Usage

## Receiver Setup
The GNSS receiver must be configured in the uBlox u-center application prior to use. Currently, this library only receives and parses data, it does not support configuration. The following packets must be enabled on the receiver:
   * UBX-NAV-PVT: this packet transmits standard accuracy position, velocity, and timing data.
   * UBX-NAV-EOE: this packet marks the end of the navigation epoch.

Optionally, if it's available, the following packet should also be enabled to use higher precision navigation:
   * UBX-NAV-HPPOSLLH: this packet transmits high accuracy position data.

You can also use u-center to configure the navigation solution and transmission rate, the baud rate for the serial interface, expected operation environment (i.e. stationary, airborne, etc), and any corrections (RTK or CORS network).

## Namespace
This library is within the namespace *bfs*.

## Methods
This driver conforms to the [GNSS interface](https://github.com/bolderflight/gnss); please refer to those documents for information on the *GnssConfig* and *GnssData* structs.

**bool Init(const GnssConfig &ref)** Initializes communication with the GNSS receiver and returns true on successfully establishing communication.

```C++
/* Ublox object */
bfs::Ublox gnss;
/* Config */
bfs::GnssConfig config = {
   .bus = &Serial3,
   .baud = 921600,
   .sampling_period_ms = 200
};
/* Init GNSS */
if (!gnss.Init(config)) {
   Serial.println("Error initializing communication with GNSS");
   while(1) {}
}
```

**bool Read(GnssData &ast; const ptr)** Reads data from the GNSS receiver and passes the data into the *GnssData* struct. Returns true on successfully reading new data.

```C++
/* GNSS data */
bfs::GnssData data;
if (gnss.Read(&data)) {

}
```
