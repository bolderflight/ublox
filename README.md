# UBLOX
Arduino library for communicating with [uBlox](https://www.u-blox.com) GPS receivers.

***Developing with the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) or [LC](https://www.pjrc.com/store/teensylc.html)? Consider buying our [Teensy GNSS Backpack](http://bolderflight.com/products/teensy/gnss/), which integrates the uBlox SAM-M8Q GNSS module in a stackable add-on module with UART, I2C, and PPS interfaces and thoughtfully designed to integrate perfectly with the Teensy. Check out all of our wonderfully small and powerful [Teensy Backpacks](http://bolderflight.com/products/teensy/)***

# Description
uBlox produces standard and high precision GPS receivers with options for RTK, PPP, and multi-constellation GNSS. These receivers feature high sensitivity, minimal acquisition times, and small form factors.

This library communicates with the uBlox receivers using the UBX protocol and the UBX-NAV-PVT packet, which is available on uBlox 7 and 8 series receivers. Hardware serial is used for receiving the data packets.

# Usage
## Installation
Simply clone or download and extract the zipped library into your Arduino/libraries folder.

Setup your uBlox receiver to output the UBX-NAV-PVT packet. Additional setup options include the desired GPS update frequency, the serial baud rate, and the expected dynamical environment. Setup of uBlox receivers can be accomplished using the [uBlox u-center software](https://www.u-blox.com/en/product/u-center-windows).

## Object Declaration
**UBLOX(HardwareSerial& bus,uint32_t baud)**
A UBLOX object should be declared, specifying the hardware serial port the uBlox GPS receiver is connected to and the baud rate. The specified baud rate should match the baud rate setup in the GPS receiver configuration. For example, the following code declares a UBLOX object called *gps* located on the hardware serial port 1 with a baud rate of 115200:

```C++
UBLOX gps(Serial1,115200);
```

## Setup Functions
**void begin()**
This should be called in your setup function. It initializes the serial communication between the microcontroller and uBlox GPS receiver.  For example, the following code begins serial communication:

```C++
gps.begin();
```

## Data Collection Functions
**bool readSensor()**
*readSensor()* reads data from the uBlox receiver and parses the UBX-NAV-PVT packet. When a complete packet is received with a valid checksum, *readSensor()* returns *true*. For example, placing the following code in the loop function will print the latitude, in degrees, to the screen at the GPS update frequency.

```C++
if(gps.readSensor()) {
	Serial.println(gps.getLatitude_deg(),10);
}
```

### Data Retrieval
The most recent valid packet is stored in the UBLOX object. Data fields can be retrieved using the following functions, which support both Imperial and SI units.

<table>
  <tr>
    <th>Imperial</th> 
    <th>SI</th>
    <th>Description</th>
  </tr>
  <tr>
    <td colspan=2> uint32_t getTow_ms()</td> 
    <td>GPS time of week of the navigation solution, ms</td>
  </tr>
  <tr>
    <td colspan=2> uint16_t getYear()</td>
    <td>UTC year</td>
  </tr>
  <tr>
    <td colspan=2> uint8_t getMonth()</td>
    <td>UTC month</td>
  </tr>
  <tr>
    <td colspan=2> uint8_t getDay()</td>
    <td>UTC day</td>
  </tr>
  <tr>
    <td colspan=2> uint8_t getHour()</td>
    <td>UTC hour</td>
  </tr>
  <tr>
    <td colspan=2> uint8_t getMin()</td>
    <td>UTC minute</td>
  </tr>
  <tr>
    <td colspan=2> uint8_t getSec()</td>
    <td>UTC second</td>
  </tr>
  <tr>
    <td colspan=2> int32_t getNanoSec()</td> 
    <td>UTC fraction of a second, ns&ast;</td>
  </tr>
  <tr>
    <td colspan=2> uint8_t getNumSatellites()</td>
    <td>Number of satellites used in the navigation solution</td>
  </tr>
  <tr>
    <td>double getLongitude_deg()</td>
    <td>double getLongitude_rad()</td>
    <td>Longitude</td>
  </tr>
  <tr>
    <td>double getLatitude_deg()</td>
    <td>double getLatitude_rad()</td>
    <td>Latitude</td>
  </tr>
  <tr>
    <td>double getEllipsoidHeight_ft()</td>
    <td>double getEllipsoidHeight_m()</td>
    <td>Height above the ellipsoid</td>
  </tr>
  <tr>
    <td>double getMSLHeight_ft()</td>
    <td>double getMSLHeight_m()</td>
    <td>Height above mean sea level</td>
  </tr>
  <tr>
    <td>double getHorizontalAccuracy_ft()</td>
    <td>double getHorizontalAccuracy_m()</td>
    <td>Horizontal accuracy estimate</td>
  </tr>
  <tr>
    <td>double getVerticalAccuracy_ft()</td>
    <td>double getVerticalAccuracy_m()</td>
    <td>Vertical accuracy estimate</td>
  </tr>
  <tr>
    <td>double getNorthVelocity_fps()</td>
    <td>double getNorthVelocity_ms()</td>
    <td>NED north velocity</td>
  </tr>
  <tr>
    <td>double getEastVelocity_fps()</td>
    <td>double getEastVelocity_ms()</td>
    <td>NED east velocity</td>
  </tr>
  <tr>
    <td>double getDownVelocity_fps()</td>
    <td>double getDownVelocity_ms()</td>
    <td>NED down velocity</td>
  </tr>
  <tr>
    <td>double getGroundSpeed_fps()</td>
    <td>double getGroundSpeed_ms()</td>
    <td>2D ground speed</td>
  </tr>
  <tr>
    <td>double getSpeedAccuracy_fps()</td>
    <td>double getSpeedAccuracy_ms()</td>
    <td>Speed accuracy estimate</td>
  </tr>
  <tr>
    <td>double getMotionHeading_deg()</td>
    <td>double getMotionHeading_rad()</td>
    <td>2D heading of motion</td>
  </tr>
  <tr>
    <td>double getVehicleHeading_deg()</td>
    <td>double getVehicleHeading_rad()</td>
    <td>2D vehicle heading</td>
  </tr>
  <tr>
    <td>double getHeadingAccuracy_deg()</td>
    <td>double getHeadingAccuracy_rad()</td>
    <td>Heading accuracy estimate</td>
  </tr>
  <tr>
    <td>float getMagneticDeclination_deg()</td>
    <td>float getMagneticDeclination_rad()</td>
    <td>Magnetic declination</td>
  </tr>
  <tr>
    <td>float getMagneticDeclinationAccuracy_deg()</td>
    <td>float getMagneticDeclinationAccuracy_rad()</td>
    <td>Magnetic declination accuracy estimate</td>
  </tr>
  <tr>
    <td colspan=2> float getpDOP()</td>
    <td>Position dilution of precision</td>
  </tr>
  <tr>
    <td colspan=2> enum FixType getFixType()</td>
    <td>Fix type, see below</td>
  </tr>
  <tr>
    <td colspan=2> enum PowerSaveMode getPowerSaveMode()</td>
    <td>Power save mode, see below</td>
  </tr>
  <tr>
    <td colspan=2> enum CarrierPhaseStatus getCarrierPhaseStatus()</td>
    <td>Carrier phase status, see below</td>
  </tr>
  <tr>
    <td colspan=2> bool isGnssFixOk()</td>
    <td>Valid fix, within DOP and accuracy masks</td>
  </tr>
  <tr>
    <td colspan=2> bool isDiffCorrApplied()</td>
    <td>Differential corrections were applied</td>
  </tr>
  <tr>
    <td colspan=2> bool isHeadingValid()</td>
    <td>Heading of vehicle is valid</td>
  </tr>
  <tr>
    <td colspan=2> bool isConfirmedDate()</td>
    <td>UTC date validity could be confirmed</td>
  </tr>
  <tr>
    <td colspan=2> bool isConfirmedTime()</td>
    <td>UTC time validity could be confirmed</td>
  </tr>
  <tr>
    <td colspan=2> bool isTimeDateConfirmationAvail()</td>
    <td>Info about UTC date and time validity confirmation is available</td>
  </tr>
  <tr>
    <td colspan=2> bool isValidDate()</td>
    <td>Valid UTC date</td>
  </tr>
  <tr>
    <td colspan=2> bool isValidTime()</td>
    <td>Valid UTC time</td>
  </tr>
  <tr>
    <td colspan=2> bool isTimeFullyResolved()</td>
    <td>UTC time of day has been fully resolved, no seconds uncertainty</td>
  </tr>
  <tr>
    <td colspan=2> bool isMagneticDeclinationValid()</td>
    <td>Valid magnetic declination estimate</td>
  </tr>
</table>

&ast; The various agencies try to keep the GNSS and UTC references synchronized at the tick event of the second. Short term perturbations in clocks may result in GPS second event being ahead of or behind the UTC second event by up to a microsecond. Hence, fraction of a second may be positive or negative.

The following enum describes the fix type:

| enum UBLOX::FixType | Description |
| -- | -- |
| NO_FIX | No Fix |
| DEAD_RECKONING | Dead reckoning only |
| FIX_2D | 2D-fix |
| FIX_3D | 3D-fix |
| GNSS_AND_DEAD_RECKONING | GNSS + dead reckoning combined |
| TIME_ONLY | time only fix |

The following enum describes the power save modes:

| enum UBLOX::PowerSaveMode | Description |
| -- | -- |
| NOT_ACTIVE | PSM is not active |
| ENABLED | Enabled (an intermediate state before Acquisition state) |
| ACQUISITION | Acquisition |
| TRACKING | Tracking |
| OPTIMIZED_TRACKING | Power optimized tracking |
| INACTIVE | Inactive |

The following enum describes the carrier phase status:

| enum UBLOX::CarrierPhaseStatus | Description |
| -- | -- |
| NO_SOL | No carrier phase range solution |
| FLOAT_SOL | Float solution (no fixed integer carrier phase measurements used to calculate the solution) |
| FIXED_SOL | Fixed solution (one or more fixed integer carrier phase range measurements used to calculate the solution) |

# Wiring
Please refer to your microcontroller documentation for hardware serial port pin information. For development purposes, the uBlox NEO 7P receiver available from [CSG Shop](http://www.csgshop.com/product.php?id_product=201) and the uBlox M8 receiver available from [HobbyKing](http://www.hobbyking.com/hobbyking/store/__86436__UBLOX_Micro_M8N_GPS_Compass_Module_1pc_.html) were used. 

For the uBlox M8 receiver available from [HobbyKing](http://www.hobbyking.com/hobbyking/store/__86436__UBLOX_Micro_M8N_GPS_Compass_Module_1pc_.html), taking the red wire as pin 1, the following is the GPS receiver pinout:

| Pin | Description |
| --- | --- 		|
| 1	| Not Connected |
| 2	| Not Connected |
| 3 | 5V |
| 4 | RX (connect to TX) |
| 5 | TX (connect to RX) |
| 6 | GND |
