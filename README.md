# UBLOX
Library for communicating with [uBlox](https://www.u-blox.com) GPS receivers using Teensy 3.x and Teensy LC devices.

# Description
uBlox produces standard and high precision GPS receivers with options for RTK, PPP, and multi-constellation GNSS. These receivers feature high sensitivity, minimal acquisition times, and small form factors.

This library communicates with the uBlox receivers using the UBX protocol and the UBX-NAV-PVT packet, which is available on uBlox 7 and 8 series receivers. Hardware serial is used for receiving the data packets.

# Usage
This library uses the [hardware serial](https://www.pjrc.com/teensy/td_uart.html) for Teensy devices.

Simply clone or download and extract the zipped library into your Arduino/libraries folder.

Setup your uBlox receiver to output the UBX-NAV-PVT packet. Additional setup options include the desired GPS update frequency, the serial baud rate, and the expected dynamical environment. Setup of uBlox receivers can be accomplished using the [uBlox u-center software](https://www.u-blox.com/en/product/u-center-windows).

**UBLOX(uint8_t bus)**
A UBLOX object should be declared, specifying the hardware serial port the uBlox GPS receiver is connected to. For example, the following code declares a UBLOX object called *gps* located on the Teensy hardware serial port 3:

```C++
UBLOX gps(3);
```

**struct gpsData**
UBX-NAV-PVT packet data is made available via the *gpsData* structure, defined below. Please note that some of the values have been scaled from the raw UBX-NAV-PVT packet to provide the data in more conventional units.

```C++
struct gpsData {
  unsigned long   iTOW;			  ///< [ms], GPS time of the navigation epoch
  unsigned short  utcYear;		  ///< [year], Year (UTC)
  unsigned char   utcMonth;		  ///< [month], Month, range 1..12 (UTC)
  unsigned char   utcDay;		  ///< [day], Day of month, range 1..31 (UTC)
  unsigned char   utcHour;		  ///< [hour], Hour of day, range 0..23 (UTC)
  unsigned char   utcMin;		  ///< [min], Minute of hour, range 0..59 (UTC)
  unsigned char   utcSec;		  ///< [s], Seconds of minute, range 0..60 (UTC)
  unsigned char   valid;		  ///< [ND], Validity flags
  unsigned long   tAcc;			  ///< [ns], Time accuracy estimate (UTC)
  long            utcNano;		  ///< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
  unsigned char   fixType;		  ///< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 
                                  ///< 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 
                                  ///< 5: time only fix
  unsigned char   flags;		  ///< [ND], Fix status flags
  unsigned char   flags2;		  ///< [ND], Additional flags
  unsigned char   numSV;		  ///< [ND], Number of satellites used in Nav Solution
  double          lon;			  ///< [deg], Longitude
  double          lat;			  ///< [deg], Latitude
  double          height;		  ///< [m], Height above ellipsoid 
  double          hMSL;			  ///< [m], Height above mean sea level
  double          hAcc;			  ///< [m], Horizontal accuracy estimate
  double          vAcc;			  ///< [m], Vertical accuracy estimate
  double          velN;			  ///< [m/s], NED north velocity
  double          velE;			  ///< [m/s], NED east velocity
  double          velD;			  ///< [m/s], NED down velocity
  double          gSpeed;		  ///< [m/s], Ground Speed (2-D)
  double          heading;		  ///< [deg], Heading of motion (2-D)
  double          sAcc;			  ///< [m/s], Speed accuracy estimate
  double          headingAcc;	  ///< [deg], Heading accuracy estimate (both motion and vehicle)
  double          pDOP;			  ///< [ND], Position DOP
  double		  headVeh;		  ///< [deg], Heading of vehicle (2-D)
};
```

A new variable of the *gpsData* structure type should be created. For example, *uBloxData* is being created of the *gpsData* structure type.

```C++
gpsData uBloxData;
```

**void begin(int baud)**
This should be called in your setup function. It sets the baud rate and initializes the serial communication between the Teensy and uBlox GPS receiver. The specified baud rate should match the baud rate setup in the GPS receiver configuration. For example, the following code begins serial communication at a baud rate of 115200, which is the baud rate that the GPS receiver is using to transmit data:

```C++
gps.begin(115200);
```
**bool read(gpsData *gpsData_ptr)**
*read(gpsData *gpsData_ptr)* reads data from the uBlox receiver and parses the UBX-NAV-PVT packet. When a complete packet is received with a valid checksum, *read(gpsData *gpsData_ptr)* returns *true* and the parsed UBX-NAV-PVT packet data is available from the gpsData structure. For example, placing the following code in the loop function will print the latitude, in degrees, to the screen at the GPS update frequency.

```C++
if( gps.read(&uBloxData) ){
	Serial.println(uBloxData.lat,10);
}
```

# Wiring
Please refer to the [Teensy pinout diagrams](https://www.pjrc.com/teensy/pinout.html) for hardware serial port pin information. For development purposes, the uBlox NEO 7P receiver available from [CSG Shop](http://www.csgshop.com/product.php?id_product=201) and the uBlox M8 receiver available from [HobbyKing](http://www.hobbyking.com/hobbyking/store/__86436__UBLOX_Micro_M8N_GPS_Compass_Module_1pc_.html) were used. 

For the uBlox M8 receiver available from [HobbyKing](http://www.hobbyking.com/hobbyking/store/__86436__UBLOX_Micro_M8N_GPS_Compass_Module_1pc_.html), taking the red wire as pin 1, the following is the GPS receiver pinout:

| Pin | Description |
| --- | --- 		|
| 1	| Not Connected |
| 2	| Not Connected |
| 3 | 5V |
| 4 | RX (connect to Teensy TX) |
| 5 | TX (connect to Teensy RX) |
| 6 | GND |
