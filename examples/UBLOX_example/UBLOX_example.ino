//
// title:     UBLOX_example.ino
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2016-07-06
// license: 
//

#include "UBLOX.h"

// a uBlox object, which is on Teensy hardware
// serial port 3
UBLOX gps(3);

// the uBlox data structure, UBX-NAV-PVT
gpsData uBloxData;

void setup() {
  // serial to display data
  Serial.begin(9600);

  // starting communication with the GPS
  // receiver at 115200 baud
  gps.begin(115200);
  
}

void loop() {

  // checking to see if a good packet has
  // been received and displaying some
  // of the packet data
  if( gps.read(&uBloxData) ){
    Serial.print(uBloxData.utcYear);  ///< [year], Year (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcMonth); ///< [month], Month, range 1..12 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcDay);   ///< [day], Day of month, range 1..31 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcHour);  ///< [hour], Hour of day, range 0..23 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcMin);   ///< [min], Minute of hour, range 0..59 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcSec);   ///< [s], Seconds of minute, range 0..60 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.numSV);    ///< [ND], Number of satellites used in Nav Solution
    Serial.print("\t");
    Serial.print(uBloxData.lat,10);   ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(uBloxData.lon,10);   ///< [deg], Longitude
    Serial.print("\t");
    Serial.println(uBloxData.hMSL);   ///< [m], Height above mean sea level
  }
}

