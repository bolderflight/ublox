/*
UBLOX_example.ino
Brian R Taylor
brian.taylor@bolderflight.com
2016-07-06

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "UBLOX.h"

// a uBlox object, which is on Teensy hardware
// serial port 3
UBLOX gps(1);

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

