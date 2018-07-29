/*
UBLOX_example.ino
Brian R Taylor
brian.taylor@bolderflight.com

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

// a uBlox object, which is on hardware
// serial port 1 with a baud rate of 115200
UBLOX gps(Serial1,115200);

void setup() {
  // serial to display data
  Serial.begin(115200);

  // starting communication with the GPS receiver
  gps.begin();
}

void loop() {
  // checking to see if a good packet has
  // been received and displaying some
  // of the packet data
  if(gps.readSensor()) {
    Serial.print(gps.getYear());                ///< [year], Year (UTC)
    Serial.print("\t");
    Serial.print(gps.getMonth());               ///< [month], Month, range 1..12 (UTC)
    Serial.print("\t");
    Serial.print(gps.getDay());                 ///< [day], Day of month, range 1..31 (UTC)
    Serial.print("\t");
    Serial.print(gps.getHour());                ///< [hour], Hour of day, range 0..23 (UTC)
    Serial.print("\t");
    Serial.print(gps.getMin());                 ///< [min], Minute of hour, range 0..59 (UTC)
    Serial.print("\t");
    Serial.print(gps.getSec());                 ///< [s], Seconds of minute, range 0..60 (UTC)
    Serial.print("\t");
    Serial.print(gps.getNumSatellites());       ///< [ND], Number of satellites used in Nav Solution
    Serial.print("\t");
    Serial.print(gps.getLatitude_deg(),10);     ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(gps.getLongitude_deg(),10);    ///< [deg], Longitude
    Serial.print("\t");
    Serial.println(gps.getMSLHeight_ft());      ///< [ft], Height above mean sea level
  }
}

