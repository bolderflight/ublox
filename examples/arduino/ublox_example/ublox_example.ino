/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "ublox.h"

/* Ublox object on hardware serial port 1 */
Ublox gps(&Serial1);

/* Rad to Deg conversion */
static constexpr double rad2deg = 180.0 / 3.14159265358979323846;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Starting communication with the GPS receiver, 115200 baud */
  if (!gps.Begin(115200)) {
    Serial.println("Cannot communicate with GPS receiver");
    while (1) {}
  }
}

void loop() {
  /* Check for new packet and display some data */
  if (gps.Read()) {
    Serial.print(gps.year());                   ///< [year], Year (UTC)
    Serial.print("\t");
    Serial.print(gps.month());                  ///< [month], Month, range 1..12 (UTC)
    Serial.print("\t");
    Serial.print(gps.day());                    ///< [day], Day of month, range 1..31 (UTC)
    Serial.print("\t");
    Serial.print(gps.hour());                   ///< [hour], Hour of day, range 0..23 (UTC)
    Serial.print("\t");
    Serial.print(gps.minute());                 ///< [min], Minute of hour, range 0..59 (UTC)
    Serial.print("\t");
    Serial.print(gps.sec());                    ///< [s], Seconds of minute, range 0..60 (UTC)
    Serial.print("\t");
    Serial.print(gps.num_satellites());         ///< [ND], Number of satellites used in Nav Solution
    Serial.print("\t");
    Serial.print(gps.lat_rad() * rad2deg, 10);  ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(gps.lon_rad() * rad2deg,10);   ///< [deg], Longitude
    Serial.print("\t");
    Serial.println(gps.alt_msl_m());            ///< [m], Height above mean sea level
  }
}

