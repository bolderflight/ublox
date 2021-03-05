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

#include "ublox/ublox.h"
#include "units/units.h"

bfs::Ublox ubx(&Serial1);

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  bool status = ubx.Begin(230400);
  if (!status) {
    Serial.println("Unable to communicate with uBlox receiver");
    while(1) {}
  }
  while (1) {
    if (ubx.Read()) {
      // Serial.print(ubx.tow_ms());
      // Serial.print("\t");
      // Serial.print(ubx.year());
      // Serial.print("\t");
      // Serial.print(ubx.month());
      // Serial.print("\t");
      // Serial.print(ubx.day());
      // Serial.print("\t");
      // Serial.print(ubx.hour());
      // Serial.print("\t");
      // Serial.print(ubx.min());
      // Serial.print("\t");
      // Serial.print(ubx.sec());
      // Serial.print("\t");
      // Serial.print(ubx.nano_sec());
      // Serial.print("\t");
      Serial.print(ubx.fix());
      Serial.print("\t");
      Serial.print(ubx.num_satellites());
      Serial.print("\t");
      Serial.print(ubx.lat_rad());
      Serial.print("\t");
      Serial.print(ubx.lon_rad());
      Serial.print("\t");
      Serial.print(ubx.alt_wgs84_m());
      Serial.print("\t");
      // Serial.print(ubx.alt_msl_m());
      // Serial.print("\t");
      // Serial.print(ubx.north_velocity_mps());
      // Serial.print("\t");
      // Serial.print(ubx.east_velocity_mps());
      // Serial.print("\t");
      // Serial.print(ubx.down_velocity_mps());
      // Serial.print("\t");
      // Serial.print(ubx.ground_speed_mps());
      // Serial.print("\t");
      // Serial.print(bfs::rad2deg(ubx.ground_track_rad()));
      // Serial.print("\t");
      // Serial.print(ubx.time_accuracy_ns());
      // Serial.print("\t");
      // Serial.print(ubx.horizontal_accuracy_m());
      // Serial.print("\t");
      // Serial.print(ubx.vertical_accuracy_m());
      // Serial.print("\t");
      // Serial.print(ubx.velocity_accuracy_mps());
      // Serial.print("\t");
      // Serial.print(bfs::rad2deg(ubx.track_accuracy_rad()));
      // Serial.print("\t");
      // Serial.print(ubx.valid_time_and_date());
      // Serial.print("\t");
      // Serial.print(ubx.valid_gnss_fix());
      Serial.print("\n");
    }
  }
}