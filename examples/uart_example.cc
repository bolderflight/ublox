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

/* Ublox object */
bfs::Ublox gnss;

/* GNSS data */
bfs::GnssData data;

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Config */
  bfs::GnssConfig config = {
    .sampling_period_ms = 200,
    .baud = 921600,
    .bus = &Serial3
  };
  /* Init GNSS */
  if (!gnss.Init(config)) {
    Serial.println("Error initializing communication with GNSS");
    while(1) {}
  }
  while (1) {
    if (gnss.Read(&data)) {
      Serial.print(data.new_data);
      Serial.print("\t");
      Serial.print(data.healthy);
      Serial.print("\t");
      Serial.print(static_cast<int8_t>(data.fix));
      Serial.print("\t");
      Serial.print(data.num_sats);
      Serial.print("\t");
      Serial.print(data.tow_ms);
      Serial.print("\t");
      Serial.print(data.week);
      Serial.print("\t");
      Serial.print(bfs::rad2deg(data.lat_rad));
      Serial.print("\t");
      Serial.print(bfs::rad2deg(data.lon_rad));
      Serial.print("\t");
      Serial.print(data.alt_wgs84_m);
      Serial.print("\t");
      Serial.print(data.ned_vel_mps(0));
      Serial.print("\t");
      Serial.print(data.ned_vel_mps(1));
      Serial.print("\t");
      Serial.print(data.ned_vel_mps(2));
      Serial.print("\t");
      Serial.print(data.horz_acc_m);
      Serial.print("\t");
      Serial.print(data.vert_acc_m);
      Serial.print("\t");
      Serial.print(data.vel_acc_mps);
      Serial.print("\n");
    }
  }
}