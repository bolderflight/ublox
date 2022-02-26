/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#include "ubx.h"

/* Ublox object, GNSS on Serial3 */
bfs::Ubx gnss(&Serial3);

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  gnss.Begin(921600);
}

void loop() {
  if(gnss.Read()) {
    Serial.print(gnss.fix());
    Serial.print("\t");
    Serial.print(gnss.num_sv());
    Serial.print("\t");
    Serial.print(gnss.lat_deg(), 6);
    Serial.print("\t");
    Serial.print(gnss.lon_deg(), 6);
    Serial.print("\t");
    Serial.print(gnss.alt_wgs84_m(), 2);
    Serial.print("\n");
  }
}
