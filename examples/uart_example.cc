/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "ublox/ublox.h"
#include "global_defs/global_defs.h"

sensors::Ublox ubx(&Serial3);

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  bool status = ubx.Begin(921600);
  if (!status) {
    Serial.println("Unable to communicate with uBlox receiver");
    while(1) {}
  }
  while (1) {
    if (ubx.Read()) {
      Serial.print(ubx.tow_ms());
      Serial.print("\t");
      Serial.print(ubx.year());
      Serial.print("\t");
      Serial.print(ubx.month());
      Serial.print("\t");
      Serial.print(ubx.day());
      Serial.print("\t");
      Serial.print(ubx.hour());
      Serial.print("\t");
      Serial.print(ubx.min());
      Serial.print("\t");
      Serial.print(ubx.sec());
      Serial.print("\t");
      Serial.print(ubx.nano_sec());
      Serial.print("\t");
      Serial.print(ubx.fix());
      Serial.print("\t");
      Serial.print(ubx.num_satellites());
      Serial.print("\t");
      Serial.print(ubx.lat_rad());
      Serial.print("\t");
      Serial.print(ubx.lon_rad());
      Serial.print("\t");
      Serial.print(ubx.alt_m());
      Serial.print("\t");
      Serial.print(ubx.north_velocity_mps());
      Serial.print("\t");
      Serial.print(ubx.east_velocity_mps());
      Serial.print("\t");
      Serial.print(ubx.down_velocity_mps());
      Serial.print("\t");
      Serial.print(ubx.time_accuracy_ns());
      Serial.print("\t");
      Serial.print(ubx.horizontal_accuracy_m());
      Serial.print("\t");
      Serial.print(ubx.vertical_accuracy_m());
      Serial.print("\t");
      Serial.print(ubx.velocity_accuracy_mps());
      Serial.print("\t");
      Serial.print(ubx.valid_time_and_date());
      Serial.print("\t");
      Serial.print(ubx.valid_gnss_fix());
      Serial.print("\n");
    }
  }
}