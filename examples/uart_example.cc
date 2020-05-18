/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "ublox/ublox.h"

sensors::Ublox ubx(&Serial1);

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  Serial.println("BEGIN");
  bool status = ubx.Begin(115200);
  Serial.println(status);
  status = ubx.EnableHighPrecision();
  Serial.println(status);
  while(1){}
  unsigned int t1, t2;
  while (1) {
    if (ubx.Read()) {
      t2 = micros();
      Serial.println(t2 - t1);
      t1 = t2;
    }
  }
}