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

#ifndef SRC_UBX_CFG_H_
#define SRC_UBX_CFG_H_

#include <cstdint>
#include <cstddef>
#include "ubx_defs.h"

// XXX Get / Set vs Poll request

namespace bfs {
/*
* Defs for UBX-CFG messages
*/
/* UBX-CFG IDs */
static constexpr uint8_t UBX_CFG_ANT_ID_ = 0x13;
static constexpr uint8_t UBX_CFG_CFG_ID_ = 0x09;
static constexpr uint8_t UBX_CFG_DAT_ID_ = 0x06;
static constexpr uint8_t UBX_CFG_DGNSS_ID_ = 0x70;
static constexpr uint8_t UBX_CFG_GEOFENCE_ID_ = 0x69;
static constexpr uint8_t UBX_CFG_GNSS_ID_ = 0x3e;
static constexpr uint8_t UBX_CFG_INF_ID_ = 0x02;
static constexpr uint8_t UBX_CFG_ITFM_ID_ = 0x39;
static constexpr uint8_t UBX_CFG_LOGFILTER_ID_ = 0x47;
static constexpr uint8_t UBX_CFG_MSG_ID_ = 0x01;
static constexpr uint8_t UBX_CFG_NAV5_ID_ = 0x24;
static constexpr uint8_t UBX_CFG_NAVX5_ID_ = 0x23;
static constexpr uint8_t UBX_CFG_NMEA_ID_ = 0x17;
static constexpr uint8_t UBX_CFG_ODO_ID_ = 0x1e;
static constexpr uint8_t UBX_CFG_PRT_ID_ = 0x00;
static constexpr uint8_t UBX_CFG_PWR_ID_ = 0x57;
static constexpr uint8_t UBX_CFG_RATE_ID_ = 0x08;
static constexpr uint8_t UBX_CFG_RINV_ID_ = 0x34;
static constexpr uint8_t UBX_CFG_RST_ID_ = 0x04;
static constexpr uint8_t UBX_CFG_SBAS_ID_ = 0x16;
static constexpr uint8_t UBX_CFG_TMODE3_ID_ = 0x71;
static constexpr uint8_t UBX_CFG_TP5_ID_ = 0x31;
static constexpr uint8_t UBX_CFG_USB_ID_ = 0x1b;
static constexpr uint8_t UBX_CFG_VALDEL_ID_ = 0x8c;
static constexpr uint8_t UBX_CFG_VALGET_ID_ = 0x8b;
static constexpr uint8_t UBX_CFG_VALSET_ID_ = 0x8a;
/* UBX-CFG Messages */
struct UbxCfgAnt {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_ANT_ID_;
  static constexpr uint16_t len = 4;
  struct {
    X2 flags;   // Antenna flag mask
    X2 pins;    // Antenna pin config
  } payload;
};
struct UbxCfgCfg {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_CFG_ID_;
  static constexpr uint16_t len = 13;
  struct {
    X4 clear_mask;
    X4 save_mask;
    X4 load_mask;
    X1 device_mask;
  } payload;
};
struct UbxCfgDatSet {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_DAT_ID_;
  static constexpr uint16_t len = 44;
  struct {
    R8 maj_a;   // Semi-major axis, m
    R8 flat;    // 1.0 / flattening
    R4 d_x;     // X axis shift at origin, m
    R4 d_y;     // Y axis shift at origin, m
    R4 d_z;     // Z axis shift at origin, m
    R4 rot_x;   // Rotation about the x axis, s
    R4 rot_y;   // Rotation about the y axis, s
    R4 rot_z;   // Rotation about the z axis, s
    R4 scale;   // Scale change, ppm
  } payload;
};
struct UbxCfgDatGet {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_DAT_ID_;
  static constexpr uint16_t len = 52;
  struct {
    U2 datum_num;       // Datum number
    CH datum_name[6];   // Datum name
    R8 maj_a;           // Semi-major axis, m
    R8 flat;            // 1.0 / flattening
    R4 d_x;             // X axis shift at origin, m
    R4 d_y;             // Y axis shift at origin, m
    R4 d_z;             // Z axis shift at origin, m
    R4 rot_x;           // Rotation about the x axis, s
    R4 rot_y;           // Rotation about the y axis, s
    R4 rot_z;           // Rotation about the z axis, s
    R4 scale;           // Scale change, ppm
  } payload;
};
struct UbxCfgDgnss {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_DGNSS_ID_;
  static constexpr uint16_t len = 4;
  struct {
    U1 dgnss_mode;    // Specifies the DGNSS mode
    U1 reserved0[3];
  } payload;
};
struct UbxCfgGeofence {

};
// struct UbxCfgGnss {
//   static constexpr uint8_t cls = UBX_CFG_CLS_;
//   static constexpr uint8_t id = UBX_CFG_GNSS_ID_;
//   uint16_t len;
//   struct {

//   } payload;
// };
struct UbxCfgInfPoll {

};
struct UbxCfgInf {

};
struct UbxCfgItfm {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_ITFM_ID_;
  static constexpr uint16_t len = 8;
  struct {
    X4 config;    // Interference configuration
    X4 config2;   // Extra settings for jamming / interference monitor
  } payload;
};
struct UbxCfgLogfilter {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_LOGFILTER_ID_;
  static constexpr uint16_t len = 12;
  struct {
    U1 version;             // Message version
    X1 flags;               // Flags
    U2 min_interval;        // Minimum time interval between logged positions, s
    U2 time_threshold;      // Time diff to trigger log, s
    U2 speed_threshold;     // Speed to trigger log, m/s
    U4 position_threshold;  // Position diff to trigger log, m
  } payload;
};
struct UbxCfgMsgPoll {

};
struct UbxCfgMsgSet {

};
struct UbxCfgMsgSet {

};
struct UbxCfgNav5 {

};
struct UbxCfgNavx5 {

};
struct UbxCfgNmea {

};
struct UbxCfgOdo {

};
struct UbxCfgPrt {

};
struct UbxCfgPrt {

};
struct UbxCfgPrt {

};
struct UbxCfgPrt {

};
struct UbxCfgPrt {

};
struct UbxCfgPwr {

};
struct UbxCfgRate {

};
struct UbxCfgRinv {

};
struct UbxCfgRst {

};
struct UbxCfgSbas {

};
struct UbxCfgTmode3 {

};
struct UbxCfgTp5 {

};
struct UbxCfgUsb {

};
struct UbxCfgValdel {

};
struct UbxCfgValdel {

};
struct UbxCfgValget {

};
struct UbxCfgValget {

};
struct UbxCfgValset {

};
struct UbxCfgValset {

};

}  // namespace bfs

#endif  // SRC_UBX_CFG_H_
