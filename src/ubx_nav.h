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

#ifndef SRC_UBX_NAV_H_
#define SRC_UBX_NAV_H_

#include <cstdint>
#include <cstddef>

namespace bfs {
/*
* Defs for UBX-NAV messages
*/
/* UBX-NAV IDs */
static constexpr uint8_t UBX_NAV_CLOCK_ID_ = 0x22;
static constexpr uint8_t UBX_NAV_DOP_ID_ = 0x04;
static constexpr uint8_t UBX_NAV_EOE_ID_ = 0x61;
static constexpr uint8_t UBX_NAV_GEOFENCE_ID_ = 0x39;
static constexpr uint8_t UBX_NAV_HPPOSECEF_ID_ = 0x13;
static constexpr uint8_t UBX_NAV_HPPOSLLH_ID_ = 0x14;
static constexpr uint8_t UBX_NAV_ODO_ID_ = 0x09;
static constexpr uint8_t UBX_NAV_ORB_ID_ = 0x34;
static constexpr uint8_t UBX_NAV_POSECEF_ID_ = 0x01;
static constexpr uint8_t UBX_NAV_POSLLH_ID_ = 0x02;
static constexpr uint8_t UBX_NAV_PVT_ID_ = 0x07;
static constexpr uint8_t UBX_NAV_RELPOSNED_ID_ = 0x3c;
static constexpr uint8_t UBX_NAV_RESETODO_ID_ = 0x10;
static constexpr uint8_t UBX_NAV_SAT_ID_ = 0x35;
static constexpr uint8_t UBX_NAV_SBAS_ID_ = 0x32;
static constexpr uint8_t UBX_NAV_SIG_ID_ = 0x43;
static constexpr uint8_t UBX_NAV_SLAS_ID_ = 0x42;
static constexpr uint8_t UBX_NAV_STATUS_ID_ = 0x03;
static constexpr uint8_t UBX_NAV_SVIN_ID_ = 0x3b;
static constexpr uint8_t UBX_NAV_TIMEBDS_ID_ = 0x24;
static constexpr uint8_t UBX_NAV_TIMEGAL_ID_ = 0x25;
static constexpr uint8_t UBX_NAV_TIMEGLO_ID_ = 0x23;
static constexpr uint8_t UBX_NAV_TIMELS_ID_ = 0x26;
static constexpr uint8_t UBX_NAV_TIMEQZSS_ID_ = 0x27;
static constexpr uint8_t UBX_NAV_TIMEUTC_ID_ = 0x21;
static constexpr uint8_t UBX_NAV_VELECEF_ID_ = 0x11;
static constexpr uint8_t UBX_NAV_VELNED_ID_ = 0x12;
/* UBX-NAV messages */
struct UbxNavClock {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_CLOCK_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U4 i_tow;   // GPS time of week, ms
    I4 clk_b;   // Clock bias, ns
    I4 clk_d;   // Clock drift, ns/s
    U4 t_acc;   // Time accuracy estimate, ns
    U4 f_acc;   // Frequency accuracy estimate, ps/s
  } payload;
};
struct UbxNavDop {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_DOP_ID_;
  static constexpr uint16_t len = 18;
  struct {
    U4 i_tow;   // GPS time of week, ms
    U2 g_dop;   // Geometric DOP, scale 0.01
    U2 p_dop;   // Position DOP, scale 0.01
    U2 t_dop;   // Time DOP, scale 0.01
    U2 v_dop;   // Vertical DOP, scale 0.01
    U2 h_dop;   // Horizontal DOP, scale 0.01
    U2 n_dop;   // Northing DOP, scale 0.01
    U2 e_dop;   // Easting DOP, scale 0.01
  } payload;
};
struct UbxNavEoe {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_EOE_ID_;
  static constexpr uint16_t len = 4;
  struct {
    U4 i_tow;   // GPS time of week, ms
  } payload;
};
struct UbxNavFence {
  U1 state;   // Geofence state
  U1 id;      // Geofence ID
};
template<std::size_t N>   // templated by the maximum number of fences
struct UbxNavGeofence {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_GEOFENCE_ID_;
  uint16_t len;
  struct {
    U4 i_tow;               // GPS time of week, ms
    U1 version;             // Message version
    U1 status;              // Geofencing status
    U1 num_fences;          // Number of geofences
    U1 comb_state;          // Combined (logical OR) state of all geofences
    UbxNavFence fence[N];   // Repeated group (num_fences times)
  } payload;
};
struct UbxNavHpposecef {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_HPPOSECEF_ID_;
  static constexpr uint16_t len = 28;
  struct {
    U1 version;         // Message version
    U1 reserved0[3];
    U4 i_tow;           // GPS time of week, ms
    I4 ecef_x;          // ECEF x coordinate, cm
    I4 ecef_y;          // ECEF y coordinate, cm
    I4 ecef_z;          // ECEF z coordinate, cm
    I1 ecef_x_hp;       // High precision component of ECEF x, mm, scale 0.1
    I1 ecef_y_hp;       // High precision component of ECEF y, mm, scale 0.1
    I1 ecef_z_hp;       // High precision component of ECEF z, mm, scale 0.1
    X1 flags;           // Flags
    U4 p_acc;           // Position accuracy estimate, mm, scale 0.1
  } payload;
};
struct UbxNavHpposllh {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_HPPOSLLH_ID_;
  static constexpr uint16_t len = 36;
  struct {
    U1 version;         // Message version
    U1 reserved0[2];
    X1 flags;           // Flags
    U4 i_tow;           // GPS time of week, ms
    I4 lon;             // Longitude, deg, scale 1e-7
    I4 lat;             // Latitude, deg, scale 1e-7
    I4 height;          // Height above ellipsoid, mm
    I4 h_msl;           // Height above MSL, mm
    I1 lon_hp;          // High precision component of lon, deg, scale 1e-9
    I1 lat_hp;          // High precision component of lat, deg, scale 1e-9
    I1 height_hp;       // High precision component of height, mm, scale 0.1
    I1 h_msl_hp;        // High precision component of MSL, mm, scale 0.1
    U4 h_acc;           // Horizontal accuracy estimate, mm, scale 0.1
    U4 v_acc;           // Vertical accuracy estimate, mm, scale 0.1
  } payload;
};

}  // namespace bfs

#endif  // SRC_UBX_NAV_H_
