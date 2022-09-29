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

#ifndef SRC_UBX_NAV_H_
#define SRC_UBX_NAV_H_

#if !defined(ARDUINO)
#include <cstdint>
#include <cstddef>
#endif
#include "ubx_defs.h"  // NOLINT

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
static constexpr uint8_t UBX_NAV_TIMEGPS_ID_ = 0x20;
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
template<size_t N>   // templated by the maximum number of fences
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
    struct {
      U1 state;             // Geofence state
      U1 id;                // Geofence ID
    } fence[N];             // Repeated group (num_fences times)
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
struct UbxNavOdo {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_ODO_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U1 version;         // Message version
    U1 reserved0[3];
    U4 i_tow;           // GPS time of week, ms
    U4 distance;        // Ground distance since last reset, m
    U4 total_distance;  // Total cumulative ground distance, m
    U4 distance_std;    // Ground distance accuracy (1-sigma), m
  } payload;
};
template<size_t N>  // templated by maximum number of satellites
struct UbxNavOrb {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_ORB_ID_;
  uint16_t len;
  struct {
    U4 i_tow;           // GPS time of week, ms
    U1 version;         // Message version
    U1 num_sv;          // Number of satellite vehicles
    U1 reserved0[2];
    struct {
      U1 gnss_id;       // GNSS ID
      U1 sv_id;         // Satellite ID
      X1 sv_flag;       // Information flags
      X1 eph;           // Ephemeris data
      X1 alm;           // Almanac data
      X1 other_orb;     // Other orbital data
    } sv[N];            // Repeated group (num_sv times)
  } payload;
};
struct UbxNavPosecef {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_POSECEF_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U4 i_tow;   // GPS time of week, ms
    I4 ecef_x;  // ECEF x coordinate, cm
    I4 ecef_y;  // ECEF y coordinate, cm
    I4 ecef_z;  // ECEF z coordinate, cm
    U4 p_acc;   // Position accuracy estimate, cm
  } payload;
};
struct UbxNavPosllh {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_POSLLH_ID_;
  static constexpr uint16_t len = 28;
  struct {
    U4 i_tow;   // GPS time of week, ms
    I4 lon;     // Longitude, deg, scale 1e-7
    I4 lat;     // Latitude, deg, scale 1e-7
    I4 height;  // Height above ellipsoid, mm
    I4 h_msl;   // Height above MSL, mm
    U4 h_acc;   // Horizontal accuracy estimate, mm
    U4 v_acc;   // Vertical accuracy estimate, mm
  } payload;
};
struct UbxNavPvt {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_PVT_ID_;
  static constexpr uint16_t len = 92;
  struct {
    U4 i_tow;       // GPS time of week, ms
    U2 year;        // Year (UTC)
    U1 month;       // Month (UTC)
    U1 day;         // Day (UTC)
    U1 hour;        // Hour (UTC)
    U1 min;         // Minute (UTC)
    U1 sec;         // Seconds (UTC)
    X1 valid;       // Validity flags
    U4 t_acc;       // Time accuracy estimate (UTC), ns
    I4 nano;        // Fraction of second (UTC), ns
    U1 fix_type;    // GNSS fix type
    X1 flags;       // Fix status flags
    X1 flags2;      // Additional flags
    U1 num_sv;      // Number of satellites used in nav solution
    I4 lon;         // Longitude, deg, scale 1e-7
    I4 lat;         // Latitude, deg, scale 1e-7
    I4 height;      // Height above the ellipsoid, mm
    I4 h_msl;       // Height above MSL, mm
    U4 h_acc;       // Horizontal accuracy estimate, mm
    U4 v_acc;       // Vertical accuracy estimate, mm
    I4 vel_n;       // NED north velocity, mm/s
    I4 vel_e;       // NED east velocity, mm/s
    I4 vel_d;       // NED down velocity, mm/s
    I4 g_speed;     // Ground speed (2D), mm/s
    I4 head_mot;    // Heading of motion (2D), deg, scale 1e-5
    U4 s_acc;       // Speed accuracy estimate, mm/s
    U4 head_acc;    // Heading accuracy estimate, deg, scale 1e-5
    U2 p_dop;       // Position DOP, scale 0.01
    X1 flags3;      // Additional flags
    U1 reserved0[5];
    I4 head_veh;    // Heading of vehicle (2D), deg, scale 1e-5
    I2 mag_dec;     // Magnetic declination, deg, scale 1e-2
    U2 mag_acc;     // Magnetic declination accuracy, deg, scale 1e-2
  } payload;
};
struct UbxNavRelposned {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_RELPOSNED_ID_;
  static constexpr uint16_t len = 64;
  struct {
    U1 version;             // Message version
    U1 reserved0;
    U2 ref_station_id;      // Reference station ID
    U4 i_tow;               // GPS time of week, ms
    I4 rel_pos_n;           // North component of relative position vector, cm
    I4 rel_pos_e;           // East component of relative position vector, cm
    I4 rel_pos_d;           // Down component of relative position vector, cm
    I4 rel_pos_length;      // Length of the relative position vector, cm
    I4 rel_pos_heading;     // Heading of the rel pos vector, deg, scale 1e-5
    U1 reserved1[4];
    I1 rel_pos_hp_n;        // High precision N. rel pos vector, mm, scale 0.1
    I1 rel_pos_hp_e;        // High precision E. rel pos vector, mm, scale 0.1
    I1 rel_pos_hp_d;        // High precision D. rel pos vector, mm, scale 0.1
    I1 rel_pos_hp_length;   // High precision rel pos vector len, mm, scale 0.1
    U4 acc_n;               // Accuracy of N. rel pos vector, mm, scale 0.1
    U4 acc_e;               // Accuracy of E. rel pos vector, mm, scale 0.1
    U4 acc_d;               // Accuracy of D. rel pos vector, mm, scale 0.1
    U4 acc_length;          // Accuracy of len of rel pos vector, mm, scale 0.1
    U4 acc_heading;         // Accuracy of heading, deg, scale 1e-5
    U1 reserved2[4];
    X4 flags;               // Flags
  } payload;
};
struct UbxNavResetodo {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_RESETODO_ID_;
  static constexpr uint16_t len = 0;
};
template<size_t N>  // templated by maximum number of satellites
struct UbxNavSat {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_SAT_ID_;
  uint16_t len;
  struct {
    U4 i_tow;         // GPS time of week, ms
    U1 version;       // Message version
    U1 num_sv;        // Number of satellites
    U1 reserved0[2];
    struct {
      U1 gnss_id;     // GNSS identifier
      U1 sv_id;       // Satellite identifier
      U1 cno;         // Carrier to noise ratio, dBHz
      I1 elev;        // Elevation, deg
      I2 azim;        // Azimuth, deg
      I2 pr_res;      // Pseudorange residual, m, scale 0.1
      X4 flags;       // Flags
    } sv[N];          // Repeated group (num_sv times)
  } payload;
};
template<size_t N>  // templated by maximum number of satellites
struct UbxNavSbas {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_SBAS_ID_;
  uint16_t len;
  struct {
    U4 i_tow;         // GPS time of week, ms
    U1 geo;           // PRN number of the GEO where correction data is from
    U1 mode;          // SBAS mode
    I1 sys;           // SBAS system
    X1 service;       // SBAS services available
    U1 cnt;           // Number of SV data following
    U1 reserved0[3];
    struct {
      U1 sv_id;       // SV ID
      U1 flags;       // Flags for this SV
      U1 udre;        // Monitoring status
      U1 sv_sys;      // System
      U1 sv_service;  // Services available
      U1 reserved1;
      I2 prc;         // pseudorange correction, cm
      U1 reserved2[2];
      I2 ic;          // ionosphere correction, cm
    } sv[N];          // Repeated group (cnt times)
  } payload;
};
template<size_t N>  // templated by maximum number of signals
struct UbxNavSig {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_SIG_ID_;
  uint16_t len;
  struct {
    U4 i_tow;           // GPS time of week, ms
    U1 version;         // Message version
    U1 num_sigs;        // Number of signals
    U1 reserved0[2];
    struct {
      U1 gnss_id;       // GNSS identifier
      U1 sv_id;         // Satellite identifier
      U1 sig_id;        // Signal identifier
      U1 freq_id;       // GLONASS frequency slot
      I2 pr_res;        // Pseudorange residuals, m, scale 0.1
      U1 cno;           // Carrier to noise ratio, dBHz
      U1 quality_ind;   // Signal quality indicator
      U1 corr_source;   // Correction source
      U1 iono_model;    // Ionospheric model
      X2 sig_flags;     // Signal related flags
      U1 reserved1[4];
    } sig[N];           // Repeated group (num_sigs times)
  } payload;
};
template<size_t N>  // templated by maximum number of corrections
struct UbxNavSlas {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_SLAS_ID_;
  uint16_t len;
  struct {
    U4 i_tow;             // GPS time of week, ms
    U1 version;           // Message version
    U1 reserved0[3];
    I4 gms_lon;           // Longitude of ground monitoring stn, deg, scale 1e-3
    I4 gms_lat;           // Latitude of ground monitoring stn, deg, scale 1e-3
    U1 gms_code;          // Ground monitoring station code
    U1 qzss_sc_id;        // Satellite id of QZS/GEO corr data used
    X1 service_flags;     // Flags regarding SLAS service
    U1 cnt;               // Number of pseudorange corr following
    struct {
      U1 gnss_id;         // GNSS ID
      U1 sv_id;           // Satellite ID
      U1 reserved1;
      U1 reserved2[3];
      I2 prc;             // Pseudorange correction, cm
    } corr[N];
  } payload;
};
struct UbxNavStatus {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_STATUS_ID_;
  static constexpr uint16_t len = 16;
  struct {
    U4 i_tow;       // GPS time of week, ms
    U1 gps_fix;     // GPS fix type
    X1 flags;       // Navigation status flags
    X1 fix_stat;    // Fix status info
    X1 flags2;      // Further info about nav output
    U4 ttff;        // Time to first fix, ms
    U4 msss;        // Milliseconds since startup, ms
  } payload;
};
struct UbxNavSvin {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_SVIN_ID_;
  static constexpr uint16_t len = 40;
  struct {
    U1 version;       // Message version
    U1 reserved0[3];
    U4 i_tow;         // GPS time of week, ms
    U4 dur;           // Passed survey-in observation time, s
    I4 mean_x;        // Current mean ECEF x, cm
    I4 mean_y;        // Current mean ECEF y, cm
    I4 mean_z;        // Current mean ECEF z, cm
    I1 mean_x_hp;     // High precision ECEF x, mm, scale 0.1
    I1 mean_y_hp;     // High precision ECEF y, mm, scale 0.1
    I1 mean_z_hp;     // High precision ECEF z, mm, scale 0.1
    U1 reserved1;
    U4 mean_acc;      // Current survey-in accuracy, mm, scale 0.1
    U4 obs;           // Number of position observations used during survey
    U1 valid;         // Survey-in position validity
    U1 active;        // Survey-in progress flag
    U1 reserved2[2];
  } payload;
};
struct UbxNavTimebds {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_TIMEBDS_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U4 i_tow;     // GPS time of week, ms
    U4 sow;       // BDS time of week, s
    I4 f_sow;     // Fractional part of SOW, ns
    I2 week;      // BDS week number
    I1 leap_s;    // BDS leap seconds
    X1 valid;     // Validity flags
    U4 t_acc;     // Time accuracy estimate, ns
  } payload;
};
struct UbxNavTimegal {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_TIMEGAL_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U4 i_tow;       // GPS time of week, ms
    U4 gal_tow;     // Galileo time of week, s
    I4 f_gal_tow;   // Fractional part of gal_tow, ns
    I2 gal_wno;     // Galileo week number
    I1 leap_s;      // Galileo leap seconds
    X1 valid;       // Validity flags
    U4 t_acc;       // Time accuracy estimate, ns
  } payload;
};
struct UbxNavTimeglo {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_TIMEGLO_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U4 i_tow;       // GPS time of week, ms
    U4 tod;         // GLONASS time of day, s
    I4 f_tod;       // Fractional part of GLONASS, ns
    U2 nt;          // Current date starting @ 1 from 1st Jan of n4
    U1 n4;          // Four year interval number
    X1 valid;       // Validity flags
    U4 t_acc;       // Time accuracy estimate, ns
  } payload;
};
struct UbxNavTimegps {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_TIMEGPS_ID_;
  static constexpr uint16_t len = 16;
  struct {
    U4 i_tow;   // GPS time of week, ms
    I4 f_tow;   // Fractional part of TOW, ns
    I2 week;    // GPS week number
    I1 leap_s;  // GPS leap seconds
    X1 valid;   // Validity flags
    U4 t_acc;   // Time accuracy estimate, ns
  } payload;
};
struct UbxNavTimels {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_TIMELS_ID_;
  static constexpr uint16_t len = 24;
  struct {
    U4 i_tow;               // GPS time of week, ms
    U1 version;             // Message version
    U1 reserved0[3];
    U1 src_of_curr_ls;      // Info src for current num of leap sec
    I1 curr_ls;             // Current number of leap sec
    U1 src_of_ls_change;    // Info src for future leap sec
    I1 ls_change;           // Future leap sec change
    I4 time_to_ls_event;    // Number of seconds until leap sec change
    U2 date_of_ls_gps_wn;   // GPS week number of leap sec change
    U2 date_of_ls_gps_dn;   // GPS day of week of leap sec change
    U1 reserved1[3];
    X1 valid;               // Validity flags
  } payload;
};
struct UbxNavTimeqzss {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_TIMEQZSS_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U4 i_tow;       // GPS time of week, ms
    U4 qzss_tow;    // QZSS time of week, s
    I4 f_qzss_tow;  // Fractional part of QZSS TOW, ns
    I2 qzss_wno;    // QZSS week number
    I1 leap_s;      // QZSS leap seconds, s
    X1 valid;       // Validity flags
    U4 t_acc;       // Time accuracy estimate, ns
  } payload;
};
struct UbxNavTimeutc {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_TIMEUTC_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U4 i_tow;       // GPS time of week, ms
    U4 t_acc;       // Time accuracy estimate (UTC), ns
    I4 nano;        // Fraction of a second (UTC), ns
    U2 year;        // Year (UTC)
    U1 month;       // Month (UTC)
    U1 day;         // Day (UTC)
    U1 hour;        // Hour (UTC)
    U1 min;         // Minute (UTC)
    U1 sec;         // Seconds (UTC)
    X1 valid;       // Validity flags
  } payload;
};
struct UbxNavVelecef {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_VELECEF_ID_;
  static constexpr uint16_t len = 20;
  struct {
    U4 i_tow;     // GPS time of week, ms
    I4 ecef_v_x;  // ECEF x velocity, cm/s
    I4 ecef_v_y;  // ECEF y velocity, cm/s
    I4 ecef_v_z;  // ECEF z velocity, cm/s
    U4 s_acc;     // Speed accuracy estimate, cm/s
  } payload;
};
struct UbxNavVelned {
  static constexpr uint8_t cls = UBX_NAV_CLS_;
  static constexpr uint8_t id = UBX_NAV_VELNED_ID_;
  static constexpr uint16_t len = 36;
  struct {
    U4 i_tow;     // GPS time of week, ms
    I4 vel_n;     // North velocity component, cm/s
    I4 vel_e;     // East velocity component, cm/s
    I4 vel_d;     // Down velocity component, cm/s
    U4 speed;     // Speed (3D), cm/s
    U4 g_speed;   // Ground speed (2D), cm/s
    I4 heading;   // Heading of motion (2D), deg, scale 1e-5
    U4 s_acc;     // Speed accuracy estimate, cm/s
    U4 c_acc;     // Course/Heading accuracy estimate, deg, scale 1e-5
  } payload;
};

}  // namespace bfs

#endif  // SRC_UBX_NAV_H_
