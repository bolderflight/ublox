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

#ifndef UBX_SRC_UBX_H_  // NOLINT
#define UBX_SRC_UBX_H_

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "ubx_defs.h"  // NOLINT
#include "ubx_nav.h"  // NOLINT

namespace bfs {

class Ubx {
 public:
  enum Fix : int8_t {
    FIX_NONE = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_DGNSS = 4,
    FIX_RTK_FLOAT = 5,
    FIX_RTK_FIXED = 6
  };
  Ubx() {}
  explicit Ubx(HardwareSerial* bus) : bus_(bus) {}
  void Config(HardwareSerial* bus);
  /* Standard begin, set the baud and test for comms */
  bool Begin(const int32_t baud);
  /* Reads NAV data and returns true on EOE */
  bool Read();
  /* Data output */
  inline Fix fix() const {return fix_;}
  inline int8_t num_sv() const {return num_sv_;}
  inline int16_t utc_year() const {return year_;}
  inline int8_t utc_month() const {return month_;}
  inline int8_t utc_day() const {return day_;}
  inline int8_t utc_hour() const {return hour_;}
  inline int8_t utc_min() const {return min_;}
  inline int8_t utc_sec() const {return sec_;}
  inline int32_t utc_nano() const {return nano_;}
  inline double gps_tow_s() const {return tow_s_;}
  inline int16_t gps_week() const {return week_;}
  inline int8_t leap_s() const {return leap_s_;}
  inline uint32_t time_acc_ns() const {return t_acc_ns_;}
  inline float north_vel_mps() const {return ned_vel_mps_[0];}
  inline float east_vel_mps() const {return ned_vel_mps_[1];}
  inline float down_vel_mps() const {return ned_vel_mps_[2];}
  inline float gnd_spd_mps() const {return gnd_spd_mps_;}
  inline float ecef_vel_x_mps() const {return ecef_vel_mps_[0];}
  inline float ecef_vel_y_mps() const {return ecef_vel_mps_[1];}
  inline float ecef_vel_z_mps() const {return ecef_vel_mps_[2];}
  inline float spd_acc_mps() const {return s_acc_mps_;}
  inline float track_deg() const {return track_deg_;}
  inline float track_rad() const {return track_deg_ * DEG2RADf_;}
  inline float track_acc_deg() const {return track_acc_deg_;}
  inline float track_acc_rad() const {return track_acc_deg_ * DEG2RADf_;}
  inline double lat_deg() const {return llh_[0];}
  inline double lat_rad() const {return llh_[0] * DEG2RADl_;}
  inline double lon_deg() const {return llh_[1];}
  inline double lon_rad() const {return llh_[1] * DEG2RADl_;}
  inline float alt_wgs84_m() const {return static_cast<float>(llh_[2]);}
  inline float alt_msl_m() const {return alt_msl_m_;}
  inline float horz_acc_m() const {return h_acc_m_;}
  inline float vert_acc_m() const {return v_acc_m_;}
  inline double ecef_pos_x_m() const {return ecef_m_[0];}
  inline double ecef_pos_y_m() const {return ecef_m_[1];}
  inline double ecef_pos_z_m() const {return ecef_m_[2];}
  inline float ecef_pos_acc_m() const {return p_acc_m_;}
  inline float gdop() const {return gdop_;}
  inline float pdop() const {return pdop_;}
  inline float tdop() const {return tdop_;}
  inline float vdop() const {return vdop_;}
  inline float hdop() const {return hdop_;}
  inline float ndop() const {return ndop_;}
  inline float edop() const {return edop_;}
  /* Relative position data */
  inline bool rel_pos_avail() const {return rel_pos_avail_;}
  inline bool rel_pos_moving_baseline() const {return rel_pos_moving_baseline_;}
  inline bool rel_pos_ref_pos_miss() const {return rel_pos_ref_pos_miss_;}
  inline bool rel_pos_ref_obs_miss() const {return rel_pos_ref_obs_miss_;}
  inline bool rel_pos_heading_valid() const {return rel_pos_heading_valid_;}
  inline bool rel_pos_normalized() const {return rel_pos_norm_;}
  inline double rel_pos_north_m() const {return rel_pos_ned_m_[0];}
  inline double rel_pos_east_m() const {return rel_pos_ned_m_[1];}
  inline double rel_pos_down_m() const {return rel_pos_ned_m_[2];}
  inline float rel_pos_acc_north_m() const {return rel_pos_ned_acc_m_[0];}
  inline float rel_pos_acc_east_m() const {return rel_pos_ned_acc_m_[1];}
  inline float rel_pos_acc_down_m() const {return rel_pos_ned_acc_m_[2];}
  inline double rel_pos_len_m() const {return rel_pos_len_m_;}
  inline float rel_pos_len_acc_m() const {return rel_pos_len_acc_m_;}
  inline float rel_pos_heading_deg() const {return rel_pos_heading_deg_;}
  inline float rel_pos_heading_acc_deg() const {
    return rel_pos_heading_acc_deg_;
  }
  inline float rel_pos_heading_rad() const {
    return rel_pos_heading_deg_ * DEG2RADf_;
  }
  inline float rel_pos_heading_acc_rad() const {
    return rel_pos_heading_acc_deg_ * DEG2RADf_;
  }
  /* Survey-in data */
  inline bool svin_valid() const {return svin_valid_;}
  inline bool svin_in_progress() const {return svin_in_progress_;}
  inline uint32_t svin_dur_s() const {return svin_dur_s_;}
  inline double svin_ecef_pos_x_m() const {return svin_ecef_m_[0];}
  inline double svin_ecef_pos_y_m() const {return svin_ecef_m_[1];}
  inline double svin_ecef_pos_z_m() const {return svin_ecef_m_[2];}
  inline float svin_ecef_pos_acc_m() const {return svin_acc_m_;}
  inline uint32_t svin_num_obs() const {return svin_num_obs_;}

 private:
  /* Parse messages, return true on valid msg received */
  bool ParseMsg();
  /* Process nav data */
  void ProcessNavData();
  /* Communication */
  HardwareSerial* bus_;
  int16_t comm_timeout_count_ = 0; 
  static const int16_t COMM_TIMEOUT_TRIES_ = 1000;
  static const int16_t COMM_TIMEOUT_DELAY_MS_ = 10; 
  /* Max payload bytes supported */
  static constexpr size_t UBX_MAX_PAYLOAD_ = 128;
  /* Parsing */
  static constexpr uint8_t UBX_HEADER_[2] = {0xB5, 0x62};
  static constexpr uint8_t UBX_CLS_POS_ = 2;
  static constexpr uint8_t UBX_ID_POS_ = 3;
  static constexpr uint8_t UBX_LEN_POS_LSB_ = 4;
  static constexpr uint8_t UBX_LEN_POS_MSB_ = 5;
  static constexpr uint8_t UBX_HEADER_LEN_ = 6;
  static constexpr uint8_t UBX_CHK_OFFSET_ = UBX_HEADER_LEN_ -
                                             sizeof(UBX_HEADER_);
  uint8_t c_, len_, chk_rx_, chk_[2];
  uint16_t chk_cmp_rx_, chk_cmp_tx_;
  size_t parser_state_ = 0;
  static constexpr float DEG2RADf_ = 3.14159265358979323846264338327950288f /
                                     180.0f;
  static constexpr double DEG2RADl_ = 3.14159265358979323846264338327950288 /
                                      180.0;
  /* Data members */
  bool eoe_ = false;
  bool use_hp_pos_ = false;
  bool svin_data_ = false;
  bool rel_pos_data_ = false;
  Fix fix_;
  bool gnss_fix_ok_, diff_soln_;
  bool valid_date_, valid_time_, fully_resolved_, validity_confirmed_;
  bool tow_valid_, week_valid_, leap_valid_;
  bool confirmed_date_, confirmed_time_, valid_time_and_date_;
  bool invalid_llh_, invalid_ecef_;
  bool rel_pos_avail_ = false;
  bool rel_pos_moving_baseline_ = false;
  bool rel_pos_ref_pos_miss_ = false;
  bool rel_pos_ref_obs_miss_ = false;
  bool rel_pos_heading_valid_ = false;
  bool rel_pos_norm_ = false;
  bool svin_valid_ = false;
  bool svin_in_progress_ = false;
  int8_t carr_soln_;
  int8_t num_sv_;
  int8_t month_, day_, hour_, min_, sec_;
  int8_t leap_s_;
  int16_t year_;
  int16_t week_;
  int32_t nano_;
  uint32_t t_acc_ns_;
  uint32_t svin_dur_s_, svin_num_obs_;
  float alt_msl_m_;
  float gnd_spd_mps_;
  float track_deg_;
  float gdop_, pdop_, tdop_, vdop_, hdop_, ndop_, edop_;
  float h_acc_m_, v_acc_m_, p_acc_m_, track_acc_deg_, s_acc_mps_;
  float rel_pos_heading_deg_;
  float rel_pos_heading_acc_deg_;
  float rel_pos_len_acc_m_;
  float svin_acc_m_;
  double rel_pos_len_m_;
  double tow_s_;
  float ecef_vel_mps_[3];
  float ned_vel_mps_[3];
  float rel_pos_ned_acc_m_[3];
  double ecef_m_[3];
  double llh_[3];
  double rel_pos_ned_m_[3];
  double svin_ecef_m_[3];
  /* Class to compute UBX checksum */
  class Checksum {
   public:
    uint16_t Compute(uint8_t const * const data, const size_t len) {
      if (!data) {
        return 0;
      }
      sum0_ = 0;
      sum1_ = 0;
      for (size_t i = 0; i < len; i++) {
        sum0_ += data[i];
        sum1_ += sum0_;
      }
      return static_cast<uint16_t>(sum1_) << 8 | sum0_;
    }
    uint16_t Update(uint8_t const * const data, const size_t len) {
      if (!data) {
        return 0;
      }
      for (size_t i = 0; i < len; i++) {
        sum0_ += data[i];
        sum1_ += sum0_;
      }
      return static_cast<uint16_t>(sum1_) << 8 | sum0_;
    }

   private:
    uint8_t sum0_, sum1_;
  } chksum_rx_, chksum_tx_;
  /* Struct for unkown messages */
  struct UbxMsg {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint8_t payload[UBX_MAX_PAYLOAD_];
  } rx_msg_;
  /* Data messages */
  UbxNavDop ubx_nav_dop_;
  UbxNavEoe ubx_nav_eoe_;
  UbxNavHpposecef ubx_nav_hp_pos_ecef_;
  UbxNavHpposllh ubx_nav_hp_pos_llh_;
  UbxNavPosecef ubx_nav_pos_ecef_;
  UbxNavRelposned ubx_nav_rel_pos_ned_;
  UbxNavVelecef ubx_nav_vel_ecef_;
  UbxNavPvt ubx_nav_pvt_;
  UbxNavTimegps ubx_nav_time_gps_;
  UbxNavSvin ubx_nav_svin_;
};

}  // namespace bfs

#endif  // UBX_SRC_UBX_H_ NOLINT
