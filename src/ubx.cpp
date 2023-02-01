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

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "ubx.h"
#include "ubx_defs.h"  // NOLINT
#include "ubx_nav.h"  // NOLINT

namespace bfs {

constexpr uint8_t Ubx::UBX_HEADER_[];
void Ubx::Config(HardwareSerial* bus) {
  bus_ = bus;
}
bool Ubx::Begin(int32_t baud) {
  bus_->begin(baud);
  bus_->flush();
  while (comm_timeout_count_++ < COMM_TIMEOUT_TRIES_) {
    if (ParseMsg()) {
      return true;
    }
    delay(COMM_TIMEOUT_DELAY_MS_);
  }
  return false;
}
bool Ubx::Read() {
  while (bus_->available()) {
    if (ParseMsg()) {
      if (rx_msg_.cls == UBX_NAV_CLS_) {
        switch (rx_msg_.id) {
          case UBX_NAV_POSECEF_ID_: {
            if (rx_msg_.len == ubx_nav_pos_ecef_.len) {
              memcpy(&ubx_nav_pos_ecef_.payload, rx_msg_.payload, rx_msg_.len);
            }
            break;
          }
          case UBX_NAV_PVT_ID_: {
            if (rx_msg_.len == ubx_nav_pvt_.len) {
              memcpy(&ubx_nav_pvt_.payload, rx_msg_.payload, rx_msg_.len);
            }
            break;
          }
          case UBX_NAV_DOP_ID_: {
            if (rx_msg_.len == ubx_nav_dop_.len) {
              memcpy(&ubx_nav_dop_.payload, rx_msg_.payload, rx_msg_.len);
            }
            break;
          }
          case UBX_NAV_VELECEF_ID_: {
            if (rx_msg_.len == ubx_nav_vel_ecef_.len) {
              memcpy(&ubx_nav_vel_ecef_.payload, rx_msg_.payload, rx_msg_.len);
            }
            break;
          }
          case UBX_NAV_HPPOSECEF_ID_: {
            if (rx_msg_.len == ubx_nav_hp_pos_ecef_.len) {
              memcpy(&ubx_nav_hp_pos_ecef_.payload, rx_msg_.payload,
                     rx_msg_.len);
              use_hp_pos_ = true;
            }
            break;
          }
          case UBX_NAV_HPPOSLLH_ID_: {
            if (rx_msg_.len == ubx_nav_hp_pos_llh_.len) {
              memcpy(&ubx_nav_hp_pos_llh_.payload, rx_msg_.payload,
                     rx_msg_.len);
              use_hp_pos_ = true;
            }
            break;
          }
          case UBX_NAV_TIMEGPS_ID_: {
            if (rx_msg_.len == ubx_nav_time_gps_.len) {
              memcpy(&ubx_nav_time_gps_.payload, rx_msg_.payload, rx_msg_.len);
            }
            break;
          }
          case UBX_NAV_SVIN_ID_: {
            if (rx_msg_.len == ubx_nav_svin_.len) {
              memcpy(&ubx_nav_svin_.payload, rx_msg_.payload, rx_msg_.len);
              svin_data_ = true;
            }
            break;
          }
          case UBX_NAV_RELPOSNED_ID_: {
            if (rx_msg_.len == ubx_nav_rel_pos_ned_.len) {
              memcpy(&ubx_nav_rel_pos_ned_.payload, rx_msg_.payload,
                     rx_msg_.len);
              rel_pos_data_ = true;
            }
            break;
          }
          case UBX_NAV_EOE_ID_: {
            if (rx_msg_.len == ubx_nav_eoe_.len) {
              memcpy(&ubx_nav_eoe_.payload, rx_msg_.payload, rx_msg_.len);
              eoe_ = true;
            }
            break;
          }
          default: {
            break;
          }
        }
      }
    }
  }
  if (eoe_) {
    eoe_ = false;
    ProcessNavData();
    return true;
  }
  return false;
}
void Ubx::ProcessNavData() {
  /* Fix */
  gnss_fix_ok_ = ubx_nav_pvt_.payload.flags & 0x01;
  diff_soln_ = ubx_nav_pvt_.payload.flags & 0x02;
  carr_soln_ = ubx_nav_pvt_.payload.flags >> 6;
  if (gnss_fix_ok_) {
    switch (ubx_nav_pvt_.payload.fix_type) {
      case 2: {
        fix_ = FIX_2D;
        break;
      }
      case 3: {
        fix_ = FIX_3D;
        if (diff_soln_) {
          fix_ = FIX_DGNSS;
        }
        if (carr_soln_ == 1) {
          fix_ = FIX_RTK_FLOAT;
        }
        if (carr_soln_ == 2) {
          fix_ = FIX_RTK_FIXED;
        }
        break;
      }
      default: {
        fix_ = FIX_NONE;
        break;
      }
    }
  } else {
    fix_ = FIX_NONE;
  }
  /* Number of satellites */
  num_sv_ = ubx_nav_pvt_.payload.num_sv;
  /* Date and time */
  valid_date_ = ubx_nav_pvt_.payload.valid & 0x01;
  valid_time_ = ubx_nav_pvt_.payload.valid & 0x02;
  fully_resolved_ = ubx_nav_pvt_.payload.valid & 0x04;
  validity_confirmed_ = ubx_nav_pvt_.payload.flags2 & 0x20;
  confirmed_date_ = ubx_nav_pvt_.payload.flags2 & 0x40;
  confirmed_time_ = ubx_nav_pvt_.payload.flags2 & 0x80;
  valid_time_and_date_ = valid_date_ && valid_time_ && fully_resolved_ &&
                         validity_confirmed_ && confirmed_date_ &&
                         confirmed_time_;
  if (valid_time_and_date_) {
    year_ = ubx_nav_pvt_.payload.year;
    month_ = ubx_nav_pvt_.payload.month;
    day_ = ubx_nav_pvt_.payload.day;
    hour_ = ubx_nav_pvt_.payload.hour;
    min_ = ubx_nav_pvt_.payload.min;
    sec_ = ubx_nav_pvt_.payload.sec;
    nano_ = ubx_nav_pvt_.payload.nano;
  } else {
    year_ = 0;
    month_ = 0;
    day_ = 0;
    hour_ = 0;
    min_ = 0;
    sec_ = 0;
    nano_ = 0;
  }
  t_acc_ns_ = ubx_nav_pvt_.payload.t_acc;
  /* GPS time */
  tow_valid_ = ubx_nav_time_gps_.payload.valid & 0x01;
  week_valid_ = ubx_nav_time_gps_.payload.valid & 0x02;
  leap_valid_ = ubx_nav_time_gps_.payload.valid & 0x04;
  if (tow_valid_) {
    tow_s_ = static_cast<double>(ubx_nav_time_gps_.payload.i_tow) * 1e-3 +
             static_cast<double>(ubx_nav_time_gps_.payload.f_tow) * 1e-9;
  } else {
    tow_s_ = 0;
  }
  if (week_valid_) {
    week_ = ubx_nav_time_gps_.payload.week;
  } else {
    week_ = 0;
  }
  if (leap_valid_) {
    leap_s_ = ubx_nav_time_gps_.payload.leap_s;
  } else {
    leap_s_ = 0;
  }
  /* DOP */
  gdop_ = static_cast<float>(ubx_nav_dop_.payload.g_dop) * 0.01f;
  pdop_ = static_cast<float>(ubx_nav_dop_.payload.p_dop) * 0.01f;
  tdop_ = static_cast<float>(ubx_nav_dop_.payload.t_dop) * 0.01f;
  vdop_ = static_cast<float>(ubx_nav_dop_.payload.v_dop) * 0.01f;
  hdop_ = static_cast<float>(ubx_nav_dop_.payload.h_dop) * 0.01f;
  ndop_ = static_cast<float>(ubx_nav_dop_.payload.n_dop) * 0.01f;
  edop_ = static_cast<float>(ubx_nav_dop_.payload.e_dop) * 0.01f;
  /* NED velocity */
  ned_vel_mps_[0] = static_cast<float>(ubx_nav_pvt_.payload.vel_n) / 1000.0f;
  ned_vel_mps_[1] = static_cast<float>(ubx_nav_pvt_.payload.vel_e) / 1000.0f;
  ned_vel_mps_[2] = static_cast<float>(ubx_nav_pvt_.payload.vel_d) / 1000.0f;
  s_acc_mps_ = static_cast<float>(ubx_nav_pvt_.payload.s_acc) / 1000.0f;
  /* ECEF velocity */
  ecef_vel_mps_[0] = static_cast<float>(ubx_nav_vel_ecef_.payload.ecef_v_x) /
                     100.0f;
  ecef_vel_mps_[1] = static_cast<float>(ubx_nav_vel_ecef_.payload.ecef_v_y) /
                     100.0f;
  ecef_vel_mps_[2] = static_cast<float>(ubx_nav_vel_ecef_.payload.ecef_v_z) /
                     100.0f;
  /* Ground track and speed */
  gnd_spd_mps_ = static_cast<float>(ubx_nav_pvt_.payload.g_speed) / 1000.0f;
  track_deg_ = static_cast<float>(ubx_nav_pvt_.payload.head_mot) / 100000.0f;
  track_acc_deg_ = static_cast<float>(ubx_nav_pvt_.payload.head_acc) /
                   100000.0f;
  /* LLH position */
  invalid_llh_ = ubx_nav_pvt_.payload.flags3 & 0x01;
  if (!invalid_llh_) {
    if (use_hp_pos_) {
      llh_[0] = (static_cast<double>(ubx_nav_hp_pos_llh_.payload.lat) +
                 static_cast<double>(ubx_nav_hp_pos_llh_.payload.lat_hp) *
                 1e-2) * 1e-7;
      llh_[1] = (static_cast<double>(ubx_nav_hp_pos_llh_.payload.lon) +
                 static_cast<double>(ubx_nav_hp_pos_llh_.payload.lon_hp) *
                 1e-2) * 1e-7;
      llh_[2] = (static_cast<double>(ubx_nav_hp_pos_llh_.payload.height) +
                 static_cast<double>(ubx_nav_hp_pos_llh_.payload.height_hp) *
                 0.1) * 1e-3;
      alt_msl_m_ = (static_cast<float>(ubx_nav_hp_pos_llh_.payload.h_msl) +
                    static_cast<float>(ubx_nav_hp_pos_llh_.payload.h_msl_hp) *
                    0.1f) / 1000.0f;
      h_acc_m_ = static_cast<float>(ubx_nav_hp_pos_llh_.payload.h_acc) /
                 10000.0f;
      v_acc_m_ = static_cast<float>(ubx_nav_hp_pos_llh_.payload.v_acc) /
                 10000.0f;
    } else {
      llh_[0] = static_cast<double>(ubx_nav_pvt_.payload.lat) * 1e-7;
      llh_[1] = static_cast<double>(ubx_nav_pvt_.payload.lon) * 1e-7;
      llh_[2] = static_cast<double>(ubx_nav_pvt_.payload.height) * 1e-3;
      alt_msl_m_ = static_cast<float>(ubx_nav_pvt_.payload.h_msl) / 1000.0f;
      h_acc_m_ = static_cast<float>(ubx_nav_pvt_.payload.h_acc) / 1000.0f;
      v_acc_m_ = static_cast<float>(ubx_nav_pvt_.payload.v_acc) / 1000.0f;
    }
  }
  /* ECEF position */
  invalid_ecef_ = ubx_nav_hp_pos_ecef_.payload.flags & 0x01;
  if (!invalid_ecef_) {
    if (use_hp_pos_) {
      ecef_m_[0] = (static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_x) +
                    static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_x_hp)
                    * 1e-2) * 1e-2;
      ecef_m_[1] = (static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_y) +
                    static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_y_hp)
                    * 1e-2) * 1e-2;
      ecef_m_[2] = (static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_z) +
                    static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_z_hp)
                    * 1e-2) * 1e-2;
      p_acc_m_ = static_cast<float>(ubx_nav_hp_pos_ecef_.payload.p_acc) /
                 10000.0f;
    } else {
      ecef_m_[0] = static_cast<double>(ubx_nav_pos_ecef_.payload.ecef_x) * 1e-2;
      ecef_m_[1] = static_cast<double>(ubx_nav_pos_ecef_.payload.ecef_y) * 1e-2;
      ecef_m_[2] = static_cast<double>(ubx_nav_pos_ecef_.payload.ecef_z) * 1e-2;
      p_acc_m_ = static_cast<float>(ubx_nav_pos_ecef_.payload.p_acc) / 100.0f;
    }
  }
  /* Relative position */
  if (rel_pos_data_) {
    rel_pos_avail_ = ubx_nav_rel_pos_ned_.payload.flags & 0x04;
    rel_pos_moving_baseline_ = ubx_nav_rel_pos_ned_.payload.flags & 0x20;
    rel_pos_ref_pos_miss_ = ubx_nav_rel_pos_ned_.payload.flags & 0x40;
    rel_pos_ref_obs_miss_ = ubx_nav_rel_pos_ned_.payload.flags & 0x80;
    rel_pos_heading_valid_ = ubx_nav_rel_pos_ned_.payload.flags & 0x100;
    rel_pos_norm_ = ubx_nav_rel_pos_ned_.payload.flags & 0x200;
    if (rel_pos_avail_) {
      rel_pos_ned_m_[0] =
        (static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_n) +
        static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_hp_n) * 1e-2) *
        1e-2;
      rel_pos_ned_m_[1] =
        (static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_e) +
        static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_hp_e) * 1e-2) *
        1e-2;
      rel_pos_ned_m_[2] =
        (static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_d) +
        static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_hp_d) * 1e-2) *
        1e-2;
      rel_pos_len_m_ =
        (static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_length) +
        static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_hp_length) *
        1e-2) * 1e-2;
      rel_pos_heading_deg_ =
        static_cast<float>(ubx_nav_rel_pos_ned_.payload.rel_pos_heading) /
        100000.0f;
      rel_pos_ned_acc_m_[0] =
        static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_n) / 10000.0f;
      rel_pos_ned_acc_m_[1] =
        static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_e) / 10000.0f;
      rel_pos_ned_acc_m_[2] =
        static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_d) / 10000.0f;
      rel_pos_len_acc_m_ =
        static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_length) / 10000.0f;
      rel_pos_heading_acc_deg_ =
        static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_heading) /
        100000.0f;
    }
  }
  /* Survey in data */
  if (svin_data_) {
    svin_dur_s_ = ubx_nav_svin_.payload.dur;
    svin_ecef_m_[0] = (static_cast<double>(ubx_nav_svin_.payload.mean_x) +
                      static_cast<double>(ubx_nav_svin_.payload.mean_x_hp)
                      * 1e-2) * 1e-2;
    svin_ecef_m_[1] = (static_cast<double>(ubx_nav_svin_.payload.mean_y) +
                      static_cast<double>(ubx_nav_svin_.payload.mean_y_hp)
                      * 1e-2) * 1e-2;
    svin_ecef_m_[2] = (static_cast<double>(ubx_nav_svin_.payload.mean_z) +
                      static_cast<double>(ubx_nav_svin_.payload.mean_z_hp)
                      * 1e-2) * 1e-2;
    svin_acc_m_ = static_cast<float>(ubx_nav_svin_.payload.mean_acc) /
                  10000.0f;
    svin_valid_ = ubx_nav_svin_.payload.valid;
    svin_in_progress_ = ubx_nav_svin_.payload.active;
    svin_num_obs_ = ubx_nav_svin_.payload.obs;
  }
}
bool Ubx::ParseMsg() {
  while (bus_->available()) {
    c_ = bus_->read();
    /* Packet header */
    if (parser_state_ < sizeof(UBX_HEADER_)) {
      if (c_ == UBX_HEADER_[parser_state_]) {
        parser_state_++;
      } else {
        parser_state_ = 0;
      }
    /* Class */
    } else if (parser_state_ == UBX_CLS_POS_) {
      rx_msg_.cls = c_;
      parser_state_++;
    /* ID */
    } else if (parser_state_ == UBX_ID_POS_) {
      rx_msg_.id = c_;
      parser_state_++;
    /* Length */
    } else if (parser_state_ == UBX_LEN_POS_LSB_) {
      len_ = c_;
      parser_state_++;
    } else if (parser_state_ == UBX_LEN_POS_MSB_) {
      rx_msg_.len = static_cast<uint16_t>(c_) << 8 | len_;
      parser_state_++;
      /* Prevent buffer overflow */
      if (rx_msg_.len > UBX_MAX_PAYLOAD_) {
        parser_state_ = 0;
      }
    /* Payload */
    } else if (parser_state_ < (rx_msg_.len + UBX_HEADER_LEN_)) {
      rx_msg_.payload[parser_state_ - UBX_HEADER_LEN_] = c_;
      parser_state_++;
    /* Checksum */
    } else if (parser_state_ == (rx_msg_.len + UBX_HEADER_LEN_)) {
      chk_rx_ = c_;
      parser_state_++;
    } else {
      chk_cmp_rx_ =
                 chksum_rx_.Compute(reinterpret_cast<const uint8_t *>(&rx_msg_),
                                    rx_msg_.len + UBX_CHK_OFFSET_);
      parser_state_ = 0;
      /* Check the computed & received checksums */
      if (chk_cmp_rx_ == (static_cast<uint16_t>(c_) << 8 | chk_rx_)) {
        /* Valid message */
        return true;
      }
    }
  }
  return false;
}

}  // namespace bfs
