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
#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "units/units.h"
#include "gnss/gnss.h"

namespace bfs {
namespace {
int GnssWeek(int year, int month, int day) {
  static const int month_day[2][12] = {
    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
    {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
  };
  static const int JAN61980 = 44244;
  static const int JAN11901 = 15385;
  int yday, mjd, leap;
  leap = (year % 4 == 0);
  yday = month_day[leap][month - 1] + day;
  mjd = ((year - 1901) / 4) * 1461 + ((year - 1901) % 4) * 365 + yday - 1 +
        JAN11901;
  return (mjd - JAN61980) / 7;
}
}  // namespace


bool Ublox::Init(const GnssConfig &ref) {
  /* Copy the config */
  config_ = ref;
  /* Initialize communication */
  config_.bus->begin(config_.baud);
  /* Read from one EOE to another EOE to see what packets are here */
  bool start = false;
  bool init_status = false;
  config_.bus->flush();
  elapsedMillis timer_ms = 0;
  while (timer_ms < TIMEOUT_MS_) {
    if (Parse()) {
      switch (msg_) {
        case UBX_NAV_DOP: {
          ubx_nav_dop_parsed_ = true;
          break;
        }
        case UBX_NAV_PVT: {
          ubx_nav_pvt_parsed_ = true;
          break;
        }
        case UBX_NAV_HPPOSLLH: {
          use_high_precision_ = true;
          break;
        }
        case UBX_NAV_EOE: {
          if (!start) {
            start = true;
          } else {
            /* Need at least UBX-NAV-DOP and UBX-NAV-PVT */
            init_status = ubx_nav_dop_parsed_ && ubx_nav_pvt_parsed_;
            /* Reset states */
            ubx_nav_dop_parsed_ = false;
            ubx_nav_pvt_parsed_ = false;
            ubx_nav_hpposllh_parsed_ = false;
            read_status_ = false;
          }
        }
      }
    }
  }
  /* Health determination */
  health_period_ms_ = 5 * config_.sampling_period_ms;
  health_timer_ms_ = 0;
  return init_status;
}
bool Ublox::Read(GnssData * const ptr) {
  /* Read through all available packets to get the newest */
  ptr->new_data = false;
  do {
    if (Epoch()) {
      ptr->new_data = true;
    }
  } while (config_.bus->available());
  /* Health status */
  ptr->healthy = (health_timer_ms_ < health_period_ms_);
  /* Parse new data, if available */
  if (ptr->new_data) {
    health_timer_ms_ = 0;
    /* GNSS fix */
    bool valid_fix = ubx_nav_pvt_.flags & 0x01;
    bool dgnss = ubx_nav_pvt_.flags & 0x02;
    uint8_t rtk = ubx_nav_pvt_.flags >> 6;
    if (valid_fix) {
      switch (ubx_nav_pvt_.fix) {
        case 2: {
          ptr->fix = GnssFix::FIX_2D;
          break;
        }
        case 3: {
          ptr->fix = GnssFix::FIX_3D;
          if (dgnss) {
            ptr->fix = GnssFix::FIX_DGNSS;
          }
          if (rtk == 1) {
            ptr->fix = GnssFix::FIX_RTK_FLOAT;
          }
          if (rtk == 2) {
            ptr->fix == GnssFix::FIX_RTK_FIXED;
          }
          break;
        }
        default: {
          ptr->fix = GnssFix::FIX_NONE;
          break;
        }
      }
    } else {
      ptr->fix = GnssFix::FIX_NONE;
    }
    ptr->num_sats = ubx_nav_pvt_.numsv;
    /* Date and time */
    bool valid_date = ubx_nav_pvt_.valid & 0x01;
    bool valid_time = ubx_nav_pvt_.valid & 0x02;
    bool fully_resolved = ubx_nav_pvt_.valid & 0x04;
    bool validity_confirmed = ubx_nav_pvt_.flags2 & 0x20;
    bool confirmed_date = ubx_nav_pvt_.flags2 & 0x40;
    bool confirmed_time = ubx_nav_pvt_.flags2 & 0x80;
    bool valid_time_and_date = valid_date && valid_time && fully_resolved &&
                               validity_confirmed && confirmed_date &&
                               confirmed_time;
    if (valid_time_and_date) {
      ptr->tow_ms = ubx_nav_pvt_.itow;
      ptr->week = GnssWeek(ubx_nav_pvt_.year, ubx_nav_pvt_.month,
                           ubx_nav_pvt_.day);
    } else {
      ptr->tow_ms = 0;
      ptr->week = 0;
    }
    /* HDOP, VDOP */
    ptr->hdop = static_cast<float>(ubx_nav_dop_.hdop) * 0.01f;
    ptr->vdop = static_cast<float>(ubx_nav_dop_.vdop) * 0.01f;
    /* Ground track and ground speed */
    ptr->spd_mps = static_cast<float>(ubx_nav_pvt_.gspeed) / 1000.0f;
    ptr->track_rad = deg2rad(static_cast<float>(ubx_nav_pvt_.headmot) /
                             100000.0f);
    ptr->track_acc_rad = deg2rad(static_cast<float>(ubx_nav_pvt_.headacc) /
                                 100000.0f);
    /* Velocity */
    ptr->ned_vel_mps(0) = static_cast<float>(ubx_nav_pvt_.veln) / 1000.0f;
    ptr->ned_vel_mps(1) = static_cast<float>(ubx_nav_pvt_.vele) / 1000.0f;
    ptr->ned_vel_mps(2) = static_cast<float>(ubx_nav_pvt_.veld) / 1000.0f;
    ptr->vel_acc_mps = static_cast<float>(ubx_nav_pvt_.sacc) / 1000.0f;
    /* Position */
    bool invalid_llh = ubx_nav_pvt_.flags3 & 0x01;
    if (!invalid_llh) {
      if (use_high_precision_) {
        ptr->alt_wgs84_m = (static_cast<float>(ubx_nav_hpposllh_.height) +
                            static_cast<float>(ubx_nav_hpposllh_.heighthp) *
                            0.1f) * 0.001f;
        ptr->alt_msl_m = (static_cast<float>(ubx_nav_hpposllh_.hmsl) +
                          static_cast<float>(ubx_nav_hpposllh_.hmslhp) *
                          0.1f) * 0.001f;
        ptr->horz_acc_m = static_cast<float>(ubx_nav_hpposllh_.hacc) /
                          10000.0f;
        ptr->vert_acc_m = static_cast<float>(ubx_nav_hpposllh_.vacc) /
                          10000.0f;
        ptr->lat_rad = deg2rad((static_cast<double>(ubx_nav_hpposllh_.lat) +
                      static_cast<double>(ubx_nav_hpposllh_.lathp) * 1e-2) *
                      1e-7);
        ptr->lon_rad = deg2rad((static_cast<double>(ubx_nav_hpposllh_.lon) +
                      static_cast<double>(ubx_nav_hpposllh_.lonhp) * 1e-2) *
                      1e-7);
      } else {
        ptr->alt_wgs84_m = static_cast<double>(ubx_nav_pvt_.height) * 0.001f;
        ptr->alt_msl_m = static_cast<double>(ubx_nav_pvt_.hmsl) * 0.001f;
        ptr->horz_acc_m = static_cast<float>(ubx_nav_hpposllh_.hacc) / 1000.0f;
        ptr->vert_acc_m = static_cast<float>(ubx_nav_hpposllh_.vacc) / 1000.0f;
        ptr->lat_rad = deg2rad(static_cast<double>(ubx_nav_pvt_.lat) * 1e-7);
        ptr->lon_rad = deg2rad(static_cast<double>(ubx_nav_pvt_.lon) * 1e-7);
      }
    } else {
      ptr->alt_wgs84_m = 0.0f;
      ptr->alt_msl_m = 0.0f;
      ptr->horz_acc_m = 0.0f;
      ptr->vert_acc_m = 0.0f;
      ptr->lat_rad = 0.0;
      ptr->lon_rad = 0.0;
    }
  }
  return ptr->new_data;
}
bool Ublox::Epoch() {
  if (Parse()) {
    switch (msg_) {
      case UBX_NAV_DOP: {
        ubx_nav_dop_parsed_ = true;
        break;
      }
      case UBX_NAV_PVT: {
        ubx_nav_pvt_parsed_ = true;
        break;
      }
      case UBX_NAV_HPPOSLLH: {
        ubx_nav_hpposllh_parsed_ = true;
        break;
      }
      case UBX_NAV_EOE: {
        read_status_ = false;
        if (use_high_precision_) {
          read_status_ = ubx_nav_dop_parsed_ && ubx_nav_pvt_parsed_ &&
                         ubx_nav_hpposllh_parsed_;
        } else {
          read_status_ = ubx_nav_dop_parsed_ && ubx_nav_pvt_parsed_;
        }
        ubx_nav_dop_parsed_ = false;
        ubx_nav_pvt_parsed_ = false;
        ubx_nav_hpposllh_parsed_ = false;
        return read_status_;
      }
    }
  }
  return false;
}
bool Ublox::Parse() {
  while (config_.bus->available()) {
    uint8_t byte_read = config_.bus->read();
    /* Identify the packet header */
    if (parser_pos_ < 2) {
      if (byte_read == UBX_HEADER_[parser_pos_]) {
        parser_pos_++;
      } else {
        parser_pos_ = 0;
      }
    /* Message class */
    } else if (parser_pos_ == 2) {
      if (byte_read == UBX_NAV_CLASS_) {
        rx_buffer_[parser_pos_ - sizeof(UBX_HEADER_)] = byte_read;
        parser_pos_++;
      } else {
        parser_pos_ = 0;
      }
    /* Message ID */
    } else if (parser_pos_ == 3) {
      if ((byte_read == UBX_NAV_DOP) ||
          (byte_read == UBX_NAV_PVT) ||
          (byte_read == UBX_NAV_HPPOSLLH) ||
          (byte_read == UBX_NAV_EOE))  {
        msg_ = static_cast<Msg>(byte_read);
        rx_buffer_[parser_pos_ - sizeof(UBX_HEADER_)] = byte_read;
        parser_pos_++;
      } else {
        parser_pos_ = 0;
      }
    /* Messgae length */
    } else if (parser_pos_ == 4) {
      msg_len_buffer_[0] = byte_read;
      rx_buffer_[parser_pos_ - sizeof(UBX_HEADER_)] = byte_read;
      parser_pos_++;
    /* Message length */
    } else if (parser_pos_ == 5) {
      msg_len_buffer_[1] = byte_read;
      msg_len_ =
        static_cast<uint16_t>(msg_len_buffer_[1]) << 8 | msg_len_buffer_[0];
      rx_buffer_[parser_pos_ - sizeof(UBX_HEADER_)] = byte_read;
      switch (msg_) {
        case UBX_NAV_DOP: {
          if (msg_len_ == UBX_DOP_LEN_) {
            parser_pos_++;
          } else {
            parser_pos_ = 0;
          }
          break;
        }
        case UBX_NAV_PVT: {
          if (msg_len_ == UBX_PVT_LEN_) {
            parser_pos_++;
          } else {
            parser_pos_ = 0;
          }
          break;
        }
        case UBX_NAV_HPPOSLLH: {
          if (msg_len_ == UBX_HPPOSLLH_LEN_) {
            parser_pos_++;
          } else {
            parser_pos_ = 0;
          }
          break;
        }
        case UBX_NAV_EOE: {
          if (msg_len_ == UBX_EOE_LEN_) {
            parser_pos_++;
          } else {
            parser_pos_ = 0;
          }
          break;
        }
      }
    /* Message payload */
    } else if (parser_pos_ < (msg_len_ + UBX_HEADER_LEN_)) {
      rx_buffer_[parser_pos_ - sizeof(UBX_HEADER_)] = byte_read;
      parser_pos_++;
    /* Checksum */
    } else  if (parser_pos_ == (msg_len_ + UBX_HEADER_LEN_)) {
      checksum_buffer_[0] = byte_read;
      parser_pos_++;
    } else {
      checksum_buffer_[1] = byte_read;
      uint16_t received_checksum =
        static_cast<uint16_t>(checksum_buffer_[1]) << 8 | checksum_buffer_[0];
      uint16_t computed_checksum =
        Checksum(rx_buffer_, msg_len_ + UBX_HEADER_LEN_);
      if (computed_checksum == received_checksum) {
        switch (msg_) {
          case UBX_NAV_DOP: {
            memcpy(&ubx_nav_dop_, rx_buffer_ + UBX_PAYLOAD_OFFSET_,
                   UBX_DOP_LEN_);
            break;
          }
          case UBX_NAV_PVT: {
            memcpy(&ubx_nav_pvt_, rx_buffer_ + UBX_PAYLOAD_OFFSET_,
                   UBX_PVT_LEN_);
            break;
          }
          case UBX_NAV_HPPOSLLH: {
            memcpy(&ubx_nav_hpposllh_, rx_buffer_ + UBX_PAYLOAD_OFFSET_,
                   UBX_HPPOSLLH_LEN_);
            break;
          }
          case UBX_NAV_EOE: {
            memcpy(&ubx_nav_eoe_, rx_buffer_ + UBX_PAYLOAD_OFFSET_,
                   UBX_EOE_LEN_);
            break;
          }
        }
        parser_pos_ = 0;
        return true;
      } else {
        parser_pos_ = 0;
        return false;
      }
    }
  }
  return false;
}
uint16_t Ublox::Checksum(uint8_t *data, uint16_t len) {
  if (!data) {
    return 0;
  }
  uint8_t checksum_buffer[2] = {0, 0};
  for (unsigned int i = 0; i < len; i++) {
    checksum_buffer[0] += data[i];
    checksum_buffer[1] += checksum_buffer[0];
  }
  return static_cast<uint16_t>(checksum_buffer_[1]) << 8 | checksum_buffer_[0];
}

}  // namespace bfs
