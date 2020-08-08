/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "ublox/ublox.h"

namespace sensors {

Ublox::Ublox(HardwareSerial* bus) {
  bus_ = bus;
}
bool Ublox::Begin(uint32_t baud) {
  bus_->begin(baud);
  parser_pos_ = 0;
  ubx_nav_pvt_parsed_ = false;
  ubx_nav_hpposllh_parsed_ = false;
  use_high_precision_ = false;
  read_status_ = false;
  bool start = false;
  elapsedMillis timer_ms = 0;
  while (timer_ms < TIMEOUT_MS_) {
    if (Parse()) {
      switch (msg_) {
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
            bool init_status = ubx_nav_pvt_parsed_;
            ubx_nav_pvt_parsed_ = false;
            ubx_nav_hpposllh_parsed_ = false;
            read_status_ = false;
            return init_status;
          }
        }
      }
    }
  }
  return false;
}
bool Ublox::Read() {
  if (Epoch()) {
    gnss_.fix(static_cast<types::Gnss<float, double>::Fix>(ubx_nav_pvt_.fix));
    gnss_.num_satellites(ubx_nav_pvt_.numsv);
    gnss_.ned_vel.north.mps(static_cast<float>(ubx_nav_pvt_.veln) / 1000.0f);
    gnss_.ned_vel.east.mps(static_cast<float>(ubx_nav_pvt_.vele) / 1000.0f);
    gnss_.ned_vel.down.mps(static_cast<float>(ubx_nav_pvt_.veld) / 1000.0f);
    if (use_high_precision_) {
      gnss_.lla.lat.deg((static_cast<double>(ubx_nav_hpposllh_.lat) + static_cast<double>(ubx_nav_hpposllh_.lathp) * 1e-2) * 1e-7);
      gnss_.lla.lon.deg((static_cast<double>(ubx_nav_hpposllh_.lon) + static_cast<double>(ubx_nav_hpposllh_.lonhp) * 1e-2) * 1e-7);
      gnss_.lla.alt.m((static_cast<double>(ubx_nav_hpposllh_.height) + static_cast<double>(ubx_nav_hpposllh_.heighthp) * 1e-1) * 1e-3);
    } else {
      gnss_.lla.lat.deg(static_cast<double>(ubx_nav_pvt_.lat) * 1e-7);
      gnss_.lla.lon.deg(static_cast<double>(ubx_nav_pvt_.lon) * 1e-7);
      gnss_.lla.alt.m(static_cast<double>(ubx_nav_pvt_.height) * 1e-3);
    }
    return true;
  }
  return false;
}
bool Ublox::Epoch() {
  if (Parse()) {
    switch (msg_) {
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
          read_status_ = ubx_nav_pvt_parsed_ && ubx_nav_hpposllh_parsed_;
        } else {
          read_status_ = ubx_nav_pvt_parsed_;
        }
        ubx_nav_pvt_parsed_ = false;
        ubx_nav_hpposllh_parsed_ = false;
        return read_status_;
      }
    }
  }
  return false;
}
types::Gnss<float, double> Ublox::gnss() {
  return gnss_;
}
bool Ublox::Parse() {
  while (bus_->available()) {
    uint8_t byte_read = bus_->read();
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
      if ((byte_read == UBX_NAV_PVT) || (byte_read == UBX_NAV_HPPOSLLH) || (byte_read == UBX_NAV_EOE))  {
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
      msg_len_ = static_cast<uint16_t>(msg_len_buffer_[1]) << 8 | msg_len_buffer_[0];
      rx_buffer_[parser_pos_ - sizeof(UBX_HEADER_)] = byte_read;
      switch (msg_) {
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
      uint16_t received_checksum = static_cast<uint16_t>(checksum_buffer_[1]) << 8 | checksum_buffer_[0];
      uint16_t computed_checksum = Checksum(rx_buffer_, msg_len_ + UBX_HEADER_LEN_);
      if (computed_checksum == received_checksum) {
        switch (msg_) {
          case UBX_NAV_PVT: {
            memcpy(&ubx_nav_pvt_, rx_buffer_ + UBX_PAYLOAD_OFFSET_, UBX_PVT_LEN_);
            break;
          }
          case UBX_NAV_HPPOSLLH: {
            memcpy(&ubx_nav_hpposllh_, rx_buffer_ + UBX_PAYLOAD_OFFSET_, UBX_HPPOSLLH_LEN_);
            break;
          }
          case UBX_NAV_EOE: {
            memcpy(&ubx_nav_eoe_, rx_buffer_ + UBX_PAYLOAD_OFFSET_, UBX_EOE_LEN_);
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

}  // namespace sensors
