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

#include "core/core.h"
#include "ubx.h"  // NOLINT

namespace bfs {

bool Ubx::Begin() {
  /* Find the baud rate */
  if (!AutoBaud()) {return false;}
  /* Restore the factory defaults */
  SetFactoryDefaults();
  /* Find the baud rate again */
  bus_->end();
  if (!AutoBaud()) {return false;}
  /* Update the baud rate */
  if (!SetBaud(UBX_COM_PORT_UART1_, DESIRED_BAUD_)) {return false;}
  bus_->end();
  bus_->begin(DESIRED_BAUD_);
  /* Disable NMEA messages */
  if (!DisableNmea(UBX_COM_PORT_UART1_)) {return false;}
  /* Set the message output rate */
  if (!SetRate(SAMPLE_PERIOD_MS_)) {return false;}
  /* Set the dynamic model */
  if (!SetDynModel(DYN_MDL_AIRBORNE_4G)) {return false;}
  /* Enable UBX messages */
  if (!EnableMsgs()) {return false;}
  return true;
}
bool Ubx::Read() {
  while (bus_->available()) {
    if (ParseMsg) {
      if (rx_msg_.cls == UBX_NAV_CLS_) {
        switch (rx_msg_.id) {
          case UBX_NAV_POSECEF_ID_: {
            break;
          }
          case UBX_NAV_POSLLH_ID_: {
            break;
          }
          case UBX_NAV_STATUS_ID_: {
            break;
          }
          case UBX_NAV_DOP_ID_: {
            break;
          }
          case UBX_NAV_VELECEF_ID_: {
            break;
          }
          case UBX_NAV_VELNED_ID_: {
            break;
          }
          case UBX_NAV_HPPOSECEF_ID_: {
            break;
          }
          case UBX_NAV_HPPOSLLH_ID_: {
            break;
          }
          case UBX_NAV_TIMEUTC_ID_: {
            break;
          }
          case UBX_NAV_TIMELS_ID_: {
            break;
          }
          case UBX_NAV_EOE_ID_: {
            break;
          }
          default: {
            break;
          }
        }
      }
    }
  }
}
bool Ubx::ParseMsg() {
  while (bus_->available()) {
    c_ = bus_->read();
    /* Packet header */
    if (parser_state_ < sizeof(UBX_HEADER_)) {
      if (c_ == UBX_HEADER_[parser_state_]){
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
      chk_cmp_rx_ = chksum_rx_.Compute((uint8_t *)(&rx_msg_),
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
void Ubx::SetFactoryDefaults() {
  ubx_cfg_cfg_.payload.clear_mask = 0xFFFF;
  ubx_cfg_cfg_.payload.save_mask = 0x00;
  ubx_cfg_cfg_.payload.load_mask = 0xFFFF;
  ubx_cfg_cfg_.payload.device_mask = 0x00;
  SendMsg(ubx_cfg_cfg_);
}
bool Ubx::EnableMsg(const uint8_t cls, const uint8_t id, const uint8_t port) {
  ubx_cfg_msg_.payload.msg_class = cls;
  ubx_cfg_msg_.payload.msg_id = id;
  ubx_cfg_msg_.payload.rate[port] = 1;
  return SendAck(ubx_cfg_msg_, LONG_TIMEOUT_MS_);
}
bool Ubx::EnableMsgs() {
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_POSECEF_ID_, UBX_COM_PORT_UART1_)) {
    return false;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_POSLLH_ID_, UBX_COM_PORT_UART1_)) {
    return false;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_STATUS_ID_, UBX_COM_PORT_UART1_)) {
    return false;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_DOP_ID_, UBX_COM_PORT_UART1_)) {
    return false;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_VELECEF_ID_, UBX_COM_PORT_UART1_)) {
    return false;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_VELNED_ID_,UBX_COM_PORT_UART1_)) {
    return false;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_HPPOSECEF_ID_, UBX_COM_PORT_UART1_)) {
    use_hp_pos_ = false;
  } else {
    use_hp_pos_ = true;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_HPPOSLLH_ID_, UBX_COM_PORT_UART1_)) {
    use_hp_pos_ = false;
  } else {
    use_hp_pos_ = true;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_TIMEUTC_ID_, UBX_COM_PORT_UART1_)) {
    return false;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_TIMELS_ID_, UBX_COM_PORT_UART1_)) {
    return false;
  }
  if (!EnableMsg(UBX_NAV_CLS_, UBX_NAV_EOE_ID_, UBX_COM_PORT_UART1_)) {
    return false;
  }
  return true;
}
bool Ubx::SetBaud(const uint8_t port, const uint32_t baud) {
  /* Get the port config */
  ubx_cfg_prt_req_.payload.port_id = port;
  if (SendResponse(ubx_cfg_prt_req_, UBX_CFG_CLS_, UBX_CFG_PRT_ID_,
                    LONG_TIMEOUT_MS_)) {
    if (rx_msg_.len != ubx_cfg_prt_.len) {return false;}
    memcpy(&ubx_cfg_prt_.payload, rx_msg_.payload, ubx_cfg_prt_.len);
    /* Update the input and output protocols */
    ubx_cfg_prt_.payload.baud_rate = baud;
  } else {return false;}
  return SendAck(ubx_cfg_prt_, LONG_TIMEOUT_MS_);
}
bool Ubx::DisableNmea(const uint8_t port) {
  /* Get the port config */
  ubx_cfg_prt_req_.payload.port_id = port;
  if (SendResponse(ubx_cfg_prt_req_, UBX_CFG_CLS_, UBX_CFG_PRT_ID_,
                    LONG_TIMEOUT_MS_)) {
    if (rx_msg_.len != ubx_cfg_prt_.len) {return false;}
    memcpy(&ubx_cfg_prt_.payload, rx_msg_.payload, ubx_cfg_prt_.len);
    /* Update the input and output protocols */
    ubx_cfg_prt_.payload.in_proto_mask = UBX_COM_PROT_UBX_ |
                                          UBX_COM_PROT_RTCM_ |
                                          UBX_COM_PROT_RTCM3_;
    ubx_cfg_prt_.payload.out_proto_mask = UBX_COM_PROT_UBX_ |
                                          UBX_COM_PROT_RTCM_ |
                                          UBX_COM_PROT_RTCM3_;
  } else {return false;}
  return SendAck(ubx_cfg_prt_, LONG_TIMEOUT_MS_);
}
bool Ubx::SetRate(const uint16_t rate) {
  /* Get the rate config */
  req_msg_.cls = UBX_CFG_CLS_;
  req_msg_.id = UBX_CFG_RATE_ID_;
  req_msg_.len = 0;
  if (SendResponse(req_msg_, UBX_CFG_CLS_, UBX_CFG_RATE_ID_,
                    LONG_TIMEOUT_MS_)) {
    if (rx_msg_.len != ubx_cfg_rate_.len) {return false;}
    memcpy(&ubx_cfg_rate_.payload, rx_msg_.payload, ubx_cfg_rate_.len);
    /* Update the rate */
    ubx_cfg_rate_.payload.meas_rate = rate;
  } else {return false;}
  return SendAck(ubx_cfg_rate_, LONG_TIMEOUT_MS_);
}
bool Ubx::SetDynModel(const DynMdl mdl) {
  ubx_cfg_nav5_.payload.mask = 0x01;  // dynamic model settings
  ubx_cfg_nav5_.payload.dyn_model = mdl;
  return SendAck(ubx_cfg_nav5_, LONG_TIMEOUT_MS_);
}
bool Ubx::TestComms() {
  /* Send a legacy request for port config to see if we get a response */
  ubx_cfg_prt_req_.payload.port_id = UBX_COM_PORT_UART1_;
  return SendResponse(ubx_cfg_prt_req_, UBX_CFG_CLS_, UBX_CFG_PRT_ID_,
                      TIMEOUT_MS_);
}
bool Ubx::AutoBaud() {
  for (std::size_t i = 0; i < sizeof(baud_rates_) / sizeof(int32_t); i++) {
    baud_ = baud_rates_[i];
    bus_->begin(baud_);
    if (TestComms()) {
      return true;
    }
    bus_->end();
  }
  return false;
}

}  // namespace bfs
