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

#ifndef SRC_UBX_H_
#define SRC_UBX_H_

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include <cstddef>
#include <cstdint>
#include "eigen.h"  // NOLINT
#include "units.h"  // NOLINT
#include "ubx_ack.h"  // NOLINT
#include "ubx_cfg.h"  // NOLINT
#include "ubx_defs.h"  // NOLINT
#include "ubx_inf.h"  // NOLINT
#include "ubx_keys.h"  // NOLINT
#include "ubx_log.h"  // NOLINT
#include "ubx_mga.h"  // NOLINT
#include "ubx_mon.h"  // NOLINT
#include "ubx_nav.h"  // NOLINT
#include "ubx_rxm.h"  // NOLINT
#include "ubx_sec.h"  // NOLINT
#include "ubx_time.h"  // NOLINT
#include "ubx_upd.h"  // NOLINT

namespace bfs {

class Ubx {
 public:
  template<typename T>
  struct Optional {
    bool has_value = false;
    T value;
  };
  enum Fix : int8_t {
    FIX_NONE = 0,
    FIX_2D = 1,
    FIX_3D = 2,
    FIX_DGNSS = 3,
    FIX_RTK_FLOAT = 4,
    FIX_RTK_FIXED = 5
  };
  enum BaseMode {
    MOVING_BASE,
    FIXED_BASE
  };
  explicit Ubx(HardwareSerial* bus) : bus_(bus) {}
  bool Begin(const uint32_t baud);
  bool ConfigRtcmInput(const bool serial1, const bool serial2);
  bool ConfigRtcmOutput(const BaseMode mode, const bool serial1,
                        const bool serial2);
  bool Read();

  /* Data */
  // Fix fix()
  // int8_t num_sats()
  // int32_t tow_ms()
  // int16_t week()

  // float time_acc()

  // float gdop()
  // float pdop()
  // float tdop()
  // float vdop()
  // float hdop()
  // float ndop()
  // float edop()

  // // double ecef_x_m()
  // // double ecef_y_m()
  // // double ecef_z_m()
  // // float ecef_acc_m()

  // // float vel_ecef_x_mps()
  // // float vel_ecef_y_mps()
  // // float vel_ecef_z_mps()
  // // float vel_ecef_acc_mps()  // is this the same as spd_acc_mps??

  // double lat_rad()
  // double lat_deg()
  // double lon_rad()
  // double lon_deg()
  // float alt_wgs84_m()
  // float alt_msl_m()
  // float horz_acc_m()
  // float vert_acc_m()

  // float vel_north_mps()
  // float vel_east_mps()
  // float vel_down_mps()
  // Eigen::Vector3f vel_ned_mps()

  // float gnd_spd_mps()
  // float spd_acc_mps()
  // float track_rad()
  // float track_deg()
  // float track_acc_rad()
  // float track_acc_deg()



//  private:
  /* Communication */
  HardwareSerial* bus_;
  /* Parsing */
  static constexpr uint8_t UBX_HEADER_[2] = {0xB5, 0x62};
  static constexpr uint8_t UBX_CLS_POS_ = 2;
  static constexpr uint8_t UBX_ID_POS_ = 3;
  static constexpr uint8_t UBX_LEN_POS_LSB_ = 4;
  static constexpr uint8_t UBX_LEN_POS_MSB_ = 5;
  static constexpr uint8_t UBX_HEADER_LEN_ = 6;
  static constexpr uint8_t UBX_CHK_OFFSET_ = UBX_HEADER_LEN_ -
                                             sizeof(UBX_HEADER_);
  UbxMsg rxmsg_;
  UbxReq req_msg_;
  /* ACK */
  int32_t DEFAULT_TIMEOUT_MS_ = 10000;
  UbxAckAck ubx_ack_ack_;
  UbxAckNak ubx_ack_nak_;
  bool ack_, new_ack_msg_ = false;
  uint8_t ack_cls_, ack_id_;
  /* CFG */
  static constexpr uint16_t PERIOD_MS_ = 100;
  UbxCfgCfg ubx_cfg_cfg_;
  UbxCfgMsgSet ubx_cfg_msg_set_;
  UbxCfgPrtReq ubx_cfg_prt_req_;
  UbxCfgPrt ubx_cfg_prt_;
  UbxCfgRate ubx_cfg_rate_;
  UbxCfgNav5 ubx_cfg_nav5_;
  bool new_cfg_msg_ = false;
  uint8_t cfg_cls_, cfg_id_;
  /* v9 cfg */
  UbxCfgValset<12> ubx_cfg_val_set_;
  /* Data */
  UbxNavTimeutc ubx_nav_time_utc_;
  UbxNavTimels ubx_nav_time_ls_;
  UbxNavStatus ubx_nav_status_;
  UbxNavDop ubx_nav_dop_;
  UbxNavEoe ubx_nav_eoe_;
  UbxNavPosecef ubx_nav_pos_ecef_;
  UbxNavHpposecef ubx_nav_hp_pos_ecef_;
  UbxNavVelecef ubx_nav_vel_ecef_;
  UbxNavPosllh ubx_nav_pos_llh_;
  UbxNavHpposllh ubx_nav_hp_pos_llh_;
  UbxNavVelned ubx_nav_vel_ned_;
  bool eoe_ = false;

  /* Helper functions */
  /*
  * Parses incoming messages given a byte from a bus and a UbxMsg pointer to
  * store received messages, returns true if a valid message is received
   */
  bool ParseMsg(const uint8_t c, UbxMsg * const msg);
  /* Sends a UBX message */
  template<typename T>
  void SendMsg(const T &ref) {
    static uint16_t chk;
    /* Write the header */
    bus_->write(UBX_HEADER_[0]);
    bus_->write(UBX_HEADER_[1]);
    /* Write the class and start computing the checksum */
    bus_->write(ref.cls);
    Checksum(&ref.cls, sizeof(ref.cls), true);
    /* Write the ID and update the checksum */
    bus_->write(ref.id);
    Checksum(&ref.id, sizeof(ref.id), false);
    /* Write the length and updated the checksum */
    bus_->write(static_cast<uint8_t>(ref.len & 0xFF));
    bus_->write(static_cast<uint8_t>(ref.len >> 8 & 0xFF));
    Checksum((uint8_t *)(&ref.len), 2, false);
    /* Write the payload and finish computing the checksum */
    bus_->write((uint8_t *)&ref.payload, ref.len);
    chk = Checksum((uint8_t *)&ref.payload, ref.len, false);
    /* Write the checksum */
    bus_->write(static_cast<uint8_t>(chk & 0xFF));
    bus_->write(static_cast<uint8_t>(chk >> 8 & 0xFF));
  }
  /* UBX checksum calculation */
  uint16_t Checksum(uint8_t const * const data, const uint16_t len,
                    bool reset_states);
  /* 
  * UBX-NAV message handler, parses messages by ID and stores in class data
  * structs. Returns true if valid & supported NAV message received
  */
  bool NavMsgHandler(const UbxMsg &ref);
  /* Processes the UBX-NAV data at EOE and preps for output */
  void ProcessNavData(); 
  /* UBX-ACK message handler Returns true if valid message received */
  bool AckMsgHandler(const UbxMsg &ref);
  /* UBX-CFG message handler */
  bool CfgMsgHandler(const UbxMsg &ref);
  /* Restore factory defaults */
  bool RestoreDefaults();
  /* Enable message */
  bool EnableMessage(const uint8_t cls, const uint8_t id, const uint8_t port);
  /* Disable message */
  bool DisableMessage(const uint8_t cls, const uint8_t id, const uint8_t port);
  /* Configure ports */
  bool ConfigPort(const uint8_t port, const uint16_t prot_in,
                  const uint16_t prot_out);
  /* Rates */
  bool ConfigRate(const uint16_t period_ms);
  /* Dynamic model */
  bool ConfigDynModel();
  /* Fix mode */
  bool ConfigFixMode();
  /* Send command */
  template<typename T> 
  bool SetConfig(const T &ref, const int32_t timeout_ms) {
    SendMsg(ref);
    bus_->flush();
    elapsedMillis t_ms;
    while (t_ms < timeout_ms) {
      if (bus_->available()) {
        if (ParseMsg(bus_->read(), &rxmsg_)) {
          Serial.println("MSG");
        }
      }
      // Read();
      // if (new_ack_msg_) {
      //   if ((ack_cls_ == ref.cls) && (ack_id_ = ref.id)) {
      //     Serial.println("NEW ACK");
      //     new_ack_msg_ = false;
      //     return ack_;
      //   }
      // }
    }
    return false;
  }
  template<typename T> 
  bool SetConfig(const T &ref) {
    return SetConfig(ref, DEFAULT_TIMEOUT_MS_);
  }
  /* Poll message */
  template<typename T>
  bool RequestConfig(const T &ref, const int32_t timeout_ms) {
    SendMsg(ref);
    elapsedMillis t_ms;
    while (t_ms < timeout_ms) {
      Read();
      if (new_cfg_msg_) {
        if ((cfg_cls_ == ref.cls) && (cfg_id_ == ref.id)) {
          new_cfg_msg_ = false;
          return true;
        }
      }
    }
    return false;
  }
  template<typename T>
  bool RequestConfig(const T &ref) {
    return RequestConfig(ref, DEFAULT_TIMEOUT_MS_);
  }
  // /* v9 Configuration */
  // /* Set a config value in RAM */
  // template<typename T>
  // bool SetCfgVal(const uint32_t key, const T val, const int32_t timeout_ms) {
  //   ubx_cfg_val_set_.len = 4 + sizeof(uint32_t) + sizeof(T);
  //   ubx_cfg_val_set_.payload.version = 0x00;
  //   ubx_cfg_val_set_.payload.layers = 0x01;  // RAM
  //   memcpy(ubx_cfg_val_set_.payload.cfg_data, &key, sizeof(uint32_t));
  //   memcpy(ubx_cfg_val_set_.payload.cfg_data + sizeof(uint32_t),
  //          &val, sizeof(T));
  //   return SendConfig(ubx_cfg_val_set_, timeout_ms);
  // }
  // template<typename T>
  // bool SetCfgVal(const uint32_t key, const T val) {
  //   return SetCfgVal(key, val, DEFAULT_TIMEOUT_MS_);
  // }
  // /* Get a config value from RAM */
  // template<typename T>
  // bool GetVal(const uint32_t key, const int32_t timeout_ms, T * const val) {
  //   if (!val) {return false;}
  // }


  // /* Legacy Configuration */
  // template<typename T>
  // bool SendLegacyCfg(const T &ref) {
  //   txmsg_.cls = ref.cls;
  //   txmsg_.id = ref.id;
  //   txmsg_.len = ref.len;
  //   memcpy(txmsg_.payload, ref.payload, ref.len);
  //   SendMsg(txmsg_);
  // }
  // /* v9 Configuration */
  // /* Set a config value in RAM */
  // template<typename T>
  // bool SetCfgVal(const uint32_t key, const T val) {
  //   static constexpr int16_t TIMEOUT_MS = 1000;
  //   if (!SendCfgValSet(key, val, &txmsg_)) {return false;}
  //   elapsedMillis t_ms;
  //   while (t_ms < TIMEOUT_MS) {
  //     if (bus_->available()) {
  //       if (ParseMsg(bus_->read(), &rxmsg_)) {
  //         if (rxmsg_.cls == UBX_ACK_CLS_) {
  //           uint8_t cls, id;
  //           bool resp;
  //           if (ParseAck(rxmsg_, &cls, &id, &resp)) {
  //             /* Check that the ACK / NACK is in response to the set cmd */
  //             if ((cls == UBX_CFG_CLS_) && (id = UBX_CFG_VALSET_ID_)) {
  //               return resp;
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }
  //   return false;
  // }
  // /* Get a config value from RAM */
  // template<typename T>
  // bool GetCfgVal(const uint32_t key, T * const val) {
  //   if (!val) {return false;}
  //   static constexpr int16_t TIMEOUT_MS = 1000;
  //   if (!SendCfgValGet(key, &txmsg_)) {return false;}
  //   elapsedMillis t_ms;
  //   while (t_ms < TIMEOUT_MS) {
  //     if (bus_->available()) {
  //       if (ParseMsg(bus_->read(), &rxmsg_)) {
  //         if ((rxmsg_.cls == UBX_CFG_CLS_) &&
  //             (rxmsg_.id == UBX_CFG_VALGET_ID_)) {
  //           uint32_t ret_key;
  //           T ret_val;
  //           if (ParseCfgValGet<T>(rxmsg_, &ret_key, &ret_val)) {
  //             if (ret_key == key) {
  //               *val = ret_val;
  //               return true;
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }
  //   return false;
  // }
  // /* UBX-ACK-ACK / UBX-ACK-NACK parsing */
  // bool ParseAck(const UbxMsg &msg, uint8_t * const cls,
  //               uint8_t * const id, bool * const resp);
  // /* Send a UBX-CFG-VALSET command */
  // template<typename T>
  // bool SendCfgValSet(const uint32_t key, const T val, UbxMsg * const msg) {
  //   if (!msg) {return false;}
  //   msg->cls = UBX_CFG_CLS_;
  //   msg->id = UBX_CFG_VALSET_ID_;
  //   msg->len = 8 + sizeof(T);
  //   msg->payload[0] = UBX_CFG_VALSET_TRANSACTIONLESS_;
  //   msg->payload[1] = 0x01;  // RAM
  //   msg->payload[2] = 0;  // unused
  //   msg->payload[3] = 0;
  //   memcpy(msg->payload + 4, &key, sizeof(uint32_t));
  //   memcpy(msg->payload + 8, &val, sizeof(T));
  //   SendMsg(*msg);
  //   return true;
  // }
  // /* Send a UBX-CFG-VALGET request */
  // bool SendCfgValGet(const uint32_t key, UbxMsg * const msg);
  // /* Parse a UBX-CFG-VALGET response */
  // template<typename T>
  // bool ParseCfgValGet(const UbxMsg &msg, uint32_t * const key, T * const val) {
  //   static constexpr uint8_t HEADER_LEN = 4;
  //   if ((!key) || (!val)) {return false;}
  //   if (msg.len != HEADER_LEN + sizeof(uint32_t) + sizeof(T)) {return false;}
  //   if (msg.payload[0] != UBX_CFG_VALGET_RESPONSE_) {return false;}
  //   if (msg.payload[1] != 0x00) {return false;}  // RAM layer
  //   memcpy(key, msg.payload + 4, sizeof(uint32_t));
  //   memcpy(val, msg.payload + 8, sizeof(T));
  //   return true;
  // }
};

}  // namespace bfs

#endif  // SRC_UBX_H_
