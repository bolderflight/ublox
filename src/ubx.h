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
#include <Arduino.h>
#else
#include "core/core.h"
#endif
#include <cstddef>
#include <cstdint>
#include "eigen.h"
#include "ubx_ack.h"
#include "ubx_cfg.h"
#include "ubx_defs.h"
#include "ubx_inf.h"
#include "ubx_keys.h"
#include "ubx_log.h"
#include "ubx_mga.h"
#include "ubx_mon.h"
#include "ubx_nav.h"
#include "ubx_rxm.h"
#include "ubx_sec.h"
#include "ubx_time.h"
#include "ubx_upd.h"

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
  enum DynMdl : uint8_t {
    DYN_MDL_PORTABLE = 0,
    DYN_MDL_STATIONARY = 2,
    DYN_MDL_PEDESTRIAN = 3,
    DYN_MDL_AUTOMOTIVE = 4,
    DYN_MDL_SEA = 5,
    DYN_MDL_AIRBORNE_1G = 6,
    DYN_MDL_AIRBORNE_2G = 7,
    DYN_MDL_AIRBORNE_4G = 8,
    DYN_MDL_WRIST = 9,
    DYN_MDL_BIKE = 10
  };
  explicit Ubx(HardwareSerial* bus) : bus_(bus) {}
  /* Automatically finds the baud rate and configures the receiver */
  bool AutoBegin();
  /* Reads NAV data and returns true on EOE */
  bool Read();
  /* Set the receiver dynamic model */
  bool SetDynModel(const DynMdl mdl);
  // bool ConfigRtcmInput(const bool serial1, const bool serial2);
  // bool ConfigRtcmOutput(const BaseMode mode, const bool serial1,
  //                       const bool serial2);
  /* Standard begin, set the baud and test for comms */
  bool Begin(const int32_t baud);
  /* Restore factory default config */
  void SetFactoryDefaults();
  /* Sets the nav solution rate */
  bool SetRate(const uint16_t period_ms);
  /* Tests for good communication */
  bool TestComms();
  /* Automatically figures out the baud rate of the receiver */
  int32_t AutoBaud();
  /* Sets the baud rate of the receiver */
  bool SetBaud(const uint8_t port, const uint32_t baud);
  /* Configures the port input & output protocols */
  bool ConfigPort(const uint8_t port, const uint8_t in_prot,
                  const uint8_t out_prot);
  /* Enables our standard set of messages */
  bool EnableMsgs();
  /* Enables a message given a class, ID, and port */
  bool EnableMsg(const uint8_t cls, const uint8_t id, const uint8_t port);
  /* Disables a message given a class, ID, and port */
  bool DisableMsg(const uint8_t cls, const uint8_t id, const uint8_t port);

 private:
  /* Send a message to the uBlox */
  template<typename T>
  void SendMsg(const T &ref) {
    /* Write the header */
    bus_->write(UBX_HEADER_[0]);
    bus_->write(UBX_HEADER_[1]);
    /* Write the class and start computing the checksum */
    bus_->write(ref.cls);
    chksum_tx_.Compute(&ref.cls, sizeof(ref.cls));
    /* Write the ID and update the checksum */
    bus_->write(ref.id);
    chksum_tx_.Update(&ref.id, sizeof(ref.id));
    /* Write the length and updated the checksum */
    bus_->write(static_cast<uint8_t>(ref.len & 0xFF));
    bus_->write(static_cast<uint8_t>(ref.len >> 8 & 0xFF));
    chksum_tx_.Update(reinterpret_cast<const uint8_t *>(&ref.len), 2);
    /* Write the payload and finish computing the checksum */
    bus_->write(reinterpret_cast<const uint8_t *>(&ref.payload), ref.len);
    chk_cmp_tx_ =
              chksum_tx_.Update(reinterpret_cast<const uint8_t *>(&ref.payload),
                                ref.len);
    /* Write the checksum */
    bus_->write(static_cast<uint8_t>(chk_cmp_tx_ & 0xFF));
    bus_->write(static_cast<uint8_t>(chk_cmp_tx_ >> 8 & 0xFF));
  }
  /* Send a message and look for a response */
  template<typename T>
  bool SendResponse(const T &ref, const uint8_t cls, const uint8_t id,
                    const uint16_t timeout_ms) {
    elapsedMillis fun_t_ms;
    do {
      SendMsg(ref);
      elapsedMillis t_ms;
      while (t_ms < TIMEOUT_MS_) {
        if (ParseMsg()) {
          if ((rx_msg_.cls == cls) && (rx_msg_.id == id)) {
            return true;
          }
        }
      }
    } while (fun_t_ms < timeout_ms);
    return false;
  }
  /* Send a message and look for an ACK */
  template<typename T>
  bool SendAck(const T &ref, const uint16_t timeout_ms) {
    elapsedMillis fun_t_ms;
    do {
      SendMsg(ref);
      elapsedMillis t_ms;
      while (t_ms < TIMEOUT_MS_) {
        if (ParseMsg()) {
          if (rx_msg_.cls == UBX_ACK_CLS_) {
            if ((rx_msg_.payload[0] == ref.cls) &&
                (rx_msg_.payload[1] == ref.id)) {
              if (rx_msg_.id == UBX_ACK_ACK_ID_) {
                return true;
              } else {
                return false;
              }
            }
          }
        }
      }
    } while (fun_t_ms < timeout_ms);
    return false;
  }
  /* Parse messages, return true on valid msg received */
  bool ParseMsg();
  /* Process nav data */
  void ProcessNavData(); 
  /* Communication */
  HardwareSerial* bus_;
  /* Potential baudrates */
  int32_t baud_rates_[8] = {38400, 9600, 19200, 57600, 115200, 230400, 460800,
                            921600};
  static constexpr int32_t DESIRED_BAUD_ = 460800;
  int32_t baud_;
  /* Configuration */
  uint16_t SAMPLE_PERIOD_MS_ = 100;
  bool use_hp_pos_ = false;
  /* Max payload bytes supported */
  static constexpr std::size_t UBX_MAX_PAYLOAD_ = 1024;
  /* Timeout for waiting for response */
  static constexpr uint16_t TIMEOUT_MS_ = 1000;
  static constexpr uint16_t LONG_TIMEOUT_MS_ = 5000;
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
  std::size_t parser_state_ = 0;
  /* Data members */
  bool eoe_ = false;
  Fix fix_;
  bool gnss_fix_ok_, diff_soln_;
  bool valid_date_, valid_time_, fully_resolved_, validity_confirmed_;
  bool confirmed_date_, confirmed_time_, valid_time_and_date_;
  bool invalid_llh_, invalid_ecef_;
  int8_t carr_soln_;
  int8_t num_sv_;
  int8_t year_, month_, day_, hour_, min_, sec_;
  int16_t week_;
  int32_t nano_;
  int32_t tow_ms_;
  uint32_t t_acc_ns_;
  float alt_msl_m_;
  float gnd_spd_mps_;
  float track_deg_;
  float gdop_, pdop_, tdop_, vdop_, hdop_, ndop_, edop_;
  float h_acc_m_, v_acc_m_, p_acc_m_, track_acc_deg_, s_acc_mps_;
  Eigen::Vector3f ecef_vel_mps_;
  Eigen::Vector3f ned_vel_mps_;
  Eigen::Vector3d ecef_m_;
  Eigen::Vector3d llh_;
  /* Class to compute UBX checksum */
  class Checksum {
   public:
    uint16_t Compute(uint8_t const * const data, const std::size_t len) {
      if (!data) {
        return 0;
      }
      sum0_ = 0;
      sum1_ = 0;
      for (std::size_t i = 0; i < len; i++) {
        sum0_ += data[i];
        sum1_ += sum0_;
      }
      return static_cast<uint16_t>(sum1_) << 8 | sum0_;
    }
    uint16_t Update(uint8_t const * const data, const std::size_t len) {
      if (!data) {
        return 0;
      }
      for (std::size_t i = 0; i < len; i++) {
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
  /* struct for polling messages that don't have specific request messages */
  struct UbxReq {
    uint8_t cls;
    uint8_t id;
    uint16_t len = 0;
    uint8_t payload;
  } req_msg_;
  /* Config messages */
  UbxCfgPrtReq ubx_cfg_prt_req_;
  UbxCfgPrt ubx_cfg_prt_;
  UbxCfgRate ubx_cfg_rate_;
  UbxCfgMsg ubx_cfg_msg_;
  UbxCfgNav5 ubx_cfg_nav5_;
  UbxCfgCfg ubx_cfg_cfg_;
  /* Data messages */
  UbxNavDop ubx_nav_dop_;
  UbxNavEoe ubx_nav_eoe_;
  UbxNavHpposecef ubx_nav_hp_pos_ecef_;
  UbxNavHpposllh ubx_nav_hp_pos_llh_;
  UbxNavPosecef ubx_nav_pos_ecef_;
  UbxNavRelposned ubx_nav_rel_pos_ned_;
  UbxNavVelecef ubx_nav_vel_ecef_;
  UbxNavPvt ubx_nav_pvt_;
};

}  // namespace bfs

#endif  // SRC_UBX_H_
