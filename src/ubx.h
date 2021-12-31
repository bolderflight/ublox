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

#include "core/core.h"

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
  bool Begin();
  bool Read();
  bool SetDynModel(const DynMdl mdl);
  bool ConfigRtcmInput(const bool serial1, const bool serial2);
  bool ConfigRtcmOutput(const BaseMode mode, const bool serial1,
                        const bool serial2);

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
    chksum_tx_.Update((uint8_t *)(&ref.len), 2);
    /* Write the payload and finish computing the checksum */
    bus_->write((uint8_t *)&ref.payload, ref.len);
    chk_cmp_tx_ = chksum_tx_.Update((uint8_t *)&ref.payload, ref.len);
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
      while(t_ms < TIMEOUT_MS_) {
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
      while(t_ms < TIMEOUT_MS_) {
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
  /* Restore factory default config */
  void SetFactoryDefaults();
  /* Enables a message given a class, ID, and port */
  bool EnableMsg(const uint8_t cls, const uint8_t id, const uint8_t port);
  /* Enables our standard set of messages */
  bool EnableMsgs();
  /* Sets the baud rate */
  bool SetBaud(const uint8_t port, const uint32_t baud);
  /* Disables the NMEA messages */
  bool DisableNmea(const uint8_t port);
  /* Sets the nav solution rate */
  bool SetRate(const uint16_t rate);
  /* Tests for good communication */
  bool TestComms();
  /* Automatically figures out the baud rate of the received */
  bool AutoBaud();
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
  /* Port definitions */
  static constexpr uint8_t UBX_COM_PORT_I2C_ = 0;
  static constexpr uint8_t UBX_COM_PORT_UART1_ = 1;
  static constexpr uint8_t UBX_COM_PORT_UART2_ = 2;
  static constexpr uint8_t UBX_COM_PORT_USB_ = 3;
  static constexpr uint8_t UBX_COM_PORT_SPI_ = 4;
  /* Port protocols */
  static constexpr uint8_t UBX_COM_PROT_UBX_ = 0x01;
  static constexpr uint8_t UBX_COM_PROT_NMEA_ = 0x02;
  static constexpr uint8_t UBX_COM_PROT_RTCM_ = 0x04;
  static constexpr uint8_t UBX_COM_PROT_RTCM3_ = 0x08;
  /* Class definitions */
  static constexpr uint8_t UBX_ACK_CLS_ = 0x05;
  static constexpr uint8_t UBX_CFG_CLS_ = 0x06;
  static constexpr uint8_t UBX_NAV_CLS_ = 0x01;
  /* ID definitions */
  static constexpr uint8_t UBX_ACK_NAK_ID_ = 0x00;
  static constexpr uint8_t UBX_ACK_ACK_ID_ = 0x01;
  static constexpr uint8_t UBX_CFG_PRT_ID_ = 0x00;
  static constexpr uint8_t UBX_CFG_MSG_ID_ = 0x01;
  static constexpr uint8_t UBX_CFG_RATE_ID_ = 0x08;
  static constexpr uint8_t UBX_CFG_CFG_ID_ = 0x09;
  static constexpr uint8_t UBX_CFG_NAV5_ID_ = 0x24;
  static constexpr uint8_t UBX_CFG_VALSET_ID_ = 0x8a;
  static constexpr uint8_t UBX_CFG_VALGET_ID_ = 0x8b;
  static constexpr uint8_t UBX_NAV_POSECEF_ID_ = 0x01;
  static constexpr uint8_t UBX_NAV_POSLLH_ID_ = 0x02;
  static constexpr uint8_t UBX_NAV_STATUS_ID_ = 0x03;
  static constexpr uint8_t UBX_NAV_DOP_ID_ = 0x04;
  static constexpr uint8_t UBX_NAV_VELECEF_ID_ = 0x11;
  static constexpr uint8_t UBX_NAV_VELNED_ID_ = 0x12;
  static constexpr uint8_t UBX_NAV_HPPOSECEF_ID_ = 0x13;
  static constexpr uint8_t UBX_NAV_HPPOSLLH_ID_ = 0x14;
  static constexpr uint8_t UBX_NAV_TIMEUTC_ID_ = 0x21;
  static constexpr uint8_t UBX_NAV_TIMELS_ID_ = 0x26;
  static constexpr uint8_t UBX_NAV_EOE_ID_ = 0x61;
  /*
  * Aliases for uBlox defined types to make defining packets from the interface
  * descriptions easier.
  */
  using U1 = uint8_t;
  using I1 = int8_t;
  using X1 = uint8_t;
  using U2 = uint16_t;
  using I2 = int16_t;
  using X2 = uint16_t;
  using U4 = uint32_t;
  using I4 = int32_t;
  using X4 = uint32_t;
  using R4 = float;
  using R8 = double;
  using CH = char;
  /* Request port config */
  struct UbxCfgPrtReq {
    static constexpr uint8_t cls = UBX_CFG_CLS_;
    static constexpr uint8_t id = UBX_CFG_PRT_ID_;
    static constexpr uint16_t len = 1;
    struct {
      U1 port_id;
    } payload;
  } ubx_cfg_prt_req_;
  /* Port config */
  struct UbxCfgPrt {
    static constexpr uint8_t cls = UBX_CFG_CLS_;
    static constexpr uint8_t id = UBX_CFG_PRT_ID_;
    static constexpr uint16_t len = 20;
    struct {
      U1 port_id;
      U1 reserved0;
      X2 tx_ready;
      X4 mode;
      U4 baud_rate;
      X2 in_proto_mask;
      X2 out_proto_mask;
      X2 flags;
      U1 reserved1[2];
    } payload;
  } ubx_cfg_prt_;
  /* Rate config */
  struct UbxCfgRate {
    static constexpr uint8_t cls = UBX_CFG_CLS_;
    static constexpr uint8_t id = UBX_CFG_RATE_ID_;
    static constexpr uint16_t len = 6;
    struct {
      U2 meas_rate;
      U2 nav_rate;
      U2 time_ref;
    } payload;
    } ubx_cfg_rate_;
  /* Configure messages */
  struct UbxCfgMsg {
    static constexpr uint8_t cls = UBX_CFG_CLS_;
    static constexpr uint8_t id = UBX_CFG_MSG_ID_;
    static constexpr uint16_t len = 8;
    struct {
      U1 msg_class;
      U1 msg_id;
      U1 rate[6];
    } payload;
  } ubx_cfg_msg_;
  /* Configure NAV5 */
  struct UbxCfgNav5 {
    static constexpr uint8_t cls = UBX_CFG_CLS_;
    static constexpr uint8_t id = UBX_CFG_NAV5_ID_;
    static constexpr uint16_t len = 36;
    struct {
      X2 mask;
      U1 dyn_model;
      U1 fix_mode;
      I4 fixed_alt;
      U4 fixed_alt_var;
      I1 min_elev;
      U1 dr_limit;
      U2 p_dop;
      U2 t_dop;
      U2 p_acc;
      U2 t_acc;
      U1 static_hold_thresh;
      U1 dgnss_timeout;
      U1 cno_thresh_num_svs;
      U1 cno_thresh;
      U1 reserved0[2];
      U2 static_hold_max_dist;
      U1 utc_standard;
      U1 reserved1[5];
    } payload;
  } ubx_cfg_nav5_;
  /* Restore default config */
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
  } ubx_cfg_cfg_;
  /* Data */
};

}  // namespace bfs

#endif  // SRC_UBX_H_
