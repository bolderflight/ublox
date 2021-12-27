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
#include "units.h"  // NOLINT
#include "ubx_defs.h"  // NOLINT
#include "ubx_cfg.h"  // NOLINT
#include "ubx_nav.h"  // NOLINT
#include "ubx_ack.h"  // NOLINT

namespace bfs {

class Ubx {
 public:
  explicit Ubx(HardwareSerial* bus) : bus_(bus) {}
  bool Begin(const uint32_t baud);
  void Read() {

  }

//  private:
  /* Communication */
  HardwareSerial* bus_;
  /* Error */
  uint8_t err_;
  /* Parsing */
  static constexpr uint8_t UBX_HEADER_[2] = {0xB5, 0x62};
  static constexpr uint8_t UBX_CLS_POS_ = 2;
  static constexpr uint8_t UBX_ID_POS_ = 3;
  static constexpr uint8_t UBX_LEN_POS_LSB_ = 4;
  static constexpr uint8_t UBX_LEN_POS_MSB_ = 5;
  static constexpr uint8_t UBX_HEADER_LEN_ = 6;
  static constexpr uint8_t UBX_CHK_OFFSET_ = UBX_HEADER_LEN_ -
                                             sizeof(UBX_HEADER_);
  UbxMsg rxmsg_, txmsg_;
  /* Helper functions */
  /*
  * Parses incoming messages given a byte from a bus and a UbxMsg pointer to
  * store received messages, returns true if a valid message is received
   */
  bool ParseMsg(const uint8_t c, UbxMsg * const msg);
  /* Sends a UBX message */
  void SendMsg(const UbxMsg &ref);
  /* UBX checksum calculation */
  uint16_t Checksum(uint8_t const * const data, const uint16_t len,
                    bool reset_states);
  /* Legacy Configuration */
  template<typename T>
  bool SendLegacyCfg(const T &ref) {
    txmsg_.cls = ref.cls;
    txmsg_.id = ref.id;
    txmsg_.len = ref.len;
    memcpy(txmsg_.payload, ref.payload, ref.len);
    SendMsg(txmsg_);
  }
  /* v9 Configuration */
  /* Set a config value in RAM */
  template<typename T>
  bool SetCfgVal(const uint32_t key, const T val) {
    static constexpr int16_t TIMEOUT_MS = 1000;
    if (!SendCfgValSet(key, val, &txmsg_)) {return false;}
    elapsedMillis t_ms;
    while (t_ms < TIMEOUT_MS) {
      if (bus_->available()) {
        if (ParseMsg(bus_->read(), &rxmsg_)) {
          if (rxmsg_.cls == UBX_ACK_CLS_) {
            uint8_t cls, id;
            bool resp;
            if (ParseAck(rxmsg_, &cls, &id, &resp)) {
              /* Check that the ACK / NACK is in response to the set cmd */
              if ((cls == UBX_CFG_CLS_) && (id = UBX_CFG_VALSET_ID_)) {
                return resp;
              }
            }
          }
        }
      }
    }
    return false;
  }
  /* Get a config value from RAM */
  template<typename T>
  bool GetCfgVal(const uint32_t key, T * const val) {
    if (!val) {return false;}
    static constexpr int16_t TIMEOUT_MS = 1000;
    if (!SendCfgValGet(key, &txmsg_)) {return false;}
    elapsedMillis t_ms;
    while (t_ms < TIMEOUT_MS) {
      if (bus_->available()) {
        if (ParseMsg(bus_->read(), &rxmsg_)) {
          if ((rxmsg_.cls == UBX_CFG_CLS_) &&
              (rxmsg_.id == UBX_CFG_VALGET_ID_)) {
            uint32_t ret_key;
            T ret_val;
            if (ParseCfgValGet<T>(rxmsg_, &ret_key, &ret_val)) {
              if (ret_key == key) {
                *val = ret_val;
                return true;
              }
            }
          }
        }
      }
    }
    return false;
  }
  /* UBX-ACK-ACK / UBX-ACK-NACK parsing */
  bool ParseAck(const UbxMsg &msg, uint8_t * const cls,
                uint8_t * const id, bool * const resp);
  /* Send a UBX-CFG-VALSET command */
  template<typename T>
  bool SendCfgValSet(const uint32_t key, const T val, UbxMsg * const msg) {
    if (!msg) {return false;}
    msg->cls = UBX_CFG_CLS_;
    msg->id = UBX_CFG_VALSET_ID_;
    msg->len = 8 + sizeof(T);
    msg->payload[0] = UBX_CFG_VALSET_TRANSACTIONLESS_;
    msg->payload[1] = 0x01;  // RAM
    msg->payload[2] = 0;  // unused
    msg->payload[3] = 0;
    memcpy(msg->payload + 4, &key, sizeof(uint32_t));
    memcpy(msg->payload + 8, &val, sizeof(T));
    SendMsg(*msg);
    return true;
  }
  /* Send a UBX-CFG-VALGET request */
  bool SendCfgValGet(const uint32_t key, UbxMsg * const msg);
  /* Parse a UBX-CFG-VALGET response */
  template<typename T>
  bool ParseCfgValGet(const UbxMsg &msg, uint32_t * const key, T * const val) {
    static constexpr uint8_t HEADER_LEN = 4;
    if ((!key) || (!val)) {return false;}
    if (msg.len != HEADER_LEN + sizeof(uint32_t) + sizeof(T)) {return false;}
    if (msg.payload[0] != UBX_CFG_VALGET_RESPONSE_) {return false;}
    if (msg.payload[1] != 0x00) {return false;}  // RAM layer
    memcpy(key, msg.payload + 4, sizeof(uint32_t));
    memcpy(val, msg.payload + 8, sizeof(T));
    return true;
  }
};

}  // namespace bfs

#endif  // SRC_UBX_H_
