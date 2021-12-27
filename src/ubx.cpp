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

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include <cstddef>
#include <cstdint>
#include "units.h"  // NOLINT
#include "ubx.h"  // NOLINT
#include "ubx_defs.h"  // NOLINT
#include "ubx_cfg.h"  // NOLINT
#include "ubx_nav.h"  // NOLINT
#include "ubx_ack.h"  // NOLINT

namespace bfs {

constexpr uint8_t Ubx::UBX_HEADER_[];

bool Ubx::Begin(const uint32_t baud) {
  bus_->begin(baud);
  /* Disable NMEA outputs */
  if (!SetCfgVal<U1>(UBX_CFG_UART1OUTPROT_NMEA_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_UART2OUTPROT_NMEA_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_USBOUTPROT_NMEA_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_I2COUTPROT_NMEA_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_SPIOUTPROT_NMEA_, 0)) {return false;}
  /* Disable NMEA inputs */
  if (!SetCfgVal<U1>(UBX_CFG_UART1INPROT_NMEA_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_UART2INPROT_NMEA_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_USBINPROT_NMEA_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_I2CINPROT_NMEA_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_SPIINPROT_NMEA_, 0)) {return false;}
  /* Disable RTCM outputs */
  if (!SetCfgVal<U1>(UBX_CFG_UART1OUTPROT_RTCM3X_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_UART2OUTPROT_RTCM3X_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_USBOUTPROT_RTCM3X_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_I2COUTPROT_RTCM3X_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_SPIOUTPROT_RTCM3X_, 0)) {return false;}
  /* Disable RTCM inputs */
  if (!SetCfgVal<U1>(UBX_CFG_UART1INPROT_RTCM3X_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_UART2INPROT_RTCM3X_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_USBINPROT_RTCM3X_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_I2CINPROT_RTCM3X_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_SPIINPROT_RTCM3X_, 0)) {return false;}
  /* Enable UBX output on UART1 */
  if (!SetCfgVal<U1>(UBX_CFG_UART1OUTPROT_UBX_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_UART2OUTPROT_UBX_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_USBOUTPROT_UBX_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_I2COUTPROT_UBX_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_SPIOUTPROT_UBX_, 0)) {return false;}
  /* Enable UBX input on UART1 */
  if (!SetCfgVal<U1>(UBX_CFG_UART1INPROT_UBX_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_UART2INPROT_UBX_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_USBINPROT_UBX_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_I2CINPROT_UBX_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_SPIINPROT_UBX_, 0)) {return false;}
  /* Select UBX Messages */
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_LOG_INFO_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_COMMS_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_HW2_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_HW3_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_HW_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_IO_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_MSGPP_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_RF_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_RXBUF_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_RXR_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_SPAN_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_MON_TXBUF_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_CLOCK_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_DOP_UART1_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_EOE_UART1_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_GEOFENCE_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_ODO_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_ORB_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_POSECEF_UART1_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_POSLLH_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_PVT_UART1_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_SAT_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_SBAS_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_SIG_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_SLAS_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_STATUS_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_SVIN_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_TIMEBDS_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_TIMEGAL_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_TIMEGLO_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_TIMELS_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_VELECEF_UART1_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_NAV_VELNED_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_RXM_MEASX_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_RXM_RAWX_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_RXM_RLM_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_RXM_RTCM_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_RXM_SFRBX_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_TIM_TM2_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_TIM_TP_UART1_, 0)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_MSGOUT_UBX_TIM_VRFY_UART1_, 0)) {return false;}
  /* Set output rate tp 10 Hz */
  if (!SetCfgVal<U2>(UBX_CFG_RATE_MEAS_, 100)) {return false;}
  /* Standard precision navigation conﬁguration */
  if (!SetCfgVal<U1>(UBX_CFG_NAVSPG_FIXMODE_,
                     UBX_CFG_NAVSPG_FIXMODE_3D_ONLY_)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_NAVSPG_INIFIX3D_, 1)) {return false;}
  if (!SetCfgVal<U1>(UBX_CFG_NAVSPG_DYNMODEL_,
                     UBX_CFG_NAVSPG_DYNMODEL_AIR4_)) {return false;}
  /* CFG-NAVHPG High precision navigation conﬁguration */
  if (!SetCfgVal<U1>(UBX_CFG_NAVHPG_DGNSSMODE_,
                     UBX_CFG_NAVHPG_DGNSSMODE_RTK_FIXED_)) {return false;}
  return true;
}
bool Ubx::ParseMsg(const uint8_t c, UbxMsg * const msg) {
  static uint8_t len, chk;
  static std::size_t parser_state;
  /* Packet header */
  if (parser_state < sizeof(UBX_HEADER_)) {
    if (c == UBX_HEADER_[parser_state]){
      parser_state++;
    } else {
      parser_state = 0;
    }
  /* Class */
  } else if (parser_state == UBX_CLS_POS_) {
    msg->cls = c;
    parser_state++;
  /* ID */
  } else if (parser_state == UBX_ID_POS_) {
    msg->id = c;
    parser_state++;
  /* Length */
  } else if (parser_state == UBX_LEN_POS_LSB_) {
    len = c;
    parser_state++;
  } else if (parser_state == UBX_LEN_POS_MSB_) {
    msg->len = static_cast<uint16_t>(c) << 8 | len;
    parser_state++;
    /* Prevent buffer overflow */
    if (msg->len > UBX_MAX_PAYLOAD_) {
      parser_state = 0;
    }
  /* Payload */
  } else if (parser_state < (msg->len + UBX_HEADER_LEN_)) {
    msg->payload[parser_state - UBX_HEADER_LEN_] = c;
    parser_state++;
  /* Checksum */
  } else if (parser_state == (msg->len + UBX_HEADER_LEN_)) {
    chk = c;
    parser_state++;
  } else {
    uint16_t chk_rx = static_cast<uint16_t>(c) << 8 | chk;
    uint16_t chk_cmp = Checksum((uint8_t *)(msg), msg->len + UBX_CHK_OFFSET_,
                                true);
    parser_state = 0;
    /* Check the computed & received checksums */
    if (chk_rx == chk_cmp) {
      /* Valid message */
      return true;
    } else {
      return false;
    }
  }
  return false;
}
void Ubx::SendMsg(const UbxMsg &ref) {
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
  bus_->write(ref.payload, ref.len);
  chk = Checksum(ref.payload, ref.len, false);
  /* Write the checksum */
  bus_->write(static_cast<uint8_t>(chk & 0xFF));
  bus_->write(static_cast<uint8_t>(chk >> 8 & 0xFF));
}
uint16_t Ubx::Checksum(uint8_t const * const data, const uint16_t len,
                       bool reset_states) {
  if (!data) {return 0;}
  static uint8_t chk[2];
  if (reset_states) {
    chk[0] = 0;
    chk[1] = 0;
  }
  for (std::size_t i = 0; i < len; i++) {
    chk[0] += data[i];
    chk[1] += chk[0];
  }
  return static_cast<uint16_t>(chk[1]) << 8 | chk[0];
}
bool Ubx::ParseAck(const UbxMsg &msg, uint8_t * const cls,
                   uint8_t * const id, bool * const resp) {
  if ((!cls) || (!id) || (!resp)) {return false;}
  if (msg.cls != UBX_ACK_CLS_) {return false;}
  *cls = msg.payload[0];
  *id = msg.payload[1];
  *resp = (msg.id == UBX_ACK_ACK_ID_);
  return true;
}
bool Ubx::SendCfgValGet(const uint32_t key, UbxMsg * const msg) {
  if (!msg) {return false;}
  msg->cls = UBX_CFG_CLS_;
  msg->id = UBX_CFG_VALGET_ID_;
  msg->len = 8;
  msg->payload[0] = UBX_CFG_VALGET_REQUEST_;
  msg->payload[1] = 0x00;  // RAM layer
  msg->payload[2] = 0;  // position
  msg->payload[3] = 0;  // position
  memcpy(msg->payload + 4, &key, sizeof(uint32_t));
  SendMsg(*msg);
  return true;
}

}  // namespace bfs
