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

#ifndef SRC_UBX_CFG_H_
#define SRC_UBX_CFG_H_

#include <cstdint>
#include <cstddef>
#include "ubx_defs.h"

namespace bfs {
/*
* Defs for UBX-CFG messages
*/
/* UBX-CFG IDs */
static constexpr uint8_t UBX_CFG_ANT_ID_ = 0x13;
static constexpr uint8_t UBX_CFG_CFG_ID_ = 0x09;
static constexpr uint8_t UBX_CFG_DAT_ID_ = 0x06;
static constexpr uint8_t UBX_CFG_DGNSS_ID_ = 0x70;
static constexpr uint8_t UBX_CFG_GEOFENCE_ID_ = 0x69;
static constexpr uint8_t UBX_CFG_GNSS_ID_ = 0x3e;
static constexpr uint8_t UBX_CFG_INF_ID_ = 0x02;
static constexpr uint8_t UBX_CFG_ITFM_ID_ = 0x39;
static constexpr uint8_t UBX_CFG_LOGFILTER_ID_ = 0x47;
static constexpr uint8_t UBX_CFG_MSG_ID_ = 0x01;
static constexpr uint8_t UBX_CFG_NAV5_ID_ = 0x24;
static constexpr uint8_t UBX_CFG_NAVX5_ID_ = 0x23;
static constexpr uint8_t UBX_CFG_NMEA_ID_ = 0x17;
static constexpr uint8_t UBX_CFG_ODO_ID_ = 0x1e;
static constexpr uint8_t UBX_CFG_PRT_ID_ = 0x00;
static constexpr uint8_t UBX_CFG_PWR_ID_ = 0x57;
static constexpr uint8_t UBX_CFG_RATE_ID_ = 0x08;
static constexpr uint8_t UBX_CFG_RINV_ID_ = 0x34;
static constexpr uint8_t UBX_CFG_RST_ID_ = 0x04;
static constexpr uint8_t UBX_CFG_SBAS_ID_ = 0x16;
static constexpr uint8_t UBX_CFG_TMODE3_ID_ = 0x71;
static constexpr uint8_t UBX_CFG_TP5_ID_ = 0x31;
static constexpr uint8_t UBX_CFG_USB_ID_ = 0x1b;
static constexpr uint8_t UBX_CFG_VALDEL_ID_ = 0x8c;
static constexpr uint8_t UBX_CFG_VALGET_ID_ = 0x8b;
static constexpr uint8_t UBX_CFG_VALSET_ID_ = 0x8a;
/* UBX-CFG Messages */
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
};
struct UbxCfgMsgSet {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_MSG_ID_;
  static constexpr uint16_t len = 8;
  struct {
    U1 msg_class;
    U1 msg_id;
    U1 rate[6];
  } payload;
};
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
};
struct UbxCfgPrtReq {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_PRT_ID_;
  static constexpr uint16_t len = 1;
  struct {
    U1 port_id;
  } payload;
};
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
};
struct UbxCfgRate {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_RATE_ID_;
  static constexpr uint16_t len = 6;
  struct {
    U2 meas_rate;
    U2 nav_rate;
    U2 time_ref;
  } payload;
};
template<std::size_t N>
struct UbxCfgValset {
  static constexpr uint8_t cls = UBX_CFG_CLS_;
  static constexpr uint8_t id = UBX_CFG_VALSET_ID_;
  uint16_t len;
  struct {
    U1 version = 0x00;
    X1 layers;
    U1 reserved0[2];
    U1 cfg_data[N];
  } payload;
};

}  // namespace bfs

#endif  // SRC_UBX_CFG_H_
