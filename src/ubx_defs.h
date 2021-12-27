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

#ifndef SRC_UBX_DEFS_H_
#define SRC_UBX_DEFS_H_

#include <cstdint>
#include <cstddef>

namespace bfs {

/*
* Common defs for UBX messages
*/

/*
* Aliases for uBlox defined types to make defining packets from the interface
* descriptions easier.
*/
using X1 = uint8_t;
using U1 = uint8_t;
using U2 = uint16_t;
using U4 = uint32_t;
using I1 = int8_t;
using I2 = int16_t;
using I4 = int32_t;
/* Max payload supported */
static constexpr std::size_t UBX_MAX_PAYLOAD_ = 1024;
/* Classes */
static constexpr uint8_t UBX_NAV_CLS_ = 0x01;
static constexpr uint8_t UBX_RXM_CLS_ = 0x02;
static constexpr uint8_t UBX_ACK_CLS_ = 0x05;
static constexpr uint8_t UBX_CFG_CLS_ = 0x06;
/* Message structure transmit / receive */
struct UbxMsg {
  uint8_t cls;
  uint8_t id;
  uint16_t len;
  uint8_t payload[UBX_MAX_PAYLOAD_];
};





// /* ID's */
// static constexpr uint8_t UBX_NAV_PVT_ID_ = 0x07;
// static constexpr uint8_t UBX_CFG_VALGET_ID_ = 0x8b;
// /* Maximum packet length that we'll receive */
// static constexpr std::size_t MAX_PACKET_LEN_ = 96;
// /* uBlox packet definitions */
// struct Ubx {
//   uint8_t cls;
//   uint8_t id;
//   uint8_t len;
//   void *payload;
// };
// struct UbxCfgValget {
//   U1 version;
//   U1 layer;
//   U2 position;
//   U4 key;
//   U1 val[8];
// };
// struct UbxNavPvt {
//   U4 itow;
//   U2 year;
//   U1 month;
//   U1 day;
//   U1 hour;
//   U1 min;
//   U1 sec;
//   X1 valid;
//   U4 tacc;
//   I4 nano;
//   U1 fix_type;
//   X1 flags;
//   X1 flags2;
//   U1 num_sv;
//   I4 lon;
//   I4 lat;
//   I4 height;
//   I4 h_msl;
//   U4 h_acc;
//   U4 v_acc;
//   I4 vel_n;
//   I4 vel_e;
//   I4 vel_d;
//   I4 g_speed;
//   I4 head_mot;
//   U4 s_acc;
//   U4 head_acc;
//   U2 pdop;
//   X1 flags3;
//   U1 resv0[5];
//   I4 head_veh;
//   I2 mag_dec;
//   U2 mag_acc;
// };

// /* Configuration keys */
// static constexpr uint32_t CFG_RATE_MEAS_ = 0x30210001;

}  // namespace bfs

#endif  // SRC_UBLOX_DEFS_H_
