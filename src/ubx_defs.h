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
/* Max payload supported */
static constexpr std::size_t UBX_MAX_PAYLOAD_ = 1024;
/* Classes */
static constexpr uint8_t UBX_ACK_CLS_ = 0x05;
static constexpr uint8_t UBX_CFG_CLS_ = 0x06;
static constexpr uint8_t UBX_INF_CLS_ = 0x04;
static constexpr uint8_t UBX_LOG_CLS_ = 0x21;
static constexpr uint8_t UBX_MGA_CLS_ = 0x13;
static constexpr uint8_t UBX_MON_CLS_ = 0x0a;
static constexpr uint8_t UBX_NAV_CLS_ = 0x01;
static constexpr uint8_t UBX_RXM_CLS_ = 0x02;
static constexpr uint8_t UBX_SEC_CLS_ = 0x27;
static constexpr uint8_t UBX_TIM_CLS_ = 0x0d;
static constexpr uint8_t UBX_UPD_CLS_ = 0x09;
/* UBX-ACK IDs */
static constexpr uint8_t UBX_ACK_ACK_ID_ = 0x01;
static constexpr uint8_t UBX_ACK_NAK_ID_ = 0x00;
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
/* UBX-INF IDs */
static constexpr uint8_t UBX_INF_DEBUG_ID_ = 0x04;
static constexpr uint8_t UBX_INF_ERROR_ID_ = 0x00;
static constexpr uint8_t UBX_INF_NOTICE_ID_ = 0x02;
static constexpr uint8_t UBX_INF_TEST_ID_ = 0x03;
static constexpr uint8_t UBX_INF_WARNING_ID_ = 0x01;
/* UBX-LOG IDs */
static constexpr uint8_t UBX_LOG_CREATE_ID_ = 0x07;
static constexpr uint8_t UBX_LOG_ERASE_ID_ = 0x03;
static constexpr uint8_t UBX_LOG_FINDTIME_ID_ = 0x0e;
static constexpr uint8_t UBX_LOG_INFO_ID_ = 0x08;
static constexpr uint8_t UBX_LOG_RETRIEVE_ID_ = 0x09;
static constexpr uint8_t UBX_LOG_RETRIEVEPOS_ID_ = 0x0b;
static constexpr uint8_t UBX_LOG_RETRIEVEPOSEXTRA_ID_ = 0x0f;
static constexpr uint8_t UBX_LOG_RETRIEVESTRING_ID_ = 0x0d;
static constexpr uint8_t UBX_LOG_STRING_ID_ = 0x04;
/* UBX-MGA IDs */
static constexpr uint8_t UBX_MGA_ACK_ID_ = 0x60;
static constexpr uint8_t UBX_MGA_BDS_ID_ = 0x03;
static constexpr uint8_t UBX_MGA_DBD_ID_ = 0x80;
static constexpr uint8_t UBX_MGA_GAL_ID_ = 0x02;
static constexpr uint8_t UBX_MGA_GLO_ID_ = 0x06;
static constexpr uint8_t UBX_MGA_GPS_ID_ = 0x00;
static constexpr uint8_t UBX_MGA_INI_ID_ = 0x40;
static constexpr uint8_t UBX_MGA_QZSS_ID_ = 0x05;
/* UBX-MON IDs */
static constexpr uint8_t UBX_MON_COMMS_ID_ = 0x36;
static constexpr uint8_t UBX_MON_GNSS_ID_ = 0x28;
static constexpr uint8_t UBX_MON_HW_ID_ = 0x09;
static constexpr uint8_t UBX_MON_HW2_ID_ = 0x0b;
static constexpr uint8_t UBX_MON_HW3_ID_ = 0x37;
static constexpr uint8_t UBX_MON_IO_ID_ = 0x02;
static constexpr uint8_t UBX_MON_MSGPP_ID_ = 0x06;
static constexpr uint8_t UBX_MON_PATCH_ID_ = 0x27;
static constexpr uint8_t UBX_MON_RF_ID_ = 0x38;
static constexpr uint8_t UBX_MON_RXBUF_ID_ = 0x07;
static constexpr uint8_t UBX_MON_RXR_ID_ = 0x21;
static constexpr uint8_t UBX_MON_SPAN_ID_ = 0x31;
static constexpr uint8_t UBX_MON_TXBUF_ID_ = 0x08;
static constexpr uint8_t UBX_MON_VER_ID_ = 0x04;

/* UBX-RXM IDs */
static constexpr uint8_t UBX_RXM_MEASX_ID_ = 0x14;
static constexpr uint8_t UBX_RXM_PMREQ_ID_ = 0x41;
static constexpr uint8_t UBX_RXM_RAWX_ID_ = 0x15;
static constexpr uint8_t UBX_RXM_RLM_ID_ = 0x59;
static constexpr uint8_t UBX_RXM_RTCM_ID_ = 0x32;
static constexpr uint8_t UBX_RXM_SFRBX_ID_ = 0x13;
/* UBX-SEC IDs */
static constexpr uint8_t UBX_SEC_UNIQID_ID_ = 0x03;
/* UBX-TIME IDs */
static constexpr uint8_t UBX_TIM_TM2_ID_ = 0x03;
static constexpr uint8_t UBX_TIM_TP_ID_ = 0x01;
static constexpr uint8_t UBX_TIM_VRFY_ID_ = 0x06;
/* UBX-UPD IDs */
static constexpr uint8_t UBX_UPD_SOS_ID_ = 0x14;











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
