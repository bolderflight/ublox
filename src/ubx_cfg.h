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

namespace bfs {

/*
* Defs for UBX-CFG messages
*/

/* ID's */
static constexpr uint8_t UBX_CFG_MSG_ID_ = 0x01;
static constexpr uint8_t UBX_CFG_VALSET_ID_ = 0x8a;
static constexpr uint8_t UBX_CFG_VALGET_ID_ = 0x8b;
/* Constants */
static constexpr uint8_t UBX_CFG_VALGET_REQUEST_ = 0x00;
static constexpr uint8_t UBX_CFG_VALGET_RESPONSE_ = 0x01;
static constexpr uint8_t UBX_CFG_VALSET_TRANSACTIONLESS_ = 0x00;
/* Legacy Configuration structs */
struct UbxCfgMsg {
  uint8_t cls = UBX_CFG_CLS_;
  uint8_t id = UBX_CFG_MSG_ID_;
  uint16_t len = 2;
  struct {
    U1 msg_class;
    U1 msg_id;
  } payload;
};

// /* v9 Configuration Keys */
// /* CFG-MSGOUT (message output configuration) */
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_LOG_INFO_I2C_ = 0x20910259;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_LOG_INFO_SPI_ = 0x2091025d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_LOG_INFO_UART1_ = 0x2091025a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_LOG_INFO_UART2_ = 0x2091025b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_LOG_INFO_USB_ = 0x2091025c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_COMMS_I2C_ = 0x2091034f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_COMMS_SPI_ = 0x20910353;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_COMMS_UART1_ = 0x20910350;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_COMMS_UART2_ = 0x20910351;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_COMMS_USB_ = 0x20910352;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW2_I2C_ = 0x209101b9;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW2_SPI_ = 0x209101bd;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW2_UART1_ = 0x209101ba;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW2_UART2_ = 0x209101bb;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW2_USB_ = 0x209101bc;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW3_I2C_ = 0x20910354;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW3_SPI_ = 0x20910358;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW3_UART1_ = 0x20910355;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW3_UART2_ = 0x20910356;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW3_USB_ = 0x20910357;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW_I2C_ = 0x209101b4;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW_SPI_ = 0x209101b8;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW_UART1_ = 0x209101b5;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW_UART2_ = 0x209101b6;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_HW_USB_ = 0x209101b7;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_IO_I2C_ = 0x209101a5;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_IO_SPI_ = 0x209101a9;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_IO_UART1_ = 0x209101a6;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_IO_UART2_ = 0x209101a7;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_IO_USB_ = 0x209101a8;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_MSGPP_I2C_ = 0x20910196;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_MSGPP_SPI_ = 0x2091019a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_MSGPP_UART1_ = 0x20910197;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_MSGPP_UART2_ = 0x20910198;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_MSGPP_USB_ = 0x20910199;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RF_I2C_ = 0x20910359;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RF_SPI_ = 0x2091035d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RF_UART1_ = 0x2091035a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RF_UART2_ = 0x2091035b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RF_USB_ = 0x2091035c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXBUF_I2C_ = 0x209101a0;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXBUF_SPI_ = 0x209101a4;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXBUF_UART1_ = 0x209101a1;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXBUF_UART2_ = 0x209101a2;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXBUF_USB_ = 0x209101a3;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXR_I2C_ = 0x20910187;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXR_SPI_ = 0x2091018b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXR_UART1_ = 0x20910188;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXR_UART2_ = 0x20910189;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_RXR_USB_ = 0x2091018a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_SPAN_I2C_ = 0x2091038b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_SPAN_SPI_ = 0x2091038f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_SPAN_UART1_ = 0x2091038c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_SPAN_UART2_ = 0x2091038d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_SPAN_USB_ = 0x2091038e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_TXBUF_I2C_ = 0x2091019b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_TXBUF_SPI_ = 0x2091019f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_TXBUF_UART1_ = 0x2091019c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_TXBUF_UART2_ = 0x2091019d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_MON_TXBUF_USB_ = 0x2091019e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ATT_I2C_ = 0x2091001f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ATT_SPI_ = 0x20910023;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ATT_UART1_ = 0x20910020;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ATT_UART2_ = 0x20910021;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ATT_USB_ = 0x20910022;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_CLOCK_I2C_ = 0x20910065;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_CLOCK_SPI_ = 0x20910069;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_CLOCK_UART1_ = 0x20910066;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_CLOCK_UART2_ = 0x20910067;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_CLOCK_USB_ = 0x20910068;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_DOP_I2C_ = 0x20910038;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_DOP_SPI_ = 0x2091003c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_DOP_UART1_ = 0x20910039;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_DOP_UART2_ = 0x2091003a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_DOP_USB_ = 0x2091003b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_EOE_I2C_ = 0x2091015f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_EOE_SPI_ = 0x20910163;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_EOE_UART1_ = 0x20910160;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_EOE_UART2_ = 0x20910161;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_EOE_USB_ = 0x20910162;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_GEOFENCE_I2C_ = 0x209100a1;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_GEOFENCE_SPI_ = 0x209100a5;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_GEOFENCE_UART1_ = 0x209100a2;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_GEOFENCE_UART2_ = 0x209100a3;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_GEOFENCE_USB_ = 0x209100a4;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_I2C_ = 0x2091002e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_SPI_ = 0x20910032;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1_ = 0x2091002f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART2_ = 0x20910030;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB_ = 0x20910031;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_I2C_ = 0x20910033;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_SPI_ = 0x20910037;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1_ = 0x20910034;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART2_ = 0x20910035;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB_ = 0x20910036;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ODO_I2C_ = 0x2091007e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ODO_SPI_ = 0x20910082;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ODO_UART1_ = 0x2091007f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ODO_UART2_ = 0x20910080;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ODO_USB_ = 0x20910081;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ORB_I2C_ = 0x20910010;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ORB_SPI_ = 0x20910014;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ORB_UART1_ = 0x20910011;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ORB_UART2_ = 0x20910012;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_ORB_USB_ = 0x20910013;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSECEF_I2C_ = 0x20910024;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSECEF_SPI_ = 0x20910028;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSECEF_UART1_ = 0x20910025;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSECEF_UART2_ = 0x20910026;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSECEF_USB_ = 0x20910027;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSLLH_I2C_ = 0x20910029;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSLLH_SPI_ = 0x2091002d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSLLH_UART1_ = 0x2091002a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSLLH_UART2_ = 0x2091002b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_POSLLH_USB_ = 0x2091002c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_PVT_I2C_ = 0x20910006;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_PVT_SPI_ = 0x2091000a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_PVT_UART1_ = 0x20910007;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_PVT_UART2_ = 0x20910008;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_PVT_USB_ = 0x20910009;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_RELPOSNED_I2C_ = 0x2091008d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_RELPOSNED_SPI_ = 0x20910091;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1_ = 0x2091008e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2_ = 0x2091008f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_RELPOSNED_USB_ = 0x20910090;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SAT_I2C_ = 0x20910015;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SAT_SPI_ = 0x20910019;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SAT_UART1_ = 0x20910016;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SAT_UART2_ = 0x20910017;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SAT_USB_ = 0x20910018;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SBAS_I2C_ = 0x2091006a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SBAS_SPI_ = 0x2091006e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SBAS_UART1_ = 0x2091006b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SBAS_UART2_ = 0x2091006c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SBAS_USB_ = 0x2091006d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SIG_I2C_ = 0x20910345;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SIG_SPI_ = 0x20910349;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SIG_UART1_ = 0x20910346;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SIG_UART2_ = 0x20910347;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SIG_USB_ = 0x20910348;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SLAS_I2C_ = 0x20910336;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SLAS_SPI_ = 0x2091033a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SLAS_UART1_ = 0x20910337;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SLAS_UART2_ = 0x20910338;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SLAS_USB_ = 0x20910339;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_STATUS_I2C_ = 0x2091001a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_STATUS_SPI_ = 0x2091001e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_STATUS_UART1_ = 0x2091001b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_STATUS_UART2_ = 0x2091001c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_STATUS_USB_ = 0x2091001d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SVIN_I2C_ = 0x20910088;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SVIN_SPI_ = 0x2091008c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SVIN_UART1_ = 0x20910089;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SVIN_UART2_ = 0x2091008a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_SVIN_USB_ = 0x2091008b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEBDS_I2C_ = 0x20910051;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEBDS_SPI_ = 0x20910055;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEBDS_UART1_ = 0x20910052;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEBDS_UART2_ = 0x20910053;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEBDS_USB_ = 0x20910054;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGAL_I2C_ = 0x20910056;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGAL_SPI_ = 0x2091005a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGAL_UART1_ = 0x20910057;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGAL_UART2_ = 0x20910058;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGAL_USB_ = 0x20910059;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGLO_I2C_ = 0x2091004c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGLO_SPI_ = 0x20910050;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGLO_UART1_ = 0x2091004d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGLO_UART2_ = 0x2091004e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGLO_USB_ = 0x2091004f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGPS_I2C_ = 0x20910047;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGPS_SPI_ = 0x2091004b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1_ = 0x20910048;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGPS_UART2_ = 0x20910049;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEGPS_USB_ = 0x2091004a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMELS_I2C_ = 0x20910060;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMELS_SPI_ = 0x20910064;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMELS_UART1_ = 0x20910061;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMELS_UART2_ = 0x20910062;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMELS_USB_ = 0x20910063;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_I2C_ = 0x20910386;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_SPI_ = 0x2091038a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_UART1_ = 0x20910387;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_UART2_ = 0x20910388;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_USB_ = 0x20910389;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEUTC_I2C_ = 0x2091005b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEUTC_SPI_ = 0x2091005f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1_ = 0x2091005c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART2_ = 0x2091005d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_TIMEUTC_USB_ = 0x2091005e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELECEF_I2C_ = 0x2091003d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELECEF_SPI_ = 0x20910041;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELECEF_UART1_ = 0x2091003e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELECEF_UART2_ = 0x2091003f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELECEF_USB_ = 0x20910040;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELNED_I2C_ = 0x20910042;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELNED_SPI_ = 0x20910046;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELNED_UART1_ = 0x20910043;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELNED_UART2_ = 0x20910044;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_NAV_VELNED_USB_ = 0x20910045;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_MEASX_I2C_ = 0x20910204;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_MEASX_SPI_ = 0x20910208;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_MEASX_UART1_ = 0x20910205;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_MEASX_UART2_ = 0x20910206;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_MEASX_USB_ = 0x20910207;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RAWX_I2C_ = 0x209102a4;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RAWX_SPI_ = 0x209102a8;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RAWX_UART1_ = 0x209102a5;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RAWX_UART2_ = 0x209102a6;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RAWX_USB_ = 0x209102a7;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RLM_I2C_ = 0x2091025e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RLM_SPI_ = 0x20910262;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RLM_UART1_ = 0x2091025f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RLM_UART2_ = 0x20910260;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RLM_USB_ = 0x20910261;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RTCM_I2C_ = 0x20910268;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RTCM_SPI_ = 0x2091026c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RTCM_UART1_ = 0x20910269;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RTCM_UART2_ = 0x2091026a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_RTCM_USB_ = 0x2091026b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_SFRBX_I2C_ = 0x20910231;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_SFRBX_SPI_ = 0x20910235;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_SFRBX_UART1_ = 0x20910232;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_SFRBX_UART2_ = 0x20910233;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_RXM_SFRBX_USB_ = 0x20910234;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TM2_I2C_ = 0x20910178;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TM2_SPI_ = 0x2091017c;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TM2_UART1_ = 0x20910179;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TM2_UART2_ = 0x2091017a;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TM2_USB_ = 0x2091017b;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TP_I2C_ = 0x2091017d;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TP_SPI_ = 0x20910181;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TP_UART1_ = 0x2091017e;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TP_UART2_ = 0x2091017f;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_TP_USB_ = 0x20910180;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_VRFY_I2C_ = 0x20910092;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_VRFY_SPI_ = 0x20910096;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_VRFY_UART1_ = 0x20910093;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_VRFY_UART2_ = 0x20910094;
// static constexpr uint32_t UBX_CFG_MSGOUT_UBX_TIM_VRFY_USB_ = 0x20910095;

// /* CFG-I2CINPROT (I2C input protocol configuration) */
// static constexpr uint32_t UBX_CFG_I2CINPROT_UBX_ = 0x10710001;
// static constexpr uint32_t UBX_CFG_I2CINPROT_NMEA_ = 0x10710002;
// static constexpr uint32_t UBX_CFG_I2CINPROT_RTCM3X_ = 0x10710004;

// /* CFG-I2COUTPROT (I2C output protocol configuration) */
// static constexpr uint32_t UBX_CFG_I2COUTPROT_UBX_ = 0x10720001;
// static constexpr uint32_t UBX_CFG_I2COUTPROT_NMEA_ = 0x10720002;
// static constexpr uint32_t UBX_CFG_I2COUTPROT_RTCM3X_ = 0x10720004;

// /* CFG-UART1INPROT (UART1 input protocol configuration) */
// static constexpr uint32_t UBX_CFG_UART1INPROT_UBX_ = 0x10730001;
// static constexpr uint32_t UBX_CFG_UART1INPROT_NMEA_ = 0x10730002;
// static constexpr uint32_t UBX_CFG_UART1INPROT_RTCM3X_ = 0x10730004;

// /* CFG-UART1OUTPROT (UART1 output protocol configuration) */
// static constexpr uint32_t UBX_CFG_UART1OUTPROT_UBX_ = 0x10740001;
// static constexpr uint32_t UBX_CFG_UART1OUTPROT_NMEA_ = 0x10740002;
// static constexpr uint32_t UBX_CFG_UART1OUTPROT_RTCM3X_ = 0x10740004;

// /* CFG-UART2INPROT (UART2 input protocol configuration) */
// static constexpr uint32_t UBX_CFG_UART2INPROT_UBX_ = 0x10750001;
// static constexpr uint32_t UBX_CFG_UART2INPROT_NMEA_ = 0x10750002;
// static constexpr uint32_t UBX_CFG_UART2INPROT_RTCM3X_ = 0x10750004;

// /* CFG-UART2OUTPROT (UART2 output protocol configuration) */
// static constexpr uint32_t UBX_CFG_UART2OUTPROT_UBX_ = 0x10760001;
// static constexpr uint32_t UBX_CFG_UART2OUTPROT_NMEA_ = 0x10760002;
// static constexpr uint32_t UBX_CFG_UART2OUTPROT_RTCM3X_ = 0x10760004;

// /* CFG-USBINPROT (USB input protocol configuration) */
// static constexpr uint32_t UBX_CFG_USBINPROT_UBX_ = 0x10770001;
// static constexpr uint32_t UBX_CFG_USBINPROT_NMEA_ = 0x10770002;
// static constexpr uint32_t UBX_CFG_USBINPROT_RTCM3X_ = 0x10770004;

// /* CFG-USBOUTPROT (USB output protocol configuration) */
// static constexpr uint32_t UBX_CFG_USBOUTPROT_UBX_ = 0x10780001;
// static constexpr uint32_t UBX_CFG_USBOUTPROT_NMEA_ = 0x10780002;
// static constexpr uint32_t UBX_CFG_USBOUTPROT_RTCM3X_ = 0x10780004;

// /* CFG-SPIINPROT (SPI input protocol configuration) */
// static constexpr uint32_t UBX_CFG_SPIINPROT_UBX_ = 0x10790001;
// static constexpr uint32_t UBX_CFG_SPIINPROT_NMEA_ = 0x10790002;
// static constexpr uint32_t UBX_CFG_SPIINPROT_RTCM3X_ = 0x10790004;

// /* CFG-SPIOUTPROT (SPI output protocol configuration) */
// static constexpr uint32_t UBX_CFG_SPIOUTPROT_UBX_ = 0x107a0001;
// static constexpr uint32_t UBX_CFG_SPIOUTPROT_NMEA_ = 0x107a0002;
// static constexpr uint32_t UBX_CFG_SPIOUTPROT_RTCM3X_ = 0x107a0004;

// /* CFG-RATE */
// static constexpr uint32_t UBX_CFG_RATE_MEAS_ = 0x30210001;
// static constexpr uint32_t UBX_CFG_RATE_NAV_ = 0x30210002;
// static constexpr uint32_t UBX_CFG_RATE_TIMEREF_ = 0x20210003;

// /* CFG-NAVHPG High precision navigation conﬁguration */
// static constexpr uint32_t UBX_CFG_NAVHPG_DGNSSMODE_ = 0x20140011;
// static constexpr uint8_t UBX_CFG_NAVHPG_DGNSSMODE_RTK_FLOAT_ = 2;
// static constexpr uint8_t UBX_CFG_NAVHPG_DGNSSMODE_RTK_FIXED_ = 3;

// /* CFG-NAVSPG Standard precision navigation conﬁguration */
// static constexpr uint32_t UBX_CFG_NAVSPG_FIXMODE_ = 0x20110011;
// static constexpr uint32_t UBX_CFG_NAVSPG_INIFIX3D_ = 0x10110013;
// static constexpr uint32_t UBX_CFG_NAVSPG_DYNMODEL_ = 0x20110021;
// static constexpr uint8_t UBX_CFG_NAVSPG_FIXMODE_2D_ONLY_ = 1;
// static constexpr uint8_t UBX_CFG_NAVSPG_FIXMODE_3D_ONLY_ = 2;
// static constexpr uint8_t UBX_CFG_NAVSPG_FIXMODE_AUTO_ = 3;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_PORT_ = 0;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_STAT_ = 2;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_PED_ = 3;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_AUTO_ = 4;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_SEA_ = 5;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_AIR1_ = 6;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_AIR2_ = 7;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_AIR4_ = 8;
// static constexpr uint8_t UBX_CFG_NAVSPG_DYNMODEL_WRIST_ = 9;



}  // namespace bfs

#endif  // SRC_UBX_CFG_H_
