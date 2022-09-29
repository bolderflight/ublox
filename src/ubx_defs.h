/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#if !defined(ARDUINO)
#include <cstdint>
#include <cstddef>
#endif

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

}  // namespace bfs

#endif  // SRC_UBX_DEFS_H_
