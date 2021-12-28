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

#ifndef SRC_UBX_MON_H_
#define SRC_UBX_MON_H_

#include <cstdint>
#include <cstddef>
#include "ubx_defs.h"  // NOLINT

namespace bfs {

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

}  // namespace bfs

#endif  // SRC_UBX_MON_H_
