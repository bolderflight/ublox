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

#ifndef SRC_UBX_RXM_H_
#define SRC_UBX_RXM_H_

#include <cstdint>
#include <cstddef>
#include "ubx_defs.h"  // NOLINT

namespace bfs {

/* UBX-RXM IDs */
static constexpr uint8_t UBX_RXM_MEASX_ID_ = 0x14;
static constexpr uint8_t UBX_RXM_PMREQ_ID_ = 0x41;
static constexpr uint8_t UBX_RXM_RAWX_ID_ = 0x15;
static constexpr uint8_t UBX_RXM_RLM_ID_ = 0x59;
static constexpr uint8_t UBX_RXM_RTCM_ID_ = 0x32;
static constexpr uint8_t UBX_RXM_SFRBX_ID_ = 0x13;

}  // namespace bfs

#endif  // SRC_UBX_RXM_H_
