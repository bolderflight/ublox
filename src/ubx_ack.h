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

#ifndef SRC_UBX_ACK_H_
#define SRC_UBX_ACK_H_

#include <cstdint>
#include <cstddef>
#include "ubx_defs.h"

namespace bfs {
/*
* Defs for UBX-ACK messages
*/
/* UBX-ACK IDs */
static constexpr uint8_t UBX_ACK_ACK_ID_ = 0x01;
static constexpr uint8_t UBX_ACK_NAK_ID_ = 0x00;
/* UBX-ACK messages */
struct UbxAckAck {
  static constexpr uint8_t cls = UBX_ACK_CLS_;
  static constexpr uint8_t id = UBX_ACK_ACK_ID_;
  static constexpr uint16_t len = 2;
  struct {
    U1 cls_id;
    U1 msg_id;
  } payload;
};
struct UbxAckNak {
  static constexpr uint8_t cls = UBX_ACK_CLS_;
  static constexpr uint8_t id = UBX_ACK_NAK_ID_;
  static constexpr uint16_t len = 2;
  struct {
    U1 cls_id;
    U1 msg_id;
  } payload;
};

}  // namespace bfs

#endif  // SRC_UBX_ACK_H_
