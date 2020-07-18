/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_UBLOX_UBLOX_H_
#define INCLUDE_UBLOX_UBLOX_H_

#include "types/types.h"
#include "core/core.h"

namespace sensors {

class Ublox {
 public:
  explicit Ublox(HardwareSerial* bus);
  bool Begin(uint32_t baud);
  bool Read();
  types::Gnss<types::NedVel<float>, types::LlaPos<double>> gnss();
 private:
  /* Communication */
  HardwareSerial* bus_;
  /* Configuration */
  static constexpr unsigned int TIMEOUT_MS_ = 5000;
  bool use_high_precision_ = false;
  /* Parsing */
  const uint8_t UBX_HEADER_[2] = {0xB5, 0x62};
  static constexpr uint8_t UBX_HEADER_LEN_ = 6;
  static constexpr uint8_t UBX_PAYLOAD_OFFSET_ = 4;
  static constexpr uint8_t UBX_NAV_CLASS_ = 0x01;
  static constexpr uint8_t UBX_PVT_LEN_ = 92;
  static constexpr uint8_t UBX_HPPOSLLH_LEN_ = 36;
  static constexpr uint8_t UBX_EOE_LEN_ = 4;
  unsigned int parser_pos_ = 0;
  uint8_t msg_len_buffer_[2];
  uint16_t msg_len_;
  uint8_t checksum_buffer_[2];
  uint8_t rx_buffer_[96];
  bool ubx_nav_pvt_parsed_ = false;
  bool ubx_nav_hpposllh_parsed_ = false;
  bool read_status_ = false;
  /* Data */
  enum Msg : uint8_t {
    UBX_NAV_PVT = 0x07,
    UBX_NAV_HPPOSLLH = 0x14,
    UBX_NAV_EOE = 0x61
  } msg_;
  struct {
    uint32_t itow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tacc;
    int32_t nano;
    uint8_t fix;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numsv;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hmsl;
    uint32_t hacc;
    uint32_t vacc;
    int32_t veln;
    int32_t vele;
    int32_t veld;
    int32_t gspeed;
    int32_t headmot;
    uint32_t sacc;
    uint32_t headacc;
    uint16_t pdop;
    uint8_t flags3;
    uint8_t reserved1[5];
    int32_t headveh;
    int16_t magdec;
    uint16_t magacc;
  } ubx_nav_pvt_;
  struct {
    uint8_t version;
    uint8_t reserved1[2];
    uint8_t flags;
    uint32_t itow;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hmsl;
    int8_t lonhp;
    int8_t lathp;
    int8_t heighthp;
    int8_t hmslhp;
    uint32_t hacc;
    uint32_t vacc;
  } ubx_nav_hpposllh_;
  struct {
    uint32_t itow;
  } ubx_nav_eoe_;
  types::Gnss<types::NedVel<float>, types::LlaPos<double>> gnss_;
  bool Epoch();
  bool Parse();
  uint16_t Checksum(uint8_t *data, uint16_t len);
};

}  // namespace sensors


#endif  // INCLUDE_UBLOX_UBLOX_H_
