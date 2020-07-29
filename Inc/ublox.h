/**
 * Copyright 2020 James Wilson <jmw@jmw.name>. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __UBLOX_H
#define __UBLOX_H

#include <cstdint>
#include "main.h"

constexpr uint8_t UBLOX_SYNC_1 = 0xb5;
constexpr uint8_t UBLOX_SYNC_2 = 0x62;

namespace UBX_CLS {
  constexpr uint8_t UBX_NAV = 0x01;
  constexpr uint8_t UBX_ACK = 0x05;
  constexpr uint8_t UBX_CFG = 0x06;
  constexpr uint8_t UBX_MON = 0x0a;
}

namespace UBX_MSG {
  constexpr uint8_t UBX_ACK_ACK = 0x01;
  constexpr uint8_t UBX_ACK_NACK = 0x00;

  constexpr uint8_t UBX_CFG_PRT = 0x00;
  constexpr uint8_t UBX_CFG_MSG = 0x01;
  constexpr uint8_t UBX_CFG_RATE = 0x08;
  constexpr uint8_t UBX_CFG_CFG = 0x09;

  constexpr uint8_t UBX_NAV_PVT = 0x07;

  constexpr uint8_t UBX_MON_VER = 0x04;
}

constexpr uint8_t UBX_PORT_I2C = 0;
constexpr uint8_t UBX_PORT_UART = 1;

constexpr uint8_t UBLOX_I2C_ADDRESS = 0x42;

#define UBX_INT16(x) (x) & 0xff, ((x) >> 8) & 0xff
#define UBX_INT32(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, ((x) >> 24) & 0xff

enum class UBXReceiveState {
  SYNC_1,
  SYNC_2,
  CLASS,
  ID,
  LENGTH_1,
  LENGTH_2,
  PAYLOAD,
  PAYLOAD_OVERFLOW,
  CHECKSUM_A,
  CHECKSUM_B,
  PACKET_COMPLETE,
};

typedef bool (*UBXPacketCallback)(const uint8_t cls,
                                    const uint8_t id,
                                    const uint8_t *payload,
                                    const uint16_t length);

constexpr uint32_t readU4(const uint8_t *buffer, const int offset)
{
  return buffer[offset]
    | buffer[offset + 1] << 8
    | buffer[offset + 2] << 16
    | buffer[offset + 3] << 24;
}

constexpr uint16_t readU2(const uint8_t *buffer, const int offset)
{
  return buffer[offset] | buffer[offset + 1] << 8;
}

uint16_t UBX_Receive(I2C_HandleTypeDef *hi2c, uint16_t device_address,
                     UBXPacketCallback callback, uint32_t timeout);
void UBX_SetSizeAndChecksum(uint8_t *packet, uint16_t length);

#endif  // __UBLOX_H
