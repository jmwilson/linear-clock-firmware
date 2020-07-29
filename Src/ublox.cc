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

#include "main.h"
#include "ublox.h"

constexpr size_t I2C_RECEIVE_BUFFER_SIZE = 64;
constexpr size_t PAYLOAD_BUFFER_SIZE = 256;

static uint32_t ReadI2C(I2C_HandleTypeDef *hi2c,
                        const uint16_t device_address,
                        const uint32_t timeout, uint8_t *buf,
                        const uint16_t size, const bool sendStop)
{
  const uint32_t tickstart = HAL_GetTick();
  uint32_t delta = 0;
  const uint32_t xferOptions =
    sendStop ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME;

  while (HAL_I2C_Master_Seq_Receive_IT(hi2c, device_address << 1,
                                       buf, size, xferOptions) != HAL_OK
         && delta < timeout) {
    delta = HAL_GetTick() - tickstart;
  }
  if (delta >= timeout) {
    return HAL_I2C_ERROR_TIMEOUT;
  }
  while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY && delta < timeout) {
    delta = HAL_GetTick() - tickstart;
    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
      break;
    }
  }
  if (delta >= timeout) {
    return HAL_I2C_ERROR_TIMEOUT;
  }
  return HAL_I2C_GetError(hi2c);
}

static uint32_t WriteI2C(I2C_HandleTypeDef *hi2c,
                         const uint16_t device_address,
                         const uint32_t timeout, const uint8_t *buf,
                         const uint16_t size, const bool sendStop)
{
  const uint32_t tickstart = HAL_GetTick();
  uint32_t delta = 0;
  const uint32_t xferOptions =
    sendStop ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME;

  while (HAL_I2C_Master_Seq_Transmit_IT(hi2c, device_address << 1,
                                        const_cast<uint8_t *>(buf), size,
                                        xferOptions) != HAL_OK
         && delta < timeout) {
    delta = HAL_GetTick() - tickstart;
  }
  if (delta >= timeout) {
    return HAL_I2C_ERROR_TIMEOUT;
  }
  while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY && delta < timeout) {
    delta = HAL_GetTick() - tickstart;
    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
      break;
    }
  }
  if (delta >= timeout) {
    return HAL_I2C_ERROR_TIMEOUT;
  }
  return HAL_I2C_GetError(hi2c);
}

void UBX_SetSizeAndChecksum(uint8_t *packet, const uint16_t length)
{
  uint8_t chkA = 0;
  uint8_t chkB = 0;
  const uint16_t payload_length = length - 8;

  packet[4] = payload_length & 0xff;
  packet[5] = payload_length >> 8;

  for(uint16_t i = 2; i < length - 2; ++i) {
    chkA += packet[i];
    chkB += chkA;
  }
  packet[length - 2] = chkA;
  packet[length - 1] = chkB;
}

uint16_t UBX_Receive(I2C_HandleTypeDef *hi2c, uint16_t device_address,
                     UBXPacketCallback callback, uint32_t timeout)
{
  uint8_t receive_buffer[I2C_RECEIVE_BUFFER_SIZE];
  uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];

  uint16_t bytes_available = 0;
  const uint8_t reg = 0xFD;

  uint16_t packets_accepted = 0;

  if (WriteI2C(hi2c, device_address, timeout, &reg, 1, false)
      != HAL_I2C_ERROR_NONE) {
    return packets_accepted;
  }
  if (ReadI2C(hi2c, device_address, timeout, receive_buffer, 2, true)
      == HAL_I2C_ERROR_NONE) {
    bytes_available = receive_buffer[1] | receive_buffer[0] << 8;
  }

  auto state = UBXReceiveState::SYNC_1;
  uint16_t payload_pos = 0;
  uint8_t chkA = 0;
  uint8_t chkB = 0;

  uint8_t packet_class;
  uint8_t packet_id;
  uint8_t packet_chkA;
  uint8_t packet_chkB;
  uint16_t payload_length;

  while (bytes_available) {
    uint16_t receive_bytes = bytes_available;
    if (receive_bytes > I2C_RECEIVE_BUFFER_SIZE) {
      receive_bytes = I2C_RECEIVE_BUFFER_SIZE;
    }

    if (ReadI2C(hi2c, device_address, timeout, receive_buffer, receive_bytes,
                true) != HAL_I2C_ERROR_NONE
    ) {
      return packets_accepted;
    }

    for(uint8_t i = 0; i < receive_bytes; ++i) {
      const uint8_t b = receive_buffer[i];
      switch (state) {
        case UBXReceiveState::SYNC_1:
          if (b == UBLOX_SYNC_1) {
            state = UBXReceiveState::SYNC_2;
          }
          break;
        case UBXReceiveState::SYNC_2:
          if (b == UBLOX_SYNC_2) {
            state = UBXReceiveState::CLASS;
          } else {
            state = UBXReceiveState::SYNC_1;
          }
          break;
        case UBXReceiveState::CLASS:
          packet_class = b;
          chkA += b;
          chkB += chkA;
          state = UBXReceiveState::ID;
          break;
        case UBXReceiveState::ID:
          packet_id = b;
          chkA += b;
          chkB += chkA;
          state = UBXReceiveState::LENGTH_1;
          break;
        case UBXReceiveState::LENGTH_1:
          payload_length = b;
          chkA += b;
          chkB += chkA;
          state = UBXReceiveState::LENGTH_2;
          break;
        case UBXReceiveState::LENGTH_2:
          payload_length |= b << 8;
          chkA += b;
          chkB += chkA;
          if (payload_length != 0) {
            state = UBXReceiveState::PAYLOAD;
          } else {
            state = UBXReceiveState::CHECKSUM_A;
          }
          break;
        case UBXReceiveState::PAYLOAD:
          payload_buffer[payload_pos++] = b;
          chkA += b;
          chkB += chkA;
          if (payload_pos == payload_length) {
            state = UBXReceiveState::CHECKSUM_A;
          } else if (payload_pos == PAYLOAD_BUFFER_SIZE) {
            state = UBXReceiveState::PAYLOAD_OVERFLOW;
          }
          break;
        case UBXReceiveState::PAYLOAD_OVERFLOW:
          // Swallow all the bytes including the checksum
          if (++payload_pos == payload_length + 2) {
            state = UBXReceiveState::PACKET_COMPLETE;
          }
          break;
        case UBXReceiveState::CHECKSUM_A:
          packet_chkA = b;
          state = UBXReceiveState::CHECKSUM_B;
          break;
        case UBXReceiveState::CHECKSUM_B:
          packet_chkB = b;
          if (chkA == packet_chkA && chkB == packet_chkB) {
            if (callback(packet_class, packet_id, payload_buffer,
                         payload_length)) {
              ++packets_accepted;
            }
          }
          state = UBXReceiveState::PACKET_COMPLETE;
          break;
      }
      if (state == UBXReceiveState::PACKET_COMPLETE) {
        // Reset frame
        state = UBXReceiveState::SYNC_1;
        payload_pos = 0;
        chkA = 0;
        chkB = 0;
      }
    }
    bytes_available -= receive_bytes;
  }
  return packets_accepted;
}
