#include "main.h"
#include "ublox.h"

#include <stdbool.h>

#define I2C_RECEIVE_BUFFER_SIZE 64
#define PAYLOAD_BUFFER_SIZE 256

static uint32_t readI2C(
  I2C_HandleTypeDef *hi2c, uint16_t device_address, uint32_t timeout,
  uint8_t *buf, uint16_t size, bool sendStop)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t delta = 0;
  uint32_t xferOptions =
    sendStop ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME;

  while (
    HAL_I2C_Master_Seq_Receive_IT(
      hi2c, device_address << 1, buf, size, xferOptions) != HAL_OK
    && delta < timeout
  ) {
    delta = HAL_GetTick() - tickstart;
  }
  if (delta >= timeout) {
    return HAL_I2C_ERROR_TIMEOUT;
  }
  while ((HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) && (delta < timeout)) {
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

static uint32_t writeI2C(
  I2C_HandleTypeDef *hi2c, uint16_t device_address, uint32_t timeout,
  uint8_t *buf, uint16_t size, bool sendStop)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t delta = 0;
  uint32_t xferOptions = sendStop ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME;

  while (
    HAL_I2C_Master_Seq_Transmit_IT(
      hi2c, device_address << 1, buf, size, xferOptions) != HAL_OK
    && delta < timeout
  ) {
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

void UBX_Set_Size_And_Checksum(uint8_t *packet, uint16_t length)
{
  uint8_t chkA = 0;
  uint8_t chkB = 0;
  uint16_t payload_length = length - 8;

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
                     UBX_Packet_Callback callback, uint32_t timeout)
{
  uint8_t receive_buffer[I2C_RECEIVE_BUFFER_SIZE];
  uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];

  uint16_t bytes_available = 0;
  uint8_t reg = 0xFD;

  uint16_t packets_accepted = 0;

  if (writeI2C(hi2c, device_address, timeout,
      &reg, 1, false) != HAL_I2C_ERROR_NONE
  ) {
    return packets_accepted;
  }
  if (readI2C(hi2c, device_address, timeout,
      receive_buffer, 2, true) == HAL_I2C_ERROR_NONE
  ) {
    bytes_available = receive_buffer[1] | ((uint16_t)receive_buffer[0] << 8);
  }

  UBX_Receive_State state = UBX_SYNC_1;
  uint16_t payload_pos = 0;
  uint8_t chkA = 0;
  uint8_t chkB = 0;

  uint8_t packet_class;
  uint8_t packet_id;
  uint8_t packet_chkA;
  uint8_t packet_chkB;
  uint16_t payload_length;

  while (bytes_available) {
    uint16_t bytes_read = bytes_available;
    if (bytes_read > I2C_RECEIVE_BUFFER_SIZE) {
      bytes_read = I2C_RECEIVE_BUFFER_SIZE;
    }

    if (readI2C(hi2c, device_address, timeout,
        receive_buffer, bytes_read, true) != HAL_I2C_ERROR_NONE
    ) {
      return packets_accepted;
    }

    for(uint8_t i = 0; i < bytes_read; ++i) {
      const uint8_t b = receive_buffer[i];
      switch (state) {
        case UBX_SYNC_1:
          if (b == UBLOX_SYNC_1) {
            state = UBX_SYNC_2;
          }
          break;
        case UBX_SYNC_2:
          if (b == UBLOX_SYNC_2) {
            state = UBX_CLASS;
          } else {
            state = UBX_SYNC_1;
          }
          break;
        case UBX_CLASS:
          packet_class = b;
          chkA += b;
          chkB += chkA;
          state = UBX_ID;
          break;
        case UBX_ID:
          packet_id = b;
          chkA += b;
          chkB += chkA;
          state = UBX_LENGTH_1;
          break;
        case UBX_LENGTH_1:
          payload_length = b;
          chkA += b;
          chkB += chkA;
          state = UBX_LENGTH_2;
          break;
        case UBX_LENGTH_2:
          payload_length |= (uint16_t)b << 8;
          chkA += b;
          chkB += chkA;
          if (payload_length != 0) {
            state = UBX_PAYLOAD;
          } else {
            state = UBX_CHECKSUM_A;
          }
          break;
        case UBX_PAYLOAD:
          payload_buffer[payload_pos++] = b;
          chkA += b;
          chkB += chkA;
          if (payload_pos == payload_length) {
            state = UBX_CHECKSUM_A;
          } else if (payload_pos == PAYLOAD_BUFFER_SIZE) {
            state = UBX_PAYLOAD_OVERFLOW;
          }
          break;
        case UBX_PAYLOAD_OVERFLOW:
          // Swallow all the bytes including the checksum
          if (++payload_pos == payload_length + 2) {
            state = UBX_PACKET_COMPLETE;
          }
          break;
        case UBX_CHECKSUM_A:
          packet_chkA = b;
          state = UBX_CHECKSUM_B;
          break;
        case UBX_CHECKSUM_B:
          packet_chkB = b;
          if (chkA == packet_chkA && chkB == packet_chkB) {
            if (callback(packet_class, packet_id, payload_buffer, payload_length)) {
              ++packets_accepted;
            }
          }
          state = UBX_PACKET_COMPLETE;
          break;
      }
      if (state == UBX_PACKET_COMPLETE) {
        // Reset frame
        state = UBX_SYNC_1;
        payload_pos = 0;
        chkA = 0;
        chkB = 0;
      }
    }
    bytes_available -= bytes_read;
  }
  return packets_accepted;
}
