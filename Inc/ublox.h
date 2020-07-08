#ifndef __UBLOX_H
#define __UBLOX_H

#include <stdint.h>
#include "main.h"

#define UBLOX_SYNC_1 0xb5
#define UBLOX_SYNC_2 0x62

#define UBX_NAV 0x01
#define UBX_ACK 0x05
#define UBX_CFG 0x06
#define UBX_MON 0x0a

#define UBX_ACK_ACK 0x01
#define UBX_ACK_NACK 0x00

#define UBX_CFG_PRT 0x00
#define UBX_CFG_MSG 0x01
#define UBX_CFG_RATE 0x08
#define UBX_CFG_CFG 0x09

#define UBX_NAV_PVT 0x07

#define UBX_MON_VER 0x04

#define UBX_PORT_I2C 0
#define UBX_PORT_UART1 1

#define UBLOX_I2C_ADDRESS 0x42

#define UBX_INT16(x) (x) & 0xff, ((x) >> 8) & 0xff
#define UBX_INT32(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, ((x) >> 24) & 0xff

typedef enum {
    UBX_SYNC_1,
    UBX_SYNC_2,
    UBX_CLASS,
    UBX_ID,
    UBX_LENGTH_1,
    UBX_LENGTH_2,
    UBX_PAYLOAD,
    UBX_PAYLOAD_OVERFLOW,
    UBX_CHECKSUM_A,
    UBX_CHECKSUM_B,
    UBX_PACKET_COMPLETE,
} UBX_Receive_State;

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t readU4(uint8_t *buffer, uint16_t offset)
{
  uint32_t result = buffer[offset];
  result |= (uint32_t)buffer[offset + 1] << 8;
  result |= (uint32_t)buffer[offset + 2] << 16;
  result |= (uint32_t)buffer[offset + 3] << 24;
  return result;
}

static inline uint16_t readU2(uint8_t *buffer, uint16_t offset)
{
  uint16_t result = buffer[offset];
  result |= (uint16_t)buffer[offset + 1] << 8;
  return result;
}

typedef void (*UBX_Packet_Callback)(uint8_t cls, uint8_t id, uint8_t *payload, uint16_t length);
void UBX_Receive(I2C_HandleTypeDef *hi2c, uint16_t device_address, UBX_Packet_Callback callback, uint32_t timeout);
void UBX_Set_Size_And_Checksum(uint8_t *packet, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
