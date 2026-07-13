#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* This file must stay identical on the STM32 master and ESP32 slave. */
#define TELEMETRY_FRAME_SIZE       32U
#define TELEMETRY_PAYLOAD_SIZE     22U
#define TELEMETRY_MAGIC_0          0xC3U
#define TELEMETRY_MAGIC_1          0x3CU
#define TELEMETRY_PROTOCOL_VERSION 1U
#define TELEMETRY_RESPONSE_BIT     0x80U

#define TELEMETRY_OFFSET_MAGIC_0   0U
#define TELEMETRY_OFFSET_MAGIC_1   1U
#define TELEMETRY_OFFSET_VERSION   2U
#define TELEMETRY_OFFSET_COMMAND   3U
#define TELEMETRY_OFFSET_SEQUENCE  4U
#define TELEMETRY_OFFSET_STATUS    5U
#define TELEMETRY_OFFSET_LENGTH    6U
#define TELEMETRY_OFFSET_RESERVED  7U
#define TELEMETRY_OFFSET_PAYLOAD   8U
#define TELEMETRY_OFFSET_CRC_LO    30U
#define TELEMETRY_OFFSET_CRC_HI    31U

enum telemetry_command {
  TELEMETRY_CMD_NOP       = 0x00,
  TELEMETRY_CMD_PING      = 0x01,
  TELEMETRY_CMD_GET_STATS = 0x02,
  TELEMETRY_CMD_SET_TIME  = 0x10,
  TELEMETRY_CMD_GET_TIME  = 0x11,
};

enum telemetry_status {
  TELEMETRY_STATUS_OK              = 0,
  TELEMETRY_STATUS_BAD_MAGIC       = 1,
  TELEMETRY_STATUS_BAD_VERSION     = 2,
  TELEMETRY_STATUS_BAD_LENGTH      = 3,
  TELEMETRY_STATUS_BAD_CRC         = 4,
  TELEMETRY_STATUS_UNKNOWN_COMMAND = 5,
  TELEMETRY_STATUS_BAD_ARGUMENT    = 6,
  TELEMETRY_STATUS_INTERNAL_ERROR  = 7,
};

/* CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF. */
static inline uint16_t telemetry_crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;

  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (unsigned int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                            : (uint16_t)(crc << 1);
    }
  }

  return crc;
}

static inline void telemetry_frame_build(uint8_t frame[TELEMETRY_FRAME_SIZE],
                                         uint8_t command, uint8_t sequence,
                                         uint8_t status, const uint8_t *payload,
                                         uint8_t payload_length)
{
  if (payload_length > TELEMETRY_PAYLOAD_SIZE) {
    payload_length = TELEMETRY_PAYLOAD_SIZE;
  }

  memset(frame, 0, TELEMETRY_FRAME_SIZE);
  frame[TELEMETRY_OFFSET_MAGIC_0] = TELEMETRY_MAGIC_0;
  frame[TELEMETRY_OFFSET_MAGIC_1] = TELEMETRY_MAGIC_1;
  frame[TELEMETRY_OFFSET_VERSION] = TELEMETRY_PROTOCOL_VERSION;
  frame[TELEMETRY_OFFSET_COMMAND] = command;
  frame[TELEMETRY_OFFSET_SEQUENCE] = sequence;
  frame[TELEMETRY_OFFSET_STATUS] = status;
  frame[TELEMETRY_OFFSET_LENGTH] = payload_length;
  if ((payload != NULL) && (payload_length != 0U)) {
    memcpy(&frame[TELEMETRY_OFFSET_PAYLOAD], payload, payload_length);
  }

  const uint16_t crc = telemetry_crc16(frame, TELEMETRY_OFFSET_CRC_LO);
  frame[TELEMETRY_OFFSET_CRC_LO] = (uint8_t)crc;
  frame[TELEMETRY_OFFSET_CRC_HI] = (uint8_t)(crc >> 8);
}

static inline uint8_t telemetry_frame_validate(
    const uint8_t frame[TELEMETRY_FRAME_SIZE])
{
  if ((frame[TELEMETRY_OFFSET_MAGIC_0] != TELEMETRY_MAGIC_0) ||
      (frame[TELEMETRY_OFFSET_MAGIC_1] != TELEMETRY_MAGIC_1)) {
    return TELEMETRY_STATUS_BAD_MAGIC;
  }
  if (frame[TELEMETRY_OFFSET_VERSION] != TELEMETRY_PROTOCOL_VERSION) {
    return TELEMETRY_STATUS_BAD_VERSION;
  }
  if (frame[TELEMETRY_OFFSET_LENGTH] > TELEMETRY_PAYLOAD_SIZE) {
    return TELEMETRY_STATUS_BAD_LENGTH;
  }

  const uint16_t received_crc =
      (uint16_t)frame[TELEMETRY_OFFSET_CRC_LO] |
      ((uint16_t)frame[TELEMETRY_OFFSET_CRC_HI] << 8);
  return (received_crc == telemetry_crc16(frame, TELEMETRY_OFFSET_CRC_LO))
             ? TELEMETRY_STATUS_OK
             : TELEMETRY_STATUS_BAD_CRC;
}

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_PROTOCOL_H */
