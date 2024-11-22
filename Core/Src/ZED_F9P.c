/*
 * ZED_F9P.cpp
 *
 *  Created on: Aug 21, 2024
 *      Author: brian
 *
 *  All references to documentation refers to:
 *  u-blox F9 high precision GNSS receiver - Interface Description
 */

#include "ZED_F9P.h"
#include "debug.h"

// Function prototypes
void F9P_parse_ubx_message(ZedF9P *gps, uint8_t *buf, int size);
int verify_ubx_buffer_checksum(uint8_t *buffer, uint32_t size);
uint16_t generate_ubx_buffer_checksum(uint8_t *buffer, int start, int end);
void parse_ubx_buffer(ZedF9P *gps, uint8_t *buf, int size);
void parse_ubx_buffer(ZedF9P *gps, uint8_t *buf, int size);
void parse_nav_msg(ZedF9P *gps, uint8_t *buf, int size);
void parse_nav_pvt_msg(ZedF9P *gps, uint8_t *buf, int size);
uint32_t extract_uint32(uint8_t *buf, int index);
uint16_t extract_uint16(uint8_t *buf, int index);
uint8_t extract_uint8(uint8_t *buf, int index);
void disable_nmea_uart1(UART_HandleTypeDef *huart);
void set_custom_polling_rate(UART_HandleTypeDef *huart, uint8_t rate);
void send_uart_ubx_message(UART_HandleTypeDef *huart, uint8_t *buf, int buffer_size);

void F9P_parse_ubx_message(ZedF9P *gps, uint8_t *buf, int size) {
  if (size < UBX_FRAME_MINIMUM_SIZE) {
    return;
  }

  // verify sync bytes [0:1]
  if (buf[0] != F9P_UBX_SYNC_CH_1 || buf[1] != F9P_UBX_SYNC_CH_2) {
    return;
  }

  // verify checksum [size - 1:size]
  if (!verify_ubx_buffer_checksum(buf, size)) {
    return;
  }

  parse_ubx_buffer(gps, buf, size);
}

void parse_ubx_buffer(ZedF9P *gps, uint8_t *buf, int size) {
  uint8_t message_class = buf[2];
  switch (message_class) {
    case UBX_NAV_CLASS:
      parse_nav_msg(gps, buf, size);
      break;
    default:
      // we don't care about other types of messages.
      break;
  }
}

// UBX-NAV (0x01 XX)
void parse_nav_msg(ZedF9P *gps, uint8_t *buf, int size) {
  uint8_t message_id = buf[3];
  switch (message_id) {
    case UBX_NAV_PVT_ID:
      parse_nav_pvt_msg(gps, buf, size);
      break;
    default:
      // we don't care about other types of messages.
      break;
  }
}

// UBX-NAV-PVT (0x01 0x07)
void parse_nav_pvt_msg(ZedF9P *gps, uint8_t *buf, int size) {
  // TODO: verify message length
  uint16_t payload_size = extract_uint16(buf, UBX_FRAME_LENGTH_OFFSET);
  const uint16_t UBX_NAV_PVT_PAYLOAD_SIZE = 92;
  if (payload_size != UBX_NAV_PVT_PAYLOAD_SIZE) {
    return;
  }

  // Offsets taken from 3.15.12 UBX-NAV-PVT (0x01 0x07) of Interface Description
  gps->iTOW = extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 0);
  gps->year = extract_uint16(buf, UBX_FRAME_PAYLOAD_OFFSET + 4);
  gps->month = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 6);
  gps->day = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 7);
  gps->hour = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 8);
  gps->min = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 9);
  gps->sec = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 10);
  gps->valid = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 11);
  gps->tAcc = extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 12);
  gps->nano = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 16);
  gps->fixType = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 20);
  gps->flags = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 21);
  gps->flags2 = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 22);
  gps->numSV = extract_uint8(buf, UBX_FRAME_PAYLOAD_OFFSET + 23);
  gps->lon = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 24);
  gps->lat = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 28);
  gps->height = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 32);
  gps->hMSL = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 36);
  gps->hAcc = extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 40);
  gps->vAcc = extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 44);
  gps->velN = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 48);
  gps->velE = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 52);
  gps->velD = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 56);
  gps->gSpeed = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 60);
  gps->headMot = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 64);
  gps->sAcc = extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 68);
  gps->headAcc = extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 72);
  gps->pDOP = extract_uint16(buf, UBX_FRAME_PAYLOAD_OFFSET + 76);
  gps->headVeh = (int32_t)extract_uint32(buf, UBX_FRAME_PAYLOAD_OFFSET + 84);
  gps->magDec = (int16_t)extract_uint16(buf, UBX_FRAME_PAYLOAD_OFFSET + 88);
  gps->magAcc = extract_uint16(buf, UBX_FRAME_PAYLOAD_OFFSET + 90);
}

int verify_ubx_buffer_checksum(uint8_t *buffer, uint32_t size) {
  return generate_ubx_buffer_checksum(buffer, 2, size - 2) == extract_uint16(buffer, size - 2);
}

uint16_t generate_ubx_buffer_checksum(uint8_t *buffer, int start, int end) {
  // Taken from pseudo code found in 3.4 UBX checksum
  // 8-bit Fletcher algorithm
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (int i = start; i < end; i++) {
    ck_a = (ck_a + buffer[i]);
    ck_b = (ck_b + ck_a);
  }

  return (uint16_t)ck_a | (uint16_t)ck_b << 8;
}

/**
 * Extracts unsigned little endian int32_t from buffer
 */
uint32_t extract_uint32(uint8_t *buf, int index) {
  return (
    (uint32_t)buf[index] |
    (uint32_t)buf[index + 1] << 8 |
    (uint32_t)buf[index + 2] << 16 |
    (uint32_t)buf[index + 3] << 24
  );
}

/**
 * Extracts little endian int32_t from buffer
 */
uint16_t extract_uint16(uint8_t *buf, int index) {
  return (uint16_t)buf[index] | (uint16_t)buf[index + 1] << 8;
}

/**
 * Extracts little endian int8_t from buffer
 */
uint8_t extract_uint8(uint8_t *buf, int index) {
  return buf[index];
}

void F9P_apply_init_config(UART_HandleTypeDef *huart) {
  disable_nmea_uart1(huart);
  set_custom_polling_rate(huart, 3);
}

void disable_nmea_uart1(UART_HandleTypeDef *huart) {
  // Refer to 3.10.26 UBX-CFG-VALSET (0x06 0x8a) and CFG-UART1OUTPROT-NMEA, 0x10740002
  uint8_t data[] = {
    F9P_UBX_SYNC_CH_1, F9P_UBX_SYNC_CH_2, // sync chars
    UBX_CFG_CLASS, // class
    UBX_CFG_VALSET, // id
    0x09, 0x00, // length (9 bytes)
    0x00, UBX_CONFIG_LAYER, 0x00, 0x00, // CFG-VALSET header
    // configuration data: 
    0x02, 0x00, 0x74, 0x10, // key id (little endian)
    0, // (L: single-bit boolean (true = 1, false = 0), stored as U1)
    0x0, 0x0 // checksum, not set
  };

  send_uart_ubx_message(huart, data, sizeof(data));
}

void set_custom_polling_rate(UART_HandleTypeDef *huart, uint8_t rate) {
  // Refer to 3.10.26 UBX-CFG-VALSET (0x06 0x8a) and CFG-MSGOUT-UBX_NAV_PVT_UART1, 0x20910007
  uint8_t data[] = {
    F9P_UBX_SYNC_CH_1, F9P_UBX_SYNC_CH_2, // sync chars
    UBX_CFG_CLASS, // class
    UBX_CFG_VALSET, // id
    0x09, 0x00, // length (9 bytes)
    0x00, UBX_CONFIG_LAYER, 0x00, 0x00, // CFG-VALSET header
    // configuration data: 
    0x07, 0x00, 0x91, 0x20, // key id (little endian)
    rate, // value (U1)
    0x0, 0x0 // checksum, not set
  };

  send_uart_ubx_message(huart, data, sizeof(data));
}

void send_uart_ubx_message(UART_HandleTypeDef *huart, uint8_t *buf, int buffer_size) {
  // fill checksum
  uint16_t checksum = generate_ubx_buffer_checksum(buf, 2, buffer_size - 2);
  buf[buffer_size - 2] = (uint8_t)(checksum);
  buf[buffer_size - 1] = (uint8_t)(checksum >> 8);

  #ifdef DEBUG
    DEBUG_LOG("Buffer as string: ");

    for (int i = 0; i < buffer_size; i++) {
      DEBUG_LOG("0x%02X ", buf[i]);  // Print each byte in hexadecimal
    }

    DEBUG_LOG("\r\n");
  #endif

  HAL_UART_Transmit(huart, buf, buffer_size, HAL_MAX_DELAY);
}
