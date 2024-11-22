/*
 * ZED_F9P.h
 *
 *  Created on: Aug 21, 2024
 *      Author: brian
 */

#ifndef INC_ZED_F9P_H_
#define INC_ZED_F9P_H_

/*
 * INCLUDES
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"

/*
 * DEFINES
 */

#define F9P_UBX_SYNC_CH_1         0xB5
#define F9P_UBX_SYNC_CH_2         0x62

// Sync (2) + body(4) + Checksum(2)
static const int UBX_FRAME_MINIMUM_SIZE = 8;
static const int UBX_FRAME_LENGTH_OFFSET = 4;
static const int UBX_FRAME_PAYLOAD_OFFSET = 6;

// UBX message classes
#define UBX_NAV_CLASS 0x01
#define UBX_CFG_CLASS 0x06

// UBX message IDs
#define UBX_NAV_PVT_ID 0x07
#define UBX_CFG_VALSET 0x8a

// UBX message lengths
#define UBX_NAV_PVT_LEN 92

#define UBX_CONFIG_LAYER 0x07

/*
 * Structs & Enums
 */

typedef struct {
    uint32_t iTOW;       // GPS time of week in milliseconds
    uint16_t year;       // Year (UTC)
    uint8_t month;       // Month (UTC)
    uint8_t day;         // Day (UTC)
    uint8_t hour;        // Hour (UTC)
    uint8_t min;         // Minute (UTC)
    uint8_t sec;         // Second (UTC)
    uint8_t valid;       // Validity flags
    uint32_t tAcc;       // Time accuracy estimate (ns)
    int32_t nano;        // Fraction of a second (ns)
    uint8_t fixType;     // GNSS fix type
    uint8_t flags;       // Fix status flags
    uint8_t flags2;      // Additional flags
    uint8_t numSV;       // Number of satellites used in the solution
    int32_t lon;         // Longitude (1e-7 degrees)
    int32_t lat;         // Latitude (1e-7 degrees)
    int32_t height;      // Height above ellipsoid (mm)
    int32_t hMSL;        // Height above mean sea level (mm)
    uint32_t hAcc;       // Horizontal accuracy estimate (mm)
    uint32_t vAcc;       // Vertical accuracy estimate (mm)
    int32_t velN;        // NED north velocity (mm/s)
    int32_t velE;        // NED east velocity (mm/s)
    int32_t velD;        // NED down velocity (mm/s)
    int32_t gSpeed;      // Ground speed (mm/s)
    int32_t headMot;     // Heading of motion (1e-5 degrees)
    uint32_t sAcc;       // Speed accuracy estimate (mm/s)
    uint32_t headAcc;    // Heading accuracy estimate (1e-5 degrees)
    uint16_t pDOP;       // Position DOP (0.01)
    int32_t headVeh;     // Heading of vehicle (1e-5 degrees)
    int16_t magDec;      // Magnetic declination (1e-2 degrees)
    uint16_t magAcc;     // Magnetic declination accuracy (1e-2 degrees)
} ZedF9P; // all fields from UBX_NAV_PVT

/*
 * Functions
 */

void F9P_apply_init_config(UART_HandleTypeDef *huart);
void F9P_parse_ubx_message(ZedF9P *gps, uint8_t *buf, int size);

#endif /* INC_ZED_F9P_H_ */
