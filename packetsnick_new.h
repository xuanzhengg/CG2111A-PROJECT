/*
 * packets.h
 * Studio 13: Sensor Mini-Project
 *
 * TPacket protocol: enums, struct, and framing constants.
 * This file must be kept in sync with the constants in pi_sensor.py.
 *
 * Current sync state (pi_sensor.py <-> packets.h):
 *   PACKET_TYPE_COMMAND  = 0  ✓
 *   PACKET_TYPE_RESPONSE = 1  ✓
 *   PACKET_TYPE_MESSAGE  = 2  ✓
 *   COMMAND_ESTOP        = 0  ✓
 *   COMMAND_COLOR        = 1  ✓
 *   RESP_OK              = 0  ✓
 *   RESP_STATUS          = 1  ✓
 *   RESP_COLOR           = 2  ✓
 *   STATE_RUNNING        = 0  ✓
 *   STATE_STOPPED        = 1  ✓
 */

#pragma once

#include <stdint.h>

// =============================================================
// TPacket protocol
// =============================================================

typedef enum {
    PACKET_TYPE_COMMAND  = 0,
    PACKET_TYPE_RESPONSE = 1,
    PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
    COMMAND_ESTOP  = 0,
    COMMAND_COLOR  = 1,   // Activity 2: request R/G/B channel frequencies from TCS3200
} TCommandType;

typedef enum {
    RESP_OK     = 0,
    RESP_STATUS = 1,
    RESP_COLOR  = 2,   // Activity 2: params[0]=R Hz, params[1]=G Hz, params[2]=B Hz
} TResponseType;

typedef enum {
    STATE_RUNNING = 0,
    STATE_STOPPED = 1,
} TState;

/*
 * TPacket — fixed 100-byte layout, identical on both ATMega2560 and Raspberry Pi.
 *
 * Layout (offsets):
 *   0        packetType  (1 B)
 *   1        command     (1 B)
 *   2–3      dummy       (2 B explicit padding — keeps params[] 4-byte aligned
 *                         on both AVR and ARM without compiler-inserted gaps)
 *   4–35     data        (32 B null-terminated debug/payload string)
 *   36–99    params      (16 × uint32_t = 64 B numeric parameters)
 *
 * Color response usage:
 *   params[0] = red   channel frequency (Hz)
 *   params[1] = green channel frequency (Hz)
 *   params[2] = blue  channel frequency (Hz)
 *
 * Status response usage:
 *   params[0] = TState value (STATE_RUNNING or STATE_STOPPED)
 */
typedef struct {
    uint8_t  packetType;
    uint8_t  command;
    uint8_t  dummy[2];
    char     data[32];
    uint32_t params[16];
} TPacket;

// =============================================================
// Framing constants
// =============================================================

#define MAGIC_HI        0xDE
#define MAGIC_LO        0xAD
#define TPACKET_SIZE    ((uint8_t)sizeof(TPacket))   // 100 bytes
#define FRAME_SIZE      (2 + TPACKET_SIZE + 1)       // 103 bytes
