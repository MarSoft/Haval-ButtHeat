/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* ButtHeat shared application types + per-control handler registry.
 * The firmware split mirrors the KiCad schematic sheets: buttheat.c (app
 * core/wiring), can.c, controls.c, display.c. */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef int32_t rotary_value_t;

typedef uint8_t ac_temp_t;  // 0 = LO; 1 = 16.5; 2 = 17; ...; 30=31; 31=31.5; 32=HI
#define AC_TEMP_HI 32
#define AC_TEMP_MAX 31
#define AC_TEMP_MIN 1
#define AC_TEMP_LO 0
// ac to degrees: (val / 2) + 16

typedef uint8_t fan_speed_t;
#define FAN_SPEED_UNKNOWN 0
// fan-off requires special handling
#define FAN_SPEED_OFF 0
#define FAN_SPEED_MIN 1
#define FAN_SPEED_MAX 7

// Airflow direction from 0x29D byte[2]: (upper_nibble >> 1)
// Raw upper nibble: 0=face, 2=face+feet, 4=feet, 6=feet+windshield, 8=windshield, E=off
typedef enum {
    AIRFLOW_FACE = 0,            // head/face vents
    AIRFLOW_FACE_FEET = 1,      // face + feet
    AIRFLOW_FEET = 2,            // feet only
    AIRFLOW_FEET_WINDSHIELD = 3, // feet + windshield defrost
    AIRFLOW_WINDSHIELD = 4,      // windshield only (defrost)
    AIRFLOW_OFF = 7,             // AC off (raw 0xE >> 1)
} airflow_dir_t;
#define AIRFLOW_FROM_CAN(b)  ((airflow_dir_t)((b) >> 5))  // extract from 0x29D byte[2]

// positive 1..3: heater, negative -1..-3: fan
typedef int8_t butt_temp_t;

typedef enum {
    MASSAGE_MODE_WAVES = 0,
    MASSAGE_MODE_DOTS = 1,
    MASSAGE_MODE_GENERAL = 2,
    MASSAGE_MODE_SHOULDER = 3,
    MASSAGE_MODE_WAIST = 4,
} massage_mode_t;

// Massage state. Layout is provisional and will change once the real CAN
// payload is decoded; do not rely on bit positions for wire serialization.
typedef struct {
    uint8_t intensity : 2;  // 0=off, 1..3
    uint8_t mode      : 3;  // massage_mode_t
    uint8_t _pad      : 3;
} __attribute__((packed)) massage_t;
#define MASSAGE_IS_OFF(m) ((m).intensity == 0)

typedef union {
    ac_temp_t ac;
    fan_speed_t fan;
    airflow_dir_t airflow;
    butt_temp_t butt;
    massage_t massage;
    uint8_t _max; // for simpler equality check
} union_temp_t;

typedef enum {
    HANDLER_TYPE_AC,
    HANDLER_TYPE_FAN,
    HANDLER_TYPE_AIRFLOW,
    HANDLER_TYPE_BUTTHEAT,
    HANDLER_TYPE_MASSAGE,
} handler_type_t;

typedef struct {
    handler_type_t type;
    QueueHandle_t control_queue; // capacity: 8, since we don't want to lost those msgs!
    QueueHandle_t can_queue; // capacity: 1, and new message overwrites existing one.
    bool leftside;
} handler_config_t;

typedef enum {
    DU_BUTTHEAT,
    DU_AC,
    DU_DISABLE_TWOZONE, // toggle two-zone AC off (only if currently on)
    DU_FANSPEED,
    DU_AIRFLOW,
    DU_MASSAGE,
    DU_SEAT_MEMORY,
    DU_CAN_ERROR, // put display to "can error" state
    DU_LONGPRESS_STAGE,    // overlay while holding: seat_mem_slot 0=clear, 1-3=slot
    DU_LONGPRESS_CONFIRM,  // confirmed on release: seat_mem_slot 1-3, triggers blink
    DU_AC_ACK,             // AC controller ack bit changed (leftside = new value)
} data_update_kind_t;
typedef struct {
    data_update_kind_t kind;
    bool leftside;
    bool fanside; // FIXME: change leftside bool into tristate (enc_number)?
    union {
        ac_temp_t ac_temp;
        fan_speed_t fan_speed;
        airflow_dir_t airflow_dir;
        butt_temp_t butt_temp;
        massage_t massage;
        uint8_t seat_mem_slot;
        bool ac_ack;
    };
} data_update_t;

// Handler registry, defined in buttheat.c. can.c overwrites can_queues with
// decoded status values; controls.c routes encoder input into control_queues.
extern handler_config_t left_ac_handler;
extern handler_config_t right_ac_handler;
extern handler_config_t fan_speed_handler;
extern handler_config_t airflow_handler;
extern handler_config_t left_butt_handler;
extern handler_config_t right_butt_handler;
extern handler_config_t massage_handler;
