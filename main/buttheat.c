/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a slave node in a TWAI network. The slave
 * node is responsible for sending data messages to the master. The example will
 * execute multiple iterations, with each iteration the slave node will do the
 * following:
 * 1) Start the TWAI driver
 * 2) Listen for ping messages from master, and send ping response
 * 3) Listen for start command from master
 * 4) Send data messages to master and listen for stop command
 * 5) Send stop response to master
 * 6) Stop the TWAI driver
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "hal/i2c_types.h"
#include "portmacro.h"

#include "esp_timer.h"
#include "sdkconfig.h"
#include "ssd1306.h"
#include "font16x28.h"  // generated with tools/gen_font16x28.py
#include "encoder.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define TAG "ButtHeat"

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

typedef uint8_t butt_temp_t;

typedef union {
    ac_temp_t ac;
    fan_speed_t fan;
    butt_temp_t butt;
    uint8_t _max; // for simpler equality check
} union_temp_t;

typedef enum {
    HANDLER_TYPE_AC,
    HANDLER_TYPE_FAN,
    HANDLER_TYPE_BUTTHEAT,
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
    DU_SEAT_MEMORY,
    DU_CAN_ERROR, // put display to "can error" state
    DU_LONGPRESS_STAGE,    // overlay while holding: seat_mem_slot 0=clear, 1-3=slot
    DU_LONGPRESS_CONFIRM,  // confirmed on release: seat_mem_slot 1-3, triggers blink
    DU_AC_ACK,             // AC controller ack bit changed (leftside = new value)
} data_update_kind_t;
typedef struct {
    data_update_kind_t kind;
    bool leftside;
    union {
        ac_temp_t ac_temp;
        fan_speed_t fan_speed;
        butt_temp_t butt_temp;
        uint8_t seat_mem_slot;
        bool ac_ack;
    };
} data_update_t;

#define CAN_ID_HEATER_STATUS 0x2D1
#define CAN_ID_HEATER_CONTROL 0x36D
#define CAN_ID_FAN_CONTROL 0x367
#define CAN_ID_AC_STATUS 0x385
#define CAN_ID_AC_CONTROL 0x1EE
#define CAN_ID_SEAT_MEMORY 0x1ED

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_TX_GPIO_NUM, CONFIG_RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
// Single filter accepting both 0x2D1 and 0x385 (and 14 other IDs, of which
// only 0x295 and 0x2C1 exist on the bus — harmlessly ignored in the switch).
// Dual filter mode is avoided due to intermittent failures on ESP32-C3
// (espressif/esp-idf#17504).
//
// Bitwise breakdown (11-bit standard IDs):
//   0x2D1 = 010 1101 0001
//   0x385 = 011 1000 0101
//   XOR   = 001 0101 0100  (= 0x154, bits that differ → must be don't-care in mask)
static const twai_filter_config_t f_config = {
    .acceptance_code = (CAN_ID_HEATER_STATUS << 21),  // 0x2D1 as base
    .acceptance_mask = ((CAN_ID_HEATER_STATUS ^ CAN_ID_AC_STATUS) << 21) | 0x1FFFFF,  // don't-care for differing bits + all non-ID bits
    .single_filter = true,
};

static handler_config_t left_ac_handler = {
    .type = HANDLER_TYPE_AC,
    .leftside = true,
};
static handler_config_t right_ac_handler = {
    .type = HANDLER_TYPE_AC,
    .leftside = false,
};
static handler_config_t fan_speed_handler = {
    .type = HANDLER_TYPE_FAN,
};
static handler_config_t left_butt_handler = {
    .type = HANDLER_TYPE_BUTTHEAT,
    .leftside = true,
};
static handler_config_t right_butt_handler = {
    .type = HANDLER_TYPE_BUTTHEAT,
    .leftside = false,
};

static QueueHandle_t tx_task_queue;
static QueueHandle_t display_queue;
static SemaphoreHandle_t done_sem;
static bool twozone_active = false; // updated by twai_receive_task, read by twai_transmit_task
static bool ac_just_updated = false; // set by twai_receive_task when AC controller acks a change
static fan_speed_t current_fan_speed = 0; // updated by twai_receive_task

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");

    data_update_t noconn_msg = {
        .kind = DU_CAN_ERROR,
    };
    bool prev_ac_ack = false;
    while (1) {
        twai_message_t rx_msg;
        esp_err_t err;
        if((err = twai_receive(&rx_msg, pdMS_TO_TICKS(500))) != ESP_OK) {
            ESP_LOGW(TAG, "TWAI recv error: %s", esp_err_to_name(err));
            if(0)xQueueSend(display_queue, &noconn_msg, 0);
            continue;
        }
        if (rx_msg.data_length_code != 8) {
            ESP_LOGI(TAG, "TWAI recv unexpected len %d", rx_msg.data_length_code);
            continue;
        }
        switch(rx_msg.identifier) {
            case CAN_ID_HEATER_STATUS:
                // parse message
                uint8_t b = rx_msg.data[1];  // the only meaningful byte
                butt_temp_t butt_left = b >> 6;
                butt_temp_t butt_right = (b >> 4) & 0b11;

                // send to the queue, overwrite any stale value
                xQueueOverwrite(left_butt_handler.can_queue, &butt_left);
                xQueueOverwrite(right_butt_handler.can_queue, &butt_right);
                break;
            case CAN_ID_AC_STATUS:
                // parse message
                // example:
                //        __ changes from 41 to 61 for 1sec when the value is changed?
                //      /          __ left
                //     |         /     __ right
                //     v        v     v
                // F6 41 6C C8 C0 02 C0 36
                ac_temp_t ac_left = (rx_msg.data[4] >> 1) & 0x3f;
                ac_temp_t ac_right = (rx_msg.data[6]) & 0x3f;
                twozone_active = (rx_msg.data[4] & 0x80) != 0;
                ac_just_updated = (rx_msg.data[1] & 0x20) != 0;
                if(ac_just_updated != prev_ac_ack) {
                    prev_ac_ack = ac_just_updated;
                    data_update_t ack_msg = {
                        .kind = DU_AC_ACK,
                        .ac_ack = ac_just_updated,
                    };
                    xQueueSend(display_queue, &ack_msg, 0);
                }
                fan_speed_t fan_speed = rx_msg.data[1] & 7;
                current_fan_speed = fan_speed;
                // if(rx_msg.data[1] == 0) then fan is off
                // int cur_temp_in_cabin_info = rx_msg.data[3]? variants:
                // bool ac_on = rx_msg.data[5] & 0x8 != 0
                // bool air_from_outside_taken = rx_msg.data[5] & 0x2 != 0
                // bool windshield_heat = rx_msg.data[5] & 0x10 != 0
                xQueueOverwrite(left_ac_handler.can_queue, &ac_left);
                xQueueOverwrite(right_ac_handler.can_queue, &ac_right);
                xQueueOverwrite(fan_speed_handler.can_queue, &fan_speed);
                break;
            default:
                // unsupported message, ignore
                break;
        }
    }
    vTaskDelete(NULL);
}

// Ensure fan is running before sending AC commands.
// Sends fan speed 1 and waits for confirmation (~60ms retry, up to 10 attempts).
static void ensure_fan_on(twai_message_t *msg_fan_set) {
    if(current_fan_speed != FAN_SPEED_OFF) return;

    ESP_LOGI(TAG, "Fan is off, turning on (speed 1) before AC command");
    for(int i = 0; i < 10 && current_fan_speed == FAN_SPEED_OFF; i++) {
        msg_fan_set->data[7] = FAN_SPEED_MIN;
        twai_transmit(msg_fan_set, portMAX_DELAY);
        msg_fan_set->data[7] = 0;
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    if(current_fan_speed == FAN_SPEED_OFF) {
        ESP_LOGW(TAG, "Fan did not turn on after retries, proceeding anyway");
    }
}

static void twai_transmit_task(void *arg)
{
    butt_temp_t butt_temp_left = 0, butt_temp_right = 0;
    data_update_t action;
    twai_message_t msg_buttheat_set = {
        // Message type and format settings
        .extd = 0,              // Standard Format message (11-bit ID)
        .rtr = 0,               // Send a data frame
        .ss = 0,                // Not single shot
        .self = 0,              // Not a self reception request
        .dlc_non_comp = 0,      // DLC is less than 8
        // Message ID and payload
        .identifier = CAN_ID_HEATER_CONTROL,
        .data_length_code = 8,
        .data = {0, 0, 0x20, 0, 0, 0, 0, 0x40},
    };
    twai_message_t msg_ac_set = {
        // Message type and format settings - default...
        // Message ID and payload
        .identifier = CAN_ID_AC_CONTROL,
        .data_length_code = 8,
        .data = {0, 0x41, 0, 2, 0, 0, 0, 0}, // data[0]: 8 = enable heat steering, 4 = disable heat steering
    };
    twai_message_t msg_fan_set = {
        .identifier = CAN_ID_FAN_CONTROL,
        .data_length_code = 8,
        .data = {
            0, // 8 = steering heat on, 4 = steering wheel heat off?
            0x10,  // ?
            0, // ac_two_zone << 4, // 0x10 = toggle_twozone, 0x00 = no_change; 0x01 = toggle_recirculation?..
            0, // 0x40 = (A) btn in a/c control panel
            0x10,
            1, // 0x9 = head, 0x11 = foot+head, 0x19=foot, 0x21 = foot+glass, 
            0, // 1 = ion toggle?.. 0x40 = ?
            0, // fan_strength: 1..7 (fan-off - some other command), 0 means no change; 0x40: fan off
        },
    };
    twai_message_t msg_seatmem_recall = {
        // Message type and format settings - default...
        // Message ID and payload
        .identifier = CAN_ID_SEAT_MEMORY,
        .data_length_code = 8,
        .data = {0, 0, 0, 0, 0, 0, 0, 0},
    };

    
    while (1) {
        if(!xQueueReceive(tx_task_queue, &action, portMAX_DELAY)) {
            continue;
        }

        switch(action.kind) {
            case DU_AC:
                ensure_fan_on(&msg_fan_set);
                msg_ac_set.data[2] = 0;
                msg_ac_set.data[3] = 2;
                if(action.leftside) {
                    msg_ac_set.data[2] = (1 + action.ac_temp) << 2;
                } else {
                    msg_ac_set.data[3] = ((1 + action.ac_temp) << 2) | 2;  // XXX: is that |2 needed?
                }
                twai_transmit(&msg_ac_set, portMAX_DELAY);
                break;
            case DU_BUTTHEAT:
                if(action.leftside) {
                    butt_temp_left = action.butt_temp;
                } else {
                    butt_temp_right = action.butt_temp;
                }
                // update msg...
                msg_buttheat_set.data[1] = (butt_temp_left << 6) | (butt_temp_right << 3);
                // ... and send it!
                twai_transmit(&msg_buttheat_set, portMAX_DELAY);
                break;
            case DU_FANSPEED:
                msg_fan_set.data[7] = action.fan_speed == FAN_SPEED_OFF ? 0x40 : action.fan_speed; // 0x40 is the command to turn fan off
                twai_transmit(&msg_fan_set, portMAX_DELAY);
                msg_fan_set.data[7] = 0; // reset for future use
                break;
            case DU_SEAT_MEMORY:
                // use action.seat_mem_slot
                // 0x04 = first slot, 0x05 = second slot
                msg_seatmem_recall.data[1] = 3 + action.seat_mem_slot;
                twai_transmit(&msg_seatmem_recall, portMAX_DELAY);
                break;
            case DU_DISABLE_TWOZONE:
                if(!twozone_active) {
                    ESP_LOGI(TAG, "Two-zone already disabled");
                    break;
                }
                ensure_fan_on(&msg_fan_set);
                // Stock head unit protocol: 5x toggle + 5x neutral at ~60ms intervals
                // This will freeze can-send task for 600ms, but we control two-zone with longpress and we properly queue other reqs
                // so this should not be too harmful.
                msg_fan_set.data[2] = 0x10;
                for(int i = 0; i < 5; i++) {
                    twai_transmit(&msg_fan_set, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(60));
                }
                msg_fan_set.data[2] = 0;
                for(int i = 0; i < 5; i++) {
                    twai_transmit(&msg_fan_set, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(60));
                }
                ESP_LOGI(TAG, "Two-zone toggle: sent 5+5 burst");
                break;
            default:
                ESP_LOGE(TAG, "Unexpected action in twai TX: %d", action.kind);
                break;
        }
    }
    vTaskDelete(NULL);
}

void seat_memory_apply(int slot) {
    // for now we do it through twai send task
    data_update_t msg = {
        .kind = DU_SEAT_MEMORY,
        .seat_mem_slot = slot,
    };
    // TODO: change timeout?
    xQueueSend(tx_task_queue, &msg, portMAX_DELAY);
}

void disable_twozone_ac(void) {
    data_update_t msg = {
        .kind = DU_DISABLE_TWOZONE,
    };
    xQueueSend(tx_task_queue, &msg, portMAX_DELAY);
}

// Draw seat icon using lines and circles
// facing_right: true = facing right (for left side), false = facing left (for right side)
static void draw_seat(SSD1306_t *dev, int x, bool facing_right) {
    // Seat is 24px wide, 32px tall
    // Backrest: inclined line leaning back
    // Seat cushion: horizontal with curved front

    if(facing_right) {
        // Backrest inclined: top-left to bottom-right
        _ssd1306_line(dev, x+1, 2, x+7, 22, false);
        _ssd1306_line(dev, x+2, 2, x+8, 22, false);
        _ssd1306_line(dev, x+3, 2, x+9, 22, false);

        // Seat cushion (y ~25-28)
        _ssd1306_line(dev, x+7, 25, x+20, 25, false);
        _ssd1306_line(dev, x+7, 26, x+21, 26, false);
        _ssd1306_line(dev, x+8, 27, x+21, 27, false);
        // Front curve
        _ssd1306_circle(dev, x+18, 27, 3, OLED_DRAW_LOWER_RIGHT, false);
    } else {
        // Backrest inclined: top-right to bottom-left
        _ssd1306_line(dev, x+22, 2, x+16, 22, false);
        _ssd1306_line(dev, x+21, 2, x+15, 22, false);
        _ssd1306_line(dev, x+20, 2, x+14, 22, false);

        // Seat cushion
        _ssd1306_line(dev, x+3, 25, x+16, 25, false);
        _ssd1306_line(dev, x+2, 26, x+16, 26, false);
        _ssd1306_line(dev, x+2, 27, x+15, 27, false);
        // Front curve
        _ssd1306_circle(dev, x+5, 27, 3, OLED_DRAW_LOWER_LEFT, false);
    }
}

// Draw a single character 16px wide x 28px tall using the custom font
// (stored as 16x32 with 2px top/bottom padding for page alignment)
// Writes to internal buffer only — call ssd1306_show_buffer() to flush.
static void draw_char_16x28(SSD1306_t *dev, int x, char ch) {
    int idx = font16x28_index(ch);
    for(int page = 0; page < 4; page++) {
        memcpy(&dev->_page[page]._segs[x], font16x28[idx][page], 16);
    }
}

// Draw 3 vertical circular dots (traffic light style), 8px wide, at position x
// level: 0=none, 1=bottom, 2=bottom+middle, 3=all three
static void draw_heat_dots(SSD1306_t *dev, int x, int level) {
    // 3 dots stacked vertically, each ~8px diameter
    // Centers at y=5 (top), y=15 (middle), y=26 (bottom)
    int cx = x + 4;  // center x (8px wide area)

    if(level >= 3) {
        _ssd1306_disc(dev, cx, 5, 3, OLED_DRAW_ALL, false);
    }
    if(level >= 2) {
        _ssd1306_disc(dev, cx, 15, 3, OLED_DRAW_ALL, false);
    }
    if(level >= 1) {
        _ssd1306_disc(dev, cx, 26, 3, OLED_DRAW_ALL, false);
    }
}

// Draw wavy heat lines above seat cushion
// x: left edge of seat area (24px wide), level: 0-3 heat level
// facing_right: true if seat faces right (affects wave positions to avoid backrest)
static void draw_heat_waves(SSD1306_t *dev, int x, int level, bool facing_right) {
    if(level == 0) return;

    // Height of waves depends on level (taller = more heat)
    // level 1: short, level 2: medium, level 3: tall
    int base_y = 22;  // just above seat cushion
    int height = 4 + (level * 3);  // 7, 10, or 13 pixels tall

    // Wave positions adjusted to avoid backrest
    // Facing right: backrest on left, waves shifted right
    // Facing left: backrest on right, waves shifted left
    int wave_xs[3];
    if(facing_right) {
        wave_xs[0] = x + 10;
        wave_xs[1] = x + 14;
        wave_xs[2] = x + 18;
    } else {
        wave_xs[0] = x + 6;
        wave_xs[1] = x + 10;
        wave_xs[2] = x + 14;
    }

    for(int w = 0; w < 3; w++) {
        int wx = wave_xs[w];

        // Draw wave segments going upward
        for(int seg = 0; seg < height; seg++) {
            // Oscillate left/right based on segment
            int offset = ((seg / 2) % 2) ? 1 : 0;
            _ssd1306_pixel(dev, wx + offset, base_y - seg, false);
        }
    }
}

// Draw fan speed as horizontal dashes at the very bottom of the display (y=30-31)
// speed: 0=off (nothing drawn), 1-7 = that many dashes spread across 128px
static void draw_fan_speed(SSD1306_t *dev, fan_speed_t speed) {
    if(speed == 0) return;
    // 7 possible dashes across 128px: each 16px wide, 2px gap
    // Total: 7*16 + 6*2 = 124px, centered with 2px margin each side
    for(int i = 0; i < speed && i < 7; i++) {
        int x = 2 + i * 18;  // 16px dash + 2px gap
        _ssd1306_line(dev, x, 30, x + 15, 30, false);
        _ssd1306_line(dev, x, 31, x + 15, 31, false);
    }
}

// Draw a digit inside a square outline, in a 24x32 area starting at x
static void draw_boxed_digit(SSD1306_t *dev, int x, uint8_t digit) {
    // Square outline
    _ssd1306_line(dev, x, 0, x+23, 0, false);       // top
    _ssd1306_line(dev, x, 31, x+23, 31, false);      // bottom
    _ssd1306_line(dev, x, 0, x, 31, false);           // left
    _ssd1306_line(dev, x+23, 0, x+23, 31, false);    // right
    // Digit centered inside (16px wide, 4px padding each side)
    draw_char_16x28(dev, x + 4, '0' + digit);
}

static void draw_active_display(SSD1306_t *dev,
                                 ac_temp_t ac_left, ac_temp_t ac_right,
                                 butt_temp_t butt_left, butt_temp_t butt_right,
                                 fan_speed_t fan_speed, bool show_just_updated,
                                 uint8_t left_overlay, uint8_t right_overlay) {
    // Clear buffer without sending to display
    uint8_t zeros[128 * 4] = {0};
    ssd1306_set_buffer(dev, zeros);

    // Layout (128px wide):
    // Left side:  [temp 32px][dots 8px][seat 24px] = 64px
    // Right side: [seat 24px][dots 8px][temp 32px] = 64px

    // Left side AC temperature (2 digits, 32px wide total)
    char temp_str[4];
    bool temp_halves = false;
    if(ac_left == AC_TEMP_LO) {
        strncpy(temp_str, "LO", 3);
    } else if(ac_left == AC_TEMP_HI) {
        strncpy(temp_str, "HI", 3);
    } else if(ac_left > AC_TEMP_HI) {
        // value of 33 means a/c is unpowered
        strncpy(temp_str, "--", 3);
    } else {
        snprintf(temp_str, sizeof(temp_str), "%2d", 16+ac_left/2);
        temp_halves = ac_left % 2;
    }
    draw_char_16x28(dev, 0, temp_str[0]);
    draw_char_16x28(dev, 16, temp_str[1]);
    if(temp_halves) {
        _ssd1306_disc(dev, 31, 8, 2, OLED_DRAW_ALL, false);
    }

    // Left heat dots (x=32)
    draw_heat_dots(dev, 32, butt_left);

    // Left seat icon (x=40), facing right — or overlay
    if(left_overlay > 0) {
        draw_boxed_digit(dev, 40, left_overlay);
    } else {
        draw_seat(dev, 40, true);
        draw_heat_waves(dev, 40, butt_left, true);
    }

    // Right seat icon (x=64), facing left — or overlay
    if(right_overlay > 0) {
        draw_boxed_digit(dev, 64, right_overlay);
    } else {
        draw_seat(dev, 64, false);
        draw_heat_waves(dev, 64, butt_right, false);
    }

    // Right heat dots (x=88)
    draw_heat_dots(dev, 88, butt_right);

    // Right side AC temperature (x=96)
    temp_halves = false;
    if(ac_right == AC_TEMP_LO) {
        strncpy(temp_str, "LO", 3);
    } else if(ac_right == AC_TEMP_HI) {
        strncpy(temp_str, "HI", 3);
    } else if(ac_right > AC_TEMP_HI) {
        strncpy(temp_str, "--", 3);
    } else {
        snprintf(temp_str, sizeof(temp_str), "%2d", 16+ac_right/2);  // TODO halves?
        temp_halves = ac_right % 2;
    }
    draw_char_16x28(dev, 96, temp_str[0]);
    draw_char_16x28(dev, 112, temp_str[1]);
    if(temp_halves) {
        _ssd1306_disc(dev, 127, 8, 2, OLED_DRAW_ALL, false);
    }

    // Fan speed indicator at the very bottom
    draw_fan_speed(dev, fan_speed);

    if(show_just_updated) {
        _ssd1306_line(dev, 0, 0, 0, 30, false);
        _ssd1306_line(dev, 127, 0, 127, 30, false);
    }

    // Show the buffer (for _ssd1306_* drawing functions)
    ssd1306_show_buffer(dev);
}

static void display_task(void *arg) {
    SSD1306_t dev;
    ESP_LOGI(TAG, "Disp task start");
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, -1);
    ssd1306_init(&dev, 128, 32);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);

    bool active = false;
    bool need_redraw = true;
    bool scroll_left = true;
    ac_temp_t ac_left_temp = 0, ac_right_temp = 0;
    butt_temp_t butt_left_temp = 0, butt_right_temp = 0;
    fan_speed_t fan_speed = 0;
    bool show_ac_ack = false;

    // Long press overlay state
    uint8_t lp_left_stage = 0;        // 0=none, 1-3=showing slot number
    uint8_t lp_right_stage = 0;
    uint8_t confirm_left_slot = 0;    // 0=none, 1-3=blinking this slot
    uint8_t confirm_left_blinks = 0;  // countdown: odd=visible, even=hidden, 0=done
    uint8_t confirm_right_slot = 0;
    uint8_t confirm_right_blinks = 0;

    while(1) {
        // Compute overlay: stage preview takes priority, then blink
        uint8_t left_overlay = 0, right_overlay = 0;
        if(lp_left_stage > 0) left_overlay = lp_left_stage;
        else if(confirm_left_slot > 0 && confirm_left_blinks % 2 == 1) left_overlay = confirm_left_slot;
        if(lp_right_stage > 0) right_overlay = lp_right_stage;
        else if(confirm_right_slot > 0 && confirm_right_blinks % 2 == 1) right_overlay = confirm_right_slot;

        if(active && need_redraw) {
            ESP_LOGI(TAG, "Disp active redraw");
            ssd1306_hardware_scroll(&dev, SCROLL_STOP);
            draw_active_display(&dev, ac_left_temp, ac_right_temp,
                                butt_left_temp, butt_right_temp,
                                fan_speed, show_ac_ack,
                                left_overlay, right_overlay);
            need_redraw = false;
        } else if(!active && need_redraw) {
            ESP_LOGI(TAG, "Disp passive");
            ssd1306_clear_screen(&dev, false);
            char *banner = "Haval Dargo    ";
            ssd1306_display_text(&dev, 1, banner, strlen(banner), false);
            char *msg = " No Signal ";
            ssd1306_display_text(&dev, 3, msg, strlen(msg), false);
            ssd1306_hardware_scroll(&dev, SCROLL_LEFT);
            scroll_left = true;
            need_redraw = false;
        } else if(!active) {
            // Alternate scroll direction for idle animation
            ssd1306_hardware_scroll(&dev, scroll_left ? SCROLL_LEFT : SCROLL_RIGHT);
            scroll_left = !scroll_left;
        }

        // Shorter timeout during blink animation
        TickType_t timeout = pdMS_TO_TICKS(500);
        if(confirm_left_blinks > 0 || confirm_right_blinks > 0) {
            timeout = pdMS_TO_TICKS(150);
        }

        data_update_t msg;
        if(xQueueReceive(display_queue, &msg, timeout) == pdTRUE) {
            switch(msg.kind) {
                case DU_AC:
                    ESP_LOGI(TAG, "Recvd AC %d = %d", msg.leftside, msg.ac_temp);
                    if(msg.leftside) {
                        ac_left_temp = msg.ac_temp;
                    } else {
                        ac_right_temp = msg.ac_temp;
                    }
                    active = true;
                    need_redraw = true;
                    break;
                case DU_BUTTHEAT:
                    ESP_LOGI(TAG, "Recvd BUTT %d = %d", msg.leftside, msg.butt_temp);
                    if(msg.leftside) {
                        butt_left_temp = msg.butt_temp;
                    } else {
                        butt_right_temp = msg.butt_temp;
                    }
                    active = true;
                    need_redraw = true;
                    break;
                case DU_FANSPEED:
                    ESP_LOGI(TAG, "Recvd FAN = %d", msg.fan_speed);
                    fan_speed = msg.fan_speed;
                    active = true;
                    need_redraw = true;
                    break;
                case DU_CAN_ERROR:
                    ESP_LOGI(TAG, "Recvd ERR");
                    active = false;
                    need_redraw = true;
                    break;
                case DU_LONGPRESS_STAGE:
                    if(msg.leftside) {
                        lp_left_stage = msg.seat_mem_slot;
                        confirm_left_slot = 0;
                        confirm_left_blinks = 0;
                    } else {
                        lp_right_stage = msg.seat_mem_slot;
                        confirm_right_slot = 0;
                        confirm_right_blinks = 0;
                    }
                    need_redraw = true;
                    break;
                case DU_LONGPRESS_CONFIRM:
                    if(msg.leftside) {
                        lp_left_stage = 0;
                        confirm_left_slot = msg.seat_mem_slot;
                        confirm_left_blinks = 5; // on-off-on-off-on then clear
                    } else {
                        lp_right_stage = 0;
                        confirm_right_slot = msg.seat_mem_slot;
                        confirm_right_blinks = 5;
                    }
                    need_redraw = true;
                    break;
                case DU_AC_ACK:
                    show_ac_ack = msg.ac_ack;
                    need_redraw = true;
                    break;
                default:
                    ESP_LOGE(TAG, "Undef msg %d", msg.kind);
                    break;
            }
        } else {
            // Timeout — advance blink countdown
            if(confirm_left_blinks > 0) {
                confirm_left_blinks--;
                if(confirm_left_blinks == 0) confirm_left_slot = 0;
                need_redraw = true;
            }
            if(confirm_right_blinks > 0) {
                confirm_right_blinks--;
                if(confirm_right_blinks == 0) confirm_right_slot = 0;
                need_redraw = true;
            }
        }
    }
}

static void encoder_cb(const rotary_encoder_event_t *evt, void *ctx) {
    QueueHandle_t queue = (QueueHandle_t)ctx;
    xQueueSendToBack(queue, evt, 0);
}

// Long press custom handling
#define RE_ET_LONGPRESS_TICK 99  // synthetic event type for timer ticks
#define LONGPRESS_INTERVAL_US (1000 * 1000)  // 1 second between stages
#define LONGPRESS_MAX_STAGE_LEFT 3   // left knob: stages 1-3, cancel at 4
#define LONGPRESS_MAX_STAGE_RIGHT 1  // right knob: stage 1 only, cancel at 2

typedef struct {
    bool active;
    uint8_t current_stage; // 0=none, 1-N=slot
    bool secondary_rotation; // rotation occurred during long press
    bool click_suppressed;   // suppress next BTN_CLICKED event
    esp_timer_handle_t timer;
} longpress_state_t;

typedef struct {
    QueueHandle_t queue;
    rotary_encoder_handle_t sender;
} longpress_timer_ctx_t;

static void longpress_timer_cb(void *arg) {
    longpress_timer_ctx_t *ctx = (longpress_timer_ctx_t *)arg;
    rotary_encoder_event_t evt = {
        .type = RE_ET_LONGPRESS_TICK,
        .sender = ctx->sender,
    };
    xQueueSendToBack(ctx->queue, &evt, 0);
}

static void encoder_router_task(void *arg) {
    QueueHandle_t encoder_queue = xQueueCreate(16, sizeof(rotary_encoder_event_t));

    rotary_encoder_config_t cfg_left = ROTARY_ENCODER_DEFAULT_CONFIG();
    cfg_left.pin_a = CONFIG_ENCODER_LEFT_A;
    cfg_left.pin_b = CONFIG_ENCODER_LEFT_B;
    cfg_left.pin_btn = CONFIG_BTN_LEFT;
    cfg_left.btn_long_press_time_us = UINT32_MAX; // disable library long press
    cfg_left.callback = encoder_cb;
    cfg_left.callback_ctx = encoder_queue;
    rotary_encoder_config_t cfg_right = ROTARY_ENCODER_DEFAULT_CONFIG();
    cfg_right.pin_a = CONFIG_ENCODER_RIGHT_A;
    cfg_right.pin_b = CONFIG_ENCODER_RIGHT_B;
    cfg_right.pin_btn = CONFIG_BTN_RIGHT;
    cfg_right.btn_long_press_time_us = UINT32_MAX; // disable library long press
    cfg_right.callback = encoder_cb;
    cfg_right.callback_ctx = encoder_queue;

    rotary_encoder_handle_t re_left = NULL, re_right = NULL;
    ESP_ERROR_CHECK(rotary_encoder_create(&cfg_left, &re_left));
    rotary_encoder_enable_acceleration(re_left, 10);
    ESP_ERROR_CHECK(rotary_encoder_create(&cfg_right, &re_right));
    rotary_encoder_enable_acceleration(re_right, 10);

    // Long press state (task never returns, so local vars stay valid for timer callbacks)
    longpress_timer_ctx_t lp_ctx_left, lp_ctx_right;
    longpress_state_t lp_left = {0}, lp_right = {0};

    lp_ctx_left = (longpress_timer_ctx_t){ .queue = encoder_queue, .sender = re_left };
    lp_ctx_right = (longpress_timer_ctx_t){ .queue = encoder_queue, .sender = re_right };

    esp_timer_create_args_t timer_args_left = {
        .callback = longpress_timer_cb,
        .arg = &lp_ctx_left,
        .name = "lp_left",
    };
    esp_timer_create_args_t timer_args_right = {
        .callback = longpress_timer_cb,
        .arg = &lp_ctx_right,
        .name = "lp_right",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args_left, &lp_left.timer));
    ESP_ERROR_CHECK(esp_timer_create(&timer_args_right, &lp_right.timer));

    // now that init is done, listen on queue and route
    while(1) {
        rotary_encoder_event_t evt;
        if(!xQueueReceive(encoder_queue, &evt, portMAX_DELAY)) {
            continue; // nothing received
        }

        bool leftside;
        if(evt.sender == re_left) {
            leftside = true;
        } else if(evt.sender == re_right) {
            leftside = false;
        } else {
            ESP_LOGE(TAG, "Unknown sender %p", evt.sender);
            continue;
        }

        longpress_state_t *lp = leftside ? &lp_left : &lp_right;
        int max_stage = leftside ? LONGPRESS_MAX_STAGE_LEFT : LONGPRESS_MAX_STAGE_RIGHT;
        uint8_t dummy = 0;

        // Cast to int: evt.type is rotary_encoder_event_type_t but we also handle
        // our synthetic RE_ET_LONGPRESS_TICK which is outside the enum.
        switch((int)evt.type) {
            case RE_ET_CHANGED:
                if(lp->active) {
                    // Cancel long press, enter secondary rotation mode
                    esp_timer_stop(lp->timer);
                    lp->active = false;
                    lp->current_stage = 0;
                    lp->secondary_rotation = true;
                    lp->click_suppressed = true;
                    data_update_t du_cancel = {
                        .kind = DU_LONGPRESS_STAGE,
                        .leftside = leftside,
                        .seat_mem_slot = 0,
                    };
                    xQueueSend(display_queue, &du_cancel, 0);
                }
                if(lp->secondary_rotation) {
                    // handle secondary rotation (button held + rotate):
                    // for now it controls fan
                    xQueueSend(fan_speed_handler.control_queue, &evt.diff, 0);
                    ESP_LOGD(TAG, "Secondary rotation %d (%s)", evt.diff, leftside ? "left" : "right");
                } else {
                    // Normal rotation → AC control
                    if(!xQueueSend((leftside ? left_ac_handler : right_ac_handler).control_queue, &evt.diff, 0)) {
                        ESP_LOGE(TAG, "Rot: queue full!");
                    }
                }
                break;

            case RE_ET_BTN_PRESSED:
                lp->active = true;
                lp->current_stage = 0;
                lp->secondary_rotation = false;
                lp->click_suppressed = false;
                esp_timer_start_once(lp->timer, LONGPRESS_INTERVAL_US);
                ESP_LOGI(TAG, "Button pressed (%s), tracking long press", leftside ? "left" : "right");
                break;

            case RE_ET_LONGPRESS_TICK:
                if(!lp->active) break; // race with release, ignore
                lp->current_stage++;
                if(lp->current_stage <= max_stage) {
                    // Show stage on display
                    data_update_t du_stage = {
                        .kind = DU_LONGPRESS_STAGE,
                        .leftside = leftside,
                        .seat_mem_slot = lp->current_stage,
                    };
                    xQueueSend(display_queue, &du_stage, 0);
                    // Schedule next tick
                    esp_timer_start_once(lp->timer, LONGPRESS_INTERVAL_US);
                    ESP_LOGI(TAG, "Long press stage %d (%s)", lp->current_stage, leftside ? "left" : "right");
                } else {
                    // Cancel — held too long
                    lp->active = false;
                    lp->current_stage = 0;
                    lp->click_suppressed = true;
                    data_update_t du_cancel = {
                        .kind = DU_LONGPRESS_STAGE,
                        .leftside = leftside,
                        .seat_mem_slot = 0,
                    };
                    xQueueSend(display_queue, &du_cancel, 0);
                    ESP_LOGI(TAG, "Long press cancelled (held too long, %s)", leftside ? "left" : "right");
                }
                break;

            case RE_ET_BTN_RELEASED: {
                esp_timer_stop(lp->timer);
                uint8_t stage = lp->current_stage;
                bool was_secondary = lp->secondary_rotation;
                lp->active = false;
                lp->current_stage = 0;
                lp->secondary_rotation = false;

                if(was_secondary) {
                    // Rotation cancelled the long press — suppress click, no action
                    lp->click_suppressed = true;
                } else if(stage >= 1 && stage <= max_stage) {
                    // Apply the action
                    if(leftside) {
                        seat_memory_apply(stage);
                    } else {
                        disable_twozone_ac();
                    }
                    // Show confirmation blink
                    data_update_t du_confirm = {
                        .kind = DU_LONGPRESS_CONFIRM,
                        .leftside = leftside,
                        .seat_mem_slot = stage,
                    };
                    xQueueSend(display_queue, &du_confirm, 0);
                    lp->click_suppressed = true;
                    ESP_LOGI(TAG, "Long press applied stage %d (%s)", stage, leftside ? "left" : "right");
                }
                // If stage == 0 and !was_secondary: short press, let BTN_CLICKED through
                break;
            }

            case RE_ET_BTN_CLICKED:
                if(lp->click_suppressed) {
                    lp->click_suppressed = false;
                    break;
                }
                // Normal click → butt heat toggle
                ESP_LOGI(TAG, "BTN click (%s)", leftside ? "left" : "right");
                if(!xQueueSend((leftside ? left_butt_handler : right_butt_handler).control_queue, &dummy, 0)) {
                    ESP_LOGE(TAG, "Btn: queue full!");
                }
                break;

            default:
                break;
        }
    }
}

// Uniform handler
void generic_handler_task(void *ctx) {
    handler_config_t *self = (handler_config_t*)ctx;
    bool is_ac = self->type == HANDLER_TYPE_AC;
    bool is_fan = self->type == HANDLER_TYPE_FAN;
    char *kind = is_ac ? "AC" : is_fan ? "FAN" : "BUTT";

    union_temp_t value = {0};
    rotary_value_t rotdiff;
    union_temp_t canval;
    data_update_t evt;

    evt.kind = is_ac ? DU_AC : is_fan ? DU_FANSPEED : DU_BUTTHEAT;
    evt.leftside = self->leftside;

    QueueSetHandle_t qset = xQueueCreateSet(8 + 1);  // sum of control_queue (8) + can_queue (1)
    xQueueAddToSet(self->control_queue, qset);
    xQueueAddToSet(self->can_queue, qset);

    TickType_t settle_until = 0;  // 0 means not settling
    TickType_t can_tx_earliest = 0;  // earliest tick we may send next CAN tx
    bool can_tx_pending = false;  // true if we have a value waiting to be sent

    #define CAN_TX_INTERVAL pdMS_TO_TICKS(100)

    while(1) {
        TickType_t now = xTaskGetTickCount();
        TickType_t timeout = portMAX_DELAY;

        // Determine nearest deadline from settle and pending CAN tx
        if(settle_until != 0) {
            if(now >= settle_until) {
                settle_until = 0;
            } else {
                timeout = settle_until - now;
            }
        }
        if(can_tx_pending) {
            if(now >= can_tx_earliest) {
                // Time to send the pending CAN message now
                xQueueSend(tx_task_queue, &evt, portMAX_DELAY);
                can_tx_earliest = now + CAN_TX_INTERVAL;
                can_tx_pending = false;
                ESP_LOGI(TAG, "%s task %d: throttled CAN tx sent", kind, self->leftside);
            } else {
                TickType_t tx_wait = can_tx_earliest - now;
                if(tx_wait < timeout) timeout = tx_wait;
            }
        }

        QueueSetMemberHandle_t active_queue = xQueueSelectFromSet(qset, timeout);

        if(active_queue == NULL) {
            // A timeout expired — loop back to check settle/pending tx
            continue;
        }

        // Always receive from whichever queue was signaled
        if(active_queue == self->can_queue) {
            xQueueReceive(self->can_queue, &canval, 0);
            // Ignore CAN events during active input (settling)
            if(settle_until != 0) continue;

            //ESP_LOGI(TAG, "%s task %d: got CAN event %d", kind, self->leftside, canval);
            if(value._max == canval._max) continue; // nothing changed
            value._max = canval._max;
            // emit to display
            if(is_ac)       evt.ac_temp = value.ac;
            else if(is_fan) evt.fan_speed = value.fan;
            else            evt.butt_temp = value.butt;
            xQueueSend(display_queue, &evt, 0);
        } else if(active_queue == self->control_queue) {
            xQueueReceive(self->control_queue, &rotdiff, 0);
            ESP_LOGI(TAG, "%s task %d: got encoder event %d", kind, self->leftside, rotdiff);
            if(is_ac) {
                if(value.ac + rotdiff < AC_TEMP_MIN) {
                    if(rotdiff > 0) {
                        value.ac = AC_TEMP_MIN;
                    } else {
                        value.ac = AC_TEMP_LO;
                    }
                } else {
                    value.ac = MAX(AC_TEMP_LO, MIN(AC_TEMP_HI, value.ac + rotdiff));
                }
            } else if(is_fan) {
                // FIXME what about int overflow when going down?
                value.fan = MAX(FAN_SPEED_OFF, MIN(FAN_SPEED_MAX, value.fan + rotdiff));
            } else {
                if(value.butt == 0) {
                    value.butt = 3;
                } else {
                    value.butt--;
                }
            }

            // emit to display immediately
            if(is_ac)       evt.ac_temp = value.ac;
            else if(is_fan) evt.fan_speed = value.fan;
            else            evt.butt_temp = value.butt;
            xQueueSend(display_queue, &evt, 0);

            now = xTaskGetTickCount();
            if(now >= can_tx_earliest) {
                // Enough time has passed — send immediately
                xQueueSend(tx_task_queue, &evt, portMAX_DELAY);
                can_tx_earliest = now + CAN_TX_INTERVAL;
                can_tx_pending = false;
            } else {
                // Too soon — mark pending, will be sent when can_tx_earliest arrives
                can_tx_pending = true;
            }

            // Enter/extend settle mode
            settle_until = xTaskGetTickCount() + pdMS_TO_TICKS(CONFIG_TX_SETTLE_TIMEOUT_MS);
        }
    }
}

void app_main(void)
{
    //Create semaphores and tasks
    tx_task_queue = xQueueCreate(8, sizeof(data_update_t));
    display_queue = xQueueCreate(8, sizeof(data_update_t));
    left_ac_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    left_ac_handler.can_queue = xQueueCreate(1, sizeof(ac_temp_t));
    right_ac_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    right_ac_handler.can_queue = xQueueCreate(1, sizeof(ac_temp_t));
    fan_speed_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    fan_speed_handler.can_queue = xQueueCreate(1, sizeof(ac_temp_t));
    left_butt_handler.control_queue = xQueueCreate(8, sizeof(uint8_t));
    left_butt_handler.can_queue = xQueueCreate(1, sizeof(butt_temp_t));
    right_butt_handler.control_queue = xQueueCreate(8, sizeof(uint8_t));
    right_butt_handler.can_queue = xQueueCreate(1, sizeof(butt_temp_t));
    done_sem  = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 8, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, 9, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(encoder_router_task, "ENC_router", 4096, NULL, 16, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(display_task, "DISP", 4096, NULL, 10, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "AC_left", 4096, (void*)&left_ac_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "AC_right", 4096, (void*)&right_ac_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "FAN", 4096, (void*)&fan_speed_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "BUTT_left", 4096, (void*)&left_butt_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "BUTT_right", 4096, (void*)&right_butt_handler, 3, NULL, tskNO_AFFINITY);

    while(!xSemaphoreTake(done_sem, portMAX_DELAY));    //Wait for tasks to complete

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(done_sem);
    vQueueDelete(tx_task_queue);
    vQueueDelete(display_queue);
}
