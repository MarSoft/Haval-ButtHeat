/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* SSD1306 128x32 rendering, mirroring the KiCad "display" sheet: passive
 * banner / active overlay / CAN-error states. */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "ssd1306.h"
#include "font16x28.h"  // generated with tools/gen_font16x28.py

#include "buttheat.h"
#include "display.h"

#define TAG "ButtHeat"

QueueHandle_t display_queue;

/* ------------------------------- Drawing ---------------------------------- */

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

// Draw an "x" mark centered at (cx, cy), ~7px wide (fan blade-like shape)
static void draw_x_mark(SSD1306_t *dev, int cx, int cy) {
    _ssd1306_line(dev, cx-3, cy-3, cx+3, cy+3, false);
    _ssd1306_line(dev, cx-3, cy+3, cx+3, cy-3, false);
    // thicken slightly for visibility
    _ssd1306_line(dev, cx-2, cy-3, cx+3, cy+2, false);
    _ssd1306_line(dev, cx-3, cy+2, cx+2, cy-3, false);
}

// Draw 3 vertical level indicators, 8px wide, at position x
// level > 0: filled circular dots for heater (1..3)
// level < 0: "x" marks for fan (-1..-3)
// In both cases the count cumulates from the bottom.
static void draw_heat_dots(SSD1306_t *dev, int x, int level) {
    // 3 markers stacked vertically, each ~8px tall
    // Centers at y=5 (top), y=15 (middle), y=26 (bottom)
    int cx = x + 4;  // center x (8px wide area)
    int n = level < 0 ? -level : level;
    bool is_fan = level < 0;

    int ys[3] = {26, 15, 5};
    for(int i = 0; i < n && i < 3; i++) {
        if(is_fan) {
            draw_x_mark(dev, cx, ys[i]);
        } else {
            _ssd1306_disc(dev, cx, ys[i], 3, OLED_DRAW_ALL, false);
        }
    }
}

// Draw massage indicator in a narrow column (driver side, x=50..58).
// Layout (subject to change once the real CAN format is decoded):
//   Left 5px  (x..x+4)  — intensity: up to 3 stacked rectangles, filled
//                         from the bottom.
//   2px gap   (x+5..x+6)
//   Right 2px (x+7..x+8) — mode: vertical line, full y=0..23
//      GENERAL  — solid
//      SHOULDER — upper half only
//      WAIST    — lower half only
//      DOTS     — every other pixel
//      WAVES    — dashed (2 on, 1 off)
static void draw_massage_indicator(SSD1306_t *dev, int x, massage_t m) {
    if(MASSAGE_IS_OFF(m)) return;

    // Intensity rectangles, filled from bottom. Each 5x6 with 2px vertical gaps.
    int rect_w = 5;
    int rect_h = 6;
    int rect_gap = 2;
    for(int i = 0; i < m.intensity && i < 3; i++) {
        int yb = 23 - i * (rect_h + rect_gap);
        int yt = yb - (rect_h - 1);
        for(int dy = yt; dy <= yb; dy++) {
            _ssd1306_line(dev, x, dy, x + rect_w - 1, dy, false);
        }
    }

    // Mode vertical line at x+7..x+8, full height (y=0..23).
    int lx0 = x + 7, lx1 = x + 8;
    int y_lo = 0, y_hi = 23;
    int y_mid = (y_lo + y_hi) / 2;

    switch(m.mode) {
        case MASSAGE_MODE_GENERAL:
            _ssd1306_line(dev, lx0, y_lo, lx0, y_hi, false);
            _ssd1306_line(dev, lx1, y_lo, lx1, y_hi, false);
            break;
        case MASSAGE_MODE_SHOULDER:
            _ssd1306_line(dev, lx0, y_lo, lx0, y_mid, false);
            _ssd1306_line(dev, lx1, y_lo, lx1, y_mid, false);
            break;
        case MASSAGE_MODE_WAIST:
            _ssd1306_line(dev, lx0, y_mid + 1, lx0, y_hi, false);
            _ssd1306_line(dev, lx1, y_mid + 1, lx1, y_hi, false);
            break;
        case MASSAGE_MODE_DOTS:
            for(int y = y_lo; y <= y_hi; y += 2) {
                _ssd1306_pixel(dev, lx0, y, false);
                _ssd1306_pixel(dev, lx1, y, false);
            }
            break;
        case MASSAGE_MODE_WAVES:
            // Zigzag: alternate left/right column every 2 rows
            for(int y = y_lo; y <= y_hi; y++) {
                int lx = ((y / 2) % 2) ? lx1 : lx0;
                _ssd1306_pixel(dev, lx, y, false);
            }
            break;
    }
}

// Draw airflow direction icon between the two seats.
// cx: horizontal center. 64 centers it (no massage); 70 shifts it right
// when the massage indicator occupies x=50..58 (leaves ~4px gap).
// Footprint: 14px wide, y=0..22 (full inter-seat vertical space). 2px strokes.
static void draw_airflow_icon(SSD1306_t *dev, int cx, airflow_dir_t dir) {
    if(dir == AIRFLOW_OFF) return;

    bool has_face = (dir == AIRFLOW_FACE || dir == AIRFLOW_FACE_FEET);
    bool has_feet = (dir == AIRFLOW_FEET || dir == AIRFLOW_FACE_FEET || dir == AIRFLOW_FEET_WINDSHIELD);
    bool has_wind = (dir == AIRFLOW_WINDSHIELD || dir == AIRFLOW_FEET_WINDSHIELD);

    // Upward arrow (face vents): tip at y=0
    if(has_face) {
        int y_top = 0;
        int y_bot = has_feet ? 9 : 11;
        // doubled shaft
        _ssd1306_line(dev, cx,   y_top,   cx,   y_bot, false);
        _ssd1306_line(dev, cx-1, y_top,   cx-1, y_bot, false);
        // doubled barbs from tip outward-down
        _ssd1306_line(dev, cx-1, y_top,   cx-6, y_top+5, false);
        _ssd1306_line(dev, cx-1, y_top+1, cx-5, y_top+5, false);
        _ssd1306_line(dev, cx,   y_top,   cx+5, y_top+5, false);
        _ssd1306_line(dev, cx,   y_top+1, cx+4, y_top+5, false);
    }

    // Downward arrow (feet)
    if(has_feet) {
        int y_top = has_face ? 12 : 0;
        int y_bot = has_face ? 22 : 11;
        _ssd1306_line(dev, cx,   y_top, cx,   y_bot,   false);
        _ssd1306_line(dev, cx-1, y_top, cx-1, y_bot,   false);
        _ssd1306_line(dev, cx-1, y_bot,   cx-6, y_bot-5, false);
        _ssd1306_line(dev, cx-1, y_bot-1, cx-5, y_bot-5, false);
        _ssd1306_line(dev, cx,   y_bot,   cx+5, y_bot-5, false);
        _ssd1306_line(dev, cx,   y_bot-1, cx+4, y_bot-5, false);
    }

    // Windshield (trapezoid)
    if(has_wind) {
        int top_y = has_feet ? 0 : 4;
        int bot_y = top_y + 11;
        _ssd1306_line(dev, cx-7, bot_y,   cx-3, top_y,   false);  // left slope
        _ssd1306_line(dev, cx-6, bot_y,   cx-2, top_y,   false);
        _ssd1306_line(dev, cx+7, bot_y,   cx+3, top_y,   false);  // right slope
        _ssd1306_line(dev, cx+6, bot_y,   cx+2, top_y,   false);
        _ssd1306_line(dev, cx-3, top_y,   cx+3, top_y,   false);  // top edge
        _ssd1306_line(dev, cx-3, top_y+1, cx+3, top_y+1, false);
        _ssd1306_line(dev, cx-7, bot_y,   cx+7, bot_y,   false);  // bottom edge
        _ssd1306_line(dev, cx-7, bot_y-1, cx+7, bot_y-1, false);
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
                                 fan_speed_t fan_speed, airflow_dir_t airflow_dir,
                                 massage_t massage,
                                 bool show_just_updated,
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
    }

    // Right seat icon (x=64), facing left — or overlay
    if(right_overlay > 0) {
        draw_boxed_digit(dev, 64, right_overlay);
    } else {
        draw_seat(dev, 64, false);
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

    // Massage indicator in the left strip (driver side), and shift airflow right
    // to free that space.
    int airflow_cx = 64;
    if(!MASSAGE_IS_OFF(massage)) {
        draw_massage_indicator(dev, 50, massage);
        airflow_cx = 70;
    }

    // Airflow direction icon between the seats.
    draw_airflow_icon(dev, airflow_cx, airflow_dir);

    // Fan speed indicator at the very bottom
    draw_fan_speed(dev, fan_speed);

    if(show_just_updated) {
        _ssd1306_line(dev, 0, 0, 0, 30, false);
        _ssd1306_line(dev, 127, 0, 127, 30, false);
    }

    // Show the buffer (for _ssd1306_* drawing functions)
    ssd1306_show_buffer(dev);
}

/* ------------------------------ Display task ------------------------------ */

void display_hw_init(SSD1306_t *dev) {
    i2c_master_init(dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, -1);
    ssd1306_init(dev, 128, 32);
    ssd1306_clear_screen(dev, false);
    ssd1306_contrast(dev, 0xff);
}

static void display_task(void *arg) {
    SSD1306_t dev;
    ESP_LOGI(TAG, "Disp task start");
    display_hw_init(&dev);

    bool active = false;
    bool need_redraw = true;
    bool scroll_left = true;
    ac_temp_t ac_left_temp = 0, ac_right_temp = 0;
    butt_temp_t butt_left_temp = 0, butt_right_temp = 0;
    fan_speed_t fan_speed = 0;
    airflow_dir_t airflow_dir = AIRFLOW_OFF;
    massage_t massage = {0};
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
                                fan_speed, airflow_dir, massage, show_ac_ack,
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
                case DU_AIRFLOW:
                    ESP_LOGI(TAG, "Recvd AIRFLOW = %d", msg.airflow_dir);
                    airflow_dir = msg.airflow_dir;
                    active = true;
                    need_redraw = true;
                    break;
                case DU_MASSAGE:
                    ESP_LOGI(TAG, "Recvd MASSAGE mode=%d intensity=%d",
                             msg.massage.mode, msg.massage.intensity);
                    massage = msg.massage;
                    active = true;
                    need_redraw = true;
                    break;
                case DU_CAN_ERROR:
                    ESP_LOGE(TAG, "Recvd CAN_ERROR! leftside=%d val=%d", msg.leftside, msg.butt_temp);
                    active = false;
                    need_redraw = true;
                    break;
                case DU_LONGPRESS_STAGE:
                    if(msg.fanside) {
                        // TODO nothing yet
                    } else if(msg.leftside) {
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
                    if(msg.fanside) {
                        // TODO nothing yet
                    } else if(msg.leftside) {
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

void display_init(void) {
    display_queue = xQueueCreate(8, sizeof(data_update_t));
    xTaskCreatePinnedToCore(display_task, "DISP", 4096, NULL, 10, NULL, tskNO_AFFINITY);
}
