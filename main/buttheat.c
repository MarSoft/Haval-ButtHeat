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
#include "portmacro.h"

#include "sdkconfig.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "encoder.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define TAG "ButtHeat"

typedef int32_t rotary_value_t;

typedef uint8_t ac_temp_t; // TODO
#define AC_TEMP_MAX 0xff
#define AC_TEMP_MIN 0

typedef uint8_t butt_temp_t;

typedef union {
    ac_temp_t ac;
    butt_temp_t butt;
    uint8_t _max; // for simpler equality check
} union_temp_t;

typedef enum {
    HANDLER_TYPE_AC,
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
    DU_SEAT_MEMORY,
    DU_CAN_ERROR, // put display to "can error" state
} data_update_kind_t;
typedef struct {
    data_update_kind_t kind;
    bool leftside;
    union {
        ac_temp_t ac_temp;
        butt_temp_t butt_temp;
        uint8_t seat_mem_slot;
    };
} data_update_t;

#define CAN_ID_HEATER_STATUS 0x2D1
#define CAN_ID_HEATER_CONTROL 0x36D

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_TX_GPIO_NUM, CONFIG_RX_GPIO_NUM, TWAI_MODE_NO_ACK);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = {
    .acceptance_code = CAN_ID_HEATER_STATUS,
    .acceptance_mask = 0xfffff800,  // only standard 11-bit ID matters
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

    while (1) {
        twai_message_t rx_msg;
        esp_err_t err;
        if((err = twai_receive(&rx_msg, pdMS_TO_TICKS(500))) != ESP_OK) {
            ESP_LOGW(TAG, "TWAI recv error: %s", esp_err_to_name(err));
            if(0)xQueueSend(display_queue, &noconn_msg, 0);
            continue;
        }
        if (rx_msg.identifier != CAN_ID_HEATER_STATUS) {  // safeguard
            ESP_LOGI(TAG, "TWAI recv unexpected ID %x", rx_msg.identifier);
            continue;
        }
        if (rx_msg.data_length_code != 8) {
            ESP_LOGI(TAG, "TWAI recv unexpected len %d", rx_msg.data_length_code);
            continue;
        }

        // parse message
        uint8_t b = rx_msg.data[1];  // the only meaningful byte
        butt_temp_t left = b >> 6;
        butt_temp_t right = (b >> 4) & 0b11;

        // send to the queue, overwrite any stale value
        xQueueOverwrite(left_butt_handler.can_queue, &left);
        xQueueOverwrite(right_butt_handler.can_queue, &right);
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    ac_temp_t ac_temp_left, ac_temp_right;
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

    
    while (1) {
        if(!xQueueReceive(tx_task_queue, &action, portMAX_DELAY)) {
            continue;
        }

        switch(action.kind) {
            case DU_AC:
                if(action.leftside) {
                    ac_temp_left = action.ac_temp;
                } else {
                    ac_temp_right = action.ac_temp;
                }
                ESP_LOGW(TAG, "AC temp transmission not implemented yet");
                // TODO!
                //twai_transmit();
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
            case DU_SEAT_MEMORY:
                // use action.seat_memory_slot
                ESP_LOGE(TAG, "Seat memory is not yet implemented");
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

// Draw a single digit 16px wide x 24px tall (scaled 2x horizontally, 3x vertically from 8x8 font)
// Vertically centered in 32px (4px padding top and bottom)
static void draw_digit_16x24(SSD1306_t *dev, int x, char digit) {
    const uint8_t *glyph = font8x8_basic_tr[(uint8_t)digit];

    // Page 0: 4px padding (top)
    uint8_t page0[16] = {0};
    for(int col = 0; col < 8; col++) {
        uint8_t src = glyph[col];
        // Bottom 1-2 bits of glyph go to top 4 bits of page0 (after 4px padding)
        uint8_t out = 0;
        if(src & 0x01) out |= 0x70;  // bit 0 -> y4-6
        if(src & 0x02) out |= 0x80;  // bit 1 -> y7 (partial)
        page0[col * 2] = out;
        page0[col * 2 + 1] = out;
    }
    ssd1306_display_image(dev, 0, x, page0, 16);

    // Page 1: bits 1-4 of glyph (scaled 3x)
    uint8_t page1[16] = {0};
    for(int col = 0; col < 8; col++) {
        uint8_t src = glyph[col];
        uint8_t out = 0;
        if(src & 0x02) out |= 0x07;  // bit 1 -> y0-2
        if(src & 0x04) out |= 0x38;  // bit 2 -> y3-5
        if(src & 0x08) out |= 0xC0;  // bit 3 -> y6-7 (partial)
        page1[col * 2] = out;
        page1[col * 2 + 1] = out;
    }
    ssd1306_display_image(dev, 1, x, page1, 16);

    // Page 2: bits 3-6 of glyph (scaled 3x)
    uint8_t page2[16] = {0};
    for(int col = 0; col < 8; col++) {
        uint8_t src = glyph[col];
        uint8_t out = 0;
        if(src & 0x08) out |= 0x01;  // bit 3 -> y0 (partial)
        if(src & 0x10) out |= 0x0E;  // bit 4 -> y1-3
        if(src & 0x20) out |= 0x70;  // bit 5 -> y4-6
        if(src & 0x40) out |= 0x80;  // bit 6 -> y7 (partial)
        page2[col * 2] = out;
        page2[col * 2 + 1] = out;
    }
    ssd1306_display_image(dev, 2, x, page2, 16);

    // Page 3: bits 6-7 of glyph + 4px padding (bottom)
    uint8_t page3[16] = {0};
    for(int col = 0; col < 8; col++) {
        uint8_t src = glyph[col];
        uint8_t out = 0;
        if(src & 0x40) out |= 0x03;  // bit 6 -> y0-1
        if(src & 0x80) out |= 0x0C;  // bit 7 -> y2-3
        // y4-7 is padding (0)
        page3[col * 2] = out;
        page3[col * 2 + 1] = out;
    }
    ssd1306_display_image(dev, 3, x, page3, 16);
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
            int offset = ((seg / 2) % 2) ? 1 : -1;
            _ssd1306_pixel(dev, wx + offset, base_y - seg, false);
        }
    }
}

static void draw_active_display(SSD1306_t *dev,
                                 ac_temp_t ac_left, ac_temp_t ac_right,
                                 butt_temp_t butt_left, butt_temp_t butt_right) {
    // Clear buffer without sending to display
    uint8_t zeros[128 * 4] = {0};
    ssd1306_set_buffer(dev, zeros);

    // Layout (128px wide):
    // Left side:  [temp 32px][dots 8px][seat 24px] = 64px
    // Right side: [seat 24px][dots 8px][temp 32px] = 64px

    // Left side AC temperature (2 digits, 32px wide total)
    char temp_str[4];
    snprintf(temp_str, sizeof(temp_str), "%2d", ac_left);
    draw_digit_16x24(dev, 0, temp_str[0]);
    draw_digit_16x24(dev, 16, temp_str[1]);

    // Left heat dots (x=32)
    draw_heat_dots(dev, 32, butt_left);

    // Left seat icon (x=40), facing right
    draw_seat(dev, 40, true);
    draw_heat_waves(dev, 40, butt_left, true);

    // Right seat icon (x=64), facing left
    draw_seat(dev, 64, false);
    draw_heat_waves(dev, 64, butt_right, false);

    // Right heat dots (x=88)
    draw_heat_dots(dev, 88, butt_right);

    // Right side AC temperature (x=96)
    snprintf(temp_str, sizeof(temp_str), "%2d", ac_right);
    draw_digit_16x24(dev, 96, temp_str[0]);
    draw_digit_16x24(dev, 112, temp_str[1]);

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

    while(1) {
        if(active && need_redraw) {
            ESP_LOGI(TAG, "Disp active redraw");
            ssd1306_hardware_scroll(&dev, SCROLL_STOP);
            draw_active_display(&dev, ac_left_temp, ac_right_temp, butt_left_temp, butt_right_temp);
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

        data_update_t msg;
        if(xQueueReceive(display_queue, &msg, pdMS_TO_TICKS(500)) == pdTRUE) {
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
                case DU_CAN_ERROR:
                    ESP_LOGI(TAG, "Recvd ERR");
                    active = false;
                    need_redraw = true;
                    break;
                default:
                    ESP_LOGE(TAG, "Undef msg %d", msg.kind);
                    break;
            }
        }
    }
}

static void encoder_router_task(void *arg) {
    QueueHandle_t encoder_queue = xQueueCreate(16, sizeof(rotary_encoder_event_t));
    rotary_encoder_init(encoder_queue);

    rotary_encoder_t re_left = {
        .pin_a = CONFIG_ENCODER_LEFT_A,
        .pin_b = CONFIG_ENCODER_LEFT_B,
        .pin_btn = CONFIG_BTN_LEFT,
    };
    rotary_encoder_add(&re_left);
    rotary_encoder_enable_acceleration(&re_left, 100);  // TODO
    rotary_encoder_t re_right = {
        .pin_a = CONFIG_ENCODER_RIGHT_A,
        .pin_b = CONFIG_ENCODER_RIGHT_B,
        .pin_btn = CONFIG_BTN_RIGHT,
    };
    rotary_encoder_add(&re_right);

    // now that init is done, listen on queue and route
    while(1) {
        rotary_encoder_event_t evt;
        if(!xQueueReceive(encoder_queue, &evt, portMAX_DELAY)) {
            continue; // nothing received
        }

        bool leftside;
        if(evt.sender == &re_left) {
            leftside = true;
        } else if(evt.sender == &re_right) {
            leftside = false;
        } else {
            ESP_LOGE(TAG, "Unknown sender %p", evt.sender);
            continue;
        }

        uint8_t dummy = 0;

        // got event, route it!
        switch(evt.type) {
            case RE_ET_CHANGED:
                if(!xQueueSend((leftside ? left_ac_handler : right_ac_handler).control_queue, &evt.diff, 0)) {
                    ESP_LOGE(TAG, "Rot: queue full!");
                }
                break;
            case RE_ET_BTN_CLICKED:
                ESP_LOGI(TAG, "BTN send");
                if(!xQueueSend((leftside ? left_butt_handler : right_butt_handler).control_queue, &dummy, 0)) {
                    ESP_LOGE(TAG, "Btn: queue full!");
                }
                break;
            case RE_ET_BTN_LONG_PRESSED:
                // seat memory
                ESP_LOGI(TAG, "Got long press");
                seat_memory_apply(leftside ? 1 : 2);
                break;
            default:
                // ignore press & release events for now
                break;
        }
    }
}

// Uniform handler
void generic_handler_task(void *ctx) {
    handler_config_t *self = (handler_config_t*)ctx;
    bool is_ac = self->type == HANDLER_TYPE_AC;
    char *kind = is_ac ? "AC" : "BUTT";

    union_temp_t value = {0};  // TODO
    rotary_value_t rotdiff;
    union_temp_t canval;
    data_update_t evt;

    evt.kind = is_ac ? DU_AC : DU_BUTTHEAT;
    evt.leftside = self->leftside;

    QueueSetHandle_t qset = xQueueCreateSet(8 + 1);  // sum of control_queue (8) + can_queue (1)
    xQueueAddToSet(self->control_queue, qset);
    xQueueAddToSet(self->can_queue, qset);

    TickType_t settle_until = 0;  // 0 means not settling

    while(1) {
        TickType_t timeout;
        if(settle_until == 0) {
            timeout = portMAX_DELAY;
        } else {
            TickType_t now = xTaskGetTickCount();
            if(now >= settle_until) {
                settle_until = 0;
                timeout = portMAX_DELAY;
            } else {
                timeout = settle_until - now;
            }
        }

        QueueSetMemberHandle_t active_queue = xQueueSelectFromSet(qset, timeout);

        if(active_queue == NULL) {
            // Settle timeout expired
            settle_until = 0;
            continue;
        }

        // Always receive from whichever queue was signaled
        if(active_queue == self->can_queue) {
            xQueueReceive(self->can_queue, &canval, 0);
            // Ignore CAN events during active input (settling)
            if(settle_until != 0) continue;

            ESP_LOGI(TAG, "%s task %d: got CAN event %d", kind, self->leftside, canval);
            if(value._max == canval._max) continue; // nothing changed
            value._max = canval._max;
            // emit to display
            if(is_ac) evt.ac_temp = value.ac;
            else      evt.butt_temp = value.butt;
            xQueueSend(display_queue, &evt, 0);
        } else if(active_queue == self->control_queue) {
            xQueueReceive(self->control_queue, &rotdiff, 0);
            ESP_LOGI(TAG, "%s task %d: got encoder event %d", kind, self->leftside, rotdiff);
            if(is_ac) {
                value.ac = MAX(AC_TEMP_MIN, MIN(AC_TEMP_MAX, value.ac + rotdiff));
            } else {
                if(value.butt == 0) {
                    value.butt = 3;
                } else {
                    value.butt--;
                }
            }

            // emit to display
            if(is_ac) evt.ac_temp = value.ac;
            else      evt.butt_temp = value.butt;
            xQueueSend(display_queue, &evt, 0);
            // and emit to CAN
            xQueueSend(tx_task_queue, &evt, portMAX_DELAY);

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
    left_butt_handler.control_queue = xQueueCreate(8, sizeof(uint8_t));
    left_butt_handler.can_queue = xQueueCreate(1, sizeof(butt_temp_t));
    right_butt_handler.control_queue = xQueueCreate(8, sizeof(uint8_t));
    right_butt_handler.can_queue = xQueueCreate(1, sizeof(butt_temp_t));

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 8, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, 9, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(encoder_router_task, "ENC_router", 4096, NULL, 16, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(display_task, "DISP", 4096, NULL, 10, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "AC_left", 4096, (void*)&left_ac_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "AC_right", 4096, (void*)&right_ac_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "BUTT_left", 4096, (void*)&left_butt_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "BUTT_right", 4096, (void*)&right_butt_handler, 3, NULL, tskNO_AFFINITY);

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(TAG, "Driver uninstalled");

    //Cleanup
    vQueueDelete(tx_task_queue);
    vQueueDelete(display_queue);
}
