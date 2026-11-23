/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* Rotary encoder + button input, mirroring the KiCad "controls" sheet:
 * three encoders share one queue; the router task disambiguates by sender
 * handle and implements staged long-press via esp_timer synthetic events. */

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "encoder.h"

#include "buttheat.h"
#include "can.h"
#include "controls.h"
#include "display.h"

#define TAG "ButtHeat"

static void encoder_cb(const rotary_encoder_event_t *evt, void *ctx) {
    QueueHandle_t queue = (QueueHandle_t)ctx;
    xQueueSendToBack(queue, evt, 0);
}

// Long press custom handling
#define RE_ET_LONGPRESS_TICK 99  // synthetic event type for timer ticks
#define LONGPRESS_INTERVAL_US (1000 * 1000)  // 1 second between stages
#define LONGPRESS_MAX_STAGE_LEFT 3   // left knob: stages 1-3, cancel at 4
#define LONGPRESS_MAX_STAGE_RIGHT 1  // right knob: stage 1 only, cancel at 2
#define LONGPRESS_MAX_STAGE_FAN 1  // fan knob: stage 1 only, cancel at 2

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

    // Future buttons are wired to the same pins as ENC3 A/B; ENC3 rests with
    // A/B disconnected between detents, so real buttons will need dedup logic
    // to tell button presses from encoder transitions.
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
    rotary_encoder_config_t cfg_fan = ROTARY_ENCODER_DEFAULT_CONFIG();
    cfg_fan.pin_a = CONFIG_ENCODER_FAN_A;
    cfg_fan.pin_b = CONFIG_ENCODER_FAN_B;
    cfg_fan.pin_btn = CONFIG_BTN_FAN;
    cfg_fan.btn_long_press_time_us = UINT32_MAX; // disable library long press
    cfg_fan.callback = encoder_cb;
    cfg_fan.callback_ctx = encoder_queue;

    rotary_encoder_handle_t re_left = NULL, re_right = NULL, re_fan;
    ESP_ERROR_CHECK(rotary_encoder_create(&cfg_left, &re_left));
    rotary_encoder_enable_acceleration(re_left, 10);
    ESP_ERROR_CHECK(rotary_encoder_create(&cfg_right, &re_right));
    rotary_encoder_enable_acceleration(re_right, 10);
    ESP_ERROR_CHECK(rotary_encoder_create(&cfg_fan, &re_fan));
    rotary_encoder_enable_acceleration(re_fan, 10);

    // Long press state (task never returns, so local vars stay valid for timer callbacks)
    longpress_timer_ctx_t lp_ctx_left, lp_ctx_right, lp_ctx_fan;
    longpress_state_t lp_left = {0}, lp_right = {0}, lp_fan = {0};

    lp_ctx_left = (longpress_timer_ctx_t){ .queue = encoder_queue, .sender = re_left };
    lp_ctx_right = (longpress_timer_ctx_t){ .queue = encoder_queue, .sender = re_right };
    lp_ctx_fan = (longpress_timer_ctx_t){ .queue = encoder_queue, .sender = re_fan };

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
    esp_timer_create_args_t timer_args_fan = {
        .callback = longpress_timer_cb,
        .arg = &lp_ctx_fan,
        .name = "lp_fan",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args_left, &lp_left.timer));
    ESP_ERROR_CHECK(esp_timer_create(&timer_args_right, &lp_right.timer));
    ESP_ERROR_CHECK(esp_timer_create(&timer_args_fan, &lp_fan.timer));

    // now that init is done, listen on queue and route
    while(1) {
        rotary_encoder_event_t evt;
        if(!xQueueReceive(encoder_queue, &evt, portMAX_DELAY)) {
            continue; // nothing received
        }

        bool leftside = false, fanside = false;
        if(evt.sender == re_left) {
            leftside = true;
        } else if(evt.sender == re_right) {
            leftside = false;
        } else if(evt.sender == re_fan) {
            fanside = true;
        } else {
            ESP_LOGE(TAG, "Unknown sender %p", evt.sender);
            continue;
        }

        longpress_state_t *lp = leftside ? &lp_left : &lp_right;
        int max_stage = fanside ? LONGPRESS_MAX_STAGE_FAN : leftside ? LONGPRESS_MAX_STAGE_LEFT : LONGPRESS_MAX_STAGE_RIGHT;
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
                    // Normal rotation → AC control (or fan for enc3)
                    if(!xQueueSend((fanside ? fan_speed_handler : leftside ? left_ac_handler : right_ac_handler).control_queue, &evt.diff, 0)) {
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
                ESP_LOGI(TAG, "Button pressed (%s), tracking long press", fanside ? "fan" : leftside ? "left" : "right");
                break;

            case RE_ET_LONGPRESS_TICK:
                if(!lp->active) break; // race with release, ignore
                lp->current_stage++;
                if(lp->current_stage <= max_stage) {
                    // Show stage on display
                    data_update_t du_stage = {
                        .kind = DU_LONGPRESS_STAGE,
                        .leftside = leftside,
                        .fanside = fanside,
                        .seat_mem_slot = lp->current_stage,
                    };
                    xQueueSend(display_queue, &du_stage, 0);
                    // Schedule next tick
                    esp_timer_start_once(lp->timer, LONGPRESS_INTERVAL_US);
                    ESP_LOGI(TAG, "Long press stage %d (%s)", lp->current_stage, fanside ? "fan" : leftside ? "left" : "right");
                } else {
                    // Cancel — held too long
                    lp->active = false;
                    lp->current_stage = 0;
                    lp->click_suppressed = true;
                    data_update_t du_cancel = {
                        .kind = DU_LONGPRESS_STAGE,
                        .leftside = leftside,
                        .fanside = fanside,
                        .seat_mem_slot = 0,
                    };
                    xQueueSend(display_queue, &du_cancel, 0);
                    ESP_LOGI(TAG, "Long press cancelled (held too long, %s)", fanside ? "fan" : leftside ? "left" : "right");
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
                    if(fanside) {
                        // TODO: no action yet
                    } else if(leftside) {
                        seat_memory_apply(stage);
                    } else {
                        disable_twozone_ac();
                    }
                    // Show confirmation blink
                    data_update_t du_confirm = {
                        .kind = DU_LONGPRESS_CONFIRM,
                        .leftside = leftside,
                        .fanside = fanside,
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
                ESP_LOGI(TAG, "BTN click (%s)", fanside ? "fan" : leftside ? "left" : "right");
                if(!xQueueSend((fanside ? fan_speed_handler : leftside ? left_butt_handler : right_butt_handler).control_queue, &dummy, 0)) {
                    ESP_LOGE(TAG, "Btn: queue full!");
                }
                break;

            default:
                break;
        }
    }
}

void controls_init(void) {
    xTaskCreatePinnedToCore(encoder_router_task, "ENC_router", 4096, NULL, 16, NULL, tskNO_AFFINITY);
}
