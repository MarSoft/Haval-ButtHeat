/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* CAN bus I/O over TWAI, mirroring the KiCad "can" sheet: node/filter setup,
 * status-frame decoding (0x2D1/0x385/0x29D) into per-handler can_queues, and
 * control-frame encoding/transmission (0x36D/0x1EE/0x367/0x1ED). */

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "buttheat.h"
#include "can.h"
#include "display.h"

#define TAG "ButtHeat"

#define CAN_ID_HEATER_STATUS 0x2D1
#define CAN_ID_HEATER_CONTROL 0x36D
#define CAN_ID_FAN_CONTROL 0x367
#define CAN_ID_AIRFLOW_STATUS 0x29D
#define CAN_ID_AC_STATUS 0x385
#define CAN_ID_AC_CONTROL 0x1EE
#define CAN_ID_SEAT_MEMORY 0x1ED
// TODO: real ID + filter wiring once the bus has been observed.
#define CAN_ID_MASSAGE_STATUS 0x000

// RX message struct for ISR → task queue transfer
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
} rx_can_msg_t;

static twai_node_handle_t twai_node;
static QueueHandle_t twai_rx_queue;
QueueHandle_t tx_task_queue;

// Cross-task file-scope statics, unsynchronized on purpose (single-core,
// latest-value-wins / benign races documented per variable):
static bool twozone_active = false; // updated by twai_receive_task, read by twai_transmit_task
static bool ac_just_updated = false; // set by twai_receive_task when AC controller acks a change
static fan_speed_t current_fan_speed = 0; // updated by twai_receive_task
// Latest seat heater/fan state per side (positive=heat, negative=fan).
// Mirrored from CAN status, overridden by our own commands; read by
// twai_transmit_task when building 0x36D frames.
static butt_temp_t butt_tx_left = 0;
static butt_temp_t butt_tx_right = 0;

/* ------------------------------ RX / TX tasks ----------------------------- */

static IRAM_ATTR bool twai_state_change_isr_cb(twai_node_handle_t handle,
                                                const twai_state_change_event_data_t *edata,
                                                void *user_ctx)
{
    if(edata->new_sta == TWAI_ERROR_BUS_OFF) {
        twai_node_recover(handle);
    }
    return false;
}

static IRAM_ATTR bool twai_rx_isr_cb(twai_node_handle_t handle,
                                       const twai_rx_done_event_data_t *edata,
                                       void *user_ctx)
{
    QueueHandle_t rx_queue = (QueueHandle_t)user_ctx;
    uint8_t buf[8];
    twai_frame_t frame = { .buffer = buf, .buffer_len = sizeof(buf) };
    if(twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        rx_can_msg_t msg = { .id = frame.header.id, .dlc = frame.header.dlc };
        memcpy(msg.data, buf, sizeof(buf));
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(rx_queue, &msg, &woken);
        return woken == pdTRUE;
    }
    return false;
}

#ifdef CAN_DEBUG
// Per-CAN-ID cache for debug change-detection. Small fixed table keyed by ID;
// linear scan is fine — we filter aggressively so only a handful of IDs reach us.
typedef struct {
    uint32_t id;
    uint8_t data[8];
    bool valid;
} can_cache_entry_t;

static bool can_data_changed(uint32_t id, const uint8_t *data) {
    static can_cache_entry_t cache[8] = {0};
    static int n_entries = 0;
    uint8_t size = 8, offset = 0;
    if(id == CAN_ID_AC_STATUS || id == 0x295) {
        // ignore first and last bytes when they are some counters/checksums
        offset = 1;
        size = 6;
    }
    for(int i = 0; i < n_entries; i++) {
        if(cache[i].id == id) {
            if(memcmp(cache[i].data+offset, data+offset, size) == 0) return false;
            memcpy(cache[i].data+offset, data+offset, size);
            return true;
        }
    }
    if(n_entries < (int)(sizeof(cache) / sizeof(cache[0]))) {
        cache[n_entries].id = id;
        memcpy(cache[n_entries].data, data, 8);
        cache[n_entries].valid = true;
        n_entries++;
    }
    return true; // first time we see this ID — log it
}
#endif

static void twai_receive_task(void *arg)
{
    bool prev_ac_ack = false;
    while (1) {
        rx_can_msg_t rx_msg;
        if(xQueueReceive(twai_rx_queue, &rx_msg, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_LOGW(TAG, "TWAI recv timeout");
            data_update_t err_msg = { .kind = DU_CAN_ERROR };
            xQueueSend(display_queue, &err_msg, 0);
            continue;
        }
        if (rx_msg.dlc != 8) {
            ESP_LOGI(TAG, "TWAI recv unexpected len %d", rx_msg.dlc);
            continue;
        }
#ifdef CAN_DEBUG
        if(can_data_changed(rx_msg.id, rx_msg.data)) {
            ESP_LOGI(TAG, "CAN 0x%03lX: %02X %02X %02X %02X %02X %02X %02X %02X",
                     (unsigned long)rx_msg.id,
                     rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
                     rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
        }
#endif
        switch(rx_msg.id) {
            case CAN_ID_HEATER_STATUS:
                // parse message
                uint8_t b = rx_msg.data[1];  // the only meaningful byte
                // data structure of this byte:
                // LLRRllrr, where L/R is heater and l/r is fan
                butt_temp_t butt_left = b >> 6;
                butt_temp_t butt_right = (b >> 4) & 0b11;

                uint8_t butt_fan_left = (b >> 2) & 0b11;
                uint8_t butt_fan_right = b & 0b11;

                if(butt_fan_left > 0) {
                    butt_left = -butt_fan_left;
                }
                if(butt_fan_right > 0) {
                    butt_right = -butt_fan_right;
                }

                butt_tx_left = butt_left;
                butt_tx_right = butt_right;

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
            case CAN_ID_AIRFLOW_STATUS:
                airflow_dir_t airflow = AIRFLOW_FROM_CAN(rx_msg.data[2]);
                xQueueOverwrite(airflow_handler.can_queue, &airflow);
                break;
            case CAN_ID_MASSAGE_STATUS:
                // TODO: real parse once the bus layout is known. For now produce
                // an "off" value so the wiring exercises end-to-end.
                massage_t massage = { .intensity = 0, .mode = 0 };
                xQueueOverwrite(massage_handler.can_queue, &massage);
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
static void ensure_fan_on(uint8_t *fan_data, twai_frame_t *msg_fan_set) {
    if(current_fan_speed != FAN_SPEED_OFF) return;

    ESP_LOGI(TAG, "Fan is off, turning on (speed 1) before AC command");
    for(int i = 0; i < 10 && current_fan_speed == FAN_SPEED_OFF; i++) {
        fan_data[7] = FAN_SPEED_MIN;
        twai_node_transmit(twai_node, msg_fan_set, -1);
        fan_data[7] = 0;
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    if(current_fan_speed == FAN_SPEED_OFF) {
        ESP_LOGW(TAG, "Fan did not turn on after retries, proceeding anyway");
    }
}

static void twai_transmit_task(void *arg)
{
    data_update_t action;

    // TX data buffers (backing storage for twai_frame_t.buffer pointers)

    uint8_t data_buttheat[8] = {
        0,
        // Control frame 0x36D layout (reverse-engineered by probing; note it
        // differs from status frame 0x2D1):
        // byte[1] = LL?RR?ll: bits 7-6 left heat, bits 4-3 right heat,
        //   bits 1-0 LEFT fan level; bits 5 and 2 are no-ops.
        0,
        // byte[2] = rr______: bits 7-6 RIGHT fan level (1..3);
        //   bit 5 (the default 0x20) is NOT required for fan control — kept only
        //   to match the stock unit's frames; bits 4-0 inert.
        // Fan wins over heat when both are set for a side (confirmed both sides).
        // bytes 0, 3-7: no seat effect.
        0x20,
        0,
        0,
        0,
        0,
        0x40,
    };
    uint8_t data_ac[8] = {0, 0x41, 0, 2, 0, 0, 0, 0}; // data[0]: 8 = enable heat steering, 4 = disable heat steering
    uint8_t data_fan[8] = {
        0,     // 8 = steering heat on, 4 = steering wheel heat off?
        0x10,  // ?
        0,     // ac_two_zone << 4, // 0x10 = toggle_twozone, 0x00 = no_change; 0x01 = toggle_recirculation?..
        0,     // 0x40 = (A) btn in a/c control panel
        0x10,
        1,     // 0x9 = head, 0x11 = foot+head, 0x19=foot, 0x21 = foot+glass,
        0,     // 1 = ion toggle?.. 0x40 = ?
        0,     // fan_strength: 1..7 (fan-off - some other command), 0 means no change; 0x40: fan off
    };
    uint8_t data_seatmem[8] = {0};

    twai_frame_t msg_buttheat_set = {
        .header = { .id = CAN_ID_HEATER_CONTROL, .dlc = 8 },
        .buffer = data_buttheat, .buffer_len = 8,
    };
    twai_frame_t msg_ac_set = {
        .header = { .id = CAN_ID_AC_CONTROL, .dlc = 8 },
        .buffer = data_ac, .buffer_len = 8,
    };
    twai_frame_t msg_fan_set = {
        .header = { .id = CAN_ID_FAN_CONTROL, .dlc = 8 },
        .buffer = data_fan, .buffer_len = 8,
    };
    twai_frame_t msg_seatmem_recall = {
        .header = { .id = CAN_ID_SEAT_MEMORY, .dlc = 8 },
        .buffer = data_seatmem, .buffer_len = 8,
    };

    while (1) {
        if(!xQueueReceive(tx_task_queue, &action, portMAX_DELAY)) {
            continue;
        }

        switch(action.kind) {
            case DU_AC:
                ensure_fan_on(data_fan, &msg_fan_set);
                data_ac[2] = 0;
                data_ac[3] = 2;
                if(action.leftside) {
                    data_ac[2] = (1 + action.ac_temp) << 2;
                } else {
                    data_ac[3] = ((1 + action.ac_temp) << 2) | 2;  // XXX: is that |2 needed?
                }
                twai_node_transmit(twai_node, &msg_ac_set, -1);
                break;
            case DU_BUTTHEAT:
                if(action.leftside) {
                    butt_tx_left = action.butt_temp;
                } else {
                    butt_tx_right = action.butt_temp;
                }
                // See the data_buttheat initializer for the 0x36D layout.
                uint8_t heat_left  = butt_tx_left  > 0 ? (uint8_t) butt_tx_left  : 0;
                uint8_t heat_right = butt_tx_right > 0 ? (uint8_t) butt_tx_right : 0;
                uint8_t fan_left   = butt_tx_left  < 0 ? (uint8_t)-butt_tx_left  : 0;
                uint8_t fan_right  = butt_tx_right < 0 ? (uint8_t)-butt_tx_right : 0;
                data_buttheat[1] = (heat_left << 6) | (heat_right << 3) | fan_left;
                data_buttheat[2] = 0x20 | (fan_right << 6);
                twai_node_transmit(twai_node, &msg_buttheat_set, -1);
                break;
            case DU_FANSPEED:
                data_fan[7] = action.fan_speed == FAN_SPEED_OFF ? 0x40 : action.fan_speed;
                twai_node_transmit(twai_node, &msg_fan_set, -1);
                data_fan[7] = 0;
                break;
            case DU_SEAT_MEMORY:
                data_seatmem[1] = 3 + action.seat_mem_slot;
                twai_node_transmit(twai_node, &msg_seatmem_recall, -1);
                break;
            case DU_DISABLE_TWOZONE:
                if(!twozone_active) {
                    ESP_LOGI(TAG, "Two-zone already disabled");
                    break;
                }
                ensure_fan_on(data_fan, &msg_fan_set);
                // Stock head unit protocol: 5x toggle + 5x neutral at ~60ms intervals
                data_fan[2] = 0x10;
                for(int i = 0; i < 5; i++) {
                    twai_node_transmit(twai_node, &msg_fan_set, -1);
                    vTaskDelay(pdMS_TO_TICKS(60));
                }
                data_fan[2] = 0;
                for(int i = 0; i < 5; i++) {
                    twai_node_transmit(twai_node, &msg_fan_set, -1);
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

/* --------------------------------- Setup ---------------------------------- */

void can_init(void) {
    twai_rx_queue = xQueueCreate(8, sizeof(rx_can_msg_t));
    tx_task_queue = xQueueCreate(8, sizeof(data_update_t));

    // TWAI node init (new driver API)
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CONFIG_TX_GPIO_NUM,
            .rx = CONFIG_RX_GPIO_NUM,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = { .bitrate = 500000 },
        .tx_queue_depth = 5,
        .fail_retry_cnt = -1,  // retry forever
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &twai_node));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(twai_node, &(twai_event_callbacks_t){
        .on_rx_done = twai_rx_isr_cb,
        .on_state_change = twai_state_change_isr_cb,
    }, twai_rx_queue));
    // Dual filter: exact 0x385 (AC), combined 0x2D1+0x29D (heater+airflow)
    // 0x2D1 ^ 0x29D = 0x04C (3 bits differ); accepts 8 IDs, only 0x295 extra on bus
    twai_mask_filter_config_t filter = twai_make_dual_filter(
        CAN_ID_AC_STATUS, 0x7FF,                                              // exact 0x385
        CAN_ID_HEATER_STATUS, ~(CAN_ID_HEATER_STATUS ^ CAN_ID_AIRFLOW_STATUS) & 0x7FF, // 0x2D1+0x29D
        false  // standard 11-bit IDs
    );
    ESP_ERROR_CHECK(twai_node_config_mask_filter(twai_node, 0, &filter));

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 8, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, 9, NULL, tskNO_AFFINITY);
}

void can_enable(void) {
    // Enable TWAI only after all handler tasks are created and have set up
    // their queue sets — see app_main for the ordering rationale.
    ESP_ERROR_CHECK(twai_node_enable(twai_node));
    ESP_LOGI(TAG, "TWAI node enabled");
}

void can_shutdown(void) {
    ESP_ERROR_CHECK(twai_node_disable(twai_node));
    ESP_ERROR_CHECK(twai_node_delete(twai_node));
    ESP_LOGI(TAG, "TWAI node deleted");
}
