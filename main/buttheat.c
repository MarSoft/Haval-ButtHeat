/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* ButtHeat application core, mirroring the KiCad "mcu" role: the per-control
 * handler tasks (controls ↔ CAN ↔ display multiplexing with TX throttling)
 * and startup wiring. Hardware sheets live in can.c / controls.c / display.c;
 * shared types and the handler registry are in buttheat.h. */

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "buttheat.h"
#include "can.h"
#include "controls.h"
#include "display.h"

#define TAG "ButtHeat"

static SemaphoreHandle_t done_sem;

handler_config_t left_ac_handler = {
    .type = HANDLER_TYPE_AC,
    .leftside = true,
};
handler_config_t right_ac_handler = {
    .type = HANDLER_TYPE_AC,
    .leftside = false,
};
handler_config_t fan_speed_handler = {
    .type = HANDLER_TYPE_FAN,
};
handler_config_t airflow_handler = {
    .type = HANDLER_TYPE_AIRFLOW,
};
handler_config_t left_butt_handler = {
    .type = HANDLER_TYPE_BUTTHEAT,
    .leftside = true,
};
handler_config_t right_butt_handler = {
    .type = HANDLER_TYPE_BUTTHEAT,
    .leftside = false,
};
// Driver-side massage only; no right-seat counterpart on this vehicle.
handler_config_t massage_handler = {
    .type = HANDLER_TYPE_MASSAGE,
    .leftside = true,
};

/* ----------------------------- Handler task ------------------------------- */

// Uniform handler
static void generic_handler_task(void *ctx) {
    handler_config_t *self = (handler_config_t*)ctx;
    bool is_ac = self->type == HANDLER_TYPE_AC;
    bool is_fan = self->type == HANDLER_TYPE_FAN;
    bool is_airflow = self->type == HANDLER_TYPE_AIRFLOW;
    bool is_massage = self->type == HANDLER_TYPE_MASSAGE;
    char *kind = is_ac ? "AC" : is_fan ? "FAN" : is_airflow ? "AIRFLOW" : is_massage ? "MASSAGE" : "BUTT";

    union_temp_t value = {0};
    rotary_value_t rotdiff;
    union_temp_t canval;
    data_update_t evt;

    evt.kind = is_ac ? DU_AC : is_fan ? DU_FANSPEED : is_airflow ? DU_AIRFLOW
             : is_massage ? DU_MASSAGE : DU_BUTTHEAT;
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
            if(is_ac)            evt.ac_temp = value.ac;
            else if(is_fan)      evt.fan_speed = value.fan;
            else if(is_airflow)  evt.airflow_dir = value.airflow;
            else if(is_massage)  evt.massage = value.massage;
            else                 evt.butt_temp = value.butt;
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
            } else if(is_airflow) {
                // TODO: airflow control via encoder (not yet wired)
                value.airflow = MAX(AIRFLOW_FACE, MIN(AIRFLOW_WINDSHIELD, value.airflow + rotdiff));
            } else if(is_massage) {
                // Massage is show-only for now; ignore control input.
                continue;
            } else {
                // Cycle heat: 3 → 2 → 1 → 0 → 3. If fan is on (negative), decrease till 0.
                if(value.butt < 0) {
                    value.butt++;  // decrease fan speed
                } else if(value.butt == 0) {
                    value.butt = 3;
                } else {
                    value.butt--;
                }
            }

            // emit to display immediately
            if(is_ac)            evt.ac_temp = value.ac;
            else if(is_fan)      evt.fan_speed = value.fan;
            else if(is_airflow)  evt.airflow_dir = value.airflow;
            else if(is_massage)  evt.massage = value.massage;
            else                 evt.butt_temp = value.butt;
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

/* -------------------------------- Startup --------------------------------- */

void app_main(void)
{
    //Create handler queues and semaphores
    left_ac_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    left_ac_handler.can_queue = xQueueCreate(1, sizeof(ac_temp_t));
    right_ac_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    right_ac_handler.can_queue = xQueueCreate(1, sizeof(ac_temp_t));
    fan_speed_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    fan_speed_handler.can_queue = xQueueCreate(1, sizeof(fan_speed_t));
    airflow_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    airflow_handler.can_queue = xQueueCreate(1, sizeof(airflow_dir_t));
    // NOTE: BUTT control queues hold uint8_t; the handler receives
    // rotary_value_t (harmless — value ignored, only presence matters).
    left_butt_handler.control_queue = xQueueCreate(8, sizeof(uint8_t));
    left_butt_handler.can_queue = xQueueCreate(1, sizeof(butt_temp_t));
    right_butt_handler.control_queue = xQueueCreate(8, sizeof(uint8_t));
    right_butt_handler.can_queue = xQueueCreate(1, sizeof(butt_temp_t));
    massage_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    massage_handler.can_queue = xQueueCreate(1, sizeof(massage_t));
    done_sem  = xSemaphoreCreateBinary();

    display_init();
    controls_init();
    can_init();

    xTaskCreatePinnedToCore(generic_handler_task, "AC_left", 4096, (void*)&left_ac_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "AC_right", 4096, (void*)&right_ac_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "FAN", 4096, (void*)&fan_speed_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "AIRFLOW", 4096, (void*)&airflow_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "BUTT_left", 4096, (void*)&left_butt_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "BUTT_right", 4096, (void*)&right_butt_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(generic_handler_task, "MASSAGE", 4096, (void*)&massage_handler, 3, NULL, tskNO_AFFINITY);

    // Enable TWAI after all handler tasks are created and have set up their queue sets.
    // On single-core ESP32-C3, higher-priority tasks (handler tasks at prio 3)
    // preempt app_main (prio 1) during xTaskCreate, run until they block on
    // xQueueSelectFromSet, then return here. So by this point all queue sets
    // are ready. Enabling earlier would race: ISR→receive_task→xQueueOverwrite
    // could fill a can_queue before its handler adds it to a queue set, making
    // the item invisible to xQueueSelectFromSet forever.
    can_enable();

    while(!xSemaphoreTake(done_sem, portMAX_DELAY));    //Wait for tasks to complete

    //Uninstall TWAI driver
    can_shutdown();

    //Cleanup
    vSemaphoreDelete(done_sem);
    vQueueDelete(tx_task_queue);
    vQueueDelete(display_queue);
}
