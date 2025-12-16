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
#include "encoder.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define DATA_PERIOD_MS                  50
#define ITER_DELAY_MS                   1000
#define RX_TASK_PRIO                    8       //Receiving task priority
#define TX_TASK_PRIO                    9       //Sending task priority
#define CTRL_TSK_PRIO                   10      //Control task priority
#define TX_GPIO_NUM                     CONFIG_TX_GPIO_NUM
#define RX_GPIO_NUM                     CONFIG_RX_GPIO_NUM
#define TAG                     "ButtHeat"

typedef int32_t rotary_value_t;

typedef uint8_t ac_temp_t; // TODO
#define AC_TEMP_MAX 0xff
#define AC_TEMP_MIN 0

typedef uint8_t butt_temp_t;

typedef struct {
    QueueHandle_t control_queue;
    QueueHandle_t can_queue;
    bool leftside;
} handler_data_t;

typedef enum {
    DU_BUTTHEAT,
    DU_AC,
    DU_SEAT_MEMORY,
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

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NO_ACK);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = {
    .acceptance_code = CAN_ID_HEATER_STATUS,
    .acceptance_mask = 0xfffff800,  // only standard 11-bit ID matters
    .single_filter = true,
};

static handler_data_t left_ac_handler = {
    .leftside = true,
};
static handler_data_t right_ac_handler = {
    .leftside = false,
};
static handler_data_t left_butt_handler = {
    .leftside = true,
};
static handler_data_t right_butt_handler = {
    .leftside = false,
};

static QueueHandle_t tx_task_queue;
static QueueHandle_t display_queue;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t stop_data_sem;
static SemaphoreHandle_t spinner_sem;
static SemaphoreHandle_t done_sem;

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");

    while (1) {
        twai_message_t rx_msg;
        esp_err_t err;
        if((err = twai_receive(&rx_msg, pdMS_TO_TICKS(500))) != ESP_OK) {
            ESP_LOGW(TAG, "TWAI recv error: %s", esp_err_to_name(err));
            taskYIELD();
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

        xSemaphoreGive(spinner_sem); // mark that we received valid data

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

static void display_task(void *arg) {
    SSD1306_t dev;
    ESP_LOGI(TAG, "Disp task start");
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, -1);
    ssd1306_init(&dev, 128, 32);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);

    bool active = 0;
    bool prev_active = 1;
    bool scroll_left = 1;
    ac_temp_t ac_left_temp, ac_right_temp;
    butt_temp_t butt_left_temp, butt_right_temp;

    while(1) {
        if(active) {
            ESP_LOGI(TAG, "Disp loop active");
            // show status - for now only heaters
            _ssd1306_circle(&dev, 16, 16, 16, OLED_DRAW_UPPER_LEFT, false);
            for(int i=0; i<butt_left_temp; i++) {
                _ssd1306_disc(&dev, 16, 26-(i*6), 4, OLED_DRAW_ALL, false);
            }
        } else if(prev_active != active) {
            ESP_LOGI(TAG, "Disp loop pasv");
            char *banner = "Haval Dargo    ";
            ssd1306_display_text(&dev, 1, banner, strlen(banner), false);
            char *msg = " No Signal ";
            ssd1306_display_text(&dev, 3, msg, strlen(msg), false);
            ssd1306_hardware_scroll(&dev, SCROLL_LEFT);
        } else {
            if(scroll_left) {
                ssd1306_hardware_scroll(&dev, SCROLL_LEFT);
            } else {
                ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);
            }
            scroll_left = !scroll_left;
        }

        prev_active = active;

        ESP_LOGI(TAG, "Recving");
        data_update_t msg;
        if(xQueueReceive(display_queue, &msg, pdMS_TO_TICKS(500)) != errQUEUE_EMPTY) {
            switch(msg.kind) {
                case DU_AC:
                    break; // TODO
                case DU_BUTTHEAT:
                    active = true;
                    if(msg.leftside) {
                        butt_left_temp = msg.butt_temp;
                    } else {
                        butt_right_temp = msg.butt_temp;
                    }
                    break;
                // TODO case for bus conn lost
                default:
                    ESP_LOGE(TAG, "Undef msg %d", msg.kind);
                    break;
            }
        }
        ESP_LOGI(TAG, "Recv end");
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
                xQueueOverwrite((leftside ? left_ac_handler : right_ac_handler).control_queue, &evt.diff);
                break;
            case RE_ET_BTN_CLICKED:
                xQueueOverwrite((leftside ? left_butt_handler : right_butt_handler).control_queue, &dummy);
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
void ac_temp_task(void *ctx) {
    handler_data_t *self = (handler_data_t*)ctx;

    ac_temp_t value = 0;  // TODO
    rotary_value_t rotdiff;
    ac_temp_t canval;
    data_update_t evt;

    evt.kind = DU_AC;
    evt.leftside = self->leftside;

    QueueSetHandle_t qset = xQueueCreateSet(2);
    xQueueAddToSet(self->control_queue, qset);
    xQueueAddToSet(self->can_queue, qset);

    while(1) {
        // wait for event on any of the queues
        xQueueSelectFromSet(qset, portMAX_DELAY);
        if(xQueueReceive(self->can_queue, &canval, 0)) {
            ESP_LOGI(TAG, "AC task %d: got CAN event %d", self->leftside, canval);
            if(value == canval) {
                continue; // nothing changed, ignore this event
            }
            value = canval;
            // TODO emit to display
        }
        int timeout = 0;
        while(xQueueReceive(self->control_queue, &rotdiff, timeout)) {
            ESP_LOGI(TAG, "AC task %d: got encoder event %d", self->leftside, rotdiff);
            value = MAX(AC_TEMP_MIN, MIN(AC_TEMP_MAX, value + rotdiff));

            evt.ac_temp = value;
            // TODO emit to display
            // and emit to CAN
            xQueueSend(tx_task_queue, &evt, portMAX_DELAY);

            // Now listen only on control queue for TX_SETTLE
            if(!timeout) {
                timeout = pdMS_TO_TICKS(CONFIG_TX_SETTLE_TIMEOUT_MS);
            }
        }
    }
}

// Uniform handler
void buttheat_task(void *ctx) {
    handler_data_t *self = (handler_data_t*)ctx;

    butt_temp_t value = 0;

    while(1) {
        uint8_t dummy;
        if(!xQueueReceive(self->control_queue, &dummy, portMAX_DELAY)) {
            continue;
        }

        // got button press!
        ESP_LOGI(TAG, "BUTT task %d: got click", self->leftside);
        if(value == 0) {
            value = 3;
        } else {
            value--;
        }
        // emit new value to CAN and to display...
        // TODO
    }
}

void app_main(void)
{
    //Create semaphores and tasks
    tx_task_queue = xQueueCreate(8, sizeof(data_update_t));
    display_queue = xQueueCreate(1, sizeof(data_update_t));
    left_ac_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    left_ac_handler.can_queue = xQueueCreate(1, sizeof(ac_temp_t));
    right_ac_handler.control_queue = xQueueCreate(8, sizeof(rotary_value_t));
    right_ac_handler.can_queue = xQueueCreate(1, sizeof(ac_temp_t));
    left_butt_handler.control_queue = xQueueCreate(8, sizeof(uint8_t));
    left_butt_handler.can_queue = xQueueCreate(1, sizeof(butt_temp_t));
    right_butt_handler.control_queue = xQueueCreate(8, sizeof(uint8_t));
    right_butt_handler.can_queue = xQueueCreate(1, sizeof(butt_temp_t));
    ctrl_task_sem = xSemaphoreCreateBinary();
    stop_data_sem  = xSemaphoreCreateBinary();
    spinner_sem  = xSemaphoreCreateBinary();
    done_sem  = xSemaphoreCreateBinary();

    xSemaphoreTake(spinner_sem, 0);  // mark that we are loading
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(display_task, "DISP", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(ac_temp_task, "AC_left", 4096, (void*)&left_ac_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(ac_temp_task, "AC_right", 4096, (void*)&right_ac_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(buttheat_task, "BUTT_left", 4096, (void*)&left_butt_handler, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(buttheat_task, "BUTT_right", 4096, (void*)&right_butt_handler, 3, NULL, tskNO_AFFINITY);

    xSemaphoreGive(ctrl_task_sem);              //Start Control task
    xSemaphoreTake(done_sem, portMAX_DELAY);    //Wait for tasks to complete

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(stop_data_sem);
    vSemaphoreDelete(spinner_sem);
    vSemaphoreDelete(done_sem);
    vQueueDelete(tx_task_queue);
    vQueueDelete(display_queue);
}
