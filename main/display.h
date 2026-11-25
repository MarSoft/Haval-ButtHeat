/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* SSD1306 rendering, mirroring the KiCad "display" sheet. */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// data_update_t queue consumed by the display task.
extern QueueHandle_t display_queue;
#include "ssd1306.h"

// Bring up the SSD1306 panel (I2C + init + clear). Used by display_init()
// and by OTA mode, which renders directly without the display task.
void display_hw_init(SSD1306_t *dev);

void display_init(void); // queue + display task
