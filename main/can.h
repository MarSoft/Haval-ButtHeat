/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* CAN bus I/O over TWAI, mirroring the KiCad "can" sheet. */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// data_update_t queue consumed by the TX task. Producers: handler tasks,
// seat_memory_apply(), disable_twozone_ac().
extern QueueHandle_t tx_task_queue;

void can_init(void);     // queues + node + filters + RX/TX tasks (node left disabled)
void can_enable(void);   // start bus traffic — only after handler tasks built their queue sets
void can_shutdown(void); // disable + delete node (teardown path)

// TX request helpers used by controls.c
void seat_memory_apply(int slot);
void disable_twozone_ac(void);
