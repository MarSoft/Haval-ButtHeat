/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* SoftAP firmware-update mode ("mcu" OTA role). Entered from app_main when
 * the left encoder button is held at power-on; takes over the display and
 * never returns (exit = power cycle). Upload protocol: join the AP, browse
 * to http://192.168.4.1, POST buttheat.bin to /update. */

#pragma once

// Enter OTA mode: SoftAP + HTTP upload server. Never returns.
void ota_mode(void) __attribute__((noreturn));

// Rollback support: if the running image is pending verification, mark it
// valid after it survived 5 s of normal operation (one-shot esp_timer).
void ota_boot_watchdog(void);
