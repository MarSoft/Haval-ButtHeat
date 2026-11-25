/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/* SoftAP firmware-update mode. Adapted from Jeija/esp32-softap-ota (MIT):
 * httpd + streaming esp_ota_write upload handler. The AP is WPA2-protected
 * with a password derived from the factory MAC (stable across boots) so the
 * car-parked attack surface is "someone who read the OLED". */

#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "display.h"
#include "sdkconfig.h"

#include "ota.h"

#define TAG "ButtHeat"

#define OTA_SSID "ButtHeat-OTA"
#define OTA_MAX_UPLOAD (2 * 1024 * 1024)  // > any 1.5 MB-slot image

extern const uint8_t ota_page_html_start[] asm("_binary_ota_page_html_start");
extern const uint8_t ota_page_html_end[] asm("_binary_ota_page_html_end");

static SSD1306_t ota_display;
static char ota_psk[16];  // "BH-" + 8 hex

// Rewrite the bottom OLED line with upload progress (once per whole %).
static void show_progress(httpd_req_t *req, int remaining) {
    static int last_pct = -1;
    int pct = 100 - remaining * 100 / req->content_len;
    if (pct != last_pct && pct >= 0 && pct <= 100) {
        char line[17];
        snprintf(line, sizeof(line), "Upload %3d%%     ", pct);
        ssd1306_display_text(&ota_display, 3, line, 16, false);
        last_pct = pct;
    }
}

static esp_err_t index_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)ota_page_html_start,
                    ota_page_html_end - ota_page_html_start);
    return ESP_OK;
}

static esp_err_t update_post_handler(httpd_req_t *req) {
    if (req->content_len > OTA_MAX_UPLOAD) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image too large");
        return ESP_FAIL;
    }

    char buf[1024];
    esp_ota_handle_t ota_handle;
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part || esp_ota_begin(part, OTA_SIZE_UNKNOWN, &ota_handle) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin failed");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int recv = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (recv == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (recv <= 0 || esp_ota_write(ota_handle, buf, recv) != ESP_OK) {
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }
        remaining -= recv;
        show_progress(req, remaining);
    }

    // esp_ota_end validates the image header + hash of what was written.
    if (esp_ota_end(ota_handle) != ESP_OK ||
        esp_ota_set_boot_partition(part) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid image");
        return ESP_FAIL;
    }

    ssd1306_display_text(&ota_display, 3, "Rebooting...     ", 16, false);
    httpd_resp_sendstr(req, "Update complete, rebooting.\n");
    vTaskDelay(pdMS_TO_TICKS(500));  // let the response reach the browser
    esp_restart();
    return ESP_OK;
}

static const httpd_uri_t index_get = {
    .uri = "/", .method = HTTP_GET, .handler = index_get_handler};
static const httpd_uri_t update_post = {
    .uri = "/update", .method = HTTP_POST, .handler = update_post_handler};

static void softap_init(void) {
    // Wi-Fi requires NVS; nothing else uses it, so a corrupt NVS is just erased.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    snprintf(ota_psk, sizeof(ota_psk), "BH-%02X%02X%02X%02X",
             mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi = {
        .ap = {
            .ssid = OTA_SSID,
            .ssid_len = strlen(OTA_SSID),
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .max_connection = 1,
        },
    };
    strlcpy((char *)wifi.ap.password, ota_psk, sizeof(wifi.ap.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void ota_mode(void) {
    display_hw_init(&ota_display);
    softap_init();

    httpd_handle_t server = NULL;
    httpd_config_t httpd = HTTPD_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(httpd_start(&server, &httpd));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &update_post));

    // NOTE: deliberately NOT committing the image here — OTA mode proves only
    // the OTA path runs. Validity is decided solely by ota_boot_watchdog()
    // after a normal boot.

    const char *ver = esp_app_get_description()->version;
    char l0[17], l2[17];
    snprintf(l0, sizeof(l0), "OTA v%-10.10s", ver);
    snprintf(l2, sizeof(l2), "PW:%.11s", ota_psk);
    ssd1306_display_text(&ota_display, 0, l0, 16, false);
    ssd1306_display_text(&ota_display, 1, "AP:" OTA_SSID " ", 16, false);
    ssd1306_display_text(&ota_display, 2, l2, 16, false);
    ssd1306_display_text(&ota_display, 3, "192.168.4.1       ", 16, false);

    ESP_LOGI(TAG, "OTA mode: SSID %s pass %s http://192.168.4.1", OTA_SSID, ota_psk);
    while (1) vTaskDelay(portMAX_DELAY);
}

static void mark_valid_cb(void *arg) {
    esp_ota_mark_app_valid_cancel_rollback();
    ESP_LOGI(TAG, "Image marked valid");
}

void ota_boot_watchdog(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK ||
        state != ESP_OTA_IMG_PENDING_VERIFY) {
        return;
    }
    const esp_timer_create_args_t args = {
        .name = "ota_valid", .callback = mark_valid_cb,
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_once(timer, 5 * 1000 * 1000));
}
