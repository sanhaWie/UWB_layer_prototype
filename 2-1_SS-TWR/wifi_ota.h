#ifndef WIFI_OTA_H
#define WIFI_OTA_H

/**
 * WiFi + mDNS + HTTP OTA 공통 모듈
 * 모든 디바이스(태그/앵커)에서 사용
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "neopixel.h"

#define WIFI_SSID    "SK_DA20_2.4G"
#define WIFI_PASS    "GGA48@6587"

static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI("WIFI", "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

/* WiFi 연결 (블로킹, 성공 시 true) */
static inline bool wifi_connect(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    s_wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

/* OTA POST 핸들러 */
static esp_err_t ota_post_handler(httpd_req_t *req) {
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }
    esp_ota_handle_t handle = 0;
    if (esp_ota_begin(update, OTA_WITH_SEQUENTIAL_WRITES, &handle) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }
    neopixel_set(0, 0, 255); /* BLUE during OTA */

    char buf[1024];
    int received;
    bool header_skipped = false;

    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        if (!header_skipped) {
            char *body = strnstr(buf, "\r\n\r\n", received);
            if (body) {
                body += 4;
                int body_len = received - (body - buf);
                if (body_len > 0) esp_ota_write(handle, body, body_len);
                header_skipped = true;
            }
            continue;
        }
        esp_ota_write(handle, buf, received);
    }

    if (esp_ota_end(handle) != ESP_OK || esp_ota_set_boot_partition(update) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA failed");
        neopixel_set(255, 0, 0);
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "OTA OK! Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* OTA GET 핸들러 */
static esp_err_t ota_get_handler(httpd_req_t *req) {
    const char *html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>OTA Update</title></head><body>"
        "<h2>OTA Update</h2>"
        "<form method='POST' action='/ota' enctype='multipart/form-data'>"
        "<input type='file' name='firmware' accept='.bin'><br><br>"
        "<input type='submit' value='Upload'>"
        "</form></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

/* mDNS + OTA 서버 시작 */
static inline void ota_server_start(const char *hostname) {
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t get = { .uri = "/ota", .method = HTTP_GET, .handler = ota_get_handler };
        httpd_uri_t post = { .uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler };
        httpd_register_uri_handler(server, &get);
        httpd_register_uri_handler(server, &post);
        ESP_LOGI("OTA", "http://%s.local/ota", hostname);
    }
}

#endif
