/**
 * 계층 1-4: DW3000 + NeoPixel + WiFi + OTA
 *
 * 1-1: SPI 통신으로 DW3000 DEV_ID 확인
 * 1-2: NeoPixel 동작 확인
 * 1-3: WiFi 연결 중 WHITE, 연결 성공 GREEN
 * 1-4: mDNS + HTTP OTA 서버 (http://tag1.local/ota)
 */
#include <stdio.h>
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
#include "dw3000_hw.h"
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "neopixel.h"

#define NEOPIXEL_PIN 8
#define WIFI_SSID    "SK_DA20_2.4G"
#define WIFI_PASS    "GGA48@6587"
#define HOSTNAME     "tag1"

static const char *TAG = "LAYER1-4";
static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT BIT0

/* ---- WiFi ---- */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi 끊김, 재시도...");
        neopixel_set(255, 255, 255);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
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
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ---- OTA HTTP handler ---- */

static const char ota_html[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>OTA - " HOSTNAME "</title></head><body>"
    "<h2>" HOSTNAME " OTA Update</h2>"
    "<form method='POST' action='/ota' enctype='multipart/form-data'>"
    "<input type='file' name='firmware' accept='.bin'><br><br>"
    "<input type='submit' value='Upload'>"
    "</form></body></html>";

static esp_err_t ota_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, ota_html, sizeof(ota_html) - 1);
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_err_t err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin 실패: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    neopixel_set(0, 0, 255); /* BLUE during OTA */
    ESP_LOGI(TAG, "OTA 시작 (size=%d)", req->content_len);

    char buf[1024];
    int received;
    int total = 0;
    bool header_skipped = false;

    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        /* multipart/form-data: skip header until \r\n\r\n */
        if (!header_skipped) {
            char *body = strnstr(buf, "\r\n\r\n", received);
            if (body) {
                body += 4;
                int body_len = received - (body - buf);
                if (body_len > 0) {
                    esp_ota_write(ota_handle, body, body_len);
                    total += body_len;
                }
                header_skipped = true;
            }
            continue;
        }
        esp_ota_write(ota_handle, buf, received);
        total += received;
    }

    /* Remove trailing multipart boundary from last chunk */
    ESP_LOGI(TAG, "OTA 수신 완료: %d bytes", total);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end 실패: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        neopixel_set(255, 0, 0);
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition 실패: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot failed");
        neopixel_set(255, 0, 0);
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OTA OK! Rebooting...");
    ESP_LOGI(TAG, "OTA 성공, 재부팅...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static void start_ota_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 서버 시작 실패");
        return;
    }

    httpd_uri_t ota_get = {
        .uri = "/ota", .method = HTTP_GET,
        .handler = ota_get_handler,
    };
    httpd_uri_t ota_post = {
        .uri = "/ota", .method = HTTP_POST,
        .handler = ota_post_handler,
    };
    httpd_register_uri_handler(server, &ota_get);
    httpd_register_uri_handler(server, &ota_post);
    ESP_LOGI(TAG, "OTA 서버 시작: http://%s.local/ota", HOSTNAME);
}

/* ---- mDNS ---- */
static void start_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(HOSTNAME));
    ESP_LOGI(TAG, "mDNS 호스트명: %s.local", HOSTNAME);
}

/* ---- app_main ---- */
void app_main(void)
{
    ESP_LOGI(TAG, "=== 계층 1-4 ===");

    /* 1-1: DW3000 */
    int rc = dw3000_hw_init();
    if (rc != 0) { ESP_LOGE(TAG, "dw3000_hw_init 실패: %d", rc); return; }
    dw3000_hw_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    rc = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
    if (rc != DWT_SUCCESS) { ESP_LOGE(TAG, "dwt_probe 실패: %d", rc); return; }

    rc = dwt_initialise(DWT_DW_INIT);
    if (rc != DWT_SUCCESS) { ESP_LOGE(TAG, "dwt_initialise 실패: %d", rc); return; }

    uint32_t dev_id = dwt_readdevid();
    ESP_LOGI(TAG, "DEV_ID = 0x%08lX", (unsigned long)dev_id);

    /* 1-2: NeoPixel */
    rc = neopixel_init(NEOPIXEL_PIN);
    if (rc != ESP_OK) { ESP_LOGE(TAG, "neopixel_init 실패: %d", rc); return; }

    /* 1-3: WiFi */
    neopixel_set(255, 255, 255);
    ESP_LOGI(TAG, "WiFi 연결 중... (WHITE)");
    s_wifi_eg = xEventGroupCreate();
    wifi_init();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        neopixel_set(255, 0, 0);
        ESP_LOGE(TAG, "WiFi 타임아웃");
        return;
    }
    neopixel_set(0, 255, 0);
    ESP_LOGI(TAG, "WiFi 연결 성공! (GREEN)");

    /* 1-4: mDNS + OTA */
    start_mdns();
    start_ota_server();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
