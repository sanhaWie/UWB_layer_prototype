/**
 * 계층 1-3: DW3000 + NeoPixel + WiFi 연결
 *
 * 1-1: SPI 통신으로 DW3000 DEV_ID 확인
 * 1-2: NeoPixel 동작 확인
 * 1-3: WiFi 연결 중 WHITE, 연결 성공 GREEN
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "dw3000_hw.h"
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "neopixel.h"

#define NEOPIXEL_PIN 8
#define WIFI_SSID    "SK_DA20_2.4G"
#define WIFI_PASS    "GGA48@6587"

static const char *TAG = "LAYER1-3";
static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi 연결 끊김, 재시도...");
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

void app_main(void)
{
    /* 1-1: DW3000 칩 ID 읽기 */
    ESP_LOGI(TAG, "=== 계층 1-3 ===");

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

    if (dev_id == (uint32_t)DWT_DW3000_DEV_ID || dev_id == (uint32_t)DWT_DW3000_PDOA_DEV_ID) {
        ESP_LOGI(TAG, "DW3000 칩 확인 완료!");
    } else {
        ESP_LOGE(TAG, "예상과 다른 칩 ID!");
    }

    /* 1-2: NeoPixel 초기화 */
    rc = neopixel_init(NEOPIXEL_PIN);
    if (rc != ESP_OK) { ESP_LOGE(TAG, "neopixel_init 실패: %d", rc); return; }

    /* 1-3: WiFi 연결 (WHITE 점등) */
    neopixel_set(255, 255, 255);
    ESP_LOGI(TAG, "WiFi 연결 중... (WHITE)");

    s_wifi_eg = xEventGroupCreate();
    wifi_init();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) {
        neopixel_set(0, 255, 0);
        ESP_LOGI(TAG, "WiFi 연결 성공! (GREEN)");
    } else {
        neopixel_set(255, 0, 0);
        ESP_LOGE(TAG, "WiFi 연결 타임아웃 (RED)");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
