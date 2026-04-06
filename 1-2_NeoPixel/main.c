/**
 * 계층 1-2: DW3000 칩 ID 읽기 + NeoPixel 동작 확인
 *
 * 1-1: SPI 통신으로 DW3000 DEV_ID 확인
 * 1-2: NeoPixel WHITE → GREEN → BLUE 1초 간격 점등
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dw3000_hw.h"
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "neopixel.h"

#define NEOPIXEL_PIN 8

static const char *TAG = "LAYER1-2";

void app_main(void)
{
    ESP_LOGI(TAG, "=== 계층 1-2 ===");

    int rc = dw3000_hw_init();
    if (rc != 0) { ESP_LOGE(TAG, "dw3000_hw_init 실패: %d", rc); return; }
    ESP_LOGI(TAG, "HW 초기화 완료");

    dw3000_hw_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    rc = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
    if (rc != DWT_SUCCESS) { ESP_LOGE(TAG, "dwt_probe 실패: %d", rc); return; }
    ESP_LOGI(TAG, "dwt_probe 성공");

    rc = dwt_initialise(DWT_DW_INIT);
    if (rc != DWT_SUCCESS) { ESP_LOGE(TAG, "dwt_initialise 실패: %d", rc); return; }
    ESP_LOGI(TAG, "dwt_initialise 성공");

    uint32_t dev_id = dwt_readdevid();
    ESP_LOGI(TAG, "=============================");
    ESP_LOGI(TAG, "  DEV_ID = 0x%08lX", (unsigned long)dev_id);
    ESP_LOGI(TAG, "=============================");

    if (dev_id == (uint32_t)DWT_DW3000_DEV_ID || dev_id == (uint32_t)DWT_DW3000_PDOA_DEV_ID) {
        ESP_LOGI(TAG, "DW3000 칩 확인 완료!");
    } else {
        ESP_LOGE(TAG, "예상과 다른 칩 ID! SPI 배선을 점검하세요.");
    }

    /* 1-2: NeoPixel 초기화 */
    rc = neopixel_init(NEOPIXEL_PIN);
    if (rc != ESP_OK) { ESP_LOGE(TAG, "neopixel_init 실패: %d", rc); return; }

    /* WHITE → GREEN → BLUE 반복 */
    const uint8_t colors[][3] = {
        {255, 255, 255},
        {0,   255, 0},
        {0,   0,   255},
    };
    int idx = 0;
    const char *names[] = {"WHITE", "GREEN", "BLUE"};

    while (1) {
        ESP_LOGI(TAG, "NeoPixel: %s", names[idx]);
        neopixel_set(colors[idx][0], colors[idx][1], colors[idx][2]);
        idx = (idx + 1) % 3;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
