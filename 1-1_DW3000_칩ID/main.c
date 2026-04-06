/**
 * 계층 1-1: DW3000 칩 ID 읽기
 *
 * SPI 통신으로 DW3000 DEV_ID 레지스터를 읽어 시리얼 출력.
 * 성공 기준: DEV_ID=0xDECA0302 또는 0xDECA0312
 * WiFi/OTA 없음.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dw3000_hw.h"
#include "deca_device_api.h"
#include "deca_probe_interface.h"

static const char *TAG = "LAYER1-1";

void app_main(void)
{
    ESP_LOGI(TAG, "=== 계층 1-1: DW3000 칩 ID 읽기 ===");

    /* HW 초기화 (GPIO + SPI) */
    int rc = dw3000_hw_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "dw3000_hw_init 실패: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "HW 초기화 완료");

    /* 하드웨어 리셋 */
    dw3000_hw_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 드라이버 probe */
    rc = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
    if (rc != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_probe 실패: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "dwt_probe 성공");

    /* 드라이버 초기화 */
    rc = dwt_initialise(DWT_DW_INIT);
    if (rc != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_initialise 실패: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "dwt_initialise 성공");

    /* 칩 ID 읽기 */
    uint32_t dev_id = dwt_readdevid();
    ESP_LOGI(TAG, "=============================");
    ESP_LOGI(TAG, "  DEV_ID = 0x%08lX", (unsigned long)dev_id);
    ESP_LOGI(TAG, "=============================");

    if (dev_id == (uint32_t)DWT_DW3000_DEV_ID || dev_id == (uint32_t)DWT_DW3000_PDOA_DEV_ID) {
        ESP_LOGI(TAG, "DW3000 칩 확인 완료!");
    } else {
        ESP_LOGE(TAG, "예상과 다른 칩 ID! SPI 배선을 점검하세요.");
    }

    /* 무한 대기 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
