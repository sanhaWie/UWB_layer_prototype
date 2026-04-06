/**
 * 계층 2-1: 태그-앵커 1:1 SS-TWR 거리 측정 + WiFi/OTA
 *
 * 흐름: NeoPixel WHITE(WiFi 연결) → GREEN(준비) → BLUE(태그 측정 중)
 * OTA: http://tag1.local/ota, http://anchor1.local/ota
 *
 * 빌드: pio run -e tag1 / pio run -e anchor1
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uwb_common.h"
#include "wifi_ota.h"
#include "neopixel.h"

#define NEOPIXEL_PIN 8

/* ============================================================
 *  TAG1: SS-TWR 개시자
 * ============================================================ */
#ifdef BUILD_TAG1

static const char *TAG = "TAG1";
static uint8_t frame_seq_nb = 0;

static float tag_range_once(void) {
    uint8_t poll[POLL_LEN];
    poll[0] = FC_LO; poll[1] = FC_HI;
    poll[2] = frame_seq_nb;
    put_u16le(&poll[IDX_PAN], UWB_PAN_ID);
    put_u16le(&poll[IDX_DST], ADDR_ANCHOR1);
    put_u16le(&poll[IDX_SRC], ADDR_TAG1);
    poll[IDX_TYPE] = MSG_POLL;

    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(POLL_LEN, poll, 0);
    dwt_writetxfctrl(POLL_LEN + 2, 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    uint32_t status_reg;
    while (!((status_reg = dwt_readsysstatuslo()) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };

    frame_seq_nb++;

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return -1.0f;
    }

    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    uint8_t resp[RESP_LEN];
    dwt_readrxdata(resp, RESP_LEN, 0);
    if (resp[IDX_TYPE] != MSG_RESP) return -1.0f;

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
    float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

    uint32_t poll_rx_ts = get_u32le(&resp[IDX_PAYLOAD + 0]);
    uint32_t resp_tx_ts = get_u32le(&resp[IDX_PAYLOAD + 4]);

    int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    double tof = ((double)rtd_init - (double)rtd_resp * (1.0 - (double)clockOffsetRatio)) / 2.0;
    if (tof < 0) tof = 0;
    return (float)(tof * DWT_TIME_UNITS * SPEED_OF_LIGHT * 1000.0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Tag1 SS-TWR + OTA ===");

    /* NeoPixel WHITE → WiFi 연결 */
    neopixel_init(NEOPIXEL_PIN);
    neopixel_set(255, 255, 255);

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi 연결 실패");
        neopixel_set(255, 0, 0);
        return;
    }
    ota_server_start("tag1");

    /* UWB 초기화 */
    int rc = uwb_init(ADDR_TAG1);
    if (rc != 0) {
        ESP_LOGE(TAG, "UWB 초기화 실패: %d", rc);
        neopixel_set(255, 0, 0);
        return;
    }
    ESP_LOGI(TAG, "준비 완료");

    /* NeoPixel GREEN → 준비 완료 */
    neopixel_set(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* NeoPixel BLUE → 측정 시작 */
    neopixel_set(0, 0, 255);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    int ok = 0, fail = 0;
    while (1) {
        float dist = tag_range_once();
        if (dist >= 0) {
            ESP_LOGI(TAG, "거리: %.0f mm", dist);
            ok++;
        } else {
            ESP_LOGW(TAG, "측정 실패");
            fail++;
        }
        if ((ok + fail) % 10 == 0 && (ok + fail) > 0) {
            ESP_LOGI(TAG, "통계: 성공=%d 실패=%d (%.1f%%)",
                     ok, fail, 100.0f * ok / (ok + fail));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

#endif /* BUILD_TAG1 */

/* ============================================================
 *  ANCHOR1: SS-TWR 응답자
 * ============================================================ */
#ifdef BUILD_ANCHOR1

static const char *TAG = "ANCHOR1";
static uint8_t frame_seq_nb = 0;

void app_main(void)
{
    ESP_LOGI(TAG, "=== Anchor1 응답자 + OTA ===");

    /* NeoPixel WHITE → WiFi 연결 */
    neopixel_init(NEOPIXEL_PIN);
    neopixel_set(255, 255, 255);

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi 연결 실패");
        neopixel_set(255, 0, 0);
        return;
    }
    ota_server_start("anchor1");

    /* UWB 초기화 */
    int rc = uwb_init(ADDR_ANCHOR1);
    if (rc != 0) {
        ESP_LOGE(TAG, "UWB 초기화 실패: %d", rc);
        neopixel_set(255, 0, 0);
        return;
    }
    ESP_LOGI(TAG, "준비 완료");

    /* NeoPixel GREEN → 준비 완료, POLL 대기 */
    neopixel_set(0, 255, 0);

    while (1) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status_reg;
        while (!((status_reg = dwt_readsysstatuslo()) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
            continue;
        }

        dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

        uint8_t poll[POLL_LEN];
        dwt_readrxdata(poll, POLL_LEN, 0);

        if (poll[IDX_TYPE] != MSG_POLL) continue;
        uint16_t dst = (uint16_t)poll[IDX_DST] | ((uint16_t)poll[IDX_DST + 1] << 8);
        if (dst != ADDR_ANCHOR1) continue;

        uint8_t poll_rx_bytes[5];
        dwt_readrxtimestamp(poll_rx_bytes, DWT_COMPAT_NONE);
        uint64_t poll_rx_ts = ((uint64_t)poll_rx_bytes[4] << 32) |
                              ((uint64_t)poll_rx_bytes[3] << 24) |
                              ((uint64_t)poll_rx_bytes[2] << 16) |
                              ((uint64_t)poll_rx_bytes[1] << 8) |
                              poll_rx_bytes[0];

        uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
                                 (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
        dwt_setdelayedtrxtime(resp_tx_time);

        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        uint8_t resp[RESP_LEN];
        resp[0] = FC_LO; resp[1] = FC_HI;
        resp[2] = frame_seq_nb++;
        put_u16le(&resp[IDX_PAN], UWB_PAN_ID);
        uint16_t initiator = (uint16_t)poll[IDX_SRC] | ((uint16_t)poll[IDX_SRC + 1] << 8);
        put_u16le(&resp[IDX_DST], initiator);
        put_u16le(&resp[IDX_SRC], ADDR_ANCHOR1);
        resp[IDX_TYPE] = MSG_RESP;

        put_u32le(&resp[IDX_PAYLOAD + 0], (uint32_t)(poll_rx_ts & 0xFFFFFFFF));
        put_u32le(&resp[IDX_PAYLOAD + 4], (uint32_t)(resp_tx_ts & 0xFFFFFFFF));

        dwt_writetxdata(RESP_LEN, resp, 0);
        dwt_writetxfctrl(RESP_LEN + 2, 0, 1);
        int ret = dwt_starttx(DWT_START_TX_DELAYED);

        if (ret == DWT_SUCCESS) {
            while (!(dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK)) { };
            dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
        }
    }
}

#endif /* BUILD_ANCHOR1 */
