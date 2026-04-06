/**
 * 계층 2-3: 안테나 딜레이 캘리브레이션
 *
 * 3m 기준 거리에서 RX_ANT_DLY를 스윕하여 최적값 탐색
 * 결과 확인: http://tag1.local/
 * OTA: http://tag1.local/ota, http://anchor1.local/ota
 *
 * 빌드: pio run -e tag1 / pio run -e anchor1
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uwb_common.h"
#include "wifi_ota.h"
#include "neopixel.h"

#define NEOPIXEL_PIN 8

/* ============================================================
 *  TAG1: 안테나 딜레이 캘리브레이션
 * ============================================================ */
#ifdef BUILD_TAG1

static const char *TAG = "TAG1";
static uint8_t frame_seq_nb = 0;

/* 웹 표시용 로그 버퍼 */
#define LOG_BUF_SIZE 8192
static char s_log_buf[LOG_BUF_SIZE];
static int s_log_len = 0;

static void log_web(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char line[256];
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    ESP_LOGI(TAG, "%s", line);
    int len = strlen(line);
    if (s_log_len + len + 2 < LOG_BUF_SIZE) {
        memcpy(&s_log_buf[s_log_len], line, len);
        s_log_buf[s_log_len + len] = '\n';
        s_log_len += len + 1;
        s_log_buf[s_log_len] = '\0';
    }
}

static esp_err_t result_get_handler(httpd_req_t *req) {
    const char *header =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta http-equiv='refresh' content='2'>"
        "<title>UWB 2-3</title>"
        "<style>body{font-family:monospace;font-size:14px;padding:20px;}"
        "pre{background:#111;color:#0f0;padding:15px;border-radius:8px;white-space:pre-wrap;}</style>"
        "</head><body><h2>UWB 2-3: 안테나 딜레이 캘리브레이션</h2><pre>";
    const char *footer = "</pre></body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, header, strlen(header));
    httpd_resp_send_chunk(req, s_log_buf, s_log_len);
    httpd_resp_send_chunk(req, footer, strlen(footer));
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

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

/* delay 값으로 10회 측정, 평균 반환 */
static float measure_with_delay(uint16_t tx_dly, uint16_t rx_dly, int n) {
    dwt_settxantennadelay(tx_dly);
    dwt_setrxantennadelay(rx_dly);
    vTaskDelay(pdMS_TO_TICKS(50));

    float sum = 0;
    int count = 0;
    int attempts = 0;
    while (count < n && attempts < n * 3) {
        float dist = tag_range_once();
        attempts++;
        if (dist >= 0) {
            sum += dist;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return (count > 0) ? (sum / count) : -1.0f;
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== 계층 2-3: 안테나 딜레이 캘리브레이션 ===");

    neopixel_init(NEOPIXEL_PIN);
    neopixel_set(255, 255, 255);

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi 연결 실패");
        neopixel_set(255, 0, 0);
        return;
    }
    ota_server_start("tag1");

    extern httpd_handle_t s_httpd;
    httpd_uri_t result_uri = { .uri = "/", .method = HTTP_GET, .handler = result_get_handler };
    httpd_register_uri_handler(s_httpd, &result_uri);

    int rc = uwb_init(ADDR_TAG1);
    if (rc != 0) {
        ESP_LOGE(TAG, "UWB 초기화 실패: %d", rc);
        neopixel_set(255, 0, 0);
        return;
    }

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    neopixel_set(0, 255, 0);
    log_web("준비 완료! http://tag1.local/ 에서 결과 확인");
    log_web("");
    log_web("3m 위치에 세팅하세요. 15초 후 캘리브레이션 시작...");

    for (int s = 15; s > 0; s--) {
        log_web("  %d초...", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* ── 1단계: 태그 RX 딜레이 거친 스윕 (16300~16500, step=20) ── */
    neopixel_set(0, 0, 255);
    log_web("");
    log_web("=== 1단계: 거친 스윕 (TX고정=16385, RX 16300~16500 step=20) ===");

    int target_mm = 3000;
    float best_err = 99999.0f;
    uint16_t best_rx = RX_ANT_DLY;

    for (uint16_t rx = 16300; rx <= 16500; rx += 20) {
        float avg = measure_with_delay(TX_ANT_DLY, rx, 10);
        if (avg < 0) {
            log_web("  RX=%u: 측정 실패", rx);
            continue;
        }
        float err = avg - (float)target_mm;
        log_web("  RX=%u: 평균=%.0fmm 오차=%.0fmm", rx, avg, err);
        if (fabsf(err) < fabsf(best_err)) {
            best_err = err;
            best_rx = rx;
        }
    }

    log_web("");
    log_web("1단계 최적: RX=%u (오차=%.0fmm)", best_rx, best_err);

    /* ── 2단계: 미세 스윕 (best ±20, step=2) ── */
    log_web("");
    log_web("=== 2단계: 미세 스윕 (RX %u~%u step=2) ===", best_rx - 20, best_rx + 20);

    uint16_t fine_start = (best_rx > 20) ? (best_rx - 20) : 0;
    uint16_t fine_end = best_rx + 20;
    best_err = 99999.0f;

    for (uint16_t rx = fine_start; rx <= fine_end; rx += 2) {
        float avg = measure_with_delay(TX_ANT_DLY, rx, 10);
        if (avg < 0) continue;
        float err = avg - (float)target_mm;
        log_web("  RX=%u: 평균=%.0fmm 오차=%.0fmm", rx, avg, err);
        if (fabsf(err) < fabsf(best_err)) {
            best_err = err;
            best_rx = rx;
        }
    }

    log_web("");
    log_web("========================================");
    log_web("  최종 결과: TX_ANT_DLY=%u, RX_ANT_DLY=%u", TX_ANT_DLY, best_rx);
    log_web("  3m 기준 오차: %.0fmm", best_err);
    log_web("========================================");

    /* ── 3단계: 최적값으로 검증 (1m/2m/3m/4m) ── */
    log_web("");
    log_web("=== 최적값으로 1~4m 검증 ===");
    log_web("최적 딜레이 적용: TX=%u, RX=%u", TX_ANT_DLY, best_rx);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxantennadelay(best_rx);

    const int distances[] = {1000, 2000, 3000, 4000};
    for (int i = 0; i < 4; i++) {
        log_web("");
        log_web(">>> %dm 위치로 이동하세요 (15초 대기)", distances[i] / 1000);
        neopixel_set(255, 255, 0);
        for (int s = 15; s > 0; s--) {
            log_web("  %d초...", s);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        neopixel_set(0, 0, 255);
        float avg = measure_with_delay(TX_ANT_DLY, best_rx, 10);
        float err = (avg >= 0) ? (avg - (float)distances[i]) : 0;
        log_web("  %dm: 평균=%.0fmm 오차=%.0fmm", distances[i] / 1000, avg, err);
    }

    neopixel_set(0, 255, 0);
    log_web("");
    log_web("=== 캘리브레이션 완료 ===");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif /* BUILD_TAG1 */

/* ============================================================
 *  ANCHOR1: SS-TWR 응답자 (변경 없음)
 * ============================================================ */
#ifdef BUILD_ANCHOR1

static const char *TAG = "ANCHOR1";
static uint8_t frame_seq_nb = 0;

void app_main(void)
{
    ESP_LOGI(TAG, "=== Anchor1 응답자 + OTA ===");

    neopixel_init(NEOPIXEL_PIN);
    neopixel_set(255, 255, 255);

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi 연결 실패");
        neopixel_set(255, 0, 0);
        return;
    }
    ota_server_start("anchor1");

    int rc = uwb_init(ADDR_ANCHOR1);
    if (rc != 0) {
        ESP_LOGE(TAG, "UWB 초기화 실패: %d", rc);
        neopixel_set(255, 0, 0);
        return;
    }
    ESP_LOGI(TAG, "준비 완료");
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
