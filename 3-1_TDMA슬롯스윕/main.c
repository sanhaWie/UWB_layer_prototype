/**
 * 계층 3-1: TDMA 4앵커 순차 측위 - 슬롯 타임 스윕
 *
 * 태그: 앵커1→2→3→4 순서로 SS-TWR, 슬롯 타임별 성공률 측정
 * 앵커: ANCHOR_ID(1~4)에 따라 주소/호스트명 자동 설정
 *
 * 결과 확인: http://tag1.local/
 * OTA: http://tag1.local/ota, http://anchor{1~4}.local/ota
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "uwb_common.h"
#include "wifi_ota.h"
#include "neopixel.h"

#define NEOPIXEL_PIN 8

/* ============================================================
 *  TAG1: TDMA 슬롯 타임 스윕 테스트
 * ============================================================ */
#ifdef BUILD_TAG1

static const char *TAG = "TAG1";
static uint8_t frame_seq_nb = 0;

#define LOG_BUF_SIZE 16384
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
        "<meta http-equiv='refresh' content='3'>"
        "<title>UWB 3-1</title>"
        "<style>body{font-family:monospace;font-size:13px;padding:20px;}"
        "pre{background:#111;color:#0f0;padding:15px;border-radius:8px;white-space:pre-wrap;}</style>"
        "</head><body><h2>UWB 3-1: TDMA 슬롯 타임 스윕</h2><pre>";
    const char *footer = "</pre></body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, header, strlen(header));
    httpd_resp_send_chunk(req, s_log_buf, s_log_len);
    httpd_resp_send_chunk(req, footer, strlen(footer));
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* 특정 앵커에 SS-TWR 1회 수행, 소요 시간(us)과 거리(mm) 반환 */
static float tag_range_to(uint16_t anchor_addr, int64_t *elapsed_us) {
    int64_t t0 = esp_timer_get_time();

    uint8_t poll[POLL_LEN];
    poll[0] = FC_LO; poll[1] = FC_HI;
    poll[2] = frame_seq_nb;
    put_u16le(&poll[IDX_PAN], UWB_PAN_ID);
    put_u16le(&poll[IDX_DST], anchor_addr);
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

    *elapsed_us = esp_timer_get_time() - t0;

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
    ESP_LOGI(TAG, "=== 계층 3-1: TDMA 슬롯 타임 스윕 ===");

    neopixel_init(NEOPIXEL_PIN);
    neopixel_set(255, 255, 255);

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi 연결 실패");
        neopixel_set(255, 0, 0);
        return;
    }
    ota_server_start("tag1");

    extern httpd_handle_t s_httpd;
    httpd_uri_t uri = { .uri = "/", .method = HTTP_GET, .handler = result_get_handler };
    httpd_register_uri_handler(s_httpd, &uri);

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
    log_web("10초 후 스윕 시작...");
    for (int s = 10; s > 0; s--) {
        log_web("  %d초...", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    neopixel_set(0, 0, 255);

    const uint16_t anchors[] = {ADDR_ANCHOR1, ADDR_ANCHOR2, ADDR_ANCHOR3, ADDR_ANCHOR4};
    const char *anchor_names[] = {"A1", "A2", "A3", "A4"};
    const int slot_times[] = {3, 5, 7, 10, 15, 20, 30, 50};
    const int n_slots = sizeof(slot_times) / sizeof(slot_times[0]);
    const int n_cycles = 100;

    log_web("");
    log_web("SLOT,앵커,슬롯ms,사이클수,성공률%%,평균us,최소us,최대us,평균거리mm,표준편차mm");

    for (int si = 0; si < n_slots; si++) {
        int slot_ms = slot_times[si];
        log_web("");
        log_web("=== 슬롯 %dms 테스트 (%d사이클) ===", slot_ms, n_cycles);

        /* 앵커별 통계 */
        int a_ok[4] = {0};
        float a_dist_sum[4] = {0};
        float a_dist_sq_sum[4] = {0};
        int64_t a_us_sum[4] = {0};
        int64_t a_us_min[4] = {999999, 999999, 999999, 999999};
        int64_t a_us_max[4] = {0};

        /* 사이클 통계 */
        int cycle_all4_ok = 0;
        int64_t cycle_us_sum = 0;
        int64_t cycle_us_min = 999999;
        int64_t cycle_us_max = 0;

        for (int c = 0; c < n_cycles; c++) {
            int64_t cycle_start = esp_timer_get_time();
            bool all4 = true;

            for (int a = 0; a < 4; a++) {
                /* 슬롯 시작 시간 맞추기 */
                int64_t slot_target = cycle_start + (int64_t)a * slot_ms * 1000;
                int64_t now = esp_timer_get_time();
                if (now < slot_target) {
                    vTaskDelay(pdMS_TO_TICKS((uint32_t)((slot_target - now) / 1000)));
                }

                int64_t elapsed_us = 0;
                float dist = tag_range_to(anchors[a], &elapsed_us);

                if (dist >= 0) {
                    a_ok[a]++;
                    a_dist_sum[a] += dist;
                    a_dist_sq_sum[a] += dist * dist;
                    a_us_sum[a] += elapsed_us;
                    if (elapsed_us < a_us_min[a]) a_us_min[a] = elapsed_us;
                    if (elapsed_us > a_us_max[a]) a_us_max[a] = elapsed_us;
                } else {
                    all4 = false;
                }
            }

            int64_t cycle_elapsed = esp_timer_get_time() - cycle_start;
            if (all4) {
                cycle_all4_ok++;
                cycle_us_sum += cycle_elapsed;
                if (cycle_elapsed < cycle_us_min) cycle_us_min = cycle_elapsed;
                if (cycle_elapsed > cycle_us_max) cycle_us_max = cycle_elapsed;
            }
        }

        /* 앵커별 결과 출력 */
        for (int a = 0; a < 4; a++) {
            if (a_ok[a] > 0) {
                float avg_dist = a_dist_sum[a] / a_ok[a];
                float var = (a_dist_sq_sum[a] / a_ok[a]) - (avg_dist * avg_dist);
                float stddev = (var > 0) ? sqrtf(var) : 0;
                log_web("SLOT,%s,%d,%d,%.1f%%,%lld,%lld,%lld,%.0f,%.1f",
                        anchor_names[a], slot_ms, n_cycles,
                        100.0f * a_ok[a] / n_cycles,
                        a_us_sum[a] / a_ok[a], a_us_min[a], a_us_max[a],
                        avg_dist, stddev);
            } else {
                log_web("SLOT,%s,%d,%d,0%%,0,0,0,0,0", anchor_names[a], slot_ms, n_cycles);
            }
        }

        /* 사이클 결과 출력 */
        if (cycle_all4_ok > 0) {
            log_web("CYCLE,%d,%d,%.1f%%,%lld,%lld,%lld",
                    slot_ms, n_cycles,
                    100.0f * cycle_all4_ok / n_cycles,
                    cycle_us_sum / cycle_all4_ok, cycle_us_min, cycle_us_max);
        } else {
            log_web("CYCLE,%d,%d,0%%,0,0,0", slot_ms, n_cycles);
        }
    }

    neopixel_set(0, 255, 0);
    log_web("");
    log_web("=== 스윕 완료 ===");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif /* BUILD_TAG1 */

/* ============================================================
 *  ANCHOR: 통합 응답자 (ANCHOR_ID로 구분)
 * ============================================================ */
#ifdef BUILD_ANCHOR

#if ANCHOR_ID == 1
  #define MY_ADDR   ADDR_ANCHOR1
  #define MY_NAME   "anchor1"
#elif ANCHOR_ID == 2
  #define MY_ADDR   ADDR_ANCHOR2
  #define MY_NAME   "anchor2"
#elif ANCHOR_ID == 3
  #define MY_ADDR   ADDR_ANCHOR3
  #define MY_NAME   "anchor3"
#elif ANCHOR_ID == 4
  #define MY_ADDR   ADDR_ANCHOR4
  #define MY_NAME   "anchor4"
#else
  #error "ANCHOR_ID must be 1~4"
#endif

static const char *TAG = MY_NAME;
static uint8_t frame_seq_nb = 0;

void app_main(void)
{
    ESP_LOGI(TAG, "=== %s 응답자 + OTA ===", MY_NAME);

    neopixel_init(NEOPIXEL_PIN);
    neopixel_set(255, 255, 255);

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi 연결 실패");
        neopixel_set(255, 0, 0);
        return;
    }
    ota_server_start(MY_NAME);

    int rc = uwb_init(MY_ADDR);
    if (rc != 0) {
        ESP_LOGE(TAG, "UWB 초기화 실패: %d", rc);
        neopixel_set(255, 0, 0);
        return;
    }
    ESP_LOGI(TAG, "준비 완료 (addr=0x%04X)", MY_ADDR);
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
        if (dst != MY_ADDR) continue;

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
        put_u16le(&resp[IDX_SRC], MY_ADDR);
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

#endif /* BUILD_ANCHOR */
