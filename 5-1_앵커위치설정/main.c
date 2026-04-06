/**
 * 계층 4-1: 태그 → 서버 UDP 전송
 *
 * TDMA 4앵커 측정 + UDP 바이너리 패킷 전송 (포트 8080)
 * 패킷: [device_id(1), pkt_type(1), reserved(2), dist1~4(float×4)] = 20바이트
 *
 * 결과: http://tag1.local/
 * OTA: http://tag1.local/ota, http://anchor{1~4}.local/ota
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "uwb_common.h"
#include "wifi_ota.h"
#include "neopixel.h"

#define NEOPIXEL_PIN 8

/* ============================================================
 *  TAG1: 4앵커 연속 측정
 * ============================================================ */
#ifdef BUILD_TAG1

static const char *TAG = "TAG1";
static uint8_t frame_seq_nb = 0;

/* 웹 표시용 최근 데이터 */
static float s_last_dist[4] = {-1,-1,-1,-1};
static int s_total_cycles = 0;
static int s_all4_ok = 0;

static esp_err_t result_get_handler(httpd_req_t *req) {
    char body[1024];
    int len = snprintf(body, sizeof(body),
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta http-equiv='refresh' content='1'>"
        "<title>UWB 4-1</title>"
        "<style>body{font-family:monospace;font-size:16px;padding:20px;}"
        ".ok{color:#0f0;}.fail{color:#f00;}"
        "table{border-collapse:collapse;}td,th{border:1px solid #555;padding:8px;}</style>"
        "</head><body><h2>3-2: 4앵커 연속 측정 (500ms 사이클)</h2>"
        "<table><tr><th>앵커</th><th>거리(mm)</th></tr>"
        "<tr><td>A1</td><td>%.0f</td></tr>"
        "<tr><td>A2</td><td>%.0f</td></tr>"
        "<tr><td>A3</td><td>%.0f</td></tr>"
        "<tr><td>A4</td><td>%.0f</td></tr></table>"
        "<br><b>사이클:</b> %d &nbsp; <b>성공률:</b> %.1f%%"
        "</body></html>",
        s_last_dist[0], s_last_dist[1], s_last_dist[2], s_last_dist[3],
        s_total_cycles,
        s_total_cycles > 0 ? 100.0f * s_all4_ok / s_total_cycles : 0.0f);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, body, len);
}

#define SERVER_IP   "192.168.45.199"
#define UDP_PORT    8080
#define DEV_TAG1    0x00
#define PKT_DIST    0x01

/* UDP 패킷 구조 (24바이트) */
typedef struct __attribute__((packed)) {
    uint8_t  device_id;
    uint8_t  pkt_type;
    uint8_t  neo_r, neo_g, neo_b;
    uint8_t  state;
    uint16_t seq;
    float    dist[4];
} udp_pkt_t;

static int s_udp_sock = -1;
static struct sockaddr_in s_server_addr;

static void udp_init(void) {
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&s_server_addr, 0, sizeof(s_server_addr));
    s_server_addr.sin_family = AF_INET;
    s_server_addr.sin_port = htons(UDP_PORT);
    s_server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
}

static uint8_t s_neo_r = 0, s_neo_g = 0, s_neo_b = 0;

static void udp_send_distances(float *dist, uint16_t seq) {
    udp_pkt_t pkt = {
        .device_id = DEV_TAG1,
        .pkt_type = PKT_DIST,
        .neo_r = s_neo_r, .neo_g = s_neo_g, .neo_b = s_neo_b,
        .state = 0,
        .seq = seq,
    };
    memcpy(pkt.dist, dist, sizeof(float) * 4);
    sendto(s_udp_sock, &pkt, sizeof(pkt), 0,
           (struct sockaddr *)&s_server_addr, sizeof(s_server_addr));
}

/* NeoPixel 설정 + 상태 저장 */
static void tag_neopixel_set(uint8_t r, uint8_t g, uint8_t b) {
    s_neo_r = r; s_neo_g = g; s_neo_b = b;
    neopixel_set(r, g, b);
}

static float tag_range_to(uint16_t anchor_addr) {
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
    ESP_LOGI(TAG, "=== 계층 4-1: 4앵커 + UDP 전송 ===");
    ota_mark_valid();

    neopixel_init(NEOPIXEL_PIN);
    tag_neopixel_set(255, 255, 255);

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi 연결 실패");
        tag_neopixel_set(255, 0, 0);
        return;
    }
    ota_server_start("tag1");

    extern httpd_handle_t s_httpd;
    httpd_uri_t uri = { .uri = "/", .method = HTTP_GET, .handler = result_get_handler };
    httpd_register_uri_handler(s_httpd, &uri);

    int rc = uwb_init(ADDR_TAG1);
    if (rc != 0) {
        ESP_LOGE(TAG, "UWB 초기화 실패: %d", rc);
        tag_neopixel_set(255, 0, 0);
        return;
    }

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    udp_init();
    ESP_LOGI(TAG, "UDP → %s:%d", SERVER_IP, UDP_PORT);

    tag_neopixel_set(0, 255, 0);
    ESP_LOGI(TAG, "준비 완료. 3초 후 측정 시작...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    tag_neopixel_set(0, 0, 255);

    const uint16_t anchors[] = {ADDR_ANCHOR1, ADDR_ANCHOR2, ADDR_ANCHOR3, ADDR_ANCHOR4};
    const int SLOT_MS = 3;
    const int CYCLE_MS = 500;

    s_total_cycles = 0;
    s_all4_ok = 0;

    while (1) {
        int64_t cycle_start = esp_timer_get_time();

        float dist[4];
        bool cycle_ok = true;

        for (int a = 0; a < 4; a++) {
            /* 슬롯 타이밍 맞추기 */
            int64_t slot_target = cycle_start + (int64_t)a * SLOT_MS * 1000;
            int64_t now = esp_timer_get_time();
            if (now < slot_target) {
                vTaskDelay(pdMS_TO_TICKS((uint32_t)((slot_target - now) / 1000)));
            }

            dist[a] = tag_range_to(anchors[a]);
            if (dist[a] < 0) cycle_ok = false;
        }

        s_total_cycles++;
        if (cycle_ok) s_all4_ok++;
        memcpy(s_last_dist, dist, sizeof(dist));

        /* UDP 전송 (reserved = 시퀀스 번호) */
        udp_send_distances(dist, (uint16_t)s_total_cycles);

        /* 4개 거리값 한 줄 출력 */
        ESP_LOGI(TAG, "A1=%.0f A2=%.0f A3=%.0f A4=%.0f %s",
                 dist[0], dist[1], dist[2], dist[3],
                 cycle_ok ? "OK" : "FAIL");

        /* 10사이클마다 통계 */
        if (s_total_cycles % 10 == 0) {
            ESP_LOGI(TAG, "  [%d사이클] 성공률=%.1f%%",
                     s_total_cycles, 100.0f * s_all4_ok / s_total_cycles);
        }

        /* 사이클 나머지 대기 (500ms) */
        int64_t elapsed_ms = (esp_timer_get_time() - cycle_start) / 1000;
        int64_t remain_ms = CYCLE_MS - elapsed_ms;
        if (remain_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)remain_ms));
        }
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
    ota_mark_valid();

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
