/**
 * 계층 5-2: 앵커 간 상호 거리 측정 + 태그 UDP 전송
 *
 * 태그: UDP STOP/RESUME으로 POLL 제어
 * 앵커: 메인루프에서 SS-TWR 응답 + UDP 레인징 명령 처리 (HTTP 없음)
 * 서버: UDP로 순차 캘리브레이션 제어
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
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
#define SERVER_IP    "192.168.45.199"
#define UDP_PORT     8080
#define CMD_PORT     8082
#define CMD_STOP     0x30
#define CMD_RESUME   0x31
#define CMD_RANGE    0x50  /* 레인징 명령: [0x50, target_hi, target_lo] */

/* ============================================================
 *  TAG1
 * ============================================================ */
#ifdef BUILD_TAG

#if TAG_ID == 1
  #define MY_TAG_ADDR  ADDR_TAG1
  #define MY_TAG_NAME  "tag1"
  #define MY_TAG_DEVID 0x00
  #define TDMA_OFFSET_MS 0
#elif TAG_ID == 2
  #define MY_TAG_ADDR  ADDR_TAG2
  #define MY_TAG_NAME  "tag2"
  #define MY_TAG_DEVID 0x01
  #define TDMA_OFFSET_MS 25   /* 태그2는 25ms 오프셋 */
#else
  #error "TAG_ID must be 1~2"
#endif

static const char *TAG = MY_TAG_NAME;
static uint8_t frame_seq_nb = 0;
static volatile bool s_tag_paused = false;

/* 8-1: DMA 정렬 버퍼 */
static DMA_ATTR uint8_t s_tx_buf[POLL_LEN] __attribute__((aligned(4)));
static DMA_ATTR uint8_t s_rx_buf[RESP_LEN] __attribute__((aligned(4)));

static float s_last_dist[4] = {-1,-1,-1,-1};
static int s_total_cycles = 0;
static int s_all4_ok = 0;

static esp_err_t result_get_handler(httpd_req_t *req) {
    char body[1024];
    int len = snprintf(body, sizeof(body),
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta http-equiv='refresh' content='1'>"
        "<title>UWB Tag1</title>"
        "<style>body{font-family:monospace;font-size:16px;padding:20px;}"
        "table{border-collapse:collapse;}td,th{border:1px solid #555;padding:8px;}</style>"
        "</head><body><h2>Tag1: 4앵커 측정 + UDP</h2>"
        "<table><tr><th>앵커</th><th>거리(mm)</th></tr>"
        "<tr><td>A1</td><td>%.0f</td></tr>"
        "<tr><td>A2</td><td>%.0f</td></tr>"
        "<tr><td>A3</td><td>%.0f</td></tr>"
        "<tr><td>A4</td><td>%.0f</td></tr></table>"
        "<br><b>사이클:</b> %d &nbsp; <b>성공률:</b> %.1f%% &nbsp; <b>%s</b>"
        "</body></html>",
        s_last_dist[0], s_last_dist[1], s_last_dist[2], s_last_dist[3],
        s_total_cycles,
        s_total_cycles > 0 ? 100.0f * s_all4_ok / s_total_cycles : 0.0f,
        s_tag_paused ? "PAUSED" : "RUNNING");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, body, len);
}

typedef struct __attribute__((packed)) {
    uint8_t  device_id;
    uint8_t  pkt_type;
    uint8_t  neo_r, neo_g, neo_b;
    uint8_t  state;
    uint16_t seq;
    float    dist[4];
} udp_pkt_t;

static int s_udp_sock = -1;
static int s_cmd_sock = -1;
static struct sockaddr_in s_server_addr;
static uint8_t s_neo_r = 0, s_neo_g = 0, s_neo_b = 0;

static void udp_init(void) {
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&s_server_addr, 0, sizeof(s_server_addr));
    s_server_addr.sin_family = AF_INET;
    s_server_addr.sin_port = htons(UDP_PORT);
    s_server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
}

static void cmd_udp_init(void) {
    s_cmd_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET, .sin_port = htons(CMD_PORT), .sin_addr.s_addr = INADDR_ANY,
    };
    bind(s_cmd_sock, (struct sockaddr *)&addr, sizeof(addr));
    int flags = fcntl(s_cmd_sock, F_GETFL, 0);
    fcntl(s_cmd_sock, F_SETFL, flags | O_NONBLOCK);
}

static void tag_neopixel_set(uint8_t r, uint8_t g, uint8_t b) {
    s_neo_r = r; s_neo_g = g; s_neo_b = b;
    neopixel_set(r, g, b);
}

static void udp_send_distances(float *dist, uint16_t seq) {
    udp_pkt_t pkt = {
        .device_id = MY_TAG_DEVID, .pkt_type = 0x01,
        .neo_r = s_neo_r, .neo_g = s_neo_g, .neo_b = s_neo_b,
        .state = 0, .seq = seq,
    };
    memcpy(pkt.dist, dist, sizeof(float) * 4);
    sendto(s_udp_sock, &pkt, sizeof(pkt), 0,
           (struct sockaddr *)&s_server_addr, sizeof(s_server_addr));
}

static float tag_range_to(uint16_t anchor_addr) {
    /* TX: DMA 정렬 버퍼 사용 */
    s_tx_buf[0] = FC_LO; s_tx_buf[1] = FC_HI;
    s_tx_buf[2] = frame_seq_nb;
    put_u16le(&s_tx_buf[IDX_PAN], UWB_PAN_ID);
    put_u16le(&s_tx_buf[IDX_DST], anchor_addr);
    put_u16le(&s_tx_buf[IDX_SRC], MY_TAG_ADDR);
    s_tx_buf[IDX_TYPE] = MSG_POLL;

    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(POLL_LEN, s_tx_buf, 0);
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

    /* RX: DMA 정렬 버퍼에 직접 수신 (Zero-Copy) */
    dwt_readrxdata(s_rx_buf, RESP_LEN, 0);
    if (s_rx_buf[IDX_TYPE] != MSG_RESP) return -1.0f;

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
    float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    uint32_t poll_rx_ts = get_u32le(&s_rx_buf[IDX_PAYLOAD + 0]);
    uint32_t resp_tx_ts = get_u32le(&s_rx_buf[IDX_PAYLOAD + 4]);

    int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);
    double tof = ((double)rtd_init - (double)rtd_resp * (1.0 - (double)clockOffsetRatio)) / 2.0;
    if (tof < 0) tof = 0;
    return (float)(tof * DWT_TIME_UNITS * SPEED_OF_LIGHT * 1000.0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Tag1 + UDP 명령 제어 ===");
    ota_mark_valid();

    neopixel_init(NEOPIXEL_PIN);
    tag_neopixel_set(255, 255, 255);

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi 연결 실패");
        tag_neopixel_set(255, 0, 0);
        return;
    }
    ota_server_start(MY_TAG_NAME);

    extern httpd_handle_t s_httpd;
    httpd_uri_t uri = { .uri = "/", .method = HTTP_GET, .handler = result_get_handler };
    httpd_register_uri_handler(s_httpd, &uri);

    int rc = uwb_init(MY_TAG_ADDR);
    if (rc != 0) {
        ESP_LOGE(TAG, "UWB 초기화 실패: %d", rc);
        tag_neopixel_set(255, 0, 0);
        return;
    }

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    udp_init();
    cmd_udp_init();

    tag_neopixel_set(0, 255, 0);
    ESP_LOGI(TAG, "준비 완료. 3초 후 측정 시작...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    tag_neopixel_set(0, 0, 255);

    const uint16_t anchors[] = {ADDR_ANCHOR1, ADDR_ANCHOR2, ADDR_ANCHOR3, ADDR_ANCHOR4};
    const int SLOT_MS = 3;
    const int CYCLE_MS = 50; /* 초당 20회 */

    while (1) {
        /* UDP 명령 확인 */
        uint8_t cmd_buf[4];
        int n = recv(s_cmd_sock, cmd_buf, sizeof(cmd_buf), 0);
        if (n > 0) {
            if (cmd_buf[0] == CMD_STOP) {
                s_tag_paused = true;
                dwt_forcetrxoff();
                tag_neopixel_set(255, 255, 0);
                ESP_LOGI(TAG, "STOP - POLL 중단");
            } else if (cmd_buf[0] == CMD_RESUME) {
                s_tag_paused = false;
                dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                tag_neopixel_set(0, 0, 255);
                ESP_LOGI(TAG, "RESUME - POLL 재개");
            }
        }

        if (s_tag_paused) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int64_t cycle_start = esp_timer_get_time();

        /* TDMA 오프셋: 태그별 시간 분리 */
        if (TDMA_OFFSET_MS > 0) {
            vTaskDelay(pdMS_TO_TICKS(TDMA_OFFSET_MS));
        }

        float dist[4];
        bool cycle_ok = true;

        for (int a = 0; a < 4; a++) {
            int64_t slot_target = cycle_start + TDMA_OFFSET_MS * 1000LL + (int64_t)a * SLOT_MS * 1000;
            int64_t now = esp_timer_get_time();
            if (now < slot_target)
                vTaskDelay(pdMS_TO_TICKS((uint32_t)((slot_target - now) / 1000)));
            dist[a] = tag_range_to(anchors[a]);
            if (dist[a] < 0) cycle_ok = false;
        }

        s_total_cycles++;
        if (cycle_ok) s_all4_ok++;
        memcpy(s_last_dist, dist, sizeof(dist));
        udp_send_distances(dist, (uint16_t)s_total_cycles);

        if (s_total_cycles % 10 == 0) {
            ESP_LOGI(TAG, "A1=%.0f A2=%.0f A3=%.0f A4=%.0f [%d] %.1f%%",
                     dist[0], dist[1], dist[2], dist[3],
                     s_total_cycles, 100.0f * s_all4_ok / s_total_cycles);
        }

        int64_t elapsed_ms = (esp_timer_get_time() - cycle_start) / 1000;
        int64_t remain_ms = CYCLE_MS - elapsed_ms;
        if (remain_ms > 0)
            vTaskDelay(pdMS_TO_TICKS((uint32_t)remain_ms));
    }
}

#endif /* BUILD_TAG */

/* ============================================================
 *  ANCHOR (ANCHOR_ID로 구분)
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

/* 8-1: DMA 정렬 버퍼 */
static DMA_ATTR uint8_t s_tx_buf[RESP_LEN] __attribute__((aligned(4)));
static DMA_ATTR uint8_t s_rx_buf[RESP_LEN] __attribute__((aligned(4)));

static int s_cmd_sock = -1;

static void cmd_udp_init(void) {
    s_cmd_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET, .sin_port = htons(CMD_PORT), .sin_addr.s_addr = INADDR_ANY,
    };
    bind(s_cmd_sock, (struct sockaddr *)&addr, sizeof(addr));
    int flags = fcntl(s_cmd_sock, F_GETFL, 0);
    fcntl(s_cmd_sock, F_SETFL, flags | O_NONBLOCK);
}

/* 서버에 레인징 결과 UDP 전송 */
static void send_range_result(uint16_t target, float dist) {
    struct __attribute__((packed)) {
        uint8_t cmd;
        uint8_t anchor_id;
        uint16_t target;
        float dist;
    } pkt = { CMD_RANGE, ANCHOR_ID, target, dist };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in dest = {
        .sin_family = AF_INET, .sin_port = htons(8081),
    };
    dest.sin_addr.s_addr = inet_addr(SERVER_IP);
    sendto(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&dest, sizeof(dest));
    close(sock);
}

/* SS-TWR 개시자 (메인루프에서 직접 실행) */
static float do_range_to(uint16_t target) {
    float sum = 0;
    int ok = 0;

    for (int i = 0; i < 10; i++) {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

        s_tx_buf[0] = FC_LO; s_tx_buf[1] = FC_HI;
        s_tx_buf[2] = frame_seq_nb++;
        put_u16le(&s_tx_buf[IDX_PAN], UWB_PAN_ID);
        put_u16le(&s_tx_buf[IDX_DST], target);
        put_u16le(&s_tx_buf[IDX_SRC], MY_ADDR);
        s_tx_buf[IDX_TYPE] = MSG_POLL;

        dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(POLL_LEN, s_tx_buf, 0);
        dwt_writetxfctrl(POLL_LEN + 2, 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        uint32_t status;
        while (!((status = dwt_readsysstatuslo()) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status & SYS_STATUS_RXFCG_BIT_MASK) {
            dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);
            dwt_readrxdata(s_rx_buf, RESP_LEN, 0);
            if (s_rx_buf[IDX_TYPE] == MSG_RESP) {
                uint32_t poll_tx = dwt_readtxtimestamplo32();
                uint32_t resp_rx = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
                float clkOff = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
                uint32_t poll_rx = get_u32le(&s_rx_buf[IDX_PAYLOAD + 0]);
                uint32_t resp_tx = get_u32le(&s_rx_buf[IDX_PAYLOAD + 4]);
                int32_t rtd_i = (int32_t)(resp_rx - poll_tx);
                int32_t rtd_r = (int32_t)(resp_tx - poll_rx);
                double tof = ((double)rtd_i - (double)rtd_r * (1.0 - (double)clkOff)) / 2.0;
                if (tof < 0) tof = 0;
                sum += (float)(tof * DWT_TIME_UNITS * SPEED_OF_LIGHT * 1000.0);
                ok++;
            }
        } else {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* UWB 강제 리셋 + 메인루프 RX 설정 복원 */
    dwt_forcetrxoff();
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR |
                         SYS_STATUS_ALL_RX_TO | SYS_STATUS_TXFRS_BIT_MASK);

    return (ok > 0) ? (sum / ok) : -1.0f;
}

/* SS-TWR 응답 (DMA 버퍼 사용) */
static void handle_ss_twr_poll(uint8_t *rx_buf) {
    uint64_t poll_rx_ts = get_rx_timestamp_u64();

    uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
                             (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
    dwt_setdelayedtrxtime(resp_tx_time);
    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    s_tx_buf[0] = FC_LO; s_tx_buf[1] = FC_HI;
    s_tx_buf[2] = frame_seq_nb++;
    put_u16le(&s_tx_buf[IDX_PAN], UWB_PAN_ID);
    uint16_t initiator = (uint16_t)rx_buf[IDX_SRC] | ((uint16_t)rx_buf[IDX_SRC + 1] << 8);
    put_u16le(&s_tx_buf[IDX_DST], initiator);
    put_u16le(&s_tx_buf[IDX_SRC], MY_ADDR);
    s_tx_buf[IDX_TYPE] = MSG_RESP;
    put_u32le(&s_tx_buf[IDX_PAYLOAD + 0], (uint32_t)(poll_rx_ts & 0xFFFFFFFF));
    put_u32le(&s_tx_buf[IDX_PAYLOAD + 4], (uint32_t)(resp_tx_ts & 0xFFFFFFFF));

    dwt_writetxdata(RESP_LEN, s_tx_buf, 0);
    dwt_writetxfctrl(RESP_LEN + 2, 0, 1);
    int ret = dwt_starttx(DWT_START_TX_DELAYED);
    if (ret == DWT_SUCCESS) {
        /* TX 완료 대기 (타임아웃 5ms) */
        int64_t t0 = esp_timer_get_time();
        while (!(dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK)) {
            if (esp_timer_get_time() - t0 > 5000) {
                dwt_forcetrxoff();
                dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK |
                                     SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
                ESP_LOGW(TAG, "TX 타임아웃");
                return;
            }
        };
        dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    } else {
        dwt_forcetrxoff();
        dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK |
                             SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
    }
}

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

    cmd_udp_init();
    ESP_LOGI(TAG, "준비 완료 (addr=0x%04X)", MY_ADDR);
    neopixel_set(0, 255, 0);

    bool anchor_stopped = false;

    while (1) {
        /* 1. UDP 명령 확인 (비블로킹) */
        uint8_t cmd_buf[4];
        int n = recv(s_cmd_sock, cmd_buf, sizeof(cmd_buf), 0);
        if (n > 0) {
            if (cmd_buf[0] == CMD_STOP) {
                anchor_stopped = true;
                dwt_forcetrxoff();
                neopixel_set(255, 255, 0); /* YELLOW: 정지 */
                ESP_LOGI(TAG, "STOP 수신");
            } else if (cmd_buf[0] == CMD_RESUME) {
                anchor_stopped = false;
                neopixel_set(0, 255, 0); /* GREEN: 복귀 */
                ESP_LOGI(TAG, "RESUME 수신");
            } else if (n >= 3 && cmd_buf[0] == CMD_RANGE) {
                uint16_t target = ((uint16_t)cmd_buf[1] << 8) | cmd_buf[2];
                ESP_LOGI(TAG, "레인징 명령 → 0x%04X", target);
                neopixel_set(255, 255, 0);

                float dist = do_range_to(target);
                ESP_LOGI(TAG, "결과: %.0fmm", dist);

                send_range_result(target, dist);
                neopixel_set(0, 255, 0);
                continue;
            }
        }

        /* STOP 상태: UWB 안 건드리고 대기 (WiFi/OTA 처리 가능) */
        if (anchor_stopped) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* 2. UWB RX: 연속 수신 대기 (polling) */
        dwt_setrxtimeout(5000); /* 5ms 타임아웃 */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status;
        while (!((status = dwt_readsysstatuslo()) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)))
        { };

        if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            continue;
        }
        dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

        dwt_readrxdata(s_rx_buf, POLL_LEN, 0);
        uint16_t dst = (uint16_t)s_rx_buf[IDX_DST] | ((uint16_t)s_rx_buf[IDX_DST + 1] << 8);
        if (dst != MY_ADDR) continue;

        if (s_rx_buf[IDX_TYPE] == MSG_POLL) {
            handle_ss_twr_poll(s_rx_buf);
        }
    }
}

#endif /* BUILD_ANCHOR */
