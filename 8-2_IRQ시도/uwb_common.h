#ifndef UWB_COMMON_H
#define UWB_COMMON_H

/**
 * DW3000 SS-TWR 공통 헤더
 * Qorvo 공식 예제(ex_06a/06b) 기준 + IRQ 지원
 */

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "dw3000_hw.h"
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000/dw3000_deca_regs.h"

/* UWB 네트워크 주소 */
#define UWB_PAN_ID     0xDECA
#define ADDR_TAG1      0x0001
#define ADDR_TAG2      0x0002
#define ADDR_ANCHOR1   0x0010
#define ADDR_ANCHOR2   0x0020
#define ADDR_ANCHOR3   0x0030
#define ADDR_ANCHOR4   0x0040

/* 프레임 구조 인덱스 */
#define FC_LO       0x41
#define FC_HI       0x88
#define IDX_PAN     3
#define IDX_DST     5
#define IDX_SRC     7
#define IDX_TYPE    9
#define IDX_PAYLOAD 10

#define POLL_LEN    10
#define RESP_LEN    18

/* 메시지 타입 */
#define MSG_POLL 0x01
#define MSG_RESP 0x02
#define MSG_DS_POLL  0x10
#define MSG_DS_RESP  0x11
#define MSG_DS_FINAL 0x12

/* 안테나 지연 */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16362

/* SS-TWR 타이밍 */
#define POLL_TX_TO_RESP_RX_DLY_UUS  1000
#define RESP_RX_TIMEOUT_UUS         2000
#define POLL_RX_TO_RESP_TX_DLY_UUS  1500
#define UUS_TO_DWT_TIME             63898ULL

/* DS-TWR 타이밍 (공식 예제 ex_05a/05b 기준) */
#define DS_POLL_TX_TO_RESP_RX_DLY_UUS  700
#define DS_RESP_RX_TIMEOUT_UUS         300
#define DS_POLL_RX_TO_RESP_TX_DLY_UUS  900
#define DS_RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define DS_FINAL_RX_TIMEOUT_UUS        220
#define DS_RESP_RX_TO_FINAL_TX_DLY_UUS 700

/* DS-TWR FINAL 메시지 길이: 헤더(10) + 3타임스탬프(12) = 22 */
#define DS_FINAL_LEN 22

/* 물리 상수 */
#define SPEED_OF_LIGHT     299702547.0
#define DWT_TIME_UNITS     (1.0 / (499.2e6 * 128.0))

/* DW3000 IRQ 핀 */
#define DW3000_IRQ_PIN 4

/* 헬퍼 함수 */
static inline void put_u32le(uint8_t *b, uint32_t v) {
    b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8); b[2]=(uint8_t)(v>>16); b[3]=(uint8_t)(v>>24);
}

static inline uint32_t get_u32le(const uint8_t *b) {
    return (uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24);
}

static inline void put_u16le(uint8_t *b, uint16_t v) {
    b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8);
}

/* ===== IRQ 기반 대기 ===== */
static SemaphoreHandle_t s_uwb_sem = NULL;

static void IRAM_ATTR uwb_irq_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_uwb_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

static inline void uwb_irq_init(void) {
    s_uwb_sem = xSemaphoreCreateBinary();
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << DW3000_IRQ_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(DW3000_IRQ_PIN, uwb_irq_isr, NULL);
}

/**
 * IRQ 기반 이벤트 대기.
 * DW3000 인터럽트 발생 시 status 반환, 타임아웃 시 0 반환.
 */
static inline uint32_t uwb_wait_event(uint32_t timeout_ms) {
    if (xSemaphoreTake(s_uwb_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return dwt_readsysstatuslo();
    }
    dwt_forcetrxoff();
    return 0;
}

/* 64비트 타임스탬프 읽기 */
static inline uint64_t get_tx_timestamp_u64(void) {
    uint8_t ts[5];
    dwt_readtxtimestamp(ts);
    return ((uint64_t)ts[4]<<32)|((uint64_t)ts[3]<<24)|
           ((uint64_t)ts[2]<<16)|((uint64_t)ts[1]<<8)|ts[0];
}

static inline uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts[5];
    dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
    return ((uint64_t)ts[4]<<32)|((uint64_t)ts[3]<<24)|
           ((uint64_t)ts[2]<<16)|((uint64_t)ts[1]<<8)|ts[0];
}

/* DS-TWR 거리 계산 (공식 예제 공식) */
static inline float ds_twr_calc_dist_mm(uint64_t poll_tx, uint64_t resp_rx, uint64_t final_tx,
                                         uint32_t poll_rx_32, uint32_t resp_tx_32, uint32_t final_rx_32) {
    double Ra = (double)(resp_rx - poll_tx);
    double Rb = (double)((uint32_t)(final_rx_32 - resp_tx_32));
    double Da = (double)(final_tx - resp_rx);
    double Db = (double)((uint32_t)(resp_tx_32 - poll_rx_32));
    double tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);
    if (tof_dtu < 0) tof_dtu = 0;
    return (float)(tof_dtu * DWT_TIME_UNITS * SPEED_OF_LIGHT * 1000.0);
}

/* UWB 초기화 */
static inline int uwb_init(uint16_t my_addr) {
    dw3000_hw_init();
    dw3000_hw_reset();
    vTaskDelay(pdMS_TO_TICKS(20));

    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS)
        return -1;
    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
        return -2;

    dwt_config_t cfg = {
        .chan           = 5,
        .txPreambLength = DWT_PLEN_128,
        .rxPAC          = DWT_PAC8,
        .txCode         = 9,
        .rxCode         = 9,
        .sfdType        = DWT_SFD_DW_8,
        .dataRate       = DWT_BR_6M8,
        .phrMode        = DWT_PHRMODE_STD,
        .phrRate        = DWT_PHRRATE_STD,
        .sfdTO          = 129 + 8 - 8,
        .stsMode        = DWT_STS_MODE_OFF,
        .stsLength      = DWT_STS_LEN_64,
        .pdoaMode       = DWT_PDOA_M0,
    };
    dwt_configure(&cfg);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setpanid(UWB_PAN_ID);
    dwt_setaddress16(my_addr);

    return 0;
}

#endif
