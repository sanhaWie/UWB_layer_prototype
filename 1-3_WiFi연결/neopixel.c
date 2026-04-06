#include "neopixel.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_check.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "NEOPIXEL";

/* WS2812 timing (ns): T0H=400, T0L=850, T1H=800, T1L=450, reset>=50us */
#define WS2812_T0H_NS 400
#define WS2812_T0L_NS 850
#define WS2812_T1H_NS 800
#define WS2812_T1L_NS 450
#define WS2812_RESET_US 50

static rmt_channel_handle_t s_channel;
static rmt_encoder_handle_t s_encoder;

/* bytes encoder wraps a built-in bytes_encoder with reset code */
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                            const void *data, size_t data_size,
                            rmt_encode_state_t *ret_state)
{
    ws2812_encoder_t *ws = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws->state) {
    case 0: /* encode GRB bytes */
        encoded_symbols += ws->bytes_encoder->encode(ws->bytes_encoder, channel, data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = (rmt_encode_state_t)RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
        }
        /* fall through */
    case 1: /* send reset code */
        encoded_symbols += ws->copy_encoder->encode(ws->copy_encoder, channel, &ws->reset_code, sizeof(ws->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws->state = 0;
            *ret_state = RMT_ENCODING_COMPLETE;
        } else {
            *ret_state = (rmt_encode_state_t)RMT_ENCODING_MEM_FULL;
        }
        return encoded_symbols;
    }
    return encoded_symbols;
}

static esp_err_t ws2812_reset(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *ws = __containerof(encoder, ws2812_encoder_t, base);
    ws->bytes_encoder->reset(ws->bytes_encoder);
    ws->copy_encoder->reset(ws->copy_encoder);
    ws->state = 0;
    return ESP_OK;
}

static esp_err_t ws2812_del(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *ws = __containerof(encoder, ws2812_encoder_t, base);
    ws->bytes_encoder->del(ws->bytes_encoder);
    ws->copy_encoder->del(ws->copy_encoder);
    free(ws);
    return ESP_OK;
}

esp_err_t neopixel_init(int gpio_num)
{
    /* RMT TX channel */
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, /* 10MHz = 100ns per tick */
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &s_channel), TAG, "tx channel");

    /* WS2812 encoder */
    ws2812_encoder_t *ws = calloc(1, sizeof(ws2812_encoder_t));
    if (!ws) return ESP_ERR_NO_MEM;

    ws->base.encode = ws2812_encode;
    ws->base.reset = ws2812_reset;
    ws->base.del = ws2812_del;

    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = { .duration0 = WS2812_T0H_NS / 100, .level0 = 1,
                   .duration1 = WS2812_T0L_NS / 100, .level1 = 0 },
        .bit1 = { .duration0 = WS2812_T1H_NS / 100, .level0 = 1,
                   .duration1 = WS2812_T1L_NS / 100, .level1 = 0 },
        .flags.msb_first = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&bytes_cfg, &ws->bytes_encoder), TAG, "bytes enc");

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_cfg, &ws->copy_encoder), TAG, "copy enc");

    ws->reset_code = (rmt_symbol_word_t){
        .duration0 = WS2812_RESET_US * 10, /* 50us in 100ns ticks */
        .level0 = 0,
        .duration1 = 0,
        .level1 = 0,
    };

    s_encoder = &ws->base;

    ESP_RETURN_ON_ERROR(rmt_enable(s_channel), TAG, "enable");
    ESP_LOGI(TAG, "NeoPixel 초기화 완료 (GPIO %d)", gpio_num);
    return ESP_OK;
}

esp_err_t neopixel_set(uint8_t r, uint8_t g, uint8_t b)
{
    /* WS2812 expects GRB order */
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    return rmt_transmit(s_channel, s_encoder, grb, sizeof(grb), &tx_config);
}
