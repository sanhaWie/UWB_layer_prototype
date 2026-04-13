#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include "esp_err.h"
#include <stdint.h>

esp_err_t neopixel_init(int gpio_num);
esp_err_t neopixel_set(uint8_t r, uint8_t g, uint8_t b);

#endif
