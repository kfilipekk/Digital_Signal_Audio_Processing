#pragma once
#include "driver/i2s_std.h"
#include "hal/gpio_types.h"

#define I2S_SCK_IO      GPIO_NUM_0
#define I2S_WS_IO       GPIO_NUM_1
#define I2S_SD_IO       GPIO_NUM_2
#define I2S_SAMPLE_RATE 16000
#define I2S_BUFFER_SIZE 256

void audio_driver_init(i2s_chan_handle_t *rx_handle);
