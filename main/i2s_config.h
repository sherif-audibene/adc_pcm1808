#pragma once

#include "driver/i2s_std.h"
#include "esp_err.h"

// Add these definitions
#define SAMPLE_RATE     39000
#define I2S_BCK_PIN     15
#define I2S_LRCK_PIN    16
#define I2S_DATA_PIN    17

// Add DMA buffer definitions
#define DMA_BUFFER_COUNT 8
#define DMA_BUFFER_LEN   1024

// Function declaration
esp_err_t init_i2s(i2s_chan_handle_t *tx_handle); 