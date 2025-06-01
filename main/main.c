#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define I2S_NUM         (0)
#define I2S_BCK_IO      (5)
#define I2S_WS_IO       (7)
#define I2S_DIN_IO      (6)
#define I2S_SAMPLE_RATE (48000) // PCM1808 default FS = 48kHz
#define MCLK_GPIO       (0)
#define MCLK_FREQ_HZ    (12288000) // 256 × 48kHz

static const char *TAG = "PCM1808_APP";

void init_mclk_output()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_1_BIT,       // 1-bit gives clean 50% square wave
        .freq_hz          = MCLK_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MCLK_GPIO,
        .duty           = 1,    // 50% duty cycle (1-bit resolution)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "MCLK (%.2f MHz) output started on GPIO%d", MCLK_FREQ_HZ / 1e6, MCLK_GPIO);
}

void init_i2s_adc_input()
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // PCM1808 outputs 24 bits, use 32 here and shift
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // PCM1808 is mono per output (L or R)
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0  // We manually provide MCLK using LEDC
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN_IO
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pin_config));
    ESP_LOGI(TAG, "I2S initialized for PCM1808 ADC input.");
}

void app_main(void)
{
    init_mclk_output();       // Start MCLK output to drive the ADC
    init_i2s_adc_input();     // Start I2S to receive audio data

    int32_t i2s_read_buff[256];
    size_t bytes_read;
    int sample_count = 0;
    int32_t max_value = 0;
    int32_t min_value = 0;
    int64_t sum = 0;

    while (1) {
        esp_err_t res = i2s_read(I2S_NUM, &i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);
        if (res == ESP_OK) {
            int samples = bytes_read / sizeof(int32_t);
            
            // Reset statistics for this batch
            max_value = INT32_MIN;
            min_value = INT32_MAX;
            sum = 0;

            // Process samples
            for (int i = 0; i < samples; i++) {
                // PCM1808 outputs 24-bit data in 32-bit container, so we need to shift right by 8
                int32_t sample = i2s_read_buff[i] >> 8;
                
                // Update statistics
                if (sample > max_value) max_value = sample;
                if (sample < min_value) min_value = sample;
                sum += abs(sample);
            }

            // Calculate average
            int32_t avg = sum / samples;

            // Print statistics every 10 batches
            if (++sample_count % 1000 == 0) {
                ESP_LOGI(TAG, "Sample Statistics:");
                ESP_LOGI(TAG, "  Max: %ld", max_value);
                ESP_LOGI(TAG, "  Min: %ld", min_value);
                ESP_LOGI(TAG, "  Avg: %ld", avg);
                ESP_LOGI(TAG, "  Samples in batch: %d", samples);
                
                // Print first few samples for debugging
                ESP_LOGI(TAG, "First 5 samples:");
                for (int i = 0; i < 5 && i < samples; i++) {
                    ESP_LOGI(TAG, "  Sample[%d]: %ld", i, i2s_read_buff[i] >> 8);
                }
            }
        }
    }
}
