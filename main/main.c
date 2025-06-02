#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>

#define I2S_NUM         (0)
#define I2S_BCK_IO      (5)
#define I2S_WS_IO       (7)
#define I2S_DIN_IO      (6)
#define I2S_DOUT_IO     (17)  // PCM5102 DIN
#define I2S_BCK_DAC_IO  (18)  // PCM5102 BCK
#define I2S_WS_DAC_IO   (16)  // PCM5102 LCK
#define I2S_SAMPLE_RATE (48000) // PCM1808 default FS = 48kHz
#define MCLK_GPIO       (0)
#define MCLK_FREQ_HZ    (12288000) // 256 × 48kHz

// Constants for signal level calculation
#define MAX_24BIT       (8388607)    // Maximum 24-bit value
#define SIGNAL_THRESHOLD (MAX_24BIT / 100)  // 1% of max value
#define BUFFER_SIZE     64           // Reduced buffer size
#define TASK_STACK_SIZE 4096         // Increased stack size

// Volume control
#define DEFAULT_GAIN    8.0f         // Increased default gain
#define MAX_GAIN        16.0f        // Increased maximum gain
#define MIN_GAIN        0.1f         // Minimum gain

static const char *TAG = "PCM1808_APP";
static float current_gain = DEFAULT_GAIN;

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

static void init_i2s_adc_input(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX,  // Enable both RX and TX
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,           // Reduced buffer count
        .dma_buf_len = BUFFER_SIZE,   // Using smaller buffer size
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = MCLK_FREQ_HZ
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = MCLK_GPIO,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DOUT_IO,  // PCM5102 DIN
        .data_in_num = I2S_DIN_IO
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pin_config));
    ESP_LOGI(TAG, "I2S driver installed");

    // Set I2S clock
    i2s_set_clk(I2S_NUM, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    ESP_LOGI(TAG, "I2S clock set to %d Hz", I2S_SAMPLE_RATE);
}

// Function to play a test tone
static void play_test_tone(void)
{
    ESP_LOGI(TAG, "Playing test tone...");
    
    int32_t test_buffer[BUFFER_SIZE];
    size_t bytes_written;
    float phase = 0.0f;
    
    // Play a sequence of tones
    const float frequencies[] = {440.0f, 880.0f, 1760.0f}; // A4, A5, A6
    const int duration_ms = 1000; // 1 second per tone
    
    for (int f = 0; f < 3; f++) {
        float phase_increment = 2.0f * M_PI * frequencies[f] / I2S_SAMPLE_RATE;
        int samples_to_play = (I2S_SAMPLE_RATE * duration_ms) / 1000;
        int buffers_to_play = samples_to_play / BUFFER_SIZE;
        
        ESP_LOGI(TAG, "Playing tone at %.1f Hz", frequencies[f]);
        
        for (int i = 0; i < buffers_to_play; i++) {
            // Generate sine wave
            for (int j = 0; j < BUFFER_SIZE; j++) {
                float sample = sinf(phase) * MAX_24BIT * 0.5f; // 50% amplitude
                test_buffer[j] = (int32_t)sample << 8; // Shift for DAC
                phase += phase_increment;
                if (phase >= 2.0f * M_PI) {
                    phase -= 2.0f * M_PI;
                }
            }
            
            // Write to DAC
            i2s_write(I2S_NUM, test_buffer, sizeof(test_buffer), &bytes_written, portMAX_DELAY);
        }
        
        // Short silence between tones
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    ESP_LOGI(TAG, "Test tone sequence completed");
}

// Audio processing task
static void audio_task(void *pvParameters)
{
    int32_t i2s_read_buff[BUFFER_SIZE];
    int32_t i2s_write_buff[BUFFER_SIZE];
    size_t bytes_read;
    size_t bytes_written;
    int sample_count = 0;
    int32_t max_value = 0;
    int32_t min_value = 0;
    int64_t sum = 0;
    int32_t peak_value = 0;
    int32_t clipped_samples = 0;

    while (1) {
        esp_err_t res = i2s_read(I2S_NUM, &i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);
        if (res == ESP_OK) {
            int samples = bytes_read / sizeof(int32_t);
            
            // Reset statistics for this batch
            max_value = INT32_MIN;
            min_value = INT32_MAX;
            sum = 0;
            clipped_samples = 0;

            // Process samples and copy to write buffer
            for (int i = 0; i < samples; i++) {
                // PCM1808 outputs 24-bit data in 32-bit container, so we need to shift right by 8
                int32_t sample = i2s_read_buff[i] >> 8;
                
                // Apply gain
                float amplified = (float)sample * current_gain;
                
                // Check for clipping
                if (amplified > MAX_24BIT) {
                    amplified = MAX_24BIT;
                    clipped_samples++;
                } else if (amplified < -MAX_24BIT) {
                    amplified = -MAX_24BIT;
                    clipped_samples++;
                }
                
                // Convert back to integer and shift for DAC
                i2s_write_buff[i] = (int32_t)amplified << 8;
                
                // Update statistics
                if (sample > max_value) max_value = sample;
                if (sample < min_value) min_value = sample;
                sum += abs(sample);
                
                // Update peak value
                if (abs(sample) > peak_value) {
                    peak_value = abs(sample);
                }
            }

            // Write to DAC
            i2s_write(I2S_NUM, i2s_write_buff, bytes_read, &bytes_written, portMAX_DELAY);

            // Calculate average
            int32_t avg = sum / samples;
            
            // Calculate signal level as percentage of max 24-bit value
            float signal_level = (float)peak_value / MAX_24BIT * 100.0f;

            // Print statistics every 1000 batches
            if (++sample_count % 1000 == 0) {
                ESP_LOGI(TAG, "=== Audio Signal Analysis ===");
                ESP_LOGI(TAG, "Current Gain: %.1fx", current_gain);
                ESP_LOGI(TAG, "Signal Level: %.2f%% of max", signal_level);
                ESP_LOGI(TAG, "Peak Value: %ld (%.2f%% of max)", peak_value, (float)peak_value/MAX_24BIT * 100.0f);
                ESP_LOGI(TAG, "Max: %ld", max_value);
                ESP_LOGI(TAG, "Min: %ld", min_value);
                ESP_LOGI(TAG, "Avg: %ld", avg);
                if (clipped_samples > 0) {
                    ESP_LOGI(TAG, "WARNING: %ld samples clipped!", clipped_samples);
                }
                
                // Print signal level indicator
                int bars = (int)(signal_level / 5); // 20 bars max
                char level_bar[21] = {0};
                for (int i = 0; i < bars && i < 20; i++) {
                    level_bar[i] = '|';
                }
                ESP_LOGI(TAG, "Level: [%-20s]", level_bar);
                
                // Print first few samples for debugging
                ESP_LOGI(TAG, "First 5 samples (raw values):");
                for (int i = 0; i < 5 && i < samples; i++) {
                    ESP_LOGI(TAG, "  Sample[%d]: %ld (%.2f%%)", 
                            i, 
                            i2s_read_buff[i] >> 8,
                            (float)(i2s_read_buff[i] >> 8) / MAX_24BIT * 100.0f);
                }
                ESP_LOGI(TAG, "===========================");
            }
        }
    }
}

void app_main(void)
{
    // Initialize hardware
    init_mclk_output();       // Start MCLK output to drive the ADC
    init_i2s_adc_input();     // Start I2S to receive audio data

    // Play test tone sequence
    play_test_tone();

    // Create audio processing task
    xTaskCreate(audio_task, "audio_task", TASK_STACK_SIZE, NULL, 5, NULL);
}
