#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>
#include "sh1107.h"

#define I2S_NUM_ADC     (0)    // I2S number for ADC (PCM1808)
#define I2S_NUM_DAC     (1)    // I2S number for DAC (PCM5102)

// ADC (PCM1808) pins
#define I2S_BCK_IO      (6)
#define I2S_WS_IO       (7)
#define I2S_DIN_IO      (5)
#define MCLK_GPIO       (0)
#define MCLK_FREQ_HZ    (12288000) // 256 × 48kHz

// DAC (PCM5102) pins
#define I2S_DOUT_IO     (17)  // PCM5102 DIN
#define I2S_BCK_DAC_IO  (5)  // PCM5102 BCK
#define I2S_WS_DAC_IO   (7)  // PCM5102 LCK

#define I2S_SAMPLE_RATE (48000) // PCM1808 default FS = 48kHz


// Constants for signal level calculation
#define SIGNAL_THRESHOLD (MAX_24BIT / 100)  // 1% of max value
#define BUFFER_SIZE     64           // Reduced buffer size
#define TASK_STACK_SIZE 4096         // Increased stack size

// Volume control
#define DEFAULT_GAIN    0.5f         // Increased default gain
#define MAX_GAIN        1.0f        // Increased maximum gain
#define MIN_GAIN        0.5f         // Minimum gain

// Display pins
#define DISPLAY_SCL_PIN 13
#define DISPLAY_SDA_PIN 12

// Display update rate
#define DISPLAY_UPDATE_INTERVAL_MS 50  // 20 FPS

// Display update task configuration
#define DISPLAY_TASK_STACK_SIZE 4096
#define DISPLAY_TASK_PRIORITY   5
#define DISPLAY_TASK_CORE_ID   1  // Run on second core

// Queue size for display updates
#define DISPLAY_QUEUE_SIZE 2

// Structure to hold display data
typedef struct {
    int32_t samples[SH1107_WIDTH];
    float signal_level;
    float current_gain;
} display_data_t;

// Global variables
static QueueHandle_t display_queue = NULL;
static int display_sample_index = 0;  // Added back the sample index
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
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,  // ADC is RX only
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // Stereo
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
        .data_out_num = I2S_PIN_NO_CHANGE,  // ADC doesn't need output
        .data_in_num = I2S_DIN_IO
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_ADC, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_ADC, &pin_config));
    ESP_LOGI(TAG, "I2S ADC driver installed");

    // Set I2S clock
    i2s_set_clk(I2S_NUM_ADC, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);  // Changed to stereo
    ESP_LOGI(TAG, "I2S ADC clock set to %d Hz", I2S_SAMPLE_RATE);
}

static void init_i2s_dac_output(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,  // DAC is TX only
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // Stereo
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0  // DAC doesn't need MCLK
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,  // DAC doesn't need MCLK
        .bck_io_num = I2S_BCK_DAC_IO,
        .ws_io_num = I2S_WS_DAC_IO,
        .data_out_num = I2S_DOUT_IO,
        .data_in_num = I2S_PIN_NO_CHANGE  // DAC doesn't need input
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_DAC, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_DAC, &pin_config));
    ESP_LOGI(TAG, "I2S DAC driver installed");

    // Set I2S clock
    i2s_set_clk(I2S_NUM_DAC, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);  // Changed to stereo
    ESP_LOGI(TAG, "I2S DAC clock set to %d Hz", I2S_SAMPLE_RATE);
}

// Function to play a test tone
static void play_test_tone(void)
{
    ESP_LOGI(TAG, "Playing test tone...");
    
    int32_t test_buffer[BUFFER_SIZE * 2];  // Doubled buffer size for stereo
    size_t bytes_written;
    float phase = 0.0f;
    
    // Play a sequence of tones
    const float frequencies[] = {440.0f}; // A4, A5, A6
    const int duration_ms = 100; // 1 second per tone
    
    for (int f = 0; f < 1; f++) {
        float phase_increment = 2.0f * M_PI * frequencies[f] / I2S_SAMPLE_RATE;
        int samples_to_play = (I2S_SAMPLE_RATE * duration_ms) / 1000;
        int buffers_to_play = samples_to_play / BUFFER_SIZE;
        
        ESP_LOGI(TAG, "Playing tone at %.1f Hz", frequencies[f]);
        
        for (int i = 0; i < buffers_to_play; i++) {
            // Generate sine wave for both channels
            for (int j = 0; j < BUFFER_SIZE; j++) {
                float sample = sinf(phase) * MAX_24BIT * 0.5f; // 50% amplitude
                int32_t sample_shifted = (int32_t)sample << 8; // Shift for DAC
                test_buffer[j * 2] = sample_shifted;     // Left channel
                test_buffer[j * 2 + 1] = sample_shifted; // Right channel
                phase += phase_increment;
                if (phase >= 2.0f * M_PI) {
                    phase -= 2.0f * M_PI;
                }
            }
            
            // Write to DAC
            i2s_write(I2S_NUM_DAC, test_buffer, sizeof(test_buffer), &bytes_written, portMAX_DELAY);
        }
        
        // Short silence between tones
        //vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    ESP_LOGI(TAG, "Test tone sequence completed");
}

// Display task running on core 1
static void display_task(void *pvParameters) {
    display_data_t display_data;
    
    while (1) {
        // Wait for new display data
        if (xQueueReceive(display_queue, &display_data, pdMS_TO_TICKS(DISPLAY_UPDATE_INTERVAL_MS)) == pdTRUE) {
            // Update the display
            sh1107_draw_wave(display_data.samples, SH1107_WIDTH);
            sh1107_display_stats(display_data.signal_level, display_data.current_gain);
            sh1107_update();
        }
    }
}

// Modified audio processing task
static void audio_task(void *pvParameters)
{
    int32_t i2s_read_buff[BUFFER_SIZE * 2];
    int32_t i2s_write_buff[BUFFER_SIZE * 2];
    size_t bytes_read;
    size_t bytes_written;
    int sample_count = 0;
    int32_t max_value = 0;
    int32_t min_value = 0;
    int64_t sum = 0;
    int32_t peak_value = 0;
    int32_t clipped_samples = 0;
    TickType_t last_display_update = xTaskGetTickCount();
    display_data_t display_data = {0};

    while (1) {
        esp_err_t res = i2s_read(I2S_NUM_ADC, &i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);
        if (res == ESP_OK) {
            int samples = bytes_read / sizeof(int32_t);
            
            // Reset statistics for this batch
            max_value = INT32_MIN;
            min_value = INT32_MAX;
            sum = 0;
            clipped_samples = 0;

            // Process samples and copy to write buffer
            for (int i = 0; i < samples; i += 2) {
                int32_t left_sample = i2s_read_buff[i] >> 8;

                int32_t right_sample = i2s_read_buff[i + 1] >> 8;
                
                // Store samples for display (use left channel)
                display_data.samples[display_sample_index] = left_sample;
                display_sample_index = (display_sample_index + 1) % SH1107_WIDTH;
                
                // Apply gain to both channels
                float left_amplified = (float)left_sample * current_gain;
                float right_amplified = (float)right_sample * current_gain;
                
                // Check for clipping
                if (left_amplified > MAX_24BIT) {
                    left_amplified = MAX_24BIT;
                    clipped_samples++;
                } else if (left_amplified < -MAX_24BIT) {
                    left_amplified = -MAX_24BIT;
                    clipped_samples++;
                }
                
                if (right_amplified > MAX_24BIT) {
                    right_amplified = MAX_24BIT;
                    clipped_samples++;
                } else if (right_amplified < -MAX_24BIT) {
                    right_amplified = -MAX_24BIT;
                    clipped_samples++;
                }
                
                // Convert back to integer and shift for DAC
                i2s_write_buff[i] = (int32_t)left_amplified << 8;
                i2s_write_buff[i + 1] = (int32_t)right_amplified << 8;
                
                // Update statistics using the maximum of both channels
                int32_t max_sample = (abs(left_sample) > abs(right_sample)) ? left_sample : right_sample;
                if (max_sample > max_value) max_value = max_sample;
                if (max_sample < min_value) min_value = max_sample;
                sum += abs(max_sample);
                
                if (abs(max_sample) > peak_value) {
                    peak_value = abs(max_sample);
                }
            }

            // Write to DAC
            i2s_write(I2S_NUM_DAC, i2s_write_buff, bytes_read, &bytes_written, portMAX_DELAY);

            // Calculate signal level as percentage of max 24-bit value
            float signal_level = (float)peak_value / MAX_24BIT * 100.0f;

            // Update display at fixed interval
            if ((xTaskGetTickCount() - last_display_update) >= pdMS_TO_TICKS(DISPLAY_UPDATE_INTERVAL_MS)) {
                // Update display data
                display_data.signal_level = signal_level;
                display_data.current_gain = current_gain;
                
                // Send to display task (non-blocking)
                xQueueSendToBack(display_queue, &display_data, 0);
                
                last_display_update = xTaskGetTickCount();
            }

            // Print statistics every second (48000 samples)
            if (++sample_count % 48000 == 0) {
                ESP_LOGI(TAG, "=== Audio Signal Analysis ===");
                ESP_LOGI(TAG, "Current Gain: %.1fx", current_gain);
                ESP_LOGI(TAG, "Signal Level: %.2f%% of max", signal_level);
                ESP_LOGI(TAG, "Peak Value: %ld", peak_value);
                if (clipped_samples > 0) {
                    ESP_LOGI(TAG, "WARNING: %ld samples clipped!", clipped_samples);
                }
            }
        }
    }
}

void app_main(void)
{
    // Initialize hardware
    init_mclk_output();       // Start MCLK output to drive the ADC
    init_i2s_adc_input();     // Start I2S to receive audio data from ADC
    init_i2s_dac_output();    // Start I2S to send audio data to DAC

    // Initialize display
    esp_err_t ret = sh1107_init(DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return;
    }

    // Create display queue
    display_queue = xQueueCreate(DISPLAY_QUEUE_SIZE, sizeof(display_data_t));
    if (display_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create display queue");
        return;
    }

    // Play test tone sequence
    play_test_tone();

    // Create display task on core 1
    BaseType_t ret_task = xTaskCreatePinnedToCore(
        display_task,
        "display_task",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        DISPLAY_TASK_PRIORITY,
        NULL,
        DISPLAY_TASK_CORE_ID
    );
    
    if (ret_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        return;
    }

    // Create audio processing task (will run on core 0 by default)
    xTaskCreate(audio_task, "audio_task", TASK_STACK_SIZE, NULL, 5, NULL);
}
