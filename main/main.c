#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>
#include "esp_mac.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

#define I2S_NUM_ADC     (0)    // I2S number for ADC (PCM1808)
#define I2S_NUM_DAC     (1)    // I2S number for DAC (PCM5102)

// ADC (PCM1808) pins
#define I2S_BCK_IO      (5)
#define I2S_WS_IO       (7)
#define I2S_DIN_IO      (6)
#define MCLK_GPIO       (0)
#define MCLK_FREQ_HZ    (12288000) // 256 × 48kHz

// DAC (PCM5102) pins
#define I2S_DOUT_IO     (17)  // PCM5102 DIN
#define I2S_BCK_DAC_IO  (18)  // PCM5102 BCK
#define I2S_WS_DAC_IO   (16)  // PCM5102 LCK

#define I2S_SAMPLE_RATE (48000) // PCM1808 default FS = 48kHz

// Constants for signal level calculation
#define MAX_24BIT       (8388607)    // Maximum 24-bit value
#define SIGNAL_THRESHOLD (MAX_24BIT / 100)  // 1% of max value
#define BUFFER_SIZE     128           // Reduced buffer size
#define TASK_STACK_SIZE 8192         // Increased stack size for complex processing

// Volume control
#define DEFAULT_GAIN    0.3f         // Increased default gain

// Add diagnostic modes after the existing defines
#define DIAGNOSTIC_MODE_PASSTHROUGH 0
#define DIAGNOSTIC_MODE_SILENCE     1  
#define DIAGNOSTIC_MODE_TEST_TONE   2
#define DIAGNOSTIC_MODE_NORMAL      3

// Add diagnostic control variable
static int diagnostic_mode = DIAGNOSTIC_MODE_NORMAL;
static bool enable_detailed_logging = false;

static const char *TAG = "PCM1808_APP";
static float current_gain = DEFAULT_GAIN;

// Add diagnostic control function
void set_diagnostic_mode(int mode) {
    if (mode >= 0 && mode <= 3) {
        diagnostic_mode = mode;
        ESP_LOGI(TAG, "Diagnostic mode set to: %d", mode);
    }
}

// Add console command for diagnostic mode
static struct {
    struct arg_int *mode;
    struct arg_end *end;
} diag_args;

static int cmd_diag_mode(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &diag_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, diag_args.end, argv[0]);
        return 1;
    }

    int mode = diag_args.mode->ival[0];
    if (mode >= 0 && mode <= 3) {
        set_diagnostic_mode(mode);
        const char* mode_names[] = {"PASSTHROUGH", "SILENCE", "TEST_TONE", "NORMAL"};
        printf("Diagnostic mode set to %d (%s)\n", mode, mode_names[mode]);
        printf("Modes:\n");
        printf("  0 - PASSTHROUGH: Simple passthrough without DC filtering\n");
        printf("  1 - SILENCE: Output silence (tests output path)\n");
        printf("  2 - TEST_TONE: Generate 1kHz test tone\n");
        printf("  3 - NORMAL: Full processing with DC filtering\n");
    } else {
        printf("Invalid mode. Use 0-3\n");
        return 1;
    }
    return 0;
}

static void register_diag_mode(void)
{
    diag_args.mode = arg_int1(NULL, NULL, "<mode>", "Diagnostic mode (0-3)");
    diag_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "diag",
        .help = "Set diagnostic mode",
        .hint = NULL,
        .func = &cmd_diag_mode,
        .argtable = &diag_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

// Add console initialization
static void initialize_console(void)
{
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
        .baud_rate = 115200,  // Use fixed baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_APB,  // Use APB clock instead
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,
            256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);
    linenoiseHistorySetMaxLen(100);

    /* Register commands */
    register_diag_mode();
}

// Console task
static void console_task(void *pvParameters)
{
    initialize_console();
    
    printf("\n\n=== PCM1808 Audio Diagnostics ===\n");
    printf("Type 'diag <mode>' to change diagnostic mode:\n");
    printf("  diag 0 - PASSTHROUGH (minimal processing)\n");
    printf("  diag 1 - SILENCE (test output path)\n");  
    printf("  diag 2 - TEST_TONE (generate 1kHz tone)\n");
    printf("  diag 3 - NORMAL (full processing)\n");
    printf("Current mode: %d\n", diagnostic_mode);
    printf("\n");

    while (1) {
        char* line = linenoise("esp32> ");
        if (line == NULL) { /* Ignore EOF and ctrl-C */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
}

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
        .dma_buf_count = 8,           // Reduced buffer count
        .dma_buf_len = BUFFER_SIZE,   // Using smaller buffer size
        .use_apll = true,
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

// Add this function before audio_task
static void generate_test_tone(int32_t *buffer, int samples, float frequency)
{
    static float phase = 0.0f;
    const float amplitude = MAX_24BIT * 0.1f; // 10% amplitude
    const float phase_increment = 2.0f * M_PI * frequency / I2S_SAMPLE_RATE;
    
    for (int i = 0; i < samples; i += 2) {
        int32_t sample = (int32_t)(amplitude * sinf(phase));
        buffer[i] = sample << 8;     // Left channel
        buffer[i + 1] = sample << 8; // Right channel
        phase += phase_increment;
        if (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;
    }
}

// Replace the audio_task function
static void audio_task(void *pvParameters)
{
    static int32_t i2s_read_buff[BUFFER_SIZE * 2];  // Make static to reduce stack usage
    static int32_t i2s_write_buff[BUFFER_SIZE * 2]; // Make static to reduce stack usage 
    size_t bytes_read;
    size_t bytes_written;
    int sample_count = 0;
    int32_t clipped_samples = 0;

    // Constants for 24-bit audio processing
    const int32_t MAX_24BIT_POSITIVE = 8388607;   
    const int32_t MAX_24BIT_NEGATIVE = -8388608;  
    
    // DC blocking filter variables
    static float left_dc_filter_x1 = 0.0f, left_dc_filter_y1 = 0.0f;
    static float right_dc_filter_x1 = 0.0f, right_dc_filter_y1 = 0.0f;
    const float dc_filter_alpha = 0.995f; // Less aggressive DC blocking
    
    // Startup settling time
    static int startup_samples = 0;
    const int STARTUP_SETTLE_SAMPLES = 48000; 
    
    // Running average for DC offset detection
    static int64_t left_dc_accumulator = 0;
    static int64_t right_dc_accumulator = 0;
    static int dc_sample_count = 0;
    
    // Signal level monitoring
    static int64_t left_signal_accumulator = 0;
    static int64_t right_signal_accumulator = 0;
    static int32_t left_peak = 0, right_peak = 0;

    ESP_LOGI(TAG, "Audio task started in diagnostic mode: %d", diagnostic_mode);

    while (1) {
        // Always read from ADC to keep I2S running
        esp_err_t res = i2s_read(I2S_NUM_ADC, &i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);
        if (res == ESP_OK) {
            int samples = bytes_read / sizeof(int32_t);
            
            switch (diagnostic_mode) {
                case DIAGNOSTIC_MODE_SILENCE:
                    // Output silence to test if noise comes from output stage
                    memset(i2s_write_buff, 0, sizeof(i2s_write_buff));
                    break;
                    
                case DIAGNOSTIC_MODE_TEST_TONE:
                    // Generate clean test tone to test output path
                    generate_test_tone(i2s_write_buff, samples, 1000.0f); // 1kHz tone
                    break;
                    
                case DIAGNOSTIC_MODE_PASSTHROUGH:
                    // Simple passthrough without any processing
                    for (int i = 0; i < samples; i += 2) {
                        int32_t left_raw = i2s_read_buff[i];
                        int32_t right_raw = i2s_read_buff[i + 1];
                        
                        // Simple gain reduction without filtering
                        int64_t left_sample = ((int64_t)left_raw * (int64_t)(current_gain * 256)) >> 8;
                        int64_t right_sample = ((int64_t)right_raw * (int64_t)(current_gain * 256)) >> 8;
                        
                        // Simple clipping
                        if (left_sample > 2147483647LL) left_sample = 2147483647LL;
                        if (left_sample < -2147483648LL) left_sample = -2147483648LL;
                        if (right_sample > 2147483647LL) right_sample = 2147483647LL;
                        if (right_sample < -2147483648LL) right_sample = -2147483648LL;
                        
                        i2s_write_buff[i] = (int32_t)left_sample;
                        i2s_write_buff[i + 1] = (int32_t)right_sample;
                    }
                    break;
                    
                case DIAGNOSTIC_MODE_NORMAL:
                default:
                    // Process samples with improved algorithm
                    for (int i = 0; i < samples; i += 2) {  
                        int32_t left_raw = i2s_read_buff[i];
                        int32_t right_raw = i2s_read_buff[i + 1];
                        
                        // Extract 24-bit data (assuming it's in upper 24 bits)
                        int32_t left_sample = left_raw >> 8;   
                        int32_t right_sample = right_raw >> 8; 
                        
                        // Accumulate for monitoring
                        left_dc_accumulator += left_sample;
                        right_dc_accumulator += right_sample;
                        left_signal_accumulator += abs(left_sample);
                        right_signal_accumulator += abs(right_sample);
                        dc_sample_count++;
                        
                        // Track peaks
                        if (abs(left_sample) > left_peak) left_peak = abs(left_sample);
                        if (abs(right_sample) > right_peak) right_peak = abs(right_sample);
                        
                        // Apply DC blocking only if DC offset is significant
                        float left_float = (float)left_sample;
                        float right_float = (float)right_sample;
                        
                        // More conservative DC blocking
                        float left_filtered = left_float;
                        float right_filtered = right_float;
                        
                        if (dc_sample_count > 4800) { // After some samples
                            int32_t left_dc_avg = left_dc_accumulator / dc_sample_count;
                            int32_t right_dc_avg = right_dc_accumulator / dc_sample_count;
                            
                            // Only apply DC blocking if offset is significant
                            if (abs(left_dc_avg) > 10000) {
                                left_filtered = dc_filter_alpha * (left_dc_filter_y1 + left_float - left_dc_filter_x1);
                                left_dc_filter_x1 = left_float;
                                left_dc_filter_y1 = left_filtered;
                            }
                            
                            if (abs(right_dc_avg) > 10000) {
                                right_filtered = dc_filter_alpha * (right_dc_filter_y1 + right_float - right_dc_filter_x1);
                                right_dc_filter_x1 = right_float;
                                right_dc_filter_y1 = right_filtered;
                            }
                        }
                        
                        // Gradual startup gain
                        float effective_gain = current_gain;
                        if (startup_samples < STARTUP_SETTLE_SAMPLES) {
                            float ramp_factor = (float)startup_samples / STARTUP_SETTLE_SAMPLES;
                            effective_gain *= ramp_factor;
                            startup_samples++;
                        }
                        
                        // Apply gain
                        float left_amplified = left_filtered * effective_gain;
                        float right_amplified = right_filtered * effective_gain;
                        
                        // Soft limiting
                        const float limit_threshold = MAX_24BIT_POSITIVE * 0.95f;
                        if (left_amplified > limit_threshold) {
                            left_amplified = limit_threshold;
                            clipped_samples++;
                        } else if (left_amplified < -limit_threshold) {
                            left_amplified = -limit_threshold;
                            clipped_samples++;
                        }
                        
                        if (right_amplified > limit_threshold) {
                            right_amplified = limit_threshold;
                            clipped_samples++;
                        } else if (right_amplified < -limit_threshold) {
                            right_amplified = -limit_threshold;
                            clipped_samples++;
                        }
                        
                        // Convert back to 32-bit format
                        i2s_write_buff[i] = ((int32_t)left_amplified) << 8;
                        i2s_write_buff[i + 1] = ((int32_t)right_amplified) << 8;
                    }
                    break;
            }

            // Write to DAC
            i2s_write(I2S_NUM_DAC, i2s_write_buff, bytes_read, &bytes_written, portMAX_DELAY);
            
            sample_count += samples;
            
            // Enhanced logging every ~2 seconds
            if (dc_sample_count >= 96000) { 
                int32_t left_dc_avg = left_dc_accumulator / dc_sample_count;
                int32_t right_dc_avg = right_dc_accumulator / dc_sample_count;
                int32_t left_signal_avg = left_signal_accumulator / dc_sample_count;
                int32_t right_signal_avg = right_signal_accumulator / dc_sample_count;
                
                ESP_LOGI(TAG, "=== AUDIO DIAGNOSTICS (Mode: %d) ===", diagnostic_mode);
                ESP_LOGI(TAG, "DC Offset - L: %ld, R: %ld", left_dc_avg, right_dc_avg);
                ESP_LOGI(TAG, "Signal Level - L: %ld, R: %ld", left_signal_avg, right_signal_avg);
                ESP_LOGI(TAG, "Peak Levels - L: %ld, R: %ld", left_peak, right_peak);
                ESP_LOGI(TAG, "Clipped samples: %ld", clipped_samples);
                ESP_LOGI(TAG, "Startup progress: %d%%", 
                    startup_samples >= STARTUP_SETTLE_SAMPLES ? 100 : 
                    (startup_samples * 100) / STARTUP_SETTLE_SAMPLES);
                
                // Check for potential issues
                if (left_signal_avg < 100 && right_signal_avg < 100) {
                    ESP_LOGW(TAG, "Very low signal level - check input connections");
                }
                if (left_peak < 1000 && right_peak < 1000) {
                    ESP_LOGW(TAG, "Very low peak levels - input may be disconnected");
                }
                if (abs(left_dc_avg) > 50000 || abs(right_dc_avg) > 50000) {
                    ESP_LOGW(TAG, "High DC offset - potential hardware issue");
                }
                
                // Reset counters
                left_dc_accumulator = 0;
                right_dc_accumulator = 0;
                left_signal_accumulator = 0;
                right_signal_accumulator = 0;
                dc_sample_count = 0;
                clipped_samples = 0;
                left_peak = 0;
                right_peak = 0;
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

    // Play test tone sequence
    //play_test_tone();

    // Create audio processing task
    xTaskCreatePinnedToCore(audio_task, "audio_task", TASK_STACK_SIZE, NULL, 5, NULL, 1);

    // Create console task
    xTaskCreatePinnedToCore(console_task, "console_task", TASK_STACK_SIZE, NULL, 5, NULL, 1);
}