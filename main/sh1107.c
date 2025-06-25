#include "sh1107.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SH1107";

#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_TIMEOUT_MS              1000
#define SH1107_ADDR                 0x3C

// Display buffer
static uint8_t display_buffer[SH1107_HEIGHT * SH1107_WIDTH / 8];

// Command definitions
#define SH1107_DISPLAYOFF          0xAE
#define SH1107_SETCONTRAST        0x81
#define SH1107_DISPLAYALLON_RESUME 0xA4
#define SH1107_DISPLAYALLON       0xA5
#define SH1107_NORMALDISPLAY     0xA6
#define SH1107_INVERTDISPLAY     0xA7
#define SH1107_DISPLAYON         0xAF
#define SH1107_SETDISPLAYOFFSET  0xD3
#define SH1107_SETDISPLAYCLOCKDIV 0xD5
#define SH1107_SETPRECHARGE      0xD9
#define SH1107_SETCOMPINS        0xDA
#define SH1107_SETVCOMDETECT     0xDB
#define SH1107_SETDCDC            0x8D

static esp_err_t sh1107_write_cmd(uint8_t cmd) {
    uint8_t write_buf[2] = {0x00, cmd};  // Control byte + command
    return i2c_master_write_to_device(I2C_MASTER_NUM, SH1107_ADDR, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t sh1107_write_data(uint8_t* data, size_t size) {
    uint8_t *write_buf = malloc(size + 1);
    if (write_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    write_buf[0] = 0x40;  // Control byte for data
    memcpy(write_buf + 1, data, size);
    
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, SH1107_ADDR, write_buf, size + 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    free(write_buf);
    return ret;
}

esp_err_t sh1107_init(int sda_pin, int scl_pin) {
    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) return ret;

    // Initialize display
    sh1107_write_cmd(SH1107_DISPLAYOFF);
    sh1107_write_cmd(SH1107_SETCONTRAST);
    sh1107_write_cmd(0x7F);  // Contrast value
    sh1107_write_cmd(SH1107_NORMALDISPLAY);
    sh1107_write_cmd(SH1107_SETDISPLAYOFFSET);
    sh1107_write_cmd(0x00);
    sh1107_write_cmd(SH1107_SETDCDC);
    sh1107_write_cmd(0x14);
    sh1107_write_cmd(SH1107_DISPLAYON);

    // Clear display buffer
    memset(display_buffer, 0, sizeof(display_buffer));
    
    ESP_LOGI(TAG, "SH1107 initialized successfully");
    return ESP_OK;
}

void sh1107_clear(void) {
    memset(display_buffer, 0, sizeof(display_buffer));
}

void sh1107_update(void) {
    for (int page = 0; page < 8; page++) {
        sh1107_write_cmd(0xB0 | page);  // Set page address
        sh1107_write_cmd(0x00);         // Set lower column address
        sh1107_write_cmd(0x10);         // Set higher column address
        
        sh1107_write_data(&display_buffer[page * SH1107_WIDTH], SH1107_WIDTH);
    }
}

void sh1107_draw_wave(const int32_t* samples, int num_samples) {
    sh1107_clear();
    
    // Use a more aggressive scaling to make the wave more visible
    // We'll use about 75% of the display height for the waveform
    float display_height = SH1107_HEIGHT * 0.75;
    float center_y = SH1107_HEIGHT / 2;
    
    // Find the maximum amplitude in the current buffer to auto-scale
    int32_t max_amplitude = 1;  // Avoid division by zero
    for (int i = 0; i < num_samples && i < SH1107_WIDTH; i++) {
        int32_t abs_sample = abs(samples[i]);
        if (abs_sample > max_amplitude) {
            max_amplitude = abs_sample;
        }
    }
    
    // Calculate scaling factor - use either auto-scaling or fixed scaling, whichever gives larger display
    float auto_scale = display_height / (max_amplitude * 2);
    float fixed_scale = display_height / (MAX_24BIT * 2);
    float scale = (auto_scale > fixed_scale) ? fixed_scale : auto_scale;
    
    // Draw the zero line
    int zero_line = center_y;
    int zero_page = zero_line / 8;
    display_buffer[zero_page * SH1107_WIDTH + 0] |= (1 << (zero_line % 8));
    display_buffer[zero_page * SH1107_WIDTH + SH1107_WIDTH-1] |= (1 << (zero_line % 8));
    
    // Draw the waveform
    int prev_y = -1;
    for (int i = 0; i < num_samples && i < SH1107_WIDTH; i++) {
        // Calculate y position
        float scaled_sample = samples[i] * scale;
        int y = center_y - (int)scaled_sample;
        
        // Clamp y to valid range
        y = (y < 0) ? 0 : (y >= SH1107_HEIGHT ? SH1107_HEIGHT-1 : y);
        
        // Draw vertical line from previous point to current point if needed
        if (prev_y != -1 && i > 0) {
            int start_y = (prev_y < y) ? prev_y : y;
            int end_y = (prev_y < y) ? y : prev_y;
            
            for (int py = start_y; py <= end_y; py++) {
                int page = py / 8;
                int bit = py % 8;
                display_buffer[page * SH1107_WIDTH + i] |= (1 << bit);
            }
        } else {
            // Draw single point
            int page = y / 8;
            int bit = y % 8;
            display_buffer[page * SH1107_WIDTH + i] |= (1 << bit);
        }
        
        prev_y = y;
    }
}

void sh1107_display_stats(float signal_level, float current_gain) {
    // Draw a simple level meter at the bottom of the screen
    int level_width = (int)((signal_level / 100.0f) * SH1107_WIDTH);
    
    // Clear the bottom area first
    for (int i = 0; i < SH1107_WIDTH; i++) {
        display_buffer[7 * SH1107_WIDTH + i] = 0x00;
    }
    
    // Draw the level meter
    for (int i = 0; i < level_width && i < SH1107_WIDTH; i++) {
        display_buffer[7 * SH1107_WIDTH + i] = 0xFF;
    }
} 