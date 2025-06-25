#ifndef SH1107_H
#define SH1107_H

#include <stdint.h>
#include "esp_err.h"

// Display dimensions
#define SH1107_WIDTH 128
#define SH1107_HEIGHT 64

// Audio constants
#define MAX_24BIT       (8388607)    // Maximum 24-bit value

// Function declarations
esp_err_t sh1107_init(int sda_pin, int scl_pin);
void sh1107_clear(void);
void sh1107_update(void);
void sh1107_draw_wave(const int32_t* samples, int num_samples);
void sh1107_display_stats(float signal_level, float current_gain);

#endif // SH1107_H 