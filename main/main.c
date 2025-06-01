#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

#define MCLK_GPIO          0    // GPIO 0 for MCLK output
#define MCLK_FREQ_HZ       12288000  // 256fs for 48kHz sampling rate

void app_main(void)
{
    // Configure MCPWM
    mcpwm_config_t pwm_config = {
        .frequency = MCLK_FREQ_HZ,
        .cmpr_a = 50.0,    // 50% duty cycle
        .cmpr_b = 50.0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MCLK_GPIO));

    ESP_LOGI("MCLK", "MCLK generation started at %d Hz", MCLK_FREQ_HZ);
}
