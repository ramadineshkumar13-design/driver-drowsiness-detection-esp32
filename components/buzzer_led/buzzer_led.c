#include "buzzer_led.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "BUZZER_LED";

esp_err_t buzzer_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_BUZZER_GPIO) |
                        (1ULL << CONFIG_LED_WARNING_GPIO) |
                        (1ULL << CONFIG_LED_CRITICAL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Buzzer/LED GPIOs: %s", esp_err_to_name(err));
        return err;
    }

    buzzer_led_all_off();
    ESP_LOGI(TAG, "Buzzer (GPIO%d), Warning LED (GPIO%d), Critical LED (GPIO%d) initialized.",
             CONFIG_BUZZER_GPIO, CONFIG_LED_WARNING_GPIO, CONFIG_LED_CRITICAL_GPIO);
    return ESP_OK;
}

void buzzer_set(bool on)
{
    gpio_set_level((gpio_num_t)CONFIG_BUZZER_GPIO, on ? 1 : 0);
}

void led_warning_set(bool on)
{
    gpio_set_level((gpio_num_t)CONFIG_LED_WARNING_GPIO, on ? 1 : 0);
}

void led_critical_set(bool on)
{
    gpio_set_level((gpio_num_t)CONFIG_LED_CRITICAL_GPIO, on ? 1 : 0);
}

void buzzer_led_all_off(void)
{
    buzzer_set(false);
    led_warning_set(false);
    led_critical_set(false);
}

void buzzer_led_update_pattern(system_state_t state, uint32_t tick_100ms)
{
    switch (state) {
        case SYSTEM_STATE_NORMAL:
            /* Quiet state: Buzzer OFF, Warning/Critical LEDs OFF.
             * Heartbeat flash on Warning LED once every 2 seconds (tick % 20 == 0) */
            buzzer_set(false);
            led_critical_set(false);
            led_warning_set((tick_100ms % 20) == 0);
            break;

        case SYSTEM_STATE_WARNING:
            /* 2 Hz cadence (250ms ON, 250ms OFF -> toggle every 2-3 ticks of 100ms)
             * Tick cycle of 5 (500 ms period): ON for ticks 0-1, OFF for ticks 2-4 */
            {
                uint32_t phase = tick_100ms % 5;
                bool active = (phase < 2);
                buzzer_set(active);
                led_warning_set(active);
                led_critical_set(false);
            }
            break;

        case SYSTEM_STATE_CRITICAL:
            /* 5 Hz rapid urgent cadence (100ms ON, 100ms OFF)
             * Toggle every tick (tick % 2) */
            {
                bool active = (tick_100ms % 2) == 0;
                buzzer_set(active);
                led_warning_set(active);
                led_critical_set(active);
            }
            break;

        case SYSTEM_STATE_EMERGENCY:
            /* High-priority alternating hazard siren:
             * Alternating strobe between Warning and Critical LED, Buzzer ON continuously */
            {
                bool strobe = (tick_100ms % 2) == 0;
                buzzer_set(true); // Continuous alarm
                led_warning_set(strobe);
                led_critical_set(!strobe);
            }
            break;

        default:
            buzzer_led_all_off();
            break;
    }
}
