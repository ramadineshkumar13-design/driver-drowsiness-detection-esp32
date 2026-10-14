#ifndef BUZZER_LED_H
#define BUZZER_LED_H

#include "esp_err.h"
#include <stdbool.h>
#include "system_types.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Buzzer and LED GPIO output pins
 * @return ESP_OK on success
 */
esp_err_t buzzer_led_init(void);

/**
 * @brief Directly set Buzzer output state
 * @param on True to turn buzzer ON, False to turn OFF
 */
void buzzer_set(bool on);

/**
 * @brief Directly set Warning LED output state
 * @param on True to illuminate LED, False to extinguish
 */
void led_warning_set(bool on);

/**
 * @brief Directly set Critical LED output state
 * @param on True to illuminate LED, False to extinguish
 */
void led_critical_set(bool on);

/**
 * @brief Update audio/visual alert patterns based on state machine state
 *        Called periodically by alert_task every 100 ms tick.
 * @param state Current system state
 * @param tick_100ms Incremental 100ms tick counter for cadence timing
 */
void buzzer_led_update_pattern(system_state_t state, uint32_t tick_100ms);

/**
 * @brief Turn off all alert indicators (buzzer, warning LED, critical LED)
 */
void buzzer_led_all_off(void);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_LED_H
