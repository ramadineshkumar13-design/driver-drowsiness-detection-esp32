#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "esp_err.h"
#include <stdbool.h>
#include "system_types.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize L298N LEDC hardware PWM and directional GPIO pins
 * @return ESP_OK on success
 */
esp_err_t motor_pwm_init(void);

/**
 * @brief Set H-Bridge direction (Forward, Reverse, Brake, Coast)
 * @param dir Direction enum
 * @return ESP_OK on success
 */
esp_err_t motor_pwm_set_direction(motor_direction_t dir);

/**
 * @brief Set instantaneous motor PWM duty cycle percentage (0.0% to 100.0%)
 * @param percent Duty cycle percentage
 * @return ESP_OK on success
 */
esp_err_t motor_pwm_set_speed(float percent);

/**
 * @brief Get currently commanded motor speed percentage
 * @return Current speed in percentage (0.0 to 100.0)
 */
float motor_pwm_get_current_speed(void);

/**
 * @brief Initiate a non-blocking linear speed deceleration/acceleration ramp
 * @param target_percent Desired end speed percentage (e.g. 0.0% to stop)
 * @param duration_ms Total duration over which the ramp should execute
 */
void motor_pwm_start_deceleration_ramp(float target_percent, uint32_t duration_ms);

/**
 * @brief Periodic ramp progression update tick.
 *        Called by alert_task every 100 ms to smoothly transition duty cycle.
 * @param delta_time_ms Elapsed time since last update (typically 100 ms)
 * @return True if target speed has been reached, False if ramp still in progress
 */
bool motor_pwm_update_ramp(uint32_t delta_time_ms);

/**
 * @brief Immediate hard stop: sets 0% PWM duty and applies active H-Bridge brake
 */
void motor_pwm_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_PWM_H
