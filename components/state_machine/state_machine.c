#include "state_machine.h"
#include "buzzer_led.h"
#include "motor_pwm.h"
#include "dsp_pipeline.h"

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
static const char *TAG = "STATE_MACHINE";
extern EventGroupHandle_t g_system_event_group;
#else
#include <stdio.h>
#define ESP_LOGI(tag, fmt, ...) printf("[INFO] [%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN] [%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERR]  [%s] " fmt "\n", tag, ##__VA_ARGS__)
void *g_system_event_group = NULL;
#endif

static system_state_t s_current_state = SYSTEM_STATE_NORMAL;
static system_state_t s_previous_state = SYSTEM_STATE_NORMAL;
static uint32_t       s_time_in_state_ms = 0;
static uint32_t       s_warning_timer_ms = 0;
static uint32_t       s_tick_counter = 0;

static const char* state_to_str(system_state_t s)
{
    switch (s) {
        case SYSTEM_STATE_NORMAL:    return "NORMAL";
        case SYSTEM_STATE_WARNING:   return "WARNING";
        case SYSTEM_STATE_CRITICAL:  return "CRITICAL";
        case SYSTEM_STATE_EMERGENCY: return "EMERGENCY";
        default:                     return "UNKNOWN";
    }
}

void state_machine_init(void)
{
    s_current_state = SYSTEM_STATE_NORMAL;
    s_previous_state = SYSTEM_STATE_NORMAL;
    s_time_in_state_ms = 0;
    s_warning_timer_ms = 0;
    s_tick_counter = 0;

    buzzer_led_all_off();
    motor_pwm_set_direction(MOTOR_DIR_FORWARD);
    motor_pwm_set_speed(100.0f);
    ESP_LOGI(TAG, "FSM Initialized: Initial State = NORMAL (Cruising Speed 100%%)");
}

system_state_t state_machine_get_state(void)
{
    return s_current_state;
}

uint32_t state_machine_get_time_in_state_ms(void)
{
    return s_time_in_state_ms;
}

static void transition_to(system_state_t new_state)
{
    if (new_state == s_current_state) return;

    ESP_LOGW(TAG, "STATE TRANSITION: [%s] -> [%s] (Spent %u ms in %s)",
             state_to_str(s_current_state),
             state_to_str(new_state),
             (unsigned int)s_time_in_state_ms,
             state_to_str(s_current_state));

    s_previous_state = s_current_state;
    s_current_state = new_state;
    s_time_in_state_ms = 0;

#ifdef ESP_PLATFORM
    if (g_system_event_group) {
        if (new_state == SYSTEM_STATE_WARNING) {
            xEventGroupSetBits(g_system_event_group, EVENT_BIT_STATE_WARNING);
        } else if (new_state == SYSTEM_STATE_CRITICAL) {
            xEventGroupSetBits(g_system_event_group, EVENT_BIT_STATE_CRITICAL);
        } else if (new_state == SYSTEM_STATE_EMERGENCY) {
            xEventGroupSetBits(g_system_event_group, EVENT_BIT_STATE_EMERGENCY);
        } else if (new_state == SYSTEM_STATE_NORMAL) {
            xEventGroupSetBits(g_system_event_group, EVENT_BIT_STATE_RECOVERED);
        }
    }
#endif

    /* State Entry Actions */
    switch (new_state) {
        case SYSTEM_STATE_NORMAL:
            s_warning_timer_ms = 0;
            buzzer_led_all_off();
            motor_pwm_set_speed(100.0f);
            dsp_pipeline_reset();
            break;

        case SYSTEM_STATE_WARNING:
            s_warning_timer_ms = 0;
            /* In WARNING, motor speed maintains cruising speed; buzzer/LED alerts driver */
            break;

        case SYSTEM_STATE_CRITICAL:
            /* Initiate smooth linear deceleration ramp down to 0% */
            motor_pwm_start_deceleration_ramp(0.0f, MOTOR_RAMP_DOWN_DURATION_MS);
            break;

        case SYSTEM_STATE_EMERGENCY:
            /* Vehicle is stationary; apply active brake and continuous hazard siren */
            motor_pwm_emergency_stop();
            break;

        default:
            break;
    }
}

system_state_t state_machine_step(const dsp_features_t *features, uint32_t delta_ms, bool *out_state_changed)
{
    s_time_in_state_ms += delta_ms;
    s_tick_counter++;
    bool changed = false;
    system_state_t start_state = s_current_state;

    /* Check Rapid Driver Recovery Condition:
     * If driver exhibits intentional steering movement (NOT inactive) OR restored normal breathing,
     * immediately recover back to NORMAL from WARNING or CRITICAL. */
    bool driver_recovered = (!features->steering_inactive) && (!features->irregular_breathing);

    switch (s_current_state) {
        case SYSTEM_STATE_NORMAL:
            /* Escalation to WARNING:
             * Irregular breathing AND Steering inactive simultaneously */
            if (features->irregular_breathing && features->steering_inactive) {
                transition_to(SYSTEM_STATE_WARNING);
            }
            break;

        case SYSTEM_STATE_WARNING:
            /* Check recovery first */
            if (driver_recovered || (!features->steering_inactive)) {
                ESP_LOGI(TAG, "Driver Recovery Detected in WARNING: Returning to NORMAL");
                transition_to(SYSTEM_STATE_NORMAL);
            } else {
                s_warning_timer_ms += delta_ms;
                /* If warning timeout expired without driver recovery, escalate to CRITICAL */
                if (s_warning_timer_ms >= TIMEOUT_WARNING_TO_CRITICAL_MS) {
                    ESP_LOGW(TAG, "Warning Timeout (%u ms) Expired with No Response: Escalate to CRITICAL",
                             (unsigned int)s_warning_timer_ms);
                    transition_to(SYSTEM_STATE_CRITICAL);
                }
            }
            break;

        case SYSTEM_STATE_CRITICAL:
            /* If driver makes an intentional steering jerk in CRITICAL, recover! */
            if (!features->steering_inactive) {
                ESP_LOGW(TAG, "Emergency Steering Override Detected in CRITICAL: Driver Aborted Deceleration");
                transition_to(SYSTEM_STATE_NORMAL);
            } else {
                /* Update motor deceleration ramp */
                bool ramp_done = motor_pwm_update_ramp(delta_ms);
                float speed = motor_pwm_get_current_speed();

                /* Once vehicle decelerates to zero speed, escalate to EMERGENCY */
                if (ramp_done || speed <= 0.01f) {
                    ESP_LOGE(TAG, "Vehicle Deceleration Complete (Speed 0%%): Escalate to EMERGENCY");
                    transition_to(SYSTEM_STATE_EMERGENCY);
                }
            }
            break;

        case SYSTEM_STATE_EMERGENCY:
            /* Vehicle is halted. Requires deliberate confirmed recovery or manual reset */
            if (!features->steering_inactive && !features->irregular_breathing && s_time_in_state_ms > 5000) {
                ESP_LOGI(TAG, "Confirmed Driver Recovery post-emergency: Returning to NORMAL");
                transition_to(SYSTEM_STATE_NORMAL);
            }
            break;

        default:
            transition_to(SYSTEM_STATE_NORMAL);
            break;
    }

    /* Update actuator patterns (buzzer tones & LED cadences) */
    buzzer_led_update_pattern(s_current_state, s_tick_counter);

    if (out_state_changed) {
        *out_state_changed = (s_current_state != start_state);
    }
    return s_current_state;
}

void state_machine_manual_reset(void)
{
    ESP_LOGI(TAG, "Manual Safety Reset Commanded");
    transition_to(SYSTEM_STATE_NORMAL);
}
