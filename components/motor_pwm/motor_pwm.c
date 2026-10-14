#include "motor_pwm.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MOTOR_PWM";

#define MOTOR_LEDC_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_TIMER        LEDC_TIMER_0
#define MOTOR_LEDC_CHANNEL      LEDC_CHANNEL_0
#define MOTOR_LEDC_DUTY_RES     LEDC_TIMER_10_BIT  // 0 to 1023
#define MOTOR_LEDC_MAX_DUTY     1023
#define MOTOR_LEDC_FREQ_HZ      20000              // 20 kHz ultrasonic (silent operation)

static float s_current_speed = 0.0f;
static float s_target_speed = 0.0f;
static float s_ramp_step_per_ms = 0.0f;
static bool  s_ramp_in_progress = false;

esp_err_t motor_pwm_init(void)
{
    /* 1. Configure Direction GPIOs */
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << CONFIG_MOTOR_IN1_GPIO) | (1ULL << CONFIG_MOTOR_IN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&dir_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure motor direction GPIOs: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. Configure LEDC Timer */
    ledc_timer_config_t timer_conf = {
        .speed_mode = MOTOR_LEDC_SPEED_MODE,
        .timer_num = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_DUTY_RES,
        .freq_hz = MOTOR_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 3. Configure LEDC Channel */
    ledc_channel_config_t chan_conf = {
        .gpio_num = CONFIG_MOTOR_PWM_GPIO,
        .speed_mode = MOTOR_LEDC_SPEED_MODE,
        .channel = MOTOR_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = MOTOR_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    err = ledc_channel_config(&chan_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Default initial state: Forward direction, 100% normal cruising speed */
    motor_pwm_set_direction(MOTOR_DIR_FORWARD);
    motor_pwm_set_speed(100.0f);

    ESP_LOGI(TAG, "L298N Motor PWM initialized (PWM Pin %d @ 20kHz, IN1=%d, IN2=%d)",
             CONFIG_MOTOR_PWM_GPIO, CONFIG_MOTOR_IN1_GPIO, CONFIG_MOTOR_IN2_GPIO);
    return ESP_OK;
}

esp_err_t motor_pwm_set_direction(motor_direction_t dir)
{
    switch (dir) {
        case MOTOR_DIR_FORWARD:
            gpio_set_level((gpio_num_t)CONFIG_MOTOR_IN1_GPIO, 1);
            gpio_set_level((gpio_num_t)CONFIG_MOTOR_IN2_GPIO, 0);
            break;
        case MOTOR_DIR_REVERSE:
            gpio_set_level((gpio_num_t)CONFIG_MOTOR_IN1_GPIO, 0);
            gpio_set_level((gpio_num_t)CONFIG_MOTOR_IN2_GPIO, 1);
            break;
        case MOTOR_DIR_BRAKE:
            gpio_set_level((gpio_num_t)CONFIG_MOTOR_IN1_GPIO, 1);
            gpio_set_level((gpio_num_t)CONFIG_MOTOR_IN2_GPIO, 1);
            break;
        case MOTOR_DIR_COAST:
        default:
            gpio_set_level((gpio_num_t)CONFIG_MOTOR_IN1_GPIO, 0);
            gpio_set_level((gpio_num_t)CONFIG_MOTOR_IN2_GPIO, 0);
            break;
    }
    return ESP_OK;
}

esp_err_t motor_pwm_set_speed(float percent)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    s_current_speed = percent;
    uint32_t duty = (uint32_t)((percent / 100.0f) * (float)MOTOR_LEDC_MAX_DUTY);

    esp_err_t err = ledc_set_duty(MOTOR_LEDC_SPEED_MODE, MOTOR_LEDC_CHANNEL, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(MOTOR_LEDC_SPEED_MODE, MOTOR_LEDC_CHANNEL);
    }
    return err;
}

float motor_pwm_get_current_speed(void)
{
    return s_current_speed;
}

void motor_pwm_start_deceleration_ramp(float target_percent, uint32_t duration_ms)
{
    if (target_percent < 0.0f) target_percent = 0.0f;
    if (target_percent > 100.0f) target_percent = 100.0f;
    if (duration_ms == 0) {
        motor_pwm_set_speed(target_percent);
        s_ramp_in_progress = false;
        return;
    }

    s_target_speed = target_percent;
    float delta = s_target_speed - s_current_speed;
    s_ramp_step_per_ms = delta / (float)duration_ms;
    s_ramp_in_progress = true;

    ESP_LOGI(TAG, "Motor Ramp Started: %.1f%% -> %.1f%% over %u ms",
             s_current_speed, s_target_speed, (unsigned int)duration_ms);
}

bool motor_pwm_update_ramp(uint32_t delta_time_ms)
{
    if (!s_ramp_in_progress) {
        return true; // Ramp already finished
    }

    float new_speed = s_current_speed + (s_ramp_step_per_ms * (float)delta_time_ms);

    /* Check completion boundary */
    if ((s_ramp_step_per_ms < 0.0f && new_speed <= s_target_speed) ||
        (s_ramp_step_per_ms > 0.0f && new_speed >= s_target_speed)) {
        motor_pwm_set_speed(s_target_speed);
        s_ramp_in_progress = false;
        ESP_LOGI(TAG, "Motor Ramp Complete: Final Speed = %.1f%%", s_target_speed);
        return true;
    }

    motor_pwm_set_speed(new_speed);
    return false;
}

void motor_pwm_emergency_stop(void)
{
    s_ramp_in_progress = false;
    motor_pwm_set_speed(0.0f);
    motor_pwm_set_direction(MOTOR_DIR_BRAKE);
    ESP_LOGW(TAG, "EMERGENCY STOP EXECUTED: Speed 0%% + Active Brake Applied");
}
