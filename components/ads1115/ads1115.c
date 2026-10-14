#include "ads1115.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "ADS1115";

#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01
#define ADS1115_REG_LO_THRESH   0x02
#define ADS1115_REG_HI_THRESH   0x03

/* Config Register Bit Definitions */
#define ADS1115_OS_START_SINGLE 0x8000
#define ADS1115_MUX_AIN0_GND    0x4000
#define ADS1115_MUX_AIN1_GND    0x5000
#define ADS1115_MUX_AIN2_GND    0x6000
#define ADS1115_MUX_AIN3_GND    0x7000
#define ADS1115_MODE_SINGLE     0x0100
#define ADS1115_DR_128SPS       0x0080
#define ADS1115_COMP_DISABLE    0x0003

static i2c_port_t s_i2c_port = I2C_NUM_0;
static uint8_t    s_dev_addr = ADS1115_I2C_ADDR;
static ads1115_pga_t s_active_pga = ADS1115_PGA_4_096V;
static bool       s_initialized = false;

esp_err_t ads1115_init(i2c_port_t i2c_port, uint8_t dev_addr)
{
    s_i2c_port = i2c_port;
    s_dev_addr = dev_addr;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = CONFIG_I2C_MASTER_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };

    esp_err_t err = i2c_param_config(s_i2c_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(s_i2c_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // Ignore if driver already installed
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Probe device address to verify physical connection */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ADS1115 detected at I2C address 0x%02X on SDA=%d, SCL=%d",
                 s_dev_addr, CONFIG_I2C_MASTER_SDA_GPIO, CONFIG_I2C_MASTER_SCL_GPIO);
        s_initialized = true;
    } else {
        ESP_LOGW(TAG, "ADS1115 not responding at 0x%02X (error: %s). Hardware may be absent.",
                 s_dev_addr, esp_err_to_name(err));
        /* Allow continuing in simulation mode */
        s_initialized = false;
    }

    return ESP_OK;
}

esp_err_t ads1115_read_raw(uint8_t channel, int16_t *out_raw)
{
    if (!out_raw) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        *out_raw = 0;
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t mux = (channel == 0) ? ADS1115_MUX_AIN0_GND : ADS1115_MUX_AIN1_GND;
    uint16_t config = ADS1115_OS_START_SINGLE |
                      mux |
                      s_active_pga |
                      ADS1115_MODE_SINGLE |
                      ADS1115_DR_128SPS |
                      ADS1115_COMP_DISABLE;

    /* Write config register */
    uint8_t write_buf[3] = {
        ADS1115_REG_CONFIG,
        (uint8_t)(config >> 8),
        (uint8_t)(config & 0xFF)
    };

    esp_err_t err = i2c_master_write_to_device(s_i2c_port, s_dev_addr,
                                                write_buf, sizeof(write_buf),
                                                pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    /* Wait for 128 SPS conversion (~8 ms) */
    vTaskDelay(pdMS_TO_TICKS(9));

    /* Read conversion register */
    uint8_t reg_addr = ADS1115_REG_CONVERSION;
    uint8_t read_buf[2] = {0};
    err = i2c_master_write_read_device(s_i2c_port, s_dev_addr,
                                       &reg_addr, 1,
                                       read_buf, 2,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    *out_raw = (int16_t)((read_buf[0] << 8) | read_buf[1]);
    return ESP_OK;
}

float ads1115_raw_to_voltage(int16_t raw, ads1115_pga_t pga)
{
    float lsb_volts = 0.000125f; // Default for +/- 4.096V
    switch (pga) {
        case ADS1115_PGA_6_144V: lsb_volts = 0.0001875f; break;
        case ADS1115_PGA_4_096V: lsb_volts = 0.000125f;  break;
        case ADS1115_PGA_2_048V: lsb_volts = 0.0000625f; break;
        case ADS1115_PGA_1_024V: lsb_volts = 0.00003125f; break;
        case ADS1115_PGA_0_512V: lsb_volts = 0.000015625f; break;
        case ADS1115_PGA_0_256V: lsb_volts = 0.0000078125f; break;
    }
    return (float)raw * lsb_volts;
}

esp_err_t ads1115_read_voltage(uint8_t channel, float *out_volts)
{
    if (!out_volts) return ESP_ERR_INVALID_ARG;
    int16_t raw = 0;
    esp_err_t err = ads1115_read_raw(channel, &raw);
    if (err != ESP_OK) {
        return err;
    }
    *out_volts = ads1115_raw_to_voltage(raw, s_active_pga);
    return ESP_OK;
}

void ads1115_simulate_sample(uint32_t tick_ms, float *out_breath_volts, float *out_steer_volts)
{
    if (!out_breath_volts || !out_steer_volts) return;

    /* 60-second repeatable test scenario:
     * 00s - 30s: NORMAL DRIVING (16 BPM regular breathing, active steering adjustments)
     * 30s - 45s: ROAD BUMP VIBRATION (Normal breathing + sudden shock bursts for artifact rejection)
     * 45s - 55s: APNEA + DROWSINESS (Breathing ceases/drops to flatline, steering locked static)
     * 55s - 60s: DRIVER SUDDEN RECOVERY (Steering wheel jerk, breathing gasps back to normal)
     */
    uint32_t cycle_ms = tick_ms % 60000;
    float time_sec = (float)tick_ms / 1000.0f;

    /* Baseline voltages */
    float base_breath = 1.65f;
    float base_steer  = 1.65f;

    if (cycle_ms < 30000) {
        /* Phase 1: Normal Driving @ 16 BPM (0.267 Hz) */
        float breath_freq = 0.267f; // 16 BPM
        float resp_wave = 0.40f * sinf(2.0f * (float)M_PI * breath_freq * time_sec);
        *out_breath_volts = base_breath + resp_wave + (0.01f * sinf(11.0f * time_sec));

        /* Active steering: smooth oscillation around center with natural jitter */
        *out_steer_volts = base_steer + (0.35f * sinf(0.35f * time_sec)) + (0.05f * sinf(3.1f * time_sec));
    }
    else if (cycle_ms < 45000) {
        /* Phase 2: Road Bump Artifacts: high-frequency large amplitude shock spikes */
        float breath_freq = 0.267f;
        float resp_wave = 0.40f * sinf(2.0f * (float)M_PI * breath_freq * time_sec);
        /* Inject high-variance vibration noise (bumps) */
        float bump_noise = (sinf(23.0f * time_sec) > 0.6f) ? 0.75f * sinf(45.0f * time_sec) : 0.02f;
        *out_breath_volts = base_breath + resp_wave + bump_noise;

        *out_steer_volts = base_steer + (0.20f * sinf(0.2f * time_sec));
    }
    else if (cycle_ms < 55000) {
        /* Phase 3: Apnea / Hypopnea Onset & Steering Inactivity (Drowsiness) */
        /* Breathing flatlines (no peaks) */
        *out_breath_volts = base_breath + (0.015f * sinf(0.1f * time_sec)); // Minimal drift, no peaks

        /* Steering completely motionless (deadband hold) */
        *out_steer_volts = base_steer; // Exact constant voltage
    }
    else {
        /* Phase 4: Driver Sudden Recovery (Wakes up, corrective steering twitch, deep breath) */
        *out_breath_volts = base_breath + 0.65f * sinf(2.0f * (float)M_PI * 0.4f * time_sec);
        /* Rapid angular steer jerk (+1.1V deflection) */
        *out_steer_volts = base_steer + 1.10f;
    }
}
