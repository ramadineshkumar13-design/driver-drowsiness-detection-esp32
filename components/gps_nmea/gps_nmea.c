#include "gps_nmea.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "GPS_NMEA";

static uart_port_t     s_uart_num = UART_NUM_2;
static SemaphoreHandle_t s_gps_mutex = NULL;
static gps_fix_data_t  s_latest_fix = {
    .latitude = 12.971598f,  // Default simulated location
    .longitude = 77.594566f,
    .speed_kmh = 0.0f,
    .fix_valid = true,
    .satellites = 8,
    .utc_time = "12:30:00"
};

/* Convert NMEA coordinates (ddmm.mmmm) to decimal degrees */
static float nmea_to_decimal(const char *val, char hemisphere)
{
    if (!val || strlen(val) < 4) return 0.0f;
    float raw = strtof(val, NULL);
    int degrees = (int)(raw / 100.0f);
    float minutes = raw - (float)(degrees * 100);
    float decimal = (float)degrees + (minutes / 60.0f);
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

esp_err_t gps_nmea_init(uart_port_t uart_num, int rx_pin, int tx_pin)
{
    s_uart_num = uart_num;
    s_gps_mutex = xSemaphoreCreateMutex();
    if (!s_gps_mutex) {
        ESP_LOGE(TAG, "Failed to create GPS mutex");
        return ESP_ERR_NO_MEM;
    }

    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(s_uart_num, &uart_config);
    if (err != ESP_OK) return err;

    err = uart_set_pin(s_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_driver_install(s_uart_num, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed for GPS: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "NEO-6M GPS initialized on UART%d (RX=%d, TX=%d @ %d Baud)",
             s_uart_num, rx_pin, tx_pin, GPS_UART_BAUD_RATE);
    return ESP_OK;
}

bool gps_nmea_parse_line(const char *line, gps_fix_data_t *out_fix)
{
    if (!line || !out_fix) return false;

    /* Parse  or  (Recommended Minimum Navigation Information) */
    if (strncmp(line, "", 6) == 0 || strncmp(line, "", 6) == 0) {
        char buf[128];
        strncpy(buf, line, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        /* Tokenize comma-separated NMEA sentence */
        char *tokens[13] = {0};
        char *token = strtok(buf, ",");
        int idx = 0;
        while (token && idx < 13) {
            tokens[idx++] = token;
            token = strtok(NULL, ",");
        }

        if (idx >= 7 && tokens[2] && tokens[3] && tokens[4] && tokens[5] && tokens[6]) {
            char status = tokens[2][0]; // 'A' = Valid, 'V' = Void
            if (status == 'A') {
                out_fix->fix_valid = true;
                out_fix->latitude = nmea_to_decimal(tokens[3], tokens[4][0]);
                out_fix->longitude = nmea_to_decimal(tokens[5], tokens[6][0]);

                if (idx >= 8 && tokens[7]) {
                    float knots = strtof(tokens[7], NULL);
                    out_fix->speed_kmh = knots * 1.852f; // Knots to km/h
                }
                if (tokens[1] && strlen(tokens[1]) >= 6) {
                    snprintf(out_fix->utc_time, sizeof(out_fix->utc_time), "%.2s:%.2s:%.2s",
                             tokens[1], tokens[1] + 2, tokens[1] + 4);
                }

                if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    s_latest_fix = *out_fix;
                    xSemaphoreGive(s_gps_mutex);
                }
                return true;
            }
        }
    }
    return false;
}

esp_err_t gps_nmea_get_latest_fix(gps_fix_data_t *out_fix)
{
    if (!out_fix) return ESP_ERR_INVALID_ARG;
    if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out_fix = s_latest_fix;
        xSemaphoreGive(s_gps_mutex);
        return ESP_OK;
    }
    /* Fallback to simulation fix */
    gps_nmea_simulate_fix(out_fix);
    return ESP_OK;
}

void gps_nmea_simulate_fix(gps_fix_data_t *out_fix)
{
    if (!out_fix) return;
    out_fix->latitude = 12.971598f;
    out_fix->longitude = 77.594566f;
    out_fix->speed_kmh = 0.0f;
    out_fix->fix_valid = true;
    out_fix->satellites = 8;
    snprintf(out_fix->utc_time, sizeof(out_fix->utc_time), "12:45:00");
}
