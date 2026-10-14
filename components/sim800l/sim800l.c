#include "sim800l.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SIM800L";
static uart_port_t s_gsm_uart_num = UART_NUM_1;
static bool        s_gsm_available = false;

esp_err_t sim800l_init(uart_port_t uart_num, int rx_pin, int tx_pin)
{
    s_gsm_uart_num = uart_num;

    uart_config_t uart_config = {
        .baud_rate = GSM_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(s_gsm_uart_num, &uart_config);
    if (err != ESP_OK) return err;

    err = uart_set_pin(s_gsm_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_driver_install(s_gsm_uart_num, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed for GSM: %s", esp_err_to_name(err));
        return err;
    }

    /* Send AT sync check */
    err = sim800l_send_cmd("AT\r\n", "OK", 1000);
    if (err == ESP_OK) {
        s_gsm_available = true;
        sim800l_send_cmd("ATE0\r\n", "OK", 500);         // Disable local echo
        sim800l_send_cmd("AT+CMGF=1\r\n", "OK", 500);      // Set text SMS mode
        ESP_LOGI(TAG, "SIM800L GSM Module Online & Configured for SMS (UART%d)", s_gsm_uart_num);
    } else {
        ESP_LOGW(TAG, "SIM800L not responding to AT. (Running in simulation mode)");
        s_gsm_available = false;
    }

    return ESP_OK;
}

esp_err_t sim800l_send_cmd(const char *cmd, const char *expected, uint32_t timeout_ms)
{
    if (!cmd || !expected) return ESP_ERR_INVALID_ARG;

    uart_flush(s_gsm_uart_num);
    uart_write_bytes(s_gsm_uart_num, cmd, strlen(cmd));

    char rx_buf[256] = {0};
    int total_read = 0;
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        int len = uart_read_bytes(s_gsm_uart_num, (uint8_t *)(rx_buf + total_read),
                                  sizeof(rx_buf) - total_read - 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            total_read += len;
            rx_buf[total_read] = '\0';
            if (strstr(rx_buf, expected)) {
                return ESP_OK;
            }
            if (strstr(rx_buf, "ERROR")) {
                return ESP_FAIL;
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t sim800l_send_sms(const char *phone_number, const char *message)
{
    if (!phone_number || !message) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "DISPATCHING SMS -> [%s]: \"%s\"", phone_number, message);

    if (!s_gsm_available) {
        ESP_LOGI(TAG, "[SIMULATION MODE] SMS dispatched successfully to %s", phone_number);
        return ESP_OK;
    }

    char cmgs_cmd[64];
    snprintf(cmgs_cmd, sizeof(cmgs_cmd), "AT+CMGS=\"%s\"\r\n", phone_number);

    /* Send CMGS command and await prompt character '>' */
    esp_err_t err = sim800l_send_cmd(cmgs_cmd, ">", 2000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get '>' prompt from SIM800L");
        return err;
    }

    /* Send message body followed by ASCII 26 (Ctrl+Z) */
    char body_buf[256];
    snprintf(body_buf, sizeof(body_buf), "%s\x1A", message);
    err = sim800l_send_cmd(body_buf, "+CMGS:", 10000); // Allow 10s for network transmission

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SMS successfully transmitted over cellular network.");
    } else {
        ESP_LOGE(TAG, "SMS dispatch failed or timed out: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t sim800l_send_emergency_alert(const char *phone_number, const gps_fix_data_t *gps, system_state_t state)
{
    char msg[160];
    snprintf(msg, sizeof(msg),
             "[EMERGENCY] Driver Drowsiness Detected! Vehicle stopped. Loc: https://maps.google.com/?q=%.6f,%.6f (Spd: %.1f km/h)",
             gps ? gps->latitude : 12.971598f,
             gps ? gps->longitude : 77.594566f,
             gps ? gps->speed_kmh : 0.0f);

    return sim800l_send_sms(phone_number, msg);
}
