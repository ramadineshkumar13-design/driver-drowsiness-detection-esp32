#ifndef SIM800L_H
#define SIM800L_H

#include "esp_err.h"
#include "driver/uart.h"
#include <stdbool.h>
#include "gps_nmea.h"
#include "system_types.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SIM800L GSM UART interface
 * @param uart_num UART port (typically UART_NUM_1)
 * @param rx_pin GPIO pin for UART RX
 * @param tx_pin GPIO pin for UART TX
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sim800l_init(uart_port_t uart_num, int rx_pin, int tx_pin);

/**
 * @brief Send an AT command and wait for expected response
 * @param cmd AT command string (including \r\n)
 * @param expected Expected response string (e.g. "OK")
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK if expected response was received
 */
esp_err_t sim800l_send_cmd(const char *cmd, const char *expected, uint32_t timeout_ms);

/**
 * @brief Dispatch an SMS text message to the specified recipient phone number
 * @param phone_number International format string (e.g. "+919876543210")
 * @param message Text message content (up to 160 characters)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sim800l_send_sms(const char *phone_number, const char *message);

/**
 * @brief Format and dispatch an emergency SMS including GPS location and vehicle status
 * @param phone_number Target recipient phone number
 * @param gps Pointer to latest GPS fix data
 * @param state Current system state
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sim800l_send_emergency_alert(const char *phone_number, const gps_fix_data_t *gps, system_state_t state);

#ifdef __cplusplus
}
#endif

#endif // SIM800L_H
