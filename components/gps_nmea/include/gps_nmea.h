#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include "esp_err.h"
#include "driver/uart.h"
#include <stdbool.h>
#include <stdint.h>
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Extracted GPS Fix Data
 */
typedef struct {
    float    latitude;       /**< Decimal degrees (+N, -S) */
    float    longitude;      /**< Decimal degrees (+E, -W) */
    float    speed_kmh;      /**< Speed over ground in km/h */
    bool     fix_valid;      /**< True if GPS satellite lock is confirmed */
    uint8_t  satellites;     /**< Number of satellites in view */
    char     utc_time[12];   /**< HH:MM:SS format string */
} gps_fix_data_t;

/**
 * @brief Initialize NEO-6M GPS UART peripheral
 * @param uart_num UART port (typically UART_NUM_2)
 * @param rx_pin GPIO pin for UART RX
 * @param tx_pin GPIO pin for UART TX
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gps_nmea_init(uart_port_t uart_num, int rx_pin, int tx_pin);

/**
 * @brief Parse a single NMEA-0183 sentence ( or )
 * @param line Raw ASCII sentence string
 * @param out_fix Pointer to structure where parsed values are written
 * @return true if a valid sentence was parsed
 */
bool gps_nmea_parse_line(const char *line, gps_fix_data_t *out_fix);

/**
 * @brief Thread-safe query of the latest valid GPS fix
 * @param out_fix Pointer to destination structure
 * @return esp_err_t ESP_OK if a fix is present
 */
esp_err_t gps_nmea_get_latest_fix(gps_fix_data_t *out_fix);

/**
 * @brief Generate a realistic synthetic GPS coordinate (for simulation / indoor testing)
 * @param out_fix Pointer to destination structure
 */
void gps_nmea_simulate_fix(gps_fix_data_t *out_fix);

#ifdef __cplusplus
}
#endif

#endif // GPS_NMEA_H
