#ifndef ADS1115_H
#define ADS1115_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "system_types.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADS1115 Programmable Gain Amplifier (PGA) Full Scale Voltage Range
 */
typedef enum {
    ADS1115_PGA_6_144V = 0x0000, /**< +/- 6.144V range, LSB = 0.1875 mV */
    ADS1115_PGA_4_096V = 0x0200, /**< +/- 4.096V range, LSB = 0.125 mV (Default for 3.3V systems) */
    ADS1115_PGA_2_048V = 0x0400, /**< +/- 2.048V range, LSB = 0.0625 mV */
    ADS1115_PGA_1_024V = 0x0600, /**< +/- 1.024V range, LSB = 0.03125 mV */
    ADS1115_PGA_0_512V = 0x0800, /**< +/- 0.512V range, LSB = 0.015625 mV */
    ADS1115_PGA_0_256V = 0x0A00  /**< +/- 0.256V range, LSB = 0.0078125 mV */
} ads1115_pga_t;

/**
 * @brief Initialize I2C bus and ADS1115 ADC driver
 * @param i2c_port I2C peripheral port number (e.g. I2C_NUM_0)
 * @param dev_addr 7-bit I2C address (typically 0x48)
 * @return ESP_OK on success, or appropriate error code
 */
esp_err_t ads1115_init(i2c_port_t i2c_port, uint8_t dev_addr);

/**
 * @brief Perform a single-ended conversion on specified analog channel
 * @param channel ADS1115 channel index (0 = A0, 1 = A1)
 * @param out_raw Pointer to store signed 16-bit ADC conversion result
 * @return ESP_OK on success, or error code
 */
esp_err_t ads1115_read_raw(uint8_t channel, int16_t *out_raw);

/**
 * @brief Read analog channel and return measured voltage in Volts
 * @param channel ADS1115 channel index
 * @param out_volts Pointer to store measured voltage
 * @return ESP_OK on success, or error code
 */
esp_err_t ads1115_read_voltage(uint8_t channel, float *out_volts);

/**
 * @brief Convert raw ADC count to voltage based on configured PGA
 * @param raw Signed 16-bit ADC count
 * @param pga PGA configuration setting
 * @return Voltage in Volts
 */
float ads1115_raw_to_voltage(int16_t raw, ads1115_pga_t pga);

/**
 * @brief Synthetic signal generator for hardware-in-the-loop simulation mode
 * @param tick_ms Current system timestamp in milliseconds
 * @param out_breath_volts Pointer to store synthetic respiratory voltage
 * @param out_steer_volts Pointer to store synthetic steering voltage
 */
void ads1115_simulate_sample(uint32_t tick_ms, float *out_breath_volts, float *out_steer_volts);

#ifdef __cplusplus
}
#endif

#endif // ADS1115_H
