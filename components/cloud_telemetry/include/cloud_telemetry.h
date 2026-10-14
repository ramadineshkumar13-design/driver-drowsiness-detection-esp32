#ifndef CLOUD_TELEMETRY_H
#define CLOUD_TELEMETRY_H

#include "esp_err.h"
#include <stdbool.h>
#include "system_types.h"
#include "gps_nmea.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Wi-Fi station mode and attempt connection
 * @param ssid Wi-Fi Access Point SSID
 * @param password Wi-Fi password
 * @return esp_err_t ESP_OK on success
 */
esp_err_t cloud_telemetry_wifi_init(const char *ssid, const char *password);

/**
 * @brief Initialize MQTT client connecting to HiveMQ / ThingSpeak broker
 * @param broker_uri Broker URI (e.g. "mqtt://broker.hivemq.com:1883")
 * @return esp_err_t ESP_OK on success
 */
esp_err_t cloud_telemetry_mqtt_init(const char *broker_uri);

/**
 * @brief Publish telemetry JSON document to MQTT broker
 * @param state Current system state
 * @param bpm Breathing rate
 * @param apnea Apnea flag
 * @param steer_deg Steering angle
 * @param speed_pct Motor speed %
 * @param gps GPS fix data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t cloud_telemetry_publish_status(system_state_t state, float bpm, bool apnea,
                                        float steer_deg, float speed_pct,
                                        const gps_fix_data_t *gps);

/**
 * @brief Check if MQTT connection is active
 * @return true if connected to broker
 */
bool cloud_telemetry_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // CLOUD_TELEMETRY_H
