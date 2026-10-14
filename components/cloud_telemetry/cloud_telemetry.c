#include "cloud_telemetry.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "CLOUD_TELEMETRY";

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_mqtt_connected = false;

#define MQTT_TOPIC_TELEMETRY "vehicle/drowsiness/telemetry"
#define MQTT_TOPIC_ALERT     "vehicle/drowsiness/emergency"

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected to Broker.");
            s_mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT Disconnected from Broker.");
            s_mqtt_connected = false;
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Event Error");
            break;
        default:
            break;
    }
}

esp_err_t cloud_telemetry_wifi_init(const char *ssid, const char *password)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {0};
    if (ssid) strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password) strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Wi-Fi Station Initialized (SSID: %s)", ssid ? ssid : "[Not Configured]");
    return ESP_OK;
}

esp_err_t cloud_telemetry_mqtt_init(const char *broker_uri)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri ? broker_uri : "mqtt://broker.hivemq.com:1883",
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);

    ESP_LOGI(TAG, "MQTT Client Initialized (Broker: %s)", mqtt_cfg.broker.address.uri);
    return ESP_OK;
}

bool cloud_telemetry_is_connected(void)
{
    return s_mqtt_connected;
}

esp_err_t cloud_telemetry_publish_status(system_state_t state, float bpm, bool apnea,
                                        float steer_deg, float speed_pct,
                                        const gps_fix_data_t *gps)
{
    char json_buf[256];
    const char *state_str = (state == SYSTEM_STATE_NORMAL) ? "NORMAL" :
                            (state == SYSTEM_STATE_WARNING) ? "WARNING" :
                            (state == SYSTEM_STATE_CRITICAL) ? "CRITICAL" : "EMERGENCY";

    snprintf(json_buf, sizeof(json_buf),
             "{\"state\":\"%s\",\"bpm\":%.1f,\"apnea\":%s,\"steer_deg\":%.1f,\"speed_pct\":%.0f,\"lat\":%.6f,\"lon\":%.6f}",
             state_str,
             bpm,
             apnea ? "true" : "false",
             steer_deg,
             speed_pct,
             gps ? gps->latitude : 12.971598f,
             gps ? gps->longitude : 77.594566f);

    const char *topic = (state == SYSTEM_STATE_EMERGENCY) ? MQTT_TOPIC_ALERT : MQTT_TOPIC_TELEMETRY;
    int qos = (state == SYSTEM_STATE_EMERGENCY) ? 1 : 0;

    ESP_LOGI(TAG, "MQTT PUB [%s] (QoS %d): %s", topic, qos, json_buf);

    if (s_mqtt_client && s_mqtt_connected) {
        int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json_buf, 0, qos, 0);
        return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
    }
    return ESP_OK; // Simulation/offline success
}
