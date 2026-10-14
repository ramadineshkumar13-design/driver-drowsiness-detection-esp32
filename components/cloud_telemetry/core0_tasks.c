#include "core0_tasks.h"
#include "gps_nmea.h"
#include "sim800l.h"
#include "cloud_telemetry.h"
#include "core1_tasks.h"
#include "state_machine.h"
#include "motor_pwm.h"
#include "dsp_pipeline.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

static const char *TAG = "CORE0_TASKS";
#define EMERGENCY_CONTACT_PHONE "+919876543210"

/* =========================================================================
 * 1. gps_gsm_task (Core 0, Priority 3) - NMEA UART stream & Emergency SMS
 * ========================================================================= */
static void gps_gsm_task(void *pvParameters)
{
    ESP_LOGI(TAG, "gps_gsm_task started on Core %d (Priority %d)",
             xPortGetCoreID(), uxTaskPriorityGet(NULL));

    char rx_line[128];
    gps_fix_data_t gps_fix = {0};

    while (1) {
        /* 1. Read UART sentences from GPS NEO-6M */
        int len = uart_read_bytes(CONFIG_GPS_UART_NUM, (uint8_t *)rx_line, sizeof(rx_line) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            rx_line[len] = '\0';
            gps_nmea_parse_line(rx_line, &gps_fix);
        }

        /* 2. Check for EMERGENCY trigger event bit from Core 1 */
        if (g_system_event_group) {
            EventBits_t bits = xEventGroupWaitBits(g_system_event_group,
                                                   EVENT_BIT_STATE_EMERGENCY,
                                                   pdTRUE,  // Clear bit on exit
                                                   pdFALSE, // Any bit
                                                   0);      // Non-blocking check
            if (bits & EVENT_BIT_STATE_EMERGENCY) {
                ESP_LOGE(TAG, "EMERGENCY EVENT BIT RECEIVED ON CORE 0! Dispatching Cellular Alert...");
                gps_nmea_get_latest_fix(&gps_fix);
                sim800l_send_emergency_alert(EMERGENCY_CONTACT_PHONE, &gps_fix, SYSTEM_STATE_EMERGENCY);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* =========================================================================
 * 2. cloud_task (Core 0, Priority 2) - MQTT Publishing Loop
 * ========================================================================= */
static void cloud_task(void *pvParameters)
{
    ESP_LOGI(TAG, "cloud_task started on Core %d (Priority %d)",
             xPortGetCoreID(), uxTaskPriorityGet(NULL));

    gps_fix_data_t gps_fix = {0};
    dsp_features_t features = {0};

    while (1) {
        /* Check for immediate state change events */
        EventBits_t bits = 0;
        if (g_system_event_group) {
            bits = xEventGroupWaitBits(g_system_event_group,
                                       EVENT_BIT_STATE_WARNING | EVENT_BIT_STATE_CRITICAL |
                                       EVENT_BIT_STATE_EMERGENCY | EVENT_BIT_STATE_RECOVERED,
                                       pdTRUE, pdFALSE, pdMS_TO_TICKS(2000)); // 2s periodic or event
            (void)bits;
        } else {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        /* Gather current system telemetry */
        system_state_t state = state_machine_get_state();
        gps_nmea_get_latest_fix(&gps_fix);
        float speed = motor_pwm_get_current_speed();

        if (g_feature_queue) {
            xQueuePeek(g_feature_queue, &features, 0);
        }

        /* Publish MQTT JSON payload */
        cloud_telemetry_publish_status(state,
                                       features.breathing_bpm,
                                       features.apnea_detected,
                                       features.steering_angle_deg,
                                       speed,
                                       &gps_fix);
    }
}

esp_err_t core0_tasks_start(void)
{
    /* 1. Initialize Core 0 hardware peripherals */
    gps_nmea_init(CONFIG_GPS_UART_NUM, CONFIG_GPS_UART_RX_GPIO, CONFIG_GPS_UART_TX_GPIO);
    sim800l_init(CONFIG_GSM_UART_NUM, CONFIG_GSM_UART_RX_GPIO, CONFIG_GSM_UART_TX_GPIO);
    cloud_telemetry_wifi_init("DriverSafety_AP", "SafetyPassword123");
    cloud_telemetry_mqtt_init("mqtt://broker.hivemq.com:1883");

    /* 2. Launch Core 0 Tasks */
    BaseType_t ret;
    ret = xTaskCreatePinnedToCore(gps_gsm_task, "gps_gsm_task",
                                  TASK_GPS_GSM_STACK_SIZE, NULL,
                                  TASK_GPS_GSM_PRIO, NULL, TASK_GPS_GSM_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to launch gps_gsm_task");
        return ESP_FAIL;
    }

    ret = xTaskCreatePinnedToCore(cloud_task, "cloud_task",
                                  TASK_CLOUD_STACK_SIZE, NULL,
                                  TASK_CLOUD_PRIO, NULL, TASK_CLOUD_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to launch cloud_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All Core 0 Telemetry & Comms Tasks successfully launched on Core 0.");
    return ESP_OK;
}
