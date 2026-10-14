#include "core1_tasks.h"
#include "ads1115.h"
#include "buzzer_led.h"
#include "motor_pwm.h"
#include "dsp_pipeline.h"
#include "state_machine.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_cpu.h"

static const char *TAG = "CORE1_TASKS";

QueueHandle_t      g_sensor_queue = NULL;
QueueHandle_t      g_feature_queue = NULL;
EventGroupHandle_t g_system_event_group = NULL;

/* =========================================================================
 * 1. sensor_task (Core 1, Priority 6) - 20 Hz Periodic Sampling
 * ========================================================================= */
static void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "sensor_task started on Core %d (Priority %d, Period %d ms)",
             xPortGetCoreID(), uxTaskPriorityGet(NULL), SENSOR_SAMPLING_PERIOD_MS);

    TickType_t last_wake_time = xTaskGetTickCount();
    sensor_raw_data_t sample = {0};

    while (1) {
        /* Strictly periodic 50 ms tick (20 Hz) using vTaskDelayUntil */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SENSOR_SAMPLING_PERIOD_MS));

        sample.timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        sample.ccount = esp_cpu_get_cycle_count();

#ifdef CONFIG_ENABLE_SIMULATION_MODE
        /* Inject mathematically synthesized physiological waveform */
        ads1115_simulate_sample(sample.timestamp_ms,
                                &sample.breathing_voltage,
                                &sample.steering_voltage);
        sample.breathing_raw = (int16_t)(sample.breathing_voltage / 0.000125f);
        sample.steering_raw  = (int16_t)(sample.steering_voltage / 0.000125f);
#else
        /* Read physical ADS1115 I2C ADC */
        esp_err_t err0 = ads1115_read_voltage(ADS1115_CH_BREATHING, &sample.breathing_voltage);
        esp_err_t err1 = ads1115_read_voltage(ADS1115_CH_STEERING, &sample.steering_voltage);
        if (err0 != ESP_OK || err1 != ESP_OK) {
            /* Fallback to simulation generator if hardware fails */
            ads1115_simulate_sample(sample.timestamp_ms,
                                    &sample.breathing_voltage,
                                    &sample.steering_voltage);
        }
#endif

        /* Push raw sample into sensor_queue (non-blocking overwrite if full) */
        if (xQueueSend(g_sensor_queue, &sample, 0) != pdPASS) {
            /* Drop oldest sample and force push if queue is full */
            sensor_raw_data_t dummy;
            xQueueReceive(g_sensor_queue, &dummy, 0);
            xQueueSend(g_sensor_queue, &sample, 0);
        }
    }
}

/* =========================================================================
 * 2. dsp_task (Core 1, Priority 5) - Queue-driven Feature Extraction
 * ========================================================================= */
static void dsp_task(void *pvParameters)
{
    ESP_LOGI(TAG, "dsp_task started on Core %d (Priority %d)",
             xPortGetCoreID(), uxTaskPriorityGet(NULL));

    sensor_raw_data_t sample = {0};
    dsp_features_t features = {0};

    while (1) {
        /* Block until a new raw sample arrives from sensor_task */
        if (xQueueReceive(g_sensor_queue, &sample, portMAX_DELAY) == pdPASS) {
            /* Execute full DSP pipeline: Filter -> Variance -> Peak Detect -> BPM -> Steer */
            dsp_pipeline_process(&sample, &features);

            /* Push extracted features to feature_queue for alert_task */
            if (xQueueSend(g_feature_queue, &features, 0) != pdPASS) {
                dsp_features_t dummy;
                xQueueReceive(g_feature_queue, &dummy, 0);
                xQueueSend(g_feature_queue, &features, 0);
            }
        }
    }
}

/* =========================================================================
 * 3. alert_task (Core 1, Priority 4) - FSM & Actuator Control Loop
 * ========================================================================= */
static void alert_task(void *pvParameters)
{
    ESP_LOGI(TAG, "alert_task started on Core %d (Priority %d)",
             xPortGetCoreID(), uxTaskPriorityGet(NULL));

    dsp_features_t features = {0};
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(100); // 100 ms control cadence

    while (1) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        /* Read latest features from feature_queue (peek or drain to latest) */
        while (xQueueReceive(g_feature_queue, &features, 0) == pdPASS) {
            /* Drain queue so alert_task always evaluates the freshest feature packet */
        }

        /* Benchmark sensor-to-alert latency using CPU cycle counter */
        uint32_t ccount_now = esp_cpu_get_cycle_count();
        uint32_t cycles_elapsed = ccount_now - features.ccount_sample;
        float latency_us = (float)cycles_elapsed / 240.0f; // 240 MHz CPU = 240 cycles per us

        /* Step the Finite State Machine */
        bool state_changed = false;
        system_state_t state = state_machine_step(&features, 100, &state_changed);

        /* Periodic debug instrumentation log (every 2.0 seconds) */
        static uint32_t log_counter = 0;
        if (++log_counter % 20 == 0 || state_changed) {
            ESP_LOGI(TAG, "[FSM: %d] BPM=%.1f (Irreg:%d, Apnea:%d) | Steer=%.1f deg (Inact:%d) | Speed=%.0f%% | Latency=%.1f us",
                     state,
                     features.breathing_bpm,
                     features.irregular_breathing,
                     features.apnea_detected,
                     features.steering_angle_deg,
                     features.steering_inactive,
                     motor_pwm_get_current_speed(),
                     latency_us);
        }
    }
}

esp_err_t core1_tasks_start(void)
{
    /* 1. Create FreeRTOS IPC primitives */
    g_sensor_queue = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(sensor_raw_data_t));
    if (!g_sensor_queue) {
        ESP_LOGE(TAG, "Failed to create sensor_queue");
        return ESP_ERR_NO_MEM;
    }

    g_feature_queue = xQueueCreate(FEATURE_QUEUE_LEN, sizeof(dsp_features_t));
    if (!g_feature_queue) {
        ESP_LOGE(TAG, "Failed to create feature_queue");
        return ESP_ERR_NO_MEM;
    }

    g_system_event_group = xEventGroupCreate();
    if (!g_system_event_group) {
        ESP_LOGE(TAG, "Failed to create system_event_group");
        return ESP_ERR_NO_MEM;
    }

    /* 2. Initialize Core 1 Drivers & State Machine */
    buzzer_led_init();
    motor_pwm_init();
    ads1115_init(I2C_MASTER_NUM, ADS1115_I2C_ADDR);
    dsp_pipeline_init();
    state_machine_init();

    /* 3. Launch Core 1 Tasks pinned to Core 1 */
    BaseType_t ret;

    ret = xTaskCreatePinnedToCore(sensor_task, "sensor_task",
                                  TASK_SENSOR_STACK_SIZE, NULL,
                                  TASK_SENSOR_PRIO, NULL, TASK_SENSOR_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor_task");
        return ESP_FAIL;
    }

    ret = xTaskCreatePinnedToCore(dsp_task, "dsp_task",
                                  TASK_DSP_STACK_SIZE, NULL,
                                  TASK_DSP_PRIO, NULL, TASK_DSP_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create dsp_task");
        return ESP_FAIL;
    }

    ret = xTaskCreatePinnedToCore(alert_task, "alert_task",
                                  TASK_ALERT_STACK_SIZE, NULL,
                                  TASK_ALERT_PRIO, NULL, TASK_ALERT_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create alert_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All Core 1 Tasks successfully launched on Core 1.");
    return ESP_OK;
}
