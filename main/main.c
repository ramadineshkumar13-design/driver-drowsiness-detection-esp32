/**
 * @file main.c
 * @brief Driver Drowsiness Detection & Emergency Response System
 *        Main application entry point for ESP32 (ESP-IDF v5.x)
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "system_config.h"
#include "system_types.h"
#include "core1_tasks.h"
#include "core0_tasks.h"
#include "state_machine.h"
#include "motor_pwm.h"
#include "dsp_pipeline.h"

static const char *TAG = "MAIN_APP";

#define TWDT_TIMEOUT_MS 5000 // 5.0 seconds Watchdog timeout

/* Task handles for stack high-water mark profiling */
extern QueueHandle_t g_sensor_queue;
extern QueueHandle_t g_feature_queue;

/**
 * @brief Periodic Diagnostic and Health Monitor Task (Priority 1, Core 1)
 *        Runs every 5.0 seconds. Profiles stack high-water marks, free heap,
 *        and resets the Task Watchdog Timer.
 */
static void diagnostics_task(void *pvParameters)
{
    ESP_LOGI(TAG, "diagnostics_task initialized on Core %d (Priority %d)",
             xPortGetCoreID(), uxTaskPriorityGet(NULL));

    /* Register diagnostics_task with Task Watchdog */
    esp_task_wdt_add(NULL);

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(2000); // 2.0s period (well within 5s TWDT)

    while (1) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        /* 1. Feed the Task Watchdog Timer */
        esp_task_wdt_reset();

        /* 2. Heap and Memory Profiling */
        uint32_t free_heap = esp_get_free_heap_size();
        uint32_t min_free_heap = esp_get_minimum_free_heap_size();

        /* 3. Query Active FSM State & Vehicle Speed */
        system_state_t state = state_machine_get_state();
        float speed = motor_pwm_get_current_speed();
        uint32_t time_in_state = state_machine_get_time_in_state_ms();

        /* 4. Queue Utilization */
        UBaseType_t sensor_q_spaces = g_sensor_queue ? uxQueueSpacesAvailable(g_sensor_queue) : 0;
        UBaseType_t feature_q_spaces = g_feature_queue ? uxQueueSpacesAvailable(g_feature_queue) : 0;

        /* Print Formatted Diagnostic Health Report */
        ESP_LOGI("DIAGNOSTICS", "===================================================");
        ESP_LOGI("DIAGNOSTICS", " SYSTEM HEALTH & RESOURCE TELEMETRY               ");
        ESP_LOGI("DIAGNOSTICS", "---------------------------------------------------");
        ESP_LOGI("DIAGNOSTICS", " FSM State: %d | Dwell Time: %u ms | Speed: %.1f%%",
                 state, (unsigned int)time_in_state, speed);
        ESP_LOGI("DIAGNOSTICS", " Free Heap: %u bytes | Min Ever Free: %u bytes",
                 (unsigned int)free_heap, (unsigned int)min_free_heap);
        ESP_LOGI("DIAGNOSTICS", " Sensor Queue Free: %u/%d | Feature Queue Free: %u/%d",
                 (unsigned int)sensor_q_spaces, SENSOR_QUEUE_LEN,
                 (unsigned int)feature_q_spaces, FEATURE_QUEUE_LEN);
        ESP_LOGI("DIAGNOSTICS", " Diagnostics Task Stack High-Water: %u bytes",
                 (unsigned int)uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGI("DIAGNOSTICS", "===================================================");
    }
}

/**
 * @brief Initialize Task Watchdog Timer (TWDT) across ESP32 cores
 */
static esp_err_t init_task_watchdog(void)
{
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = TWDT_TIMEOUT_MS,
        .idle_core_mask = (1 << 0) | (1 << 1), // Monitor Idle tasks on Core 0 & Core 1
        .trigger_panic = true                 // Trigger hard reboot upon task hang
    };

    esp_err_t err = esp_task_wdt_reconfigure(&twdt_config);
    if (err != ESP_OK) {
        err = esp_task_wdt_init(&twdt_config);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Task Watchdog Timer (TWDT) enabled (%d ms timeout, Panic on timeout)",
                 TWDT_TIMEOUT_MS);
    } else {
        ESP_LOGW(TAG, "TWDT init/reconfigure status: %s", esp_err_to_name(err));
    }
    return err;
}

/**
 * @brief Application Entry Point
 */
void app_main(void)
{
    /* Print Startup Banner */
    printf("\n");
    printf("======================================================================\n");
    printf("  DRIVER DROWSINESS DETECTION & EMERGENCY RESPONSE SYSTEM             \n");
    printf("  Firmware: ESP-IDF v5.x | FreeRTOS Dual-Core Architecture            \n");
    printf("======================================================================\n");

    /* Print Chip Details */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("  SoC Target:       ESP32 (Revision %d)\n", chip_info.revision);
    printf("  CPU Cores:        %d Cores @ 240 MHz\n", chip_info.cores);
    printf("  Flash Memory:     %s\n", (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Embedded" : "External SPI");
    printf("  Free Heap Boot:   %u bytes\n", (unsigned int)esp_get_free_heap_size());
#ifdef CONFIG_ENABLE_SIMULATION_MODE
    printf("  Operating Mode:   SIMULATION MODE (Synthetic Bio-Signals Active)\n");
#else
    printf("  Operating Mode:   HARDWARE MODE (ADS1115 I2C Physical Sampling)\n");
#endif
    printf("======================================================================\n\n");

    /* 1. Initialize Task Watchdog Timer */
    init_task_watchdog();

    /* 2. Launch Core 1 Tasks (Hard Real-Time Physical Sensing & Control) */
    ESP_LOGI(TAG, "Starting Core 1 Real-Time Sensing & DSP Pipeline...");
    esp_err_t err = core1_tasks_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: Failed to start Core 1 tasks: %s", esp_err_to_name(err));
        return;
    }

    /* 3. Launch Core 0 Tasks (Networking, GPS NMEA, SIM800L GSM, MQTT) */
    ESP_LOGI(TAG, "Starting Core 0 Networking & Telemetry Tasks...");
    err = core0_tasks_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: Failed to start Core 0 tasks: %s", esp_err_to_name(err));
        return;
    }

    /* 4. Launch Health & Diagnostics Monitor Task on Core 1 (Priority 1) */
    xTaskCreatePinnedToCore(diagnostics_task, "diagnostics_task",
                            TASK_DIAGNOSTICS_STACK_SIZE, NULL,
                            TASK_DIAGNOSTICS_PRIO, NULL, TASK_DIAGNOSTICS_CORE);

    ESP_LOGI(TAG, "All Subsystems Initialized. System actively monitoring driver.");
}
