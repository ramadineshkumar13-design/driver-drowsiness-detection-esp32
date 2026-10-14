#ifndef CORE1_TASKS_H
#define CORE1_TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "system_types.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Global handles accessible across tasks */
extern QueueHandle_t      g_sensor_queue;
extern QueueHandle_t      g_feature_queue;
extern EventGroupHandle_t g_system_event_group;

/**
 * @brief Initialize FreeRTOS queues, event groups, and launch all Core 1 tasks:
 *        - sensor_task (Priority 6, Core 1)
 *        - dsp_task    (Priority 5, Core 1)
 *        - alert_task  (Priority 4, Core 1)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t core1_tasks_start(void);

#ifdef __cplusplus
}
#endif

#endif // CORE1_TASKS_H
