#ifndef CORE0_TASKS_H
#define CORE0_TASKS_H

#include "esp_err.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and launch Core 0 Networking & Telemetry Tasks:
 *        - gps_gsm_task (Priority 3, Core 0)
 *        - cloud_task   (Priority 2, Core 0)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t core0_tasks_start(void);

#ifdef __cplusplus
}
#endif

#endif // CORE0_TASKS_H
