#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include "system_types.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the state machine to SYSTEM_STATE_NORMAL
 */
void state_machine_init(void);

/**
 * @brief Process new DSP features and evaluate FSM transitions
 * @param features Pointer to latest DSP features computed by dsp_task
 * @param delta_ms Elapsed time in milliseconds since previous evaluation
 * @param out_state_changed Optional pointer returning true if a state transition occurred
 * @return Current system state after evaluation
 */
system_state_t state_machine_step(const dsp_features_t *features, uint32_t delta_ms, bool *out_state_changed);

/**
 * @brief Get the active system state
 * @return Active system state enum
 */
system_state_t state_machine_get_state(void);

/**
 * @brief Get elapsed time spent in the current state in milliseconds
 * @return Milliseconds in active state
 */
uint32_t state_machine_get_time_in_state_ms(void);

/**
 * @brief Perform a manual safety reset back to SYSTEM_STATE_NORMAL
 *        Restores motor speed, resets all alarms, and clears emergency bits.
 */
void state_machine_manual_reset(void);

#ifdef __cplusplus
}
#endif

#endif // STATE_MACHINE_H
