#ifndef DSP_PIPELINE_H
#define DSP_PIPELINE_H

#include <stdint.h>
#include <stdbool.h>
#include "system_types.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize all DSP filter states, variance buffers, and peak detection timers
 */
void dsp_pipeline_init(void);

/**
 * @brief Reset internal filter buffers and timers (called on state recovery/reset)
 */
void dsp_pipeline_reset(void);

/**
 * @brief Primary signal processing pipeline entrypoint.
 *        Executes filter -> variance -> artifact gating -> peak detect -> BPM -> steering check.
 * @param sample Pointer to input raw sensor sample (acquired at 20 Hz)
 * @param out_features Pointer to output feature structure to be filled
 * @return true if processing succeeded
 */
bool dsp_pipeline_process(const sensor_raw_data_t *sample, dsp_features_t *out_features);

/* =========================================================================
 * Modular sub-functions exposed for isolated unit testing
 * ========================================================================= */

/**
 * @brief Apply cascaded moving-average and IIR low-pass smoothing filter
 * @param raw_input New raw respiratory voltage sample
 * @return Filtered sample value
 */
float dsp_filter_step(float raw_input);

/**
 * @brief Compute rolling variance over 1.0 second sliding window (20 samples)
 * @param input_sample Current voltage sample
 * @return Rolling variance sigma^2 in V^2
 */
float dsp_variance_step(float input_sample);

/**
 * @brief Peak detection and instantaneous BPM estimator
 * @param filtered_sample Smoothed respiratory sample
 * @param is_artifact_active If true, peak detection is gated (ignored)
 * @param out_bpm Pointer to output calculated BPM
 * @param out_apnea Pointer to output apnea flag (true if no peak for >= timeout)
 * @return true if a valid respiratory peak was detected at this sample
 */
bool dsp_peak_detect_step(float filtered_sample, bool is_artifact_active, float *out_bpm, bool *out_apnea);

/**
 * @brief Evaluate steering inactivity over sliding window (7.0 seconds = 140 samples)
 * @param steer_voltage Current steering sensor analog voltage
 * @param out_angle_deg Pointer to output estimated steering angle (-90 to +90 deg)
 * @return true if steering inactivity is detected (voltage delta < deadband)
 */
bool dsp_steering_inactivity_step(float steer_voltage, float *out_angle_deg);

#ifdef __cplusplus
}
#endif

#endif // DSP_PIPELINE_H
