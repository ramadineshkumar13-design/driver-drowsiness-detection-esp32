#include "dsp_pipeline.h"
#include <math.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_log.h"
static const char *TAG = "DSP_PIPELINE";
#define DSP_LOGD(tag, fmt, ...) ESP_LOGD(tag, fmt, ##__VA_ARGS__)
#define DSP_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define DSP_LOGW(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#else
#include <stdio.h>
#define DSP_LOGD(tag, fmt, ...)
#define DSP_LOGI(tag, fmt, ...)
#define DSP_LOGW(tag, fmt, ...)
#endif

/* =========================================================================
 * Internal State Buffers
 * ========================================================================= */

/* Moving Average Low-Pass Filter State */
#define MA_SIZE DSP_MOVING_AVG_WINDOW_SIZE // 10 samples (0.5s @ 20Hz)
static float s_ma_buffer[MA_SIZE];
static uint32_t s_ma_idx = 0;
static float s_ma_sum = 0.0f;
static bool  s_ma_filled = false;
static float s_iir_smoothed = 1.65f; // Initial center baseline
static const float S_IIR_ALPHA = 0.20f; // Single-pole smoothing factor

/* Rolling Variance Buffer (1.0s = 20 samples) */
#define VAR_SIZE DSP_VARIANCE_WINDOW_SIZE
static float s_var_buffer[VAR_SIZE];
static uint32_t s_var_idx = 0;
static bool  s_var_filled = false;

/* Peak Detection State */
static float s_prev_sample_1 = 0.0f; // y[n-1]
static float s_prev_sample_2 = 0.0f; // y[n-2]
static uint32_t s_samples_since_last_peak = 0;
static uint32_t s_refractory_counter = 0;
static float s_baseline_estimate = 1.65f;
static float s_last_calculated_bpm = 16.0f; // Default normal baseline

/* Breath Interval Median Filter (stores last 4 intervals for stability) */
#define RECENT_INTERVALS_LEN 4
static float s_recent_intervals[RECENT_INTERVALS_LEN] = {3.75f, 3.75f, 3.75f, 3.75f}; // 16 BPM = 3.75s
static uint32_t s_interval_idx = 0;

/* Steering Inactivity Buffer (7.0s @ 20Hz = 140 samples) */
#define STEER_WINDOW_SAMPLES (STEERING_INACTIVE_TIMEOUT_SEC * SENSOR_SAMPLING_RATE_HZ)
static float s_steer_buffer[STEER_WINDOW_SAMPLES];
static uint32_t s_steer_idx = 0;
static bool  s_steer_filled = false;

/* =========================================================================
 * Implementation Functions
 * ========================================================================= */

void dsp_pipeline_init(void)
{
    dsp_pipeline_reset();
}

void dsp_pipeline_reset(void)
{
    memset(s_ma_buffer, 0, sizeof(s_ma_buffer));
    s_ma_idx = 0;
    s_ma_sum = 0.0f;
    s_ma_filled = false;
    s_iir_smoothed = 1.65f;

    memset(s_var_buffer, 0, sizeof(s_var_buffer));
    s_var_idx = 0;
    s_var_filled = false;

    s_prev_sample_1 = 0.0f;
    s_prev_sample_2 = 0.0f;
    s_samples_since_last_peak = 0;
    s_refractory_counter = 0;
    s_baseline_estimate = 1.65f;
    s_last_calculated_bpm = 16.0f;

    for (int i = 0; i < RECENT_INTERVALS_LEN; i++) {
        s_recent_intervals[i] = 3.75f;
    }
    s_interval_idx = 0;

    memset(s_steer_buffer, 0, sizeof(s_steer_buffer));
    s_steer_idx = 0;
    s_steer_filled = false;
}

float dsp_filter_step(float raw_input)
{
    /* 1. Update moving average circular buffer */
    s_ma_sum -= s_ma_buffer[s_ma_idx];
    s_ma_buffer[s_ma_idx] = raw_input;
    s_ma_sum += raw_input;

    s_ma_idx = (s_ma_idx + 1) % MA_SIZE;
    if (s_ma_idx == 0) s_ma_filled = true;

    uint32_t count = s_ma_filled ? MA_SIZE : s_ma_idx;
    float ma_out = (count > 0) ? (s_ma_sum / (float)count) : raw_input;

    /* 2. Exponential single-pole IIR low-pass smoothing */
    s_iir_smoothed = (S_IIR_ALPHA * ma_out) + ((1.0f - S_IIR_ALPHA) * s_iir_smoothed);

    return s_iir_smoothed;
}

float dsp_variance_step(float input_sample)
{
    s_var_buffer[s_var_idx] = input_sample;
    s_var_idx = (s_var_idx + 1) % VAR_SIZE;
    if (s_var_idx == 0) s_var_filled = true;

    uint32_t count = s_var_filled ? VAR_SIZE : s_var_idx;
    if (count < 2) return 0.0f;

    /* Calculate mean */
    float sum = 0.0f;
    for (uint32_t i = 0; i < count; i++) {
        sum += s_var_buffer[i];
    }
    float mean = sum / (float)count;

    /* Calculate variance sigma^2 */
    float var_sum = 0.0f;
    for (uint32_t i = 0; i < count; i++) {
        float diff = s_var_buffer[i] - mean;
        var_sum += (diff * diff);
    }

    return var_sum / (float)count;
}

bool dsp_peak_detect_step(float filtered_sample, bool is_artifact_active, float *out_bpm, bool *out_apnea)
{
    bool peak_found = false;
    s_samples_since_last_peak++;

    if (s_refractory_counter > 0) {
        s_refractory_counter--;
    }

    /* Slowly adapt baseline estimate (time constant ~ 5 seconds) */
    s_baseline_estimate = (0.01f * filtered_sample) + (0.99f * s_baseline_estimate);

    /* Three-point peak detection: y[n-1] is a local maximum if:
     * 1. y[n-1] > y[n] and y[n-1] > y[n-2]
     * 2. y[n-1] > baseline + threshold
     * 3. Refractory counter has expired
     * 4. Motion artifact is not corrupting the current window
     */
    if (!is_artifact_active && s_refractory_counter == 0) {
        float threshold = s_baseline_estimate + DSP_BREATH_PEAK_THRESHOLD_MIN;
        if (s_prev_sample_1 > s_prev_sample_2 &&
            s_prev_sample_1 > filtered_sample &&
            s_prev_sample_1 > threshold) {

            peak_found = true;
            s_refractory_counter = DSP_REFRACTORY_PERIOD_SAMPLES; // 1.2s refractory

            /* Calculate breath interval in seconds */
            float interval_sec = (float)s_samples_since_last_peak / (float)SENSOR_SAMPLING_RATE_HZ;
            s_samples_since_last_peak = 0; // Reset peak distance timer

            /* Filter intervals: ignore physiological impossibilities (> 60 BPM or < 5 BPM) */
            if (interval_sec >= 1.0f && interval_sec <= 12.0f) {
                s_recent_intervals[s_interval_idx] = interval_sec;
                s_interval_idx = (s_interval_idx + 1) % RECENT_INTERVALS_LEN;

                /* Sort a local copy to compute robust median */
                float sorted[RECENT_INTERVALS_LEN];
                memcpy(sorted, s_recent_intervals, sizeof(sorted));
                for (int i = 0; i < RECENT_INTERVALS_LEN - 1; i++) {
                    for (int j = i + 1; j < RECENT_INTERVALS_LEN; j++) {
                        if (sorted[i] > sorted[j]) {
                            float tmp = sorted[i];
                            sorted[i] = sorted[j];
                            sorted[j] = tmp;
                        }
                    }
                }
                float median_interval = (sorted[1] + sorted[2]) * 0.5f;
                if (median_interval > 0.01f) {
                    s_last_calculated_bpm = 60.0f / median_interval;
                }
            }
        }
    }

    /* Shift history */
    s_prev_sample_2 = s_prev_sample_1;
    s_prev_sample_1 = filtered_sample;

    /* Apnea detection: check if elapsed time without peak exceeds timeout */
    uint32_t apnea_threshold_samples = APNEA_DETECTION_TIMEOUT_SEC * SENSOR_SAMPLING_RATE_HZ;
    bool apnea = (s_samples_since_last_peak >= apnea_threshold_samples);

    if (apnea) {
        /* If apnea is active, decay estimated BPM down towards zero */
        s_last_calculated_bpm = 0.0f;
    }

    if (out_bpm) *out_bpm = s_last_calculated_bpm;
    if (out_apnea) *out_apnea = apnea;

    return peak_found;
}

bool dsp_steering_inactivity_step(float steer_voltage, float *out_angle_deg)
{
    /* Estimate angle in degrees: assumes 1.65V center (0 deg), 0V = -90 deg, 3.3V = +90 deg */
    if (out_angle_deg) {
        *out_angle_deg = ((steer_voltage - 1.65f) / 1.65f) * 90.0f;
    }

    s_steer_buffer[s_steer_idx] = steer_voltage;
    s_steer_idx = (s_steer_idx + 1) % STEER_WINDOW_SAMPLES;
    if (s_steer_idx == 0) s_steer_filled = true;

    uint32_t count = s_steer_filled ? STEER_WINDOW_SAMPLES : s_steer_idx;
    if (count < (SENSOR_SAMPLING_RATE_HZ * 2)) {
        return false; // Still priming buffer (first 2 seconds)
    }

    /* Find min and max voltage in the sliding window */
    float min_v = s_steer_buffer[0];
    float max_v = s_steer_buffer[0];
    for (uint32_t i = 1; i < count; i++) {
        if (s_steer_buffer[i] < min_v) min_v = s_steer_buffer[i];
        if (s_steer_buffer[i] > max_v) max_v = s_steer_buffer[i];
    }

    float delta_v = max_v - min_v;

    /* Inactivity triggered if voltage deviation is below deadband */
    return (delta_v < STEERING_DEADBAND_VOLTS);
}

bool dsp_pipeline_process(const sensor_raw_data_t *sample, dsp_features_t *out_features)
{
    if (!sample || !out_features) return false;

    out_features->timestamp_ms = sample->timestamp_ms;
    out_features->ccount_sample = sample->ccount;

    /* 1. Low-Pass Filtering */
    out_features->breathing_filtered = dsp_filter_step(sample->breathing_voltage);

    /* 2. Motion Artifact Rejection (Variance Gating) */
    out_features->breathing_variance = dsp_variance_step(sample->breathing_voltage);
    out_features->motion_artifact_active = (out_features->breathing_variance > DSP_MOTION_ARTIFACT_THRESHOLD);

    /* 3. Peak Detection & BPM */
    dsp_peak_detect_step(out_features->breathing_filtered,
                         out_features->motion_artifact_active,
                         &out_features->breathing_bpm,
                         &out_features->apnea_detected);

    /* 4. Irregular Breathing Evaluation */
    out_features->irregular_breathing = out_features->apnea_detected ||
                                       (out_features->breathing_bpm < BREATHING_NORMAL_MIN_BPM) ||
                                       (out_features->breathing_bpm > BREATHING_NORMAL_MAX_BPM);

    /* 5. Steering Inactivity Evaluation */
    out_features->steering_inactive = dsp_steering_inactivity_step(sample->steering_voltage,
                                                                   &out_features->steering_angle_deg);

    return true;
}
