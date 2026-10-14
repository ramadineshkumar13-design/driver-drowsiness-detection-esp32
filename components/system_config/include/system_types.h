#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief System Operating States for Hierarchical FSM
 */
typedef enum {
    SYSTEM_STATE_NORMAL = 0,    /**< Normal driving, regular breathing, active steering */
    SYSTEM_STATE_WARNING = 1,   /**< Irregular breathing & steering inactive; audio/visual alert */
    SYSTEM_STATE_CRITICAL = 2,  /**< Persistent drowsiness; automated PWM motor deceleration */
    SYSTEM_STATE_EMERGENCY = 3  /**< Vehicle stopped; GPS capture, SMS dispatch, cloud alert */
} system_state_t;

/**
 * @brief Raw sensor sample acquired by sensor_task
 */
typedef struct {
    uint32_t timestamp_ms;      /**< System tick timestamp in milliseconds */
    uint32_t ccount;            /**< CPU cycle counter at exact time of sampling */
    int16_t  breathing_raw;     /**< Raw 16-bit ADC value from ADS1115 A0 (FSR/Flex) */
    int16_t  steering_raw;      /**< Raw 16-bit ADC value from ADS1115 A1 (Steering) */
    float    breathing_voltage; /**< Converted voltage (Volts) */
    float    steering_voltage;  /**< Converted voltage (Volts) */
} sensor_raw_data_t;

/**
 * @brief Extracted DSP features computed by dsp_task
 */
typedef struct {
    uint32_t timestamp_ms;           /**< Feature computation timestamp */
    uint32_t ccount_sample;          /**< Original sample CPU cycle counter for latency profiling */
    float    breathing_filtered;     /**< Low-pass filtered respiratory signal */
    float    breathing_variance;     /**< Rolling variance for motion artifact rejection */
    float    breathing_bpm;          /**< Instantaneous respiratory rate in Breaths Per Minute */
    bool     motion_artifact_active; /**< True if road bump / shock artifact is detected */
    bool     irregular_breathing;    /**< True if BPM is outside healthy range or apnea present */
    bool     apnea_detected;         /**< True if no breath peak detected for >= timeout */
    bool     steering_inactive;      /**< True if steering angular deviation is below deadband */
    float    steering_angle_deg;     /**< Estimated steering angle in degrees (-90 to +90) */
} dsp_features_t;

/**
 * @brief Motor driver direction enumeration
 */
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE,
    MOTOR_DIR_BRAKE,
    MOTOR_DIR_COAST
} motor_direction_t;

/**
 * @brief Event Group bitmask definitions for inter-task synchronization
 */
#define EVENT_BIT_STATE_WARNING     (1 << 0)
#define EVENT_BIT_STATE_CRITICAL    (1 << 1)
#define EVENT_BIT_STATE_EMERGENCY   (1 << 2)
#define EVENT_BIT_STATE_RECOVERED   (1 << 3)
#define EVENT_BIT_GPS_FIX_READY     (1 << 4)
#define EVENT_BIT_SMS_SENT          (1 << 5)
#define EVENT_BIT_MQTT_CONNECTED    (1 << 6)

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_TYPES_H
