#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "sdkconfig.h"
#include "system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Hardware Pinout Configurations
 * ========================================================================= */
#ifndef CONFIG_I2C_MASTER_SDA_GPIO
#define CONFIG_I2C_MASTER_SDA_GPIO       21
#endif

#ifndef CONFIG_I2C_MASTER_SCL_GPIO
#define CONFIG_I2C_MASTER_SCL_GPIO       22
#endif

#define I2C_MASTER_NUM                   0         /**< I2C port number */
#define I2C_MASTER_FREQ_HZ               100000    /**< 100 kHz standard mode */
#define I2C_MASTER_TX_BUF_DISABLE        0
#define I2C_MASTER_RX_BUF_DISABLE        0
#define I2C_MASTER_TIMEOUT_MS            100

/* ADS1115 Device Parameters */
#define ADS1115_I2C_ADDR                 0x48      /**< ADDR pin to GND */
#define ADS1115_CH_BREATHING             0         /**< Channel A0 */
#define ADS1115_CH_STEERING              1         /**< Channel A1 */

/* L298N Motor Driver Pins */
#ifndef CONFIG_MOTOR_PWM_GPIO
#define CONFIG_MOTOR_PWM_GPIO            18        /**< ENA (PWM) Pin */
#endif
#ifndef CONFIG_MOTOR_IN1_GPIO
#define CONFIG_MOTOR_IN1_GPIO            19        /**< Direction Pin 1 */
#endif
#ifndef CONFIG_MOTOR_IN2_GPIO
#define CONFIG_MOTOR_IN2_GPIO            5         /**< Direction Pin 2 */
#endif

/* Buzzer and Status LED Pins */
#ifndef CONFIG_BUZZER_GPIO
#define CONFIG_BUZZER_GPIO               23
#endif
#ifndef CONFIG_LED_WARNING_GPIO
#define CONFIG_LED_WARNING_GPIO          2
#endif
#ifndef CONFIG_LED_CRITICAL_GPIO
#define CONFIG_LED_CRITICAL_GPIO         4
#endif

/* GPS NEO-6M UART Configuration */
#ifndef CONFIG_GPS_UART_NUM
#define CONFIG_GPS_UART_NUM              2
#endif
#ifndef CONFIG_GPS_UART_RX_GPIO
#define CONFIG_GPS_UART_RX_GPIO          16
#endif
#ifndef CONFIG_GPS_UART_TX_GPIO
#define CONFIG_GPS_UART_TX_GPIO          17
#endif
#define GPS_UART_BAUD_RATE               9600

/* GSM SIM800L UART Configuration */
#ifndef CONFIG_GSM_UART_NUM
#define CONFIG_GSM_UART_NUM              1
#endif
#ifndef CONFIG_GSM_UART_RX_GPIO
#define CONFIG_GSM_UART_RX_GPIO          27
#endif
#ifndef CONFIG_GSM_UART_TX_GPIO
#define CONFIG_GSM_UART_TX_GPIO          26
#endif
#define GSM_UART_BAUD_RATE               115200

/* =========================================================================
 * FreeRTOS Concurrency & Task Configuration
 * ========================================================================= */
#define TASK_SENSOR_CORE                 1
#define TASK_SENSOR_PRIO                 6
#define TASK_SENSOR_STACK_SIZE           3072

#define TASK_DSP_CORE                    1
#define TASK_DSP_PRIO                    5
#define TASK_DSP_STACK_SIZE              4096

#define TASK_ALERT_CORE                  1
#define TASK_ALERT_PRIO                  4
#define TASK_ALERT_STACK_SIZE            3584

#define TASK_GPS_GSM_CORE                0
#define TASK_GPS_GSM_PRIO                3
#define TASK_GPS_GSM_STACK_SIZE          4608

#define TASK_CLOUD_CORE                  0
#define TASK_CLOUD_PRIO                  2
#define TASK_CLOUD_STACK_SIZE            5120

#define TASK_DIAGNOSTICS_CORE            1
#define TASK_DIAGNOSTICS_PRIO            1
#define TASK_DIAGNOSTICS_STACK_SIZE      3072

#define SENSOR_QUEUE_LEN                 16
#define FEATURE_QUEUE_LEN                8

/* Sampling Parameters */
#define SENSOR_SAMPLING_RATE_HZ          20
#define SENSOR_SAMPLING_PERIOD_MS        (1000 / SENSOR_SAMPLING_RATE_HZ) // 50 ms

/* =========================================================================
 * DSP Algorithmic Thresholds & Timings
 * ========================================================================= */
#define DSP_MOVING_AVG_WINDOW_SIZE       10        /**< 10 samples = 0.5s filter */
#define DSP_VARIANCE_WINDOW_SIZE         20        /**< 20 samples = 1.0s window */
#define DSP_MOTION_ARTIFACT_THRESHOLD    0.08f     /**< Voltage variance threshold */

#define DSP_BREATH_PEAK_THRESHOLD_MIN    0.15f     /**< Peak detection baseline threshold */
#define DSP_REFRACTORY_PERIOD_SAMPLES    24        /**< 1.2s refractory @ 20Hz (max 50 BPM) */

#define BREATHING_NORMAL_MIN_BPM         10        /**< Bradypnea boundary */
#define BREATHING_NORMAL_MAX_BPM         30        /**< Tachypnea boundary */
#define APNEA_DETECTION_TIMEOUT_SEC      8         /**< Apnea duration trigger */

#define STEERING_DEADBAND_VOLTS          0.04f     /**< Deadband threshold in Volts */
#define STEERING_INACTIVE_TIMEOUT_SEC    7         /**< Inactivity duration trigger */

/* =========================================================================
 * State Machine Escalation Timeouts
 * ========================================================================= */
#define TIMEOUT_WARNING_TO_CRITICAL_MS   5000      /**< 5.0 seconds in WARNING before CRITICAL */
#define MOTOR_RAMP_DOWN_DURATION_MS      4000      /**< 4.0 seconds linear deceleration ramp */

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_CONFIG_H
