idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES system_config ads1115 buzzer_led motor_pwm dsp_pipeline state_machine gps_nmea sim800l cloud_telemetry esp_system esp_timer log freertos
)
