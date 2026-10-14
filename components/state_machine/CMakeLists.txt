idf_component_register(
    SRCS "state_machine.c" "core1_tasks.c"
    INCLUDE_DIRS "include"
    REQUIRES system_config ads1115 buzzer_led motor_pwm dsp_pipeline log freertos esp_timer
)
