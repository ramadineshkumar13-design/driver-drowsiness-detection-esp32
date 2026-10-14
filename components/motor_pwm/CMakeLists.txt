idf_component_register(
    SRCS "motor_pwm.c"
    INCLUDE_DIRS "include"
    REQUIRES driver system_config log
)
