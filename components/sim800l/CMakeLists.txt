idf_component_register(
    SRCS "sim800l.c"
    INCLUDE_DIRS "include"
    REQUIRES driver system_config gps_nmea log freertos
)
