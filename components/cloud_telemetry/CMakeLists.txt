idf_component_register(
    SRCS "cloud_telemetry.c" "core0_tasks.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif nvs_flash mqtt system_config gps_nmea sim800l state_machine log freertos
)
