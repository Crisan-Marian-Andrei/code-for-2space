idf_component_register(
    SRCS "imu_baro.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer bus_manager
)
