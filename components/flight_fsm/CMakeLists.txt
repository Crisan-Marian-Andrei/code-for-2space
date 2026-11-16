idf_component_register(
    SRCS "flight_fsm.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc esp_timer imu_baro
)
