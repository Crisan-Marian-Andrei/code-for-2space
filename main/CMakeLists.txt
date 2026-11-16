idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        bus_manager
        strain_gauge
        imu_baro
        gnss
        lora_telemetry
        sd_logger
        flight_fsm
        esp_timer
        driver
)
