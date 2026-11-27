idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        board_bsp bus_manager strain_gauge accel_baro lora_telemetry
        sd_logger flight_fsm
        esp_timer esp_wifi nvs_flash
)
