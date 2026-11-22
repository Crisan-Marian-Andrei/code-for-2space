idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer sd_logger strain_gauge flight_fsm board_bsp bus_manager
)
