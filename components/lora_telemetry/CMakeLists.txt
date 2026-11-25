idf_component_register(
    SRCS "lora.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer board_bsp bus_manager
)
