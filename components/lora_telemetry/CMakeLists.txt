idf_component_register(
    SRCS "lora.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer bus_manager
)
