idf_component_register(
    SRCS "ads124s06.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer bus_manager
)
