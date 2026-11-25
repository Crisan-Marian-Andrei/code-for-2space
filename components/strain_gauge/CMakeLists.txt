idf_component_register(
    SRCS "ads124s06.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer board_bsp bus_manager
)
