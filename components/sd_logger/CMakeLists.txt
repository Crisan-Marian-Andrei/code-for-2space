idf_component_register(
    SRCS "sd_logger.c"
    INCLUDE_DIRS "."
    REQUIRES driver sdmmc fatfs esp_timer
)
