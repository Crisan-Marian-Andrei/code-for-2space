idf_component_register(
    SRCS "accel_baro.c"
    INCLUDE_DIRS "."
    REQUIRES driver bus_manager
)
