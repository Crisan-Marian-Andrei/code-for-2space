idf_component_register(
    SRCS "flight_fsm.c" "state_estimator.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer
)
