idf_component_register(
    SRCS
        station_example_main.cpp
    INCLUDE_DIRS
        .
    REQUIRES
        esp_event
        log
        esp_system
        esp_wifi
        nvs_flash
        lwip
        staircase-automate
        staircase-automate-hal
)

idf_component_set_property(
    staircase-automate
        COMPILE_OPTIONS "-Wall -Werror" APPEND
)