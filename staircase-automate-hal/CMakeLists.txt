idf_component_register(
    SRCS
        src/ESP32BinaryValueReader.cxx
        src/ESP32BinaryValueWriter.cxx
        src/ESP32Persistence.cxx
        src/ESP32Task.cxx
    INCLUDE_DIRS
        ./include
    REQUIRES
        staircase-automate
        log
        driver
        nvs_flash
        esp_timer
        esp_system
        console
)

idf_component_set_property(
    staircase-automate-hal
        COMPILE_OPTIONS "-Wall -Werror" APPEND
)