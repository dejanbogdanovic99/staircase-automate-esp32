#include <ESP32Timing.hxx>

#include <hal/Timing.hxx>
#include <hal/ITiming.hxx>

#include <cstdint>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

using namespace esp32;

ESP32Timing::ESP32Timing() noexcept
    : mCurrentTime{esp_timer_get_time()} {}

void ESP32Timing::sleepFor(hal::Milliseconds millis) const noexcept {
    vTaskDelay(millis / portTICK_PERIOD_MS);
}

hal::Milliseconds ESP32Timing::getDelta() const noexcept {
    std::int64_t currentTime = esp_timer_get_time();
    std::int64_t delta = currentTime - mCurrentTime;
    mCurrentTime = currentTime;

    if (delta < 0)
    {
        delta += INT64_MAX;
    }

    if (delta > kMaxDelta)
    {
        delta = kMaxDelta;
    }

    return delta / 1000;
}