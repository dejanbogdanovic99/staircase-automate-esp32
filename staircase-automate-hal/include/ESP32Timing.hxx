#pragma once

#include <hal/Timing.hxx>
#include <hal/ITiming.hxx>

#include <cstdint>

#include "driver/gpio.h"
#include "esp_timer.h"

namespace esp32 {

class ESP32Timing : public hal::ITiming {
  public:
    ESP32Timing() noexcept;
    ~ESP32Timing() = default;

    void sleepFor(hal::Milliseconds millis) const noexcept final;
    hal::Milliseconds getDelta() const noexcept final;

  private:
    static constexpr std::int64_t kMaxDelta = 15000;
    static constexpr const char *kTag = "ESP32Timing";
    mutable std::int64_t mCurrentTime;
};

} // namespace esp32