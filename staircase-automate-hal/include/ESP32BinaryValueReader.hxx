#pragma once

#include <hal/BinaryValue.hxx>
#include <hal/IBinaryValueReader.hxx>

#include "driver/gpio.h"

namespace esp32 {

class ESP32BinaryValueReader : public hal::IBinaryValueReader {
  public:
    ESP32BinaryValueReader(gpio_num_t pin) noexcept;
    ~ESP32BinaryValueReader();

    esp_err_t init() noexcept;
    hal::BinaryValue readValue() noexcept final;

  private:
    hal::BinaryValue getValue(uint32_t level) const noexcept;

    static constexpr const char *kTag = "ESP32BinaryValueReader";
    gpio_num_t mPin;
};

} // namespace esp32