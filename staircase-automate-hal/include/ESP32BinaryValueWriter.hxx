#pragma once

#include <hal/BinaryValue.hxx>
#include <hal/IBinaryValueWriter.hxx>

#include <driver/gpio.h>

#if CONFIG_BINARY_OUTPUT_DEFAULT_VALUE_LOW
#define BINARY_OUTPUT_DEFAULT_VALUE hal::BinaryValue::LOW
#else
#define BINARY_OUTPUT_DEFAULT_VALUE hal::BinaryValue::HIGH
#endif

namespace esp32 {

class ESP32BinaryValueWriter : public hal::IBinaryValueWriter {
  public:
    ESP32BinaryValueWriter(gpio_num_t pin) noexcept;
    ~ESP32BinaryValueWriter();

    void writeValue(hal::BinaryValue value) noexcept final;

  private:
    uint32_t getLevel(hal::BinaryValue value) const noexcept;

    static constexpr hal::BinaryValue kDefaultValue =
        BINARY_OUTPUT_DEFAULT_VALUE;
    static constexpr const char *kTag = "ESP32BinaryValueWriter";
    gpio_num_t mPin;
};

} // namespace esp32