#include <ESP32BinaryValueWriter.hxx>

#include <hal/BinaryValue.hxx>

#include <driver/gpio.h>
#include <esp_log.h>

using namespace esp32;

ESP32BinaryValueWriter::ESP32BinaryValueWriter(gpio_num_t pin) noexcept
    : mPin{pin} {
    ESP_ERROR_CHECK(gpio_reset_pin(mPin));
    ESP_ERROR_CHECK(gpio_set_direction(mPin, GPIO_MODE_OUTPUT));
    writeValue(kDefaultValue);
}

ESP32BinaryValueWriter::~ESP32BinaryValueWriter() {
    esp_err_t ret = gpio_reset_pin(mPin);
    if (ret != ESP_OK) {
        ESP_LOGW(kTag, "Failed to reset pin %d due to %d",
                 static_cast<int>(mPin), static_cast<int>(ret));
    }
}

void ESP32BinaryValueWriter::writeValue(hal::BinaryValue value) noexcept {
    uint32_t level = getLevel(value);
    esp_err_t ret = gpio_set_level(mPin, level);
    if (ret != ESP_OK) {
        ESP_LOGW(kTag, "Failed to set %d due to %d", static_cast<int>(value),
                 static_cast<int>(ret));
    }
}

uint32_t
ESP32BinaryValueWriter::getLevel(hal::BinaryValue value) const noexcept {
    switch (value) {
    case hal::BinaryValue::HIGH:
        return 1;
    case hal::BinaryValue::LOW:
    default:
        return 0;
    }
}