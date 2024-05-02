#include <ESP32BinaryValueReader.hxx>

#include <hal/BinaryValue.hxx>

#include <driver/gpio.h>
#include <esp_log.h>

using namespace esp32;

ESP32BinaryValueReader::ESP32BinaryValueReader(gpio_num_t pin) noexcept
    : mPin{pin} {
    ESP_ERROR_CHECK(gpio_reset_pin(mPin));
    ESP_ERROR_CHECK(gpio_set_direction(mPin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(mPin, GPIO_PULLUP_ONLY));
}

ESP32BinaryValueReader::~ESP32BinaryValueReader() {
    esp_err_t ret = gpio_reset_pin(mPin);
    if (ret != ESP_OK) {
        ESP_LOGW(kTag, "Failed to reset pin %d due to %d",
                 static_cast<int>(mPin), static_cast<int>(ret));
    }
}

hal::BinaryValue ESP32BinaryValueReader::readValue() noexcept {
    return getValue(gpio_get_level(mPin));
}

hal::BinaryValue
ESP32BinaryValueReader::getValue(uint32_t level) const noexcept {
    switch (level) {
    case 0:
        return hal::BinaryValue::LOW;
    case 1:
    default:
        return hal::BinaryValue::HIGH;
    }
}