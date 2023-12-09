#include <ESP32BinaryValueReader.hxx>

#include <hal/BinaryValue.hxx>

#include "driver/gpio.h"
#include "esp_log.h"

using namespace esp32;

ESP32BinaryValueReader::ESP32BinaryValueReader(gpio_num_t pin) noexcept
    : mPin{pin} {}

ESP32BinaryValueReader::~ESP32BinaryValueReader() {
    esp_err_t ret = gpio_reset_pin(mPin);
    if (ret != ESP_OK) {
        ESP_LOGW(kTag, "Failed to reset pin %d due to %d",
                 static_cast<int>(mPin), static_cast<int>(ret));
    }
}

esp_err_t ESP32BinaryValueReader::init() noexcept {
    esp_err_t ret = gpio_reset_pin(mPin);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to reset pin %d due to %d",
                 static_cast<int>(mPin), static_cast<int>(ret));
        return ret;
    }

    ret = gpio_set_direction(mPin, GPIO_MODE_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "Failed to set input direction for pin %d due to %d",
                 static_cast<int>(mPin), static_cast<int>(ret));
        return ret;
    }

    return ESP_OK;
}

hal::BinaryValue ESP32BinaryValueReader::readValue() noexcept {
    return getValue(gpio_get_level(mPin));
}

hal::BinaryValue
ESP32BinaryValueReader::getValue(uint32_t level) const noexcept {
    switch (level) {
    case 1:
        return hal::BinaryValue::HIGH;
    case 0:
    default:
        return hal::BinaryValue::LOW;
    }
}