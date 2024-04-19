#include <ESP32Persistence.hxx>

#include <hal/BinaryValue.hxx>

#include <string>

#include <nvs.h>
#include <nvs_flash.h>

using namespace esp32;

ESP32Persistence::ESP32Persistence(nvs_handle_t handle) noexcept
    : mHandle{handle} {}

bool ESP32Persistence::keyExists(const std::string &key) const noexcept {
    int32_t value;
    return nvs_get_i32(mHandle, key.c_str(), &value) == ESP_OK;
}

std::int32_t ESP32Persistence::getValue(const std::string &key) const noexcept {
    int32_t value = -1;
    nvs_get_i32(mHandle, key.c_str(), &value);
    return value;
}

void ESP32Persistence::setValue(const std::string &key,
                                std::int32_t value) noexcept {
    nvs_set_i32(mHandle, key.c_str(), value);
}