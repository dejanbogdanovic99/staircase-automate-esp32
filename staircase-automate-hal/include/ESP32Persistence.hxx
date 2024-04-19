#pragma once

#include <hal/IPersistence.hxx>

#include <nvs.h>
#include <nvs_flash.h>

namespace esp32 {

class ESP32Persistence : public hal::IPersistence {
  public:
    ESP32Persistence(nvs_handle_t handle) noexcept;
    ~ESP32Persistence() = default;

    bool keyExists(const std::string &key) const noexcept override;
    std::int32_t getValue(const std::string &key) const noexcept override;
    void setValue(const std::string &key, std::int32_t value) noexcept override;

  private:
    static constexpr const char *kTag = "ESP32Persistence";

    nvs_handle_t mHandle;
};

} // namespace esp32