#pragma once

#include <hal/ITask.hxx>
#include <hal/Timing.hxx>

#include <staircase/IRunnable.hxx>

#include <cstdint>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esp32 {

class ESP32Task : public hal::ITask {

  public:
    ESP32Task(const std::string &name, std::uint16_t stackSize,
              std::uint16_t priority, staircase::IRunnable &runnable,
              hal::Milliseconds period);

    void init() noexcept;
    hal::Milliseconds getDelta() const noexcept final;

  private:
    static void handler(void *data);

    void sleep() noexcept final;

    static constexpr const char *kTag = "ESP32Task";

    std::string mName;
    std::uint16_t mStackSize;
    std::uint16_t mPriority;
    mutable std::int64_t mCurrentTime;
    TaskHandle_t mTaskHandle;
};

} // namespace esp32
