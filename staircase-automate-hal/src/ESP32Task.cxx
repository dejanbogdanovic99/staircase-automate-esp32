#include <ESP32Task.hxx>

#include <hal/ITask.hxx>
#include <hal/Timing.hxx>

#include <staircase/IRunnable.hxx>

#include <cstdint>
#include <string>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace esp32;

ESP32Task::ESP32Task(const std::string &name, std::uint16_t stackSize,
                     std::uint16_t priority, staircase::IRunnable &runnable,
                     hal::Milliseconds period)
    : ITask{runnable, period}, mName{name}, mStackSize{stackSize},
      mPriority{priority}, mCurrentTime{esp_timer_get_time()},
      mTaskHandle{NULL} {}

void ESP32Task::init() {
    if (mTaskHandle != NULL) {
        ESP_LOGE(kTag, "Task already initialized");
        return;
    }

    xTaskCreate(&ESP32Task::handler, mName.c_str(), mStackSize, this,
                mPriority, &mTaskHandle);
}

hal::Milliseconds ESP32Task::getDelta() const noexcept {
    std::int64_t currentTime = esp_timer_get_time();
    std::int64_t delta = currentTime - mCurrentTime;
    mCurrentTime = currentTime;

    if (delta < 0) {
        delta += INT64_MAX;
    }

    return delta / 1000;
}

void ESP32Task::sleep() noexcept { vTaskDelay(mPeriod / portTICK_PERIOD_MS); }

void ESP32Task::handler(void *data) {
    if (data == NULL) {
        ESP_LOGE(kTag, "Data is NULL, aborting...");
        return;
    }

    ESP32Task &task = *reinterpret_cast<ESP32Task *>(data);

    task.loop();
}