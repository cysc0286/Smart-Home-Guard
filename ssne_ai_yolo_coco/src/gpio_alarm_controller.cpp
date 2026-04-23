#include "../include/gpio_alarm_controller.hpp"

#include <chrono>
#include <stdio.h>

#include "gpio_api.h"

const uint16_t GpioAlarmController::kLedPin = GPIO_PIN_8;
const uint16_t GpioAlarmController::kBuzzerPin = GPIO_PIN_10;
const uint16_t GpioAlarmController::kOutputPins =
    static_cast<uint16_t>(GpioAlarmController::kLedPin | GpioAlarmController::kBuzzerPin);
const int GpioAlarmController::kBuzzerOnMs = 200;
const int GpioAlarmController::kBuzzerOffMs = 100;
const int GpioAlarmController::kUpdateIntervalMs = 20;

GpioAlarmController::GpioAlarmController()
    : gpio_(NULL),
      initialized_(false),
      buzzer_on_(false),
      led_level_high_(false),
      buzzer_level_high_(false),
      last_toggle_ms_(0),
      last_update_call_ms_(0) {}

GpioAlarmController::~GpioAlarmController() {
  Release();
}

bool GpioAlarmController::Initialize() {
  if (initialized_) {
    return true;
  }

  gpio_handle_t handle = gpio_init();
  if (handle == NULL) {
    fprintf(stderr, "[GPIO] gpio_init failed.\n");
    return false;
  }

  if (gpio_set_enable(handle, kOutputPins, true) != 0) {
    fprintf(stderr, "[GPIO] gpio_set_enable failed.\n");
    gpio_close(handle);
    return false;
  }

  if (gpio_set_mode(handle, kOutputPins, GPIO_MODE_OUTPUT) != 0) {
    fprintf(stderr, "[GPIO] gpio_set_mode OUTPUT failed.\n");
    gpio_close(handle);
    return false;
  }

  if (gpio_write_pin(handle, kOutputPins, GPIO_PIN_RESET) != 0) {
    fprintf(stderr, "[GPIO] initial gpio_write_pin RESET failed.\n");
    gpio_close(handle);
    return false;
  }

  gpio_ = handle;
  initialized_ = true;
  buzzer_on_ = false;
  led_level_high_ = false;
  buzzer_level_high_ = false;
  last_toggle_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
  last_update_call_ms_ = last_toggle_ms_;

  printf("[GPIO] Initialized. LED=GPIO8, BUZZER=GPIO10\n");
  return true;
}

void GpioAlarmController::SetLed(bool on) {
  if (!initialized_) return;
  if (led_level_high_ == on) return;

  gpio_write_pin(static_cast<gpio_handle_t>(gpio_), kLedPin,
                 on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  led_level_high_ = on;
}

void GpioAlarmController::SetBuzzer(bool on) {
  if (!initialized_) return;
  if (buzzer_level_high_ == on) return;

  gpio_write_pin(static_cast<gpio_handle_t>(gpio_), kBuzzerPin,
                 on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  buzzer_level_high_ = on;
}

void GpioAlarmController::Update(bool has_object) {
  if (!initialized_) {
    return;
  }

  const long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();

  if (!has_object) {
    SetLed(false);
    SetBuzzer(false);
    buzzer_on_ = false;
    last_toggle_ms_ = now_ms;
    last_update_call_ms_ = now_ms;
    return;
  }

  // 限频执行GPIO更新，避免每帧都进入控制逻辑
  if ((now_ms - last_update_call_ms_) < kUpdateIntervalMs) {
    return;
  }
  last_update_call_ms_ = now_ms;

  SetLed(true);

  const int phase_ms = buzzer_on_ ? kBuzzerOnMs : kBuzzerOffMs;
  if ((now_ms - last_toggle_ms_) >= phase_ms) {
    buzzer_on_ = !buzzer_on_;
    last_toggle_ms_ = now_ms;
  }

  SetBuzzer(buzzer_on_);
}

void GpioAlarmController::Release() {
  if (!initialized_) {
    return;
  }

  gpio_handle_t handle = static_cast<gpio_handle_t>(gpio_);
  if (led_level_high_ || buzzer_level_high_) {
    gpio_write_pin(handle, kOutputPins, GPIO_PIN_RESET);
  }
  gpio_close(handle);

  gpio_ = NULL;
  initialized_ = false;
  buzzer_on_ = false;
  led_level_high_ = false;
  buzzer_level_high_ = false;
  last_toggle_ms_ = 0;
  last_update_call_ms_ = 0;

  printf("[GPIO] Released.\n");
}
