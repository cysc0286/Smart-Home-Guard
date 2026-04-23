#pragma once

#include <stdint.h>

class GpioAlarmController {
 public:
  GpioAlarmController();
  ~GpioAlarmController();

  bool Initialize();
  void Update(bool has_object);
  void Release();

 private:
  void SetLed(bool on);
  void SetBuzzer(bool on);

  void* gpio_;
  bool initialized_;
  bool buzzer_on_;
  bool led_level_high_;
  bool buzzer_level_high_;
  long long last_toggle_ms_;
  long long last_update_call_ms_;

  static const uint16_t kLedPin;
  static const uint16_t kBuzzerPin;
  static const uint16_t kOutputPins;
  static const int kBuzzerOnMs;
  static const int kBuzzerOffMs;
  static const int kUpdateIntervalMs;
};
