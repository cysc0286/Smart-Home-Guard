#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <chrono>

#include "include/gpio_alarm_controller.hpp"

#include "include/coco_config.hpp"
#include "include/coco_detector.hpp"
#include "include/debounce_tracker.hpp"
#include "include/utils.hpp"
#include "include/common.hpp"

using namespace std;

bool g_exit_flag = false;
std::mutex g_mtx;

void keyboard_listener() {
  std::string input;
  std::cout << "Input q to quit..." << std::endl;
  while (true) {
    std::cin >> input;
    std::lock_guard<std::mutex> lock(g_mtx);
    if (input == "q" || input == "Q") {
      g_exit_flag = true;
      break;
    }
  }
}

bool check_exit_flag() {
  std::lock_guard<std::mutex> lock(g_mtx);
  return g_exit_flag;
}

// Restore crop-offset: coordinates from the 1440x1080 crop must be shifted
// back to the full 1920x1080 image space before display.
void ConvertCropBoxesToOriginal(CocoDetectionResult* result) {
  for (auto& det : result->detections) {
    det.box_xyxy[0] += static_cast<float>(coco_config::kCropOffsetX);
    det.box_xyxy[2] += static_cast<float>(coco_config::kCropOffsetX);
  }
}

int main() {
  std::array<int, 2> img_shape  = coco_config::kImageShape;
  std::array<int, 2> crop_shape = coco_config::kCropShape;
  std::array<int, 2> det_shape  = coco_config::kDetShape;
  std::string        model_path = coco_config::kModelPath;

  if (ssne_initial()) {
    fprintf(stderr, "[INIT] SSNE initialization failed!\n");
    return -1;
  }
  printf("[INIT] SSNE initialized\n");

  IMAGEPROCESSOR processor;
  processor.Initialize(&img_shape);

  COCO_DETECTOR detector;
  detector.Initialize(model_path, &crop_shape, &det_shape);
  printf("[INIT] Detector loaded: %s\n", model_path.c_str());

  VISUALIZER visualizer;
  visualizer.Initialize(img_shape, "shared_colorLUT.sscl");

  GpioAlarmController gpio_alarm;
  if (!gpio_alarm.Initialize()) {
    visualizer.Release();
    detector.Release();
    processor.Release();
    ssne_release();
    return -1;
  }
  printf("[INIT] GPIO alarm ready\n");

  usleep(200000);
  printf("[INIT] System ready -- input q to quit\n");

  ssne_tensor_t       img_sensor;
  CocoDetectionResult det_result;
  DebounceTracker     tracker;

  std::thread listener_thread(keyboard_listener);

  auto last_log_time = std::chrono::steady_clock::now();
  constexpr int kDetLogIntervalMs  = 500;
  constexpr int kIdleLogIntervalMs = 5000;

  while (!check_exit_flag()) {
    processor.GetImage(&img_sensor);
    detector.Predict(&img_sensor, &det_result, coco_config::kConfThreshold);
    ConvertCropBoxesToOriginal(&det_result);

    // Debounce: only show detections stable across >= 3 consecutive frames
    tracker.Update(det_result);
    CocoDetectionResult stable = tracker.ConfirmedDetections();

    const bool has_object = !stable.detections.empty();
    gpio_alarm.Update(has_object);

    const auto now = std::chrono::steady_clock::now();
    const auto log_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_log_time).count();
    const int log_interval = has_object ? kDetLogIntervalMs : kIdleLogIntervalMs;
    if (log_elapsed_ms >= log_interval) {
      if (has_object) {
        for (const auto& det : stable.detections) {
          printf("[DET]  %-10s  conf=%.2f  [%.0f,%.0f,%.0f,%.0f]\n",
                 det.label.c_str(), det.score,
                 det.box_xyxy[0], det.box_xyxy[1], det.box_xyxy[2], det.box_xyxy[3]);
        }
      } else {
        printf("[IDLE] no detection\n");
      }
      last_log_time = now;
    }

    visualizer.Draw(tracker.ConfirmedBoxes());
  }

  if (listener_thread.joinable()) {
    listener_thread.join();
  }

  gpio_alarm.Release();
  detector.Release();
  processor.Release();
  visualizer.Release();

  if (ssne_release()) {
    fprintf(stderr, "SSNE release failed!\n");
    return -1;
  }

  return 0;
}
