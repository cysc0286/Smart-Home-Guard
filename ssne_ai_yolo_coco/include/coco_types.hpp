#pragma once

#include <array>
#include <string>
#include <vector>

struct CocoDetection {
  std::array<float, 4> box_xyxy = {0.f, 0.f, 0.f, 0.f};
  float score    = 0.f;
  int   class_id = -1;
  std::string label;
};

struct CocoDetectionResult {
  std::vector<CocoDetection> detections;

  void Clear() { detections.clear(); }

  std::vector<std::array<float, 4>> ToBoxes() const {
    std::vector<std::array<float, 4>> boxes;
    boxes.reserve(detections.size());
    for (const auto& det : detections) {
      boxes.emplace_back(det.box_xyxy);
    }
    return boxes;
  }
};
