#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "coco_config.hpp"
#include "coco_types.hpp"

// Temporal debounce tracker for object detections.
//
// A detection must remain matched for coco_config::kAlarmConfirmMs before being
// shown. Once confirmed, it stays visible until absent for
// coco_config::kAlarmClearMs. Box coordinates are smoothed with exponential
// moving average to reduce jitter.
class DebounceTracker {
 public:
  struct Track {
    std::array<float, 4> box;   // smoothed box [x1,y1,x2,y2]
    int   class_id  = -1;
    std::string label;
    float score     = 0.f;
    int   on_count  = 0;        // consecutive frames present
    int   off_count = 0;        // consecutive frames absent
    bool  confirmed = false;    // visible to caller
    long long first_seen_ms = 0;
    long long last_seen_ms  = 0;
  };

  void Update(const CocoDetectionResult& detections) {
    const long long now_ms = NowMs();
    // Mark all tracks as unmatched
    std::vector<bool> track_matched(tracks_.size(), false);
    std::vector<bool> det_matched(detections.detections.size(), false);

    // Greedy IoU matching
    for (size_t di = 0; di < detections.detections.size(); ++di) {
      const auto& det = detections.detections[di];
      float best_iou = kIoUThreshold;
      int   best_ti  = -1;

      for (size_t ti = 0; ti < tracks_.size(); ++ti) {
        if (track_matched[ti]) continue;
        if (tracks_[ti].class_id != det.class_id) continue;
        float iou = IoU(tracks_[ti].box, det.box_xyxy);
        if (iou > best_iou) {
          best_iou = iou;
          best_ti  = static_cast<int>(ti);
        }
      }

      if (best_ti >= 0) {
        // Matched: update track
        Track& t = tracks_[best_ti];
        t.box       = Smooth(t.box, det.box_xyxy);
        t.score     = det.score;
        t.on_count  = std::min(t.on_count + 1, 30);
        t.off_count = 0;
        t.last_seen_ms = now_ms;
        t.confirmed = ((now_ms - t.first_seen_ms) >= coco_config::kAlarmConfirmMs);
        track_matched[best_ti] = true;
        det_matched[di]        = true;
      }
    }

    // Unmatched tracks: increment off_count
    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (!track_matched[ti]) {
        tracks_[ti].off_count++;
        tracks_[ti].on_count = 0;
      }
    }

    // Remove stale tracks
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [now_ms](const Track& t) {
                         return (now_ms - t.last_seen_ms) > coco_config::kAlarmClearMs;
                       }),
        tracks_.end());

    // Add new tracks for unmatched detections
    for (size_t di = 0; di < detections.detections.size(); ++di) {
      if (!det_matched[di]) {
        const auto& det = detections.detections[di];
        Track t;
        t.box       = det.box_xyxy;
        t.class_id  = det.class_id;
        t.label     = det.label;
        t.score     = det.score;
        t.on_count  = 1;
        t.off_count = 0;
        t.confirmed = false;
        t.first_seen_ms = now_ms;
        t.last_seen_ms  = now_ms;
        tracks_.push_back(t);
      }
    }
  }

  // Returns only confirmed (stable) tracks as boxes for OSD display
  std::vector<std::array<float, 4>> ConfirmedBoxes() const {
    std::vector<std::array<float, 4>> boxes;
    for (const auto& t : tracks_) {
      if (t.confirmed) boxes.emplace_back(t.box);
    }
    return boxes;
  }

  // Returns confirmed tracks as a CocoDetectionResult for logging
  CocoDetectionResult ConfirmedDetections() const {
    CocoDetectionResult result;
    for (const auto& t : tracks_) {
      if (!t.confirmed) continue;
      CocoDetection det;
      det.box_xyxy = t.box;
      det.class_id = t.class_id;
      det.label    = t.label;
      det.score    = t.score;
      result.detections.push_back(det);
    }
    return result;
  }

 private:
  // Minimum IoU to match a detection to an existing track
  static constexpr float kIoUThreshold = 0.3f;
  // Exponential smoothing factor (0=no update, 1=no smoothing)
  static constexpr float kSmoothing    = 0.4f;

  std::vector<Track> tracks_;

  static long long NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  static float IoU(const std::array<float, 4>& a,
                   const std::array<float, 4>& b) {
    const float x1 = std::max(a[0], b[0]);
    const float y1 = std::max(a[1], b[1]);
    const float x2 = std::min(a[2], b[2]);
    const float y2 = std::min(a[3], b[3]);
    const float inter = std::max(0.f, x2 - x1) * std::max(0.f, y2 - y1);
    const float area_a = std::max(0.f, a[2]-a[0]) * std::max(0.f, a[3]-a[1]);
    const float area_b = std::max(0.f, b[2]-b[0]) * std::max(0.f, b[3]-b[1]);
    return inter / std::max(1e-6f, area_a + area_b - inter);
  }

  static std::array<float, 4> Smooth(const std::array<float, 4>& prev,
                                      const std::array<float, 4>& curr) {
    return {
        kSmoothing * curr[0] + (1.f - kSmoothing) * prev[0],
        kSmoothing * curr[1] + (1.f - kSmoothing) * prev[1],
        kSmoothing * curr[2] + (1.f - kSmoothing) * prev[2],
        kSmoothing * curr[3] + (1.f - kSmoothing) * prev[3],
    };
  }
};
