#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "coco_config.hpp"
#include "coco_types.hpp"

// Lightweight single-class tracker used to debounce alarm decisions.  Display
// output is intentionally more responsive than alarm output: new tracks appear
// immediately and may bridge a short miss, whereas alarm candidates must be
// confirmed and matched in the current frame.
class DebounceTracker {
 public:
  struct Track {
    std::array<float, 4> box;
    std::array<float, 4> raw_box;
    std::array<float, 4> velocity{{0.0f, 0.0f, 0.0f, 0.0f}};
    int class_id = -1;
    std::string label;
    float score = 0.0f;
    int on_count = 0;
    int off_count = 0;
    bool confirmed = false;
    bool matched_this_update = false;
    long long first_seen_ms = 0;
    long long last_seen_ms = 0;
  };

  struct UpdateStats {
    int detections = 0;
    int matched_iou = 0;
    int matched_center = 0;
    int new_tracks = 0;
    int unmatched_tracks = 0;
    int expired_tracks = 0;
    int visible_tracks = 0;
    int confirmed_tracks = 0;
    float best_match_iou_sum = 0.0f;
    float best_match_iou_max = 0.0f;
  };

  void Update(const CocoDetectionResult& detections) {
    const long long now_ms = NowMs();
    last_stats_ = UpdateStats();
    last_stats_.detections = static_cast<int>(detections.detections.size());

    std::vector<bool> track_matched(tracks_.size(), false);
    std::vector<bool> detection_matched(detections.detections.size(), false);
    for (auto& track : tracks_) {
      track.matched_this_update = false;
    }

    // Greedy one-to-one assignment.  IoU remains the primary signal; a
    // predicted-centre gate recovers matches when fast movement destroys IoU.
    for (std::size_t detection_index = 0;
         detection_index < detections.detections.size(); ++detection_index) {
      const CocoDetection& detection = detections.detections[detection_index];
      int best_track = -1;
      bool best_is_iou = false;
      float best_rank = -std::numeric_limits<float>::infinity();
      float best_iou = 0.0f;

      for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
        if (track_matched[track_index] ||
            tracks_[track_index].class_id != detection.class_id) {
          continue;
        }
        const std::array<float, 4> predicted = Predict(tracks_[track_index]);
        const float iou = IoU(predicted, detection.box_xyxy);
        const float center_distance = NormalizedCenterDistance(
            predicted, detection.box_xyxy);
        const float size_ratio = SizeRatio(predicted, detection.box_xyxy);
        const bool iou_match = iou >= coco_config::kTrackerIoUThreshold;
        const bool center_match =
            center_distance <= coco_config::kTrackerCenterGate &&
            size_ratio <= coco_config::kTrackerSizeRatioGate;
        if (!iou_match && !center_match) {
          continue;
        }
        const float rank = iou_match
            ? 2.0f + iou
            : 1.0f - center_distance /
                         std::max(0.01f, coco_config::kTrackerCenterGate);
        if (rank > best_rank) {
          best_rank = rank;
          best_track = static_cast<int>(track_index);
          best_is_iou = iou_match;
          best_iou = iou;
        }
      }

      if (best_track < 0) {
        continue;
      }
      Track& track = tracks_[static_cast<std::size_t>(best_track)];
      for (std::size_t coordinate = 0; coordinate < 4; ++coordinate) {
        const float delta = detection.box_xyxy[coordinate] - track.raw_box[coordinate];
        track.velocity[coordinate] =
            coco_config::kTrackerVelocityAlpha * delta +
            (1.0f - coco_config::kTrackerVelocityAlpha) *
                track.velocity[coordinate];
      }
      track.raw_box = detection.box_xyxy;
      track.box = Smooth(track.box, detection.box_xyxy);
      track.label = detection.label;
      track.score = detection.score;
      ++track.on_count;
      track.off_count = 0;
      track.last_seen_ms = now_ms;
      track.matched_this_update = true;
      track.confirmed = track.confirmed ||
          now_ms - track.first_seen_ms >= coco_config::kAlarmConfirmMs;
      track_matched[static_cast<std::size_t>(best_track)] = true;
      detection_matched[detection_index] = true;
      if (best_is_iou) {
        ++last_stats_.matched_iou;
      } else {
        ++last_stats_.matched_center;
      }
      last_stats_.best_match_iou_sum += best_iou;
      last_stats_.best_match_iou_max =
          std::max(last_stats_.best_match_iou_max, best_iou);
    }

    for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
      if (!track_matched[track_index]) {
        ++tracks_[track_index].off_count;
        ++last_stats_.unmatched_tracks;
      }
    }

    const std::size_t tracks_before_expiry = tracks_.size();
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [now_ms](const Track& track) {
                         return now_ms - track.last_seen_ms >
                                coco_config::kTrackRetentionMs;
                       }),
        tracks_.end());
    last_stats_.expired_tracks = static_cast<int>(
        tracks_before_expiry - tracks_.size());

    for (std::size_t detection_index = 0;
         detection_index < detections.detections.size(); ++detection_index) {
      if (detection_matched[detection_index]) {
        continue;
      }
      const CocoDetection& detection = detections.detections[detection_index];
      Track track;
      track.box = detection.box_xyxy;
      track.raw_box = detection.box_xyxy;
      track.class_id = detection.class_id;
      track.label = detection.label;
      track.score = detection.score;
      track.on_count = 1;
      track.matched_this_update = true;
      track.first_seen_ms = now_ms;
      track.last_seen_ms = now_ms;
      tracks_.push_back(track);
      ++last_stats_.new_tracks;
    }

    for (const auto& track : tracks_) {
      if (track.matched_this_update ||
          now_ms - track.last_seen_ms <= coco_config::kDisplayHoldMs) {
        ++last_stats_.visible_tracks;
      }
      if (track.confirmed && track.matched_this_update) {
        ++last_stats_.confirmed_tracks;
      }
    }
  }

  CocoDetectionResult DisplayDetections() const {
    CocoDetectionResult result;
    const long long now_ms = NowMs();
    for (const auto& track : tracks_) {
      if (!track.matched_this_update &&
          now_ms - track.last_seen_ms > coco_config::kDisplayHoldMs) {
        continue;
      }
      result.detections.push_back(ToDetection(track));
    }
    return result;
  }

  std::vector<std::array<float, 4>> ConfirmedBoxes() const {
    std::vector<std::array<float, 4>> boxes;
    for (const auto& track : tracks_) {
      if (track.confirmed && track.matched_this_update) {
        boxes.push_back(track.box);
      }
    }
    return boxes;
  }

  CocoDetectionResult ConfirmedDetections() const {
    CocoDetectionResult result;
    for (const auto& track : tracks_) {
      if (track.confirmed && track.matched_this_update) {
        result.detections.push_back(ToDetection(track));
      }
    }
    return result;
  }

  const UpdateStats& LastUpdateStats() const { return last_stats_; }

  void Reset() {
    tracks_.clear();
    last_stats_ = UpdateStats();
  }

 private:
  static constexpr float kSmoothing = 0.4f;
  std::vector<Track> tracks_;
  UpdateStats last_stats_;

  static long long NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  static CocoDetection ToDetection(const Track& track) {
    CocoDetection detection;
    detection.box_xyxy = track.box;
    detection.class_id = track.class_id;
    detection.label = track.label;
    detection.score = track.score;
    return detection;
  }

  static std::array<float, 4> Predict(const Track& track) {
    std::array<float, 4> predicted = track.raw_box;
    for (std::size_t coordinate = 0; coordinate < 4; ++coordinate) {
      predicted[coordinate] += track.velocity[coordinate];
    }
    return predicted;
  }

  static float IoU(const std::array<float, 4>& a,
                   const std::array<float, 4>& b) {
    const float x1 = std::max(a[0], b[0]);
    const float y1 = std::max(a[1], b[1]);
    const float x2 = std::min(a[2], b[2]);
    const float y2 = std::min(a[3], b[3]);
    const float intersection =
        std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    const float area_a =
        std::max(0.0f, a[2] - a[0]) * std::max(0.0f, a[3] - a[1]);
    const float area_b =
        std::max(0.0f, b[2] - b[0]) * std::max(0.0f, b[3] - b[1]);
    return intersection / std::max(1e-6f, area_a + area_b - intersection);
  }

  static float NormalizedCenterDistance(const std::array<float, 4>& a,
                                        const std::array<float, 4>& b) {
    const float center_ax = (a[0] + a[2]) * 0.5f;
    const float center_ay = (a[1] + a[3]) * 0.5f;
    const float center_bx = (b[0] + b[2]) * 0.5f;
    const float center_by = (b[1] + b[3]) * 0.5f;
    const float dx = center_ax - center_bx;
    const float dy = center_ay - center_by;
    const float diagonal_a = std::hypot(a[2] - a[0], a[3] - a[1]);
    const float diagonal_b = std::hypot(b[2] - b[0], b[3] - b[1]);
    return std::hypot(dx, dy) /
        std::max(1.0f, (diagonal_a + diagonal_b) * 0.5f);
  }

  static float SizeRatio(const std::array<float, 4>& a,
                         const std::array<float, 4>& b) {
    const float width_a = std::max(1.0f, a[2] - a[0]);
    const float height_a = std::max(1.0f, a[3] - a[1]);
    const float width_b = std::max(1.0f, b[2] - b[0]);
    const float height_b = std::max(1.0f, b[3] - b[1]);
    return std::max(
        std::max(width_a / width_b, width_b / width_a),
        std::max(height_a / height_b, height_b / height_a));
  }

  static std::array<float, 4> Smooth(const std::array<float, 4>& previous,
                                     const std::array<float, 4>& current) {
    std::array<float, 4> smoothed;
    for (std::size_t coordinate = 0; coordinate < 4; ++coordinate) {
      smoothed[coordinate] = previous[coordinate] * (1.0f - kSmoothing) +
                             current[coordinate] * kSmoothing;
    }
    return smoothed;
  }
};
