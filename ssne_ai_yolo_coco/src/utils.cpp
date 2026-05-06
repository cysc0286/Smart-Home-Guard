/*
 * utils.cpp — OSD可视化 + 原始人脸检测排序/NMS工具
 */
#include "../include/utils.hpp"
#include "../include/coco_config.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdio>

namespace utils {

void Merge(FaceDetectionResult* result, size_t low, size_t mid, size_t high) {
  std::vector<std::array<float, 4>>& boxes = result->boxes;
  std::vector<float>& scores = result->scores;
  std::vector<std::array<float, 4>> temp_boxes(boxes);
  std::vector<float> temp_scores(scores);
  size_t i = low;
  size_t j = mid + 1;
  size_t k = i;
  for (; i <= mid && j <= high; k++) {
    if (temp_scores[i] >= temp_scores[j]) {
      scores[k] = temp_scores[i];
      boxes[k] = temp_boxes[i];
      i++;
    } else {
      scores[k] = temp_scores[j];
      boxes[k] = temp_boxes[j];
      j++;
    }
  }
  while (i <= mid) {
    scores[k] = temp_scores[i];
    boxes[k] = temp_boxes[i];
    k++;
    i++;
  }
  while (j <= high) {
    scores[k] = temp_scores[j];
    boxes[k] = temp_boxes[j];
    k++;
    j++;
  }
}

void MergeSort(FaceDetectionResult* result, size_t low, size_t high) {
  if (low < high) {
    size_t mid = (high - low) / 2 + low;
    MergeSort(result, low, mid);
    MergeSort(result, mid + 1, high);
    Merge(result, low, mid, high);
  }
}

void SortDetectionResult(FaceDetectionResult* result) {
  size_t low = 0;
  size_t high = result->scores.size();
  if (high == 0) {
    return;
  }
  high = high - 1;
  MergeSort(result, low, high);
}

// NMS: 按分数排序后，逐个与后续框比IoU，超阈值则抑制
void NMS(FaceDetectionResult* result, float iou_threshold, int top_k) {
  SortDetectionResult(result);

  int res_count = static_cast<int>(result->boxes.size());
  result->Resize(std::min(res_count, top_k));

  std::vector<float> area_of_boxes(result->boxes.size());
  std::vector<int> suppressed(result->boxes.size(), 0);
  for (size_t i = 0; i < result->boxes.size(); ++i) {
    area_of_boxes[i] = (result->boxes[i][2] - result->boxes[i][0] + 1) *
                       (result->boxes[i][3] - result->boxes[i][1] + 1);
  }

  for (size_t i = 0; i < result->boxes.size(); ++i) {
    if (suppressed[i] == 1) {
      continue;
    }
    for (size_t j = i + 1; j < result->boxes.size(); ++j) {
      if (suppressed[j] == 1) {
        continue;
      }
      float xmin = std::max(result->boxes[i][0], result->boxes[j][0]);
      float ymin = std::max(result->boxes[i][1], result->boxes[j][1]);
      float xmax = std::min(result->boxes[i][2], result->boxes[j][2]);
      float ymax = std::min(result->boxes[i][3], result->boxes[j][3]);
      float overlap_w = std::max(0.0f, xmax - xmin + 1);
      float overlap_h = std::max(0.0f, ymax - ymin + 1);
      float overlap_area = overlap_w * overlap_h;
      float overlap_ratio =
          overlap_area / (area_of_boxes[i] + area_of_boxes[j] - overlap_area);
      if (overlap_ratio > iou_threshold) {
        suppressed[j] = 1;
      }
    }
  }

  FaceDetectionResult backup(*result);
  int landmarks_per_face = result->landmarks_per_face;

  result->Clear();
  result->landmarks_per_face = landmarks_per_face;
  result->Reserve(suppressed.size());
  for (size_t i = 0; i < suppressed.size(); ++i) {
    if (suppressed[i] == 1) {
      continue;
    }
    result->boxes.emplace_back(backup.boxes[i]);
    result->scores.push_back(backup.scores[i]);
    if (result->landmarks_per_face > 0) {
      for (size_t j = 0; j < result->landmarks_per_face; ++j) {
        result->landmarks.emplace_back(
            backup.landmarks[i * result->landmarks_per_face + j]);
      }
    }
  }
}
}  // namespace utils


void FaceDetectionResult::Free() {
  std::vector<std::array<float, 4>>().swap(boxes);
  std::vector<float>().swap(scores);
  std::vector<std::array<float, 2>>().swap(landmarks);
  landmarks_per_face = 0;
}

void FaceDetectionResult::Clear() {
  boxes.clear();
  scores.clear();
  landmarks.clear();
  landmarks_per_face = 0;
}

void FaceDetectionResult::Reserve(int size) {
  boxes.reserve(size);
  scores.reserve(size);
  if (landmarks_per_face > 0) {
    landmarks.reserve(size * landmarks_per_face);
  }
}

void FaceDetectionResult::Resize(int size) {
  boxes.resize(size);
  scores.resize(size);
  if (landmarks_per_face > 0) {
    landmarks.resize(size * landmarks_per_face);
  }
}

FaceDetectionResult::FaceDetectionResult(const FaceDetectionResult& res) {
  boxes.assign(res.boxes.begin(), res.boxes.end());
  landmarks.assign(res.landmarks.begin(), res.landmarks.end());
  scores.assign(res.scores.begin(), res.scores.end());
  landmarks_per_face = res.landmarks_per_face;
}


void VISUALIZER::Initialize(std::array<int, 2>& in_img_shape, const std::string& bitmap_lut_path) {
    m_width = in_img_shape[0];
    m_height = in_img_shape[1];
    const char* lut_path = nullptr;
    if (!bitmap_lut_path.empty()) {
        m_bitmap_lut_path_full = "/app_demo/app_assets/" + bitmap_lut_path;
        lut_path = m_bitmap_lut_path_full.c_str();
    }
    osd_device.Initialize(m_width, m_height, lut_path);
}


void VISUALIZER::Draw() {
    std::vector<sst::device::osd::OsdQuadRangle> quad_rangle_vec;

	sst::device::osd::OsdQuadRangle q;
	q.color = 0;
	q.box = {100, 100, 200, 200};
	q.border = 3;
	q.alpha = fdevice::TYPE_ALPHA75;
	q.type = fdevice::TYPE_HOLLOW;
	quad_rangle_vec.emplace_back(q);

    osd_device.Draw(quad_rangle_vec);
}

// 单色框绘制（兼容旧调用）
void VISUALIZER::Draw(const std::vector<std::array<float, 4>>& boxes) {
    DrawDetections(boxes, {});
}

// 同帧绘制正常框(绿) + 报警框(红)到 layer 0
void VISUALIZER::DrawDetections(const std::vector<std::array<float, 4>>& normal_boxes,
                                const std::vector<std::array<float, 4>>& alarm_boxes) {
    std::vector<sst::device::osd::OsdQuadRangle> quad_rangle_vec;

    auto append = [&](const std::vector<std::array<float, 4>>& boxes, int color_idx) {
        for (const auto& b : boxes) {
            sst::device::osd::OsdQuadRangle q;
            q.box = {b[0], b[1], b[2], b[3]};
            q.color = color_idx;
            q.border = coco_config::kBoxBorderPx;
            q.alpha = fdevice::TYPE_ALPHA100;
            q.type = fdevice::TYPE_HOLLOW;
            q.layer_id = DETECTION_LAYER_ID;
            quad_rangle_vec.emplace_back(q);
        }
    };
    append(normal_boxes, coco_config::kColorNormalBox);
    append(alarm_boxes,  coco_config::kColorAlarmBox);

    osd_device.Draw(quad_rangle_vec, DETECTION_LAYER_ID);
}

void VISUALIZER::DrawZoneRect(int x1, int y1, int x2, int y2) {
    if (x1 > x2) std::swap(x1, x2);
    if (y1 > y2) std::swap(y1, y2);
    x1 = std::max(0, std::min(x1, m_width - 1));
    y1 = std::max(0, std::min(y1, m_height - 1));
    x2 = std::max(0, std::min(x2, m_width - 1));
    y2 = std::max(0, std::min(y2, m_height - 1));

    std::vector<std::array<float, 4>> boxes;
    boxes.push_back({static_cast<float>(x1), static_cast<float>(y1),
                     static_cast<float>(x2), static_cast<float>(y2)});
    osd_device.Draw(boxes,
                    coco_config::kZoneBorderPx,
                    ZONE_LAYER_ID,
                    fdevice::TYPE_HOLLOW,
                    fdevice::TYPE_ALPHA100,
                    coco_config::kColorZoneBox);
}

// OSD不支持多段线，用多边形的外接矩形框代替显示；实际判断仍走 point-in-polygon
void VISUALIZER::DrawZonePolygonBBox(const std::vector<std::array<int, 2>>& points) {
    if (points.size() < 3) return;
    int xmin = points[0][0];
    int ymin = points[0][1];
    int xmax = xmin;
    int ymax = ymin;
    for (const auto& p : points) {
        xmin = std::min(xmin, p[0]);
        ymin = std::min(ymin, p[1]);
        xmax = std::max(xmax, p[0]);
        ymax = std::max(ymax, p[1]);
    }
    DrawZoneRect(xmin, ymin, xmax, ymax);
}

// 用小实心方块沿边采样拟合多边形轮廓，MAX_QUADS上限防爆DMA buffer
void VISUALIZER::DrawZonePolygon(const std::vector<std::array<int, 2>>& points) {
    if (points.size() < 3) return;

    const int T = coco_config::kZoneBorderPx;
    const int MAX_QUADS = 450;
    const std::size_t n = points.size();

    int total_len = 0;
    for (std::size_t i = 0; i < n; ++i) {
        int dx = points[(i + 1) % n][0] - points[i][0];
        int dy = points[(i + 1) % n][1] - points[i][1];
        total_len += std::max(std::abs(dx), std::abs(dy));
    }
    if (total_len == 0) return;

    const int step = std::max(T, (total_len + MAX_QUADS - 1) / MAX_QUADS);
    const int half = step / 2;

    std::vector<sst::device::osd::OsdQuadRangle> quads;
    quads.reserve(MAX_QUADS);

    for (std::size_t i = 0; i < n; ++i) {
        const int ax = points[i][0];
        const int ay = points[i][1];
        const int bx = points[(i + 1) % n][0];
        const int by = points[(i + 1) % n][1];
        const int dx = bx - ax;
        const int dy = by - ay;
        const int len = std::max(std::abs(dx), std::abs(dy));
        if (len == 0) continue;

        for (int s = 0; s <= len; s += step) {
            const float t  = static_cast<float>(s) / static_cast<float>(len);
            const int   px = ax + static_cast<int>(t * dx + 0.5f);
            const int   py = ay + static_cast<int>(t * dy + 0.5f);

            const int x1 = std::max(0, px - half);
            const int y1 = std::max(0, py - half);
            const int x2 = std::min(m_width  - 1, px + half);
            const int y2 = std::min(m_height - 1, py + half);
            if (x2 <= x1 || y2 <= y1) continue;

            sst::device::osd::OsdQuadRangle q;
            q.box      = {static_cast<float>(x1), static_cast<float>(y1),
                          static_cast<float>(x2), static_cast<float>(y2)};
            q.color    = coco_config::kColorZoneBox;
            q.border   = 0;
            q.alpha    = fdevice::TYPE_ALPHA100;
            q.type     = fdevice::TYPE_SOLID;
            q.layer_id = ZONE_LAYER_ID;
            quads.emplace_back(q);
        }
    }

    if (!quads.empty())
        osd_device.Draw(quads, ZONE_LAYER_ID);
}

void VISUALIZER::ClearZoneOverlay() {
    osd_device.ClearLayer(ZONE_LAYER_ID);
}

void VISUALIZER::ShowAlarmIndicator(int pos_x, int pos_y) {
    if (m_alarm_indicator_visible) return;
    const std::string path = "/app_demo/app_assets/" + std::string(coco_config::kAlarmBitmapName);
    osd_device.DrawTexture(path.c_str(), nullptr, ALARM_LAYER_ID, pos_x, pos_y);
    m_alarm_indicator_visible = true;
}

void VISUALIZER::HideAlarmIndicator() {
    if (!m_alarm_indicator_visible) return;
    osd_device.ClearLayer(ALARM_LAYER_ID);
    m_alarm_indicator_visible = false;
}

void VISUALIZER::DrawFixedSquare(int x_min, int y_min, int x_max, int y_max, int layer_id) {
    int abs_x_min = x_min;
    int abs_y_min = y_min;
    int abs_x_max = x_max;
    int abs_y_max = y_max;
    if (abs_x_min > abs_x_max) std::swap(abs_x_min, abs_x_max);
    if (abs_y_min > abs_y_max) std::swap(abs_y_min, abs_y_max);
    abs_x_min = std::max(0, std::min(abs_x_min, m_width - 1));
    abs_y_min = std::max(0, std::min(abs_y_min, m_height - 1));
    abs_x_max = std::max(0, std::min(abs_x_max, m_width - 1));
    abs_y_max = std::max(0, std::min(abs_y_max, m_height - 1));

    std::vector<std::array<float, 4>> square_box;
    square_box.push_back({static_cast<float>(abs_x_min),
                         static_cast<float>(abs_y_min),
                         static_cast<float>(abs_x_max),
                         static_cast<float>(abs_y_max)});
    osd_device.Draw(square_box, 0, layer_id,
                    fdevice::TYPE_SOLID, fdevice::TYPE_ALPHA100, 2);
    std::cout << "[VISUALIZER] Fixed square drawn: (" << abs_x_min << ", " << abs_y_min
              << ") to (" << abs_x_max << ", " << abs_y_max << "), layer_id=" << layer_id << std::endl;
}

void VISUALIZER::DrawBitmap(const std::string& bitmap_path, const std::string& lut_path,
                            int pos_x, int pos_y, int layer_id) {
    std::string full_bitmap_path = "/app_demo/app_assets/" + bitmap_path;
    const char* full_lut_path = nullptr;
    osd_device.DrawTexture(full_bitmap_path.c_str(), full_lut_path, layer_id, pos_x, pos_y);
}

void VISUALIZER::Release() {
    osd_device.Release();
}
