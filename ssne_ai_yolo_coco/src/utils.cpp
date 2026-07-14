/*
 * utils.cpp — OSD可视化 + 原始人脸检测排序/NMS工具
 */
#include "../include/utils.hpp"
#include "../include/coco_config.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdio>
#include <cstdint>

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

// 外接矩形降级显示；实际判断始终走 point-in-polygon。
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

// 将多边形轮廓栅格化为透明底SSBMP。相比“小方块拼线”，RLE位图不受每层32个四边形、
// 同一扫描行最多4个四边形的限制；危险区只在配置变化时更新，不进入逐帧检测热路径。
bool VISUALIZER::DrawZonePolygon(const std::vector<std::array<int, 2>>& points) {
    if (points.size() < 3 || m_width <= 0 || m_height <= 0) return false;

    std::vector<std::array<int, 2>> clipped;
    clipped.reserve(points.size());
    for (const auto& p : points) {
        std::array<int, 2> q = {{
            std::max(0, std::min(p[0], m_width - 1)),
            std::max(0, std::min(p[1], m_height - 1))
        }};
        if (clipped.empty() || clipped.back() != q) clipped.push_back(q);
    }
    if (clipped.size() > 1 && clipped.front() == clipped.back()) clipped.pop_back();
    if (clipped.size() < 3) return false;

    long long twice_area = 0;
    for (std::size_t i = 0; i < clipped.size(); ++i) {
        const auto& a = clipped[i];
        const auto& b = clipped[(i + 1) % clipped.size()];
        twice_area += static_cast<long long>(a[0]) * b[1] -
                      static_cast<long long>(b[0]) * a[1];
    }
    if (twice_area == 0) return false;

    int xmin = clipped[0][0], xmax = xmin;
    int ymin = clipped[0][1], ymax = ymin;
    for (const auto& p : clipped) {
        xmin = std::min(xmin, p[0]);
        xmax = std::max(xmax, p[0]);
        ymin = std::min(ymin, p[1]);
        ymax = std::max(ymax, p[1]);
    }

    const int radius = std::max(1, coco_config::kZoneBorderPx / 2);
    xmin = std::max(0, xmin - radius);
    ymin = std::max(0, ymin - radius);
    xmax = std::min(m_width - 1, xmax + radius);
    ymax = std::min(m_height - 1, ymax + radius);
    // YUV相邻像素共享色度，贴图左上角对齐偶数坐标可减少彩色轮廓边缘失真。
    if ((xmin & 1) != 0) --xmin;
    if ((ymin & 1) != 0) --ymin;
    const int bitmap_width = xmax - xmin + 1;
    const int bitmap_height = ymax - ymin + 1;
    if (bitmap_width <= 0 || bitmap_height <= 0) return false;

    const uint8_t transparent_index = 31;
    const uint8_t zone_color = static_cast<uint8_t>(coco_config::kColorZoneBox);
    std::vector<uint8_t> pixels(static_cast<std::size_t>(bitmap_width) * bitmap_height,
                                transparent_index);

    auto stamp = [&](int x, int y) {
        for (int dy = -radius; dy <= radius; ++dy) {
            const int py = y + dy;
            if (py < 0 || py >= bitmap_height) continue;
            for (int dx = -radius; dx <= radius; ++dx) {
                const int px = x + dx;
                if (px < 0 || px >= bitmap_width) continue;
                pixels[static_cast<std::size_t>(py) * bitmap_width + px] = zone_color;
            }
        }
    };

    // 整数Bresenham逐边绘制，顶点处重复stamp可自然封闭接缝。
    for (std::size_t i = 0; i < clipped.size(); ++i) {
        int x0 = clipped[i][0] - xmin;
        int y0 = clipped[i][1] - ymin;
        const int x1 = clipped[(i + 1) % clipped.size()][0] - xmin;
        const int y1 = clipped[(i + 1) % clipped.size()][1] - ymin;
        const int dx = std::abs(x1 - x0);
        const int sx = x0 < x1 ? 1 : -1;
        const int dy = -std::abs(y1 - y0);
        const int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true) {
            stamp(x0, y0);
            if (x0 == x1 && y0 == y1) break;
            const int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    const std::string temp_path = m_zone_bitmap_path + ".tmp";
    std::ofstream file(temp_path.c_str(), std::ios::binary | std::ios::trunc);
    if (!file) {
        std::cerr << "[ZONE][OSD] cannot create " << temp_path << std::endl;
        return false;
    }
    const char magic[4] = {'M', 'B', 'S', 'S'};
    const int32_t header[3] = {bitmap_width, bitmap_height, 32};
    file.write(magic, sizeof(magic));
    file.write(reinterpret_cast<const char*>(header), sizeof(header));
    file.write(reinterpret_cast<const char*>(pixels.data()),
               static_cast<std::streamsize>(pixels.size()));
    file.close();
    if (!file || std::rename(temp_path.c_str(), m_zone_bitmap_path.c_str()) != 0) {
        std::cerr << "[ZONE][OSD] failed to finalize polygon bitmap" << std::endl;
        std::remove(temp_path.c_str());
        return false;
    }

    const bool drawn = osd_device.DrawTexture(m_zone_bitmap_path.c_str(), nullptr,
                                               ZONE_BITMAP_LAYER_ID, xmin, ymin,
                                               fdevice::TYPE_ALPHA100);
    if (drawn) {
        std::cout << "[ZONE][OSD] polygon-rle points=" << clipped.size()
                  << " bitmap=" << bitmap_width << "x" << bitmap_height
                  << " pos=" << xmin << "," << ymin << std::endl;
    }
    return drawn;
}

void VISUALIZER::ClearZoneOverlay() {
    osd_device.ClearLayer(ZONE_LAYER_ID);
    osd_device.ClearLayer(ZONE_BITMAP_LAYER_ID);
}

bool VISUALIZER::ShowStatusCard(const std::string& bitmap_name, int pos_x, int pos_y) {
    if (bitmap_name == m_status_bitmap_name) return true;

    const std::string path = "/app_demo/app_assets/" + bitmap_name;
    osd_device.ClearLayer(STATUS_LAYER_ID);
    if (!osd_device.DrawTexture(path.c_str(), nullptr, STATUS_LAYER_ID, pos_x, pos_y)) {
        m_status_bitmap_name.clear();
        return false;
    }
    m_status_bitmap_name = bitmap_name;
    return true;
}

void VISUALIZER::ClearStatusCard() {
    if (m_status_bitmap_name.empty()) return;
    osd_device.ClearLayer(STATUS_LAYER_ID);
    m_status_bitmap_name.clear();
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
    std::remove(m_zone_bitmap_path.c_str());
    std::remove((m_zone_bitmap_path + ".tmp").c_str());
    osd_device.Release();
}
