/*
 * utils.hpp — OSD可视化封装 + 人脸检测排序/NMS工具
 */
#pragma once

#include "osd-device.hpp"
#include <algorithm>

namespace utils {
  void Merge(FaceDetectionResult* result, size_t low, size_t mid, size_t high);
  void MergeSort(FaceDetectionResult* result, size_t low, size_t high);
  void SortDetectionResult(FaceDetectionResult* result);
  void NMS(FaceDetectionResult* result, float iou_threshold, int top_k);
} // namespace utils

class VISUALIZER {
  public:
    void Initialize(std::array<int, 2>& in_img_shape, const std::string& bitmap_lut_path = "");
    void Release();
    void Draw();
    void Draw(const std::vector<std::array<float, 4>>& boxes);

    // OSD 图层分配: 0=检测框  1=矩形危险区域  2=多边形危险区域位图  3=报警位图
    static const int DETECTION_LAYER_ID = 0;
    static const int ZONE_LAYER_ID      = 1;
    static const int ZONE_BITMAP_LAYER_ID = 2;
    static const int ALARM_LAYER_ID     = 3;

    // 正常框(绿) + 报警框(红) 同帧绘制到 layer 0
    void DrawDetections(const std::vector<std::array<float, 4>>& normal_boxes,
                        const std::vector<std::array<float, 4>>& alarm_boxes);

    void DrawZoneRect(int x1, int y1, int x2, int y2);
    // 外接矩形仅作为RLE多边形显示失败时的降级路径。
    void DrawZonePolygonBBox(const std::vector<std::array<int, 2>>& points);
    bool DrawZonePolygon(const std::vector<std::array<int, 2>>& points);
    void ClearZoneOverlay();

    void ShowAlarmIndicator(int pos_x = 30, int pos_y = 30);
    void HideAlarmIndicator();

    void DrawFixedSquare(int x_min, int y_min, int x_max, int y_max, int layer_id = 1);
    void DrawBitmap(const std::string& bitmap_path, const std::string& lut_path = "",
                    int pos_x = 0, int pos_y = 0, int layer_id = 2);

  private:
    sst::device::osd::OsdDevice osd_device;
    int m_width = 0;
    int m_height = 0;
    std::string m_bitmap_lut_path_full;
    bool m_alarm_indicator_visible = false;
    std::string m_zone_bitmap_path = "/tmp/smart_home_guard_zone.ssbmp";
};
