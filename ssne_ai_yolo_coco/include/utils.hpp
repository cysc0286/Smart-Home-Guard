/*
 * @Filename: utils.hpp
 * @Author: Hongying He
 * @Email: hongying.he@smartsenstech.com
 * @Date: 2025-12-30 14-57-47
 * @Copyright (c) 2025 SmartSens
 */
#pragma once

#include "osd-device.hpp"
#include <algorithm>

namespace utils {
  // 人脸检测模型所需的函数
  /* 合并两段结果 */
  void Merge(FaceDetectionResult* result, size_t low, size_t mid, size_t high);
  /* 归并排序算法 */
  void MergeSort(FaceDetectionResult* result, size_t low, size_t high);
  /* 对检测结果进行排序 */
  void SortDetectionResult(FaceDetectionResult* result);
  /* 非极大值抑制 */
  void NMS(FaceDetectionResult* result, float iou_threshold, int top_k);
} // namespace utils

class VISUALIZER {
  public:
    void Initialize(std::array<int, 2>& in_img_shape, const std::string& bitmap_lut_path = "");
    void Release();
    void Draw();
    void Draw(const std::vector<std::array<float, 4>>& boxes);

    // OSD 图层布局
    static const int DETECTION_LAYER_ID = 0;  // 检测框（白/红混合）
    static const int ZONE_LAYER_ID      = 1;  // 危险区域黄色框
    static const int ALARM_LAYER_ID     = 2;  // 英文 ALERT 报警位图

    /**
     * @brief 绘制检测框（区分正常/报警），单次调用同时输出两种颜色到同一图层
     * @param normal_boxes 正常检测框（白/绿色）
     * @param alarm_boxes 进入危险区域的报警框（红色）
     */
    void DrawDetections(const std::vector<std::array<float, 4>>& normal_boxes,
                        const std::vector<std::array<float, 4>>& alarm_boxes);

    /**
     * @brief 绘制危险区域黄色矩形外框（hollow）
     */
    void DrawZoneRect(int x1, int y1, int x2, int y2);

    /**
     * @brief 绘制多边形外接黄色矩形框（OSD 不支持多段线，用 bbox 近似）
     */
    void DrawZonePolygonBBox(const std::vector<std::array<int, 2>>& points);

    /** 清空危险区域图层 */
    void ClearZoneOverlay();

    /**
     * @brief 显示英文报警位图（ALERT）
     */
    void ShowAlarmIndicator(int pos_x = 30, int pos_y = 30);

    /** 隐藏英文报警位图 */
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
};
