/*
 * @Filename: utils.cpp
 * @Author: Hongying He
 * @Email: hongying.he@smartsenstech.com
 * @Date: 2025-12-30 14-57-47
 * @Copyright (c) 2025 SmartSens
 */
#include "../include/utils.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdio>

namespace utils {

/**
 * @brief 归并排序的合并操作
 * @param result 人脸检测结果结构体指针
 * @param low 合并区间的起始索引
 * @param mid 合并区间的中间索引
 * @param high 合并区间的结束索引
 * @description 将两个已排序的子数组合并成一个有序数组，按照分数从高到低排序
 */
void Merge(FaceDetectionResult* result, size_t low, size_t mid, size_t high) {
  // 获取检测框和分数的引用
  std::vector<std::array<float, 4>>& boxes = result->boxes;
  std::vector<float>& scores = result->scores;
  // 创建临时副本用于合并操作
  std::vector<std::array<float, 4>> temp_boxes(boxes);
  std::vector<float> temp_scores(scores);
  size_t i = low;      // 左半部分的索引
  size_t j = mid + 1;  // 右半部分的索引
  size_t k = i;        // 合并结果的索引
  // 合并两个有序子数组，选择分数更高的检测框
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
  // 将左半部分剩余元素复制到结果中
  while (i <= mid) {
    scores[k] = temp_scores[i];
    boxes[k] = temp_boxes[i];
    k++;
    i++;
  }
  // 将右半部分剩余元素复制到结果中
  while (j <= high) {
    scores[k] = temp_scores[j];
    boxes[k] = temp_boxes[j];
    k++;
    j++;
  }
}

/**
 * @brief 归并排序递归函数
 * @param result 人脸检测结果结构体指针
 * @param low 排序区间的起始索引
 * @param high 排序区间的结束索引
 * @description 使用归并排序算法对检测结果按分数从高到低排序
 */
void MergeSort(FaceDetectionResult* result, size_t low, size_t high) {
  if (low < high) {
    size_t mid = (high - low) / 2 + low;  // 计算中间索引
    MergeSort(result, low, mid);          // 递归排序左半部分
    MergeSort(result, mid + 1, high);     // 递归排序右半部分
    Merge(result, low, mid, high);        // 合并两个有序子数组
  }
}

/**
 * @brief 对检测结果进行排序
 * @param result 人脸检测结果结构体指针
 * @description 按照检测分数从高到低对检测结果进行排序
 */
void SortDetectionResult(FaceDetectionResult* result) {
  size_t low = 0;
  size_t high = result->scores.size();
  if (high == 0) {
    return;  // 如果没有检测结果，直接返回
  }
  high = high - 1;  // 转换为最大索引
  MergeSort(result, low, high);
}

/**
 * @brief 非极大值抑制（NMS）算法
 * @param result 人脸检测结果结构体指针
 * @param iou_threshold IoU阈值，超过此阈值的重叠框将被抑制
 * @param top_k 保留前k个检测结果
 * @description 去除重叠的检测框，保留分数最高的检测结果
 */
void NMS(FaceDetectionResult* result, float iou_threshold, int top_k) {
  // 根据检测分数对检测结果进行排序整理
  SortDetectionResult(result);

  // 保留其中的top-K个值
  int res_count = static_cast<int>(result->boxes.size());
  result->Resize(std::min(res_count, top_k));

  // 计算每个检测框的面积
  std::vector<float> area_of_boxes(result->boxes.size());
  std::vector<int> suppressed(result->boxes.size(), 0);  // 标记被抑制的框
  for (size_t i = 0; i < result->boxes.size(); ++i) {
    // 计算检测框面积：(x2-x1+1) * (y2-y1+1)
    area_of_boxes[i] = (result->boxes[i][2] - result->boxes[i][0] + 1) *
                       (result->boxes[i][3] - result->boxes[i][1] + 1);
  }

  // NMS过程：遍历所有检测框，抑制与高分框重叠度高的低分框
  for (size_t i = 0; i < result->boxes.size(); ++i) {
    if (suppressed[i] == 1) {
      continue;  // 跳过已被抑制的框
    }
    for (size_t j = i + 1; j < result->boxes.size(); ++j) {
      if (suppressed[j] == 1) {
        continue;  // 跳过已被抑制的框
      }
      // 计算两个框的交集区域
      float xmin = std::max(result->boxes[i][0], result->boxes[j][0]);
      float ymin = std::max(result->boxes[i][1], result->boxes[j][1]);
      float xmax = std::min(result->boxes[i][2], result->boxes[j][2]);
      float ymax = std::min(result->boxes[i][3], result->boxes[j][3]);
      float overlap_w = std::max(0.0f, xmax - xmin + 1);
      float overlap_h = std::max(0.0f, ymax - ymin + 1);
      float overlap_area = overlap_w * overlap_h;
      // 计算IoU（交并比）：交集面积 / 并集面积
      float overlap_ratio =
          overlap_area / (area_of_boxes[i] + area_of_boxes[j] - overlap_area);
      // 如果IoU超过阈值，抑制低分框
      if (overlap_ratio > iou_threshold) {
        suppressed[j] = 1;
      }
    }
  }
  // 备份原始结果
  FaceDetectionResult backup(*result);
  int landmarks_per_face = result->landmarks_per_face;

  result->Clear();
  // 在调用Reserve方法之前，不要忘记重置landmarks_per_face
  result->landmarks_per_face = landmarks_per_face;
  result->Reserve(suppressed.size());
  // 只保留未被抑制的检测结果
  for (size_t i = 0; i < suppressed.size(); ++i) {
    if (suppressed[i] == 1) {
      continue;  // 跳过被抑制的框
    }
    result->boxes.emplace_back(backup.boxes[i]);
    result->scores.push_back(backup.scores[i]);
    // 如果有关键点信息，也一并复制
    if (result->landmarks_per_face > 0) {
      for (size_t j = 0; j < result->landmarks_per_face; ++j) {
        result->landmarks.emplace_back(
            backup.landmarks[i * result->landmarks_per_face + j]);
      }
    }
  }
}
}  // namespace utils


/**
 * @brief 释cFaceDetectionResult的内存
 * @description 使用swap技巧释cvector占用的内存
 */
void FaceDetectionResult::Free() {
  std::vector<std::array<float, 4>>().swap(boxes);
  std::vector<float>().swap(scores);
  std::vector<std::array<float, 2>>().swap(landmarks);
  landmarks_per_face = 0;
}

/**
 * @brief 清空FaceDetectionResult的内容
 * @description 清空所有检测框、分数和关键点，但保留内存分配
 */
void FaceDetectionResult::Clear() {
  boxes.clear();
  scores.clear();
  landmarks.clear();
  landmarks_per_face = 0;
}

/**
 * @brief 预分配内存空间
 * @param size 要保留的元素数量
 * @description 为检测框、分数和关键点预分配内存，提高性能
 */
void FaceDetectionResult::Reserve(int size) {
  boxes.reserve(size);
  scores.reserve(size);
  if (landmarks_per_face > 0) {
    landmarks.reserve(size * landmarks_per_face);
  }
}

/**
 * @brief 调整FaceDetectionResult的大小
 * @param size 新的元素数量
 * @description 调整检测框、分数和关键点的数量
 */
void FaceDetectionResult::Resize(int size) {
  boxes.resize(size);
  scores.resize(size);
  if (landmarks_per_face > 0) {
    landmarks.resize(size * landmarks_per_face);
  }
}

/**
 * @brief FaceDetectionResult的拷贝构造函数
 * @param res 要拷贝的FaceDetectionResult对象
 * @description 深拷贝检测结果的所有数据
 */
FaceDetectionResult::FaceDetectionResult(const FaceDetectionResult& res) {
  boxes.assign(res.boxes.begin(), res.boxes.end());
  landmarks.assign(res.landmarks.begin(), res.landmarks.end());
  scores.assign(res.scores.begin(), res.scores.end());
  landmarks_per_face = res.landmarks_per_face;
}


/**
 * @brief OSD可视化器初始化函数
 * @param in_img_shape 图像尺寸 [宽度, 高度]
 * @description 初始化OSD设备
 */
void VISUALIZER::Initialize(std::array<int, 2>& in_img_shape, const std::string& bitmap_lut_path) {
    // 初始化OSD设备，配置图像宽度和高度
    m_width = in_img_shape[0];
    m_height = in_img_shape[1];
    // 如果提供了位图LUT路径，在初始化时加载
    // 位图LUT和默认LUT都在app_assets目录下，使用相同的路径格式
    // 注意：需要将完整路径保存为成员变量，确保生命周期
    const char* lut_path = nullptr;
    if (!bitmap_lut_path.empty()) {
        m_bitmap_lut_path_full = "/app_demo/app_assets/" + bitmap_lut_path;
        lut_path = m_bitmap_lut_path_full.c_str();
    }
    osd_device.Initialize(m_width, m_height, lut_path);
}


/**
 * @brief 绘制测试矩形框（用于测试OSD功能）
 * @description 在OSD上绘制一个固定位置的测试矩形框
 */
void VISUALIZER::Draw() {
    std::vector<sst::device::osd::OsdQuadRangle> quad_rangle_vec;

	sst::device::osd::OsdQuadRangle q;

	// 配置测试矩形框参数
	q.color = 0;                         // 颜色索引0
	q.box = {100, 100, 200, 200};        // 矩形框坐标 [xmin, ymin, xmax, ymax]
	q.border = 3;                        // 边框宽度3像素
	q.alpha = fdevice::TYPE_ALPHA75;     // 透明度75%
	q.type = fdevice::TYPE_HOLLOW;       // 空心矩形
	quad_rangle_vec.emplace_back(q);


    // 调用OSD设备绘制测试矩形框
    osd_device.Draw(quad_rangle_vec);
}

/**
 * @brief 根据检测框绘制OSD矩形
 * @param boxes 检测框向量，每个元素为[xmin, ymin, xmax, ymax]
 * @description 将所有检测到的人脸框绘制到OSD显示层（使用layer 0，不影响固定正方形所在的layer 1）
 */
void VISUALIZER::Draw(const std::vector<std::array<float, 4>>& boxes) {

    std::vector<sst::device::osd::OsdQuadRangle> quad_rangle_vec;  // OSD矩形框向量

    // 遍历所有检测框，转换为OSD矩形格式
    for (size_t i = 0; i < boxes.size(); i++) {
        sst::device::osd::OsdQuadRangle q;

        // 将检测框坐标从float转换为int [xmin, ymin, xmax, ymax]
        int xmin = static_cast<int>(boxes[i][0]);  // 左上角x坐标
        int ymin = static_cast<int>(boxes[i][1]);  // 左上角y坐标
        int xmax = static_cast<int>(boxes[i][2]);  // 右下角x坐标
        int ymax = static_cast<int>(boxes[i][3]);  // 右下角y坐标

        q.box = {xmin, ymin, xmax, ymax};  // 设置矩形框坐标

        // 设置矩形框样式参数
        q.color = 2;                         // 颜色索引1（不同于测试框）
        q.border = 3;                        // 边框宽度3像素
        q.alpha = fdevice::TYPE_ALPHA75;     // 透明度75%
        q.type = fdevice::TYPE_HOLLOW;       // 空心矩形
        q.layer_id = DETECTION_LAYER_ID;     // 使用layer 0，避免影响固定正方形的layer 1
        quad_rangle_vec.emplace_back(q);     // 添加到矩形框向量
    }
    // 调用OSD设备绘制所有矩形框到指定图层（layer 0）
    // 使用指定图层版本，避免清除所有图层（包括layer 1的固定正方形）
    osd_device.Draw(quad_rangle_vec, DETECTION_LAYER_ID);
}
/**
 * @brief 绘制固定实心正方形
 * @param x_min 左上角X坐标（绝对坐标，左上角为原点）
 * @param y_min 左上角Y坐标（绝对坐标，左上角为原点）
 * @param x_max 右下角X坐标（绝对坐标，左上角为原点）
 * @param y_max 右下角Y坐标（绝对坐标，左上角为原点）
 * @param layer_id 使用的OSD layer ID（建议使用1，避免与检测框冲突）
 * @description 绘制一次后，正方形会一直显示，不随帧数消失。坐标系统以画面左上角为原点(0,0)，X向右为正，Y向下为正。
 */
void VISUALIZER::DrawFixedSquare(int x_min, int y_min, int x_max, int y_max, int layer_id) {
    // 确保坐标顺序正确（xmin < xmax, ymin < ymax）
    int abs_x_min = x_min;
    int abs_y_min = y_min;
    int abs_x_max = x_max;
    int abs_y_max = y_max;
    if (abs_x_min > abs_x_max) std::swap(abs_x_min, abs_x_max);
    if (abs_y_min > abs_y_max) std::swap(abs_y_min, abs_y_max);
    // 确保坐标在画面范围内
    abs_x_min = std::max(0, std::min(abs_x_min, m_width - 1));
    abs_y_min = std::max(0, std::min(abs_y_min, m_height - 1));
    abs_x_max = std::max(0, std::min(abs_x_max, m_width - 1));
    abs_y_max = std::max(0, std::min(abs_y_max, m_height - 1));
    // 创建正方形框
    std::vector<std::array<float, 4>> square_box;
    square_box.push_back({static_cast<float>(abs_x_min),
                         static_cast<float>(abs_y_min),
                         static_cast<float>(abs_x_max),
                         static_cast<float>(abs_y_max)});
    // 使用指定的layer_id绘制正方形
    osd_device.Draw(square_box,
                    0,                              // 边框宽度0（实心矩形不需要边框）
                    layer_id,                       // 使用指定的layer_id
                    fdevice::TYPE_SOLID,            // 实心矩形
                    fdevice::TYPE_ALPHA100,        // 完全不透明
                    2);                             // 颜色索引2（可根据需要修改）
    std::cout << "[VISUALIZER] Fixed square drawn: (" << abs_x_min << ", " << abs_y_min
              << ") to (" << abs_x_max << ", " << abs_y_max << "), layer_id=" << layer_id << std::endl;
}
/**
 * @brief 绘制位图
 * @param bitmap_path 位图文件路径（相对于app_assets目录）
 * @param lut_path LUT文件路径（相对于app_assets目录，如果为空则使用默认LUT）
 * @param pos_x 位图左上角X坐标（绝对坐标，左上角为原点）
 * @param pos_y 位图左上角Y坐标（绝对坐标，左上角为原点）
 * @param layer_id 使用的OSD layer ID（建议使用2，避免与其他图层冲突）
 * @description 绘制一次后，位图会一直显示，不随帧数消失。坐标系统以画面左上角为原点(0,0)，X向右为正，Y向下为正。
 */
void VISUALIZER::DrawBitmap(const std::string& bitmap_path, const std::string& lut_path,
                            int pos_x, int pos_y, int layer_id) {
    // 构建完整路径
    std::string full_bitmap_path = "/app_demo/app_assets/" + bitmap_path;
    // 构建LUT完整路径（如果提供了）
    // 注意：LUT在初始化时已加载，这里只是用于日志记录
    const char* full_lut_path = nullptr;
    /*std::string lut_full_path;
    if (!lut_path.empty()) {
        lut_full_path = "/app_demo/app_assets/" + lut_path;
        full_lut_path = lut_full_path.c_str();
    }
    std::cout << "[VISUALIZER] Drawing bitmap: " << full_bitmap_path
              << " at position (" << pos_x << ", " << pos_y
              << "), layer_id=" << layer_id << std::endl;*/
    // 调用OSD设备绘制位图（传入绝对坐标）
    //osd_device.DrawTexture(full_bitmap_path.c_str(), full_lut_path, layer_id, pos_x, pos_y);
    osd_device.DrawTexture(full_bitmap_path.c_str(), full_lut_path, layer_id, pos_x, pos_y);
}

/**
 * @brief 释放OSD可视化器资源
 * @description 清理OSD设备占用的资源
 */
void VISUALIZER::Release() {
    osd_device.Release();  // 释放OSD设备资源
}

