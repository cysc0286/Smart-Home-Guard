#pragma once

#include <array>
#include <string>
#include <vector>

#include "common.hpp"
#include "coco_config.hpp"
#include "coco_types.hpp"

class COCO_DETECTOR {
 public:
  std::string ModelName() const { return "yolov8n_coco_head6"; }

  void Initialize(std::string& model_path,
                  std::array<int, 2>* in_img_shape,
                  std::array<int, 2>* in_det_shape);

  void Predict(ssne_tensor_t* img_in,
               CocoDetectionResult* result,
               float conf_threshold = coco_config::kConfThreshold);

  void Release();

 private:
  float DecodeDFLEdge(const float* logits) const;
  float Sigmoid(float x) const;
  float IoU(const std::array<float, 4>& a, const std::array<float, 4>& b) const;

  void BuildDetectionsForScale(const float* box_ptr,
                               const float* cls_ptr,
                               int h, int w, int stride,
                               float conf_threshold,
                               std::vector<CocoDetection>* detections) const;

  void ApplyNms(std::vector<CocoDetection>* detections) const;

  uint16_t     model_id = 0;
  ssne_tensor_t inputs[1];
  ssne_tensor_t outputs[coco_config::kNumHeads];
  AiPreprocessPipe pipe_offline = GetAIPreprocessPipe();

  std::array<int, 2> img_shape = {0, 0};
  std::array<int, 2> det_shape = {0, 0};
  float w_scale = 1.f;
  float h_scale = 1.f;
};
