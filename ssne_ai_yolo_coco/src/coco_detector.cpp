#include "../include/coco_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

void COCO_DETECTOR::Initialize(std::string& model_path,
                                std::array<int, 2>* in_img_shape,
                                std::array<int, 2>* in_det_shape) {
  img_shape = *in_img_shape;
  det_shape = *in_det_shape;
  w_scale = static_cast<float>(img_shape[0]) / static_cast<float>(det_shape[0]);
  h_scale = static_cast<float>(img_shape[1]) / static_cast<float>(det_shape[1]);

  char* model_path_char = const_cast<char*>(model_path.c_str());
  model_id = ssne_loadmodel(model_path_char, SSNE_STATIC_ALLOC);
  printf("[INFO] Model loaded, model_id = %d\n", (int)model_id);

  // Query model's expected input dtype (SSNE_UINT8=0 / SSNE_INT8=1 / SSNE_FLOAT32=2)
  int input_dtype = SSNE_FLOAT32;
  ssne_get_model_input_dtype(model_id, &input_dtype);
  printf("[INFO] Model input dtype = %d (0=uint8, 1=int8, 2=float32)\n", input_dtype);

  // Align preprocessing pipeline normalization to model requirements
  SetNormalize(pipe_offline, model_id);

  const uint32_t det_width  = static_cast<uint32_t>(det_shape[0]);
  const uint32_t det_height = static_cast<uint32_t>(det_shape[1]);
  inputs[0] = create_tensor(det_width, det_height, SSNE_RGB, SSNE_BUF_AI);
  // Set tensor dtype to match model expectation to avoid "Wrong input tensor!" error
  set_data_type(inputs[0], static_cast<uint8_t>(input_dtype));

  printf("[INFO] COCO_DETECTOR initialized with input shape [%d, %d]\n",
         det_shape[0], det_shape[1]);
  printf("[INFO] Expect Head6 outputs: P3_box, P3_cls, P4_box, P4_cls, P5_box, P5_cls\n");
}

float COCO_DETECTOR::Sigmoid(float x) const {
  return 1.0f / (1.0f + std::exp(-x));
}

float COCO_DETECTOR::DecodeDFLEdge(const float* logits) const {
  float max_logit = logits[0];
  for (int i = 1; i < coco_config::kRegMax; ++i) {
    max_logit = std::max(max_logit, logits[i]);
  }

  float exp_values[coco_config::kRegMax];
  float sum_exp = 0.0f;
  for (int i = 0; i < coco_config::kRegMax; ++i) {
    exp_values[i] = std::exp(logits[i] - max_logit);
    sum_exp += exp_values[i];
  }

  float value = 0.0f;
  for (int i = 0; i < coco_config::kRegMax; ++i) {
    value += (exp_values[i] / sum_exp) * static_cast<float>(i);
  }
  return value;
}

float COCO_DETECTOR::IoU(const std::array<float, 4>& a,
                          const std::array<float, 4>& b) const {
  const float x1 = std::max(a[0], b[0]);
  const float y1 = std::max(a[1], b[1]);
  const float x2 = std::min(a[2], b[2]);
  const float y2 = std::min(a[3], b[3]);
  const float inter_w = std::max(0.0f, x2 - x1);
  const float inter_h = std::max(0.0f, y2 - y1);
  const float inter   = inter_w * inter_h;
  const float area_a  = std::max(0.0f, a[2] - a[0]) * std::max(0.0f, a[3] - a[1]);
  const float area_b  = std::max(0.0f, b[2] - b[0]) * std::max(0.0f, b[3] - b[1]);
  return inter / std::max(1e-6f, area_a + area_b - inter);
}

void COCO_DETECTOR::BuildDetectionsForScale(const float* box_ptr,
                                             const float* cls_ptr,
                                             int h, int w, int stride,
                                             float conf_threshold,
                                             std::vector<CocoDetection>* detections) const {
  const int box_channels = 4 * coco_config::kRegMax;
  const int cls_channels = coco_config::kNumClasses;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int cls_offset = (y * w + x) * cls_channels;

      int   best_cls   = 0;
      float best_score = Sigmoid(cls_ptr[cls_offset]);
      for (int c = 1; c < cls_channels; ++c) {
        const float score = Sigmoid(cls_ptr[cls_offset + c]);
        if (score > best_score) {
          best_score = score;
          best_cls   = c;
        }
      }

      if (best_score < conf_threshold) {
        continue;
      }

      // Only keep person(0), cat(15), dog(16)
      if (best_cls != 0 && best_cls != 15 && best_cls != 16) {
        continue;
      }

      const int box_offset = (y * w + x) * box_channels;
      float ltrb[4] = {0.f, 0.f, 0.f, 0.f};
      for (int edge = 0; edge < 4; ++edge) {
        ltrb[edge] = DecodeDFLEdge(box_ptr + box_offset + edge * coco_config::kRegMax);
      }

      const float anchor_x = (static_cast<float>(x) + 0.5f) * static_cast<float>(stride);
      const float anchor_y = (static_cast<float>(y) + 0.5f) * static_cast<float>(stride);
      float x1 = (anchor_x - ltrb[0]) * w_scale;
      float y1 = (anchor_y - ltrb[1]) * h_scale;
      float x2 = (anchor_x + ltrb[2]) * w_scale;
      float y2 = (anchor_y + ltrb[3]) * h_scale;

      x1 = std::max(0.0f, std::min(x1, static_cast<float>(img_shape[0] - 1)));
      y1 = std::max(0.0f, std::min(y1, static_cast<float>(img_shape[1] - 1)));
      x2 = std::max(0.0f, std::min(x2, static_cast<float>(img_shape[0] - 1)));
      y2 = std::max(0.0f, std::min(y2, static_cast<float>(img_shape[1] - 1)));

      CocoDetection det;
      det.box_xyxy = {x1, y1, x2, y2};
      det.score    = best_score;
      det.class_id = best_cls;
      det.label    = coco_config::kClassNames[best_cls];
      detections->push_back(det);
    }
  }
}

void COCO_DETECTOR::ApplyNms(std::vector<CocoDetection>* detections) const {
  std::sort(detections->begin(), detections->end(),
            [](const CocoDetection& a, const CocoDetection& b) {
              return a.score > b.score;
            });

  std::vector<int> suppressed(detections->size(), 0);
  std::vector<CocoDetection> kept;
  kept.reserve(detections->size());

  for (size_t i = 0; i < detections->size(); ++i) {
    if (suppressed[i] == 1) continue;
    kept.push_back(detections->at(i));
    for (size_t j = i + 1; j < detections->size(); ++j) {
      if (suppressed[j] == 1) continue;
      if (detections->at(i).class_id != detections->at(j).class_id) continue;
      if (IoU(detections->at(i).box_xyxy, detections->at(j).box_xyxy) >
          coco_config::kNmsThreshold) {
        suppressed[j] = 1;
      }
    }
    if (static_cast<int>(kept.size()) >= coco_config::kKeepTopK) break;
  }

  detections->swap(kept);
}

void COCO_DETECTOR::Predict(ssne_tensor_t* img_in,
                             CocoDetectionResult* result,
                             float conf_threshold) {
  result->Clear();

  const int ret = RunAiPreprocessPipe(pipe_offline, *img_in, inputs[0]);
  if (ret != 0) {
    printf("[ERROR] Failed to run AI preprocess pipe: %d\n", ret);
    return;
  }

  if (ssne_inference(model_id, 1, inputs)) {
    fprintf(stderr, "ssne inference fail!\n");
    return;
  }

  ssne_getoutput(model_id, coco_config::kNumHeads, outputs);

  std::vector<CocoDetection> detections;
  detections.reserve(256);

  for (int scale_idx = 0; scale_idx < 3; ++scale_idx) {
    const int box_idx = scale_idx * 2;
    const int cls_idx = scale_idx * 2 + 1;
    const int h       = get_height(outputs[box_idx]);
    const int w       = get_width(outputs[box_idx]);
    const int stride  = coco_config::kStrides[scale_idx];

    const float* box_ptr = static_cast<float*>(get_data(outputs[box_idx]));
    const float* cls_ptr = static_cast<float*>(get_data(outputs[cls_idx]));
    BuildDetectionsForScale(box_ptr, cls_ptr, h, w, stride, conf_threshold, &detections);
  }

  ApplyNms(&detections);
  result->detections = detections;
}

void COCO_DETECTOR::Release() {
  release_tensor(inputs[0]);
  for (int i = 0; i < coco_config::kNumHeads; ++i) {
    release_tensor(outputs[i]);
  }
  ReleaseAIPreprocessPipe(pipe_offline);
}
