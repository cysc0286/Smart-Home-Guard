#pragma once

#include <array>

namespace coco_config {

// Camera / image pipeline (same geometry as guard project)
static const std::array<int, 2> kImageShape  = {1920, 1080};
static const std::array<int, 2> kCropShape   = {1440, 1080};
static const int                kCropOffsetX = 240;
static const std::array<int, 2> kDetShape    = {256, 256};

// Model path on the board filesystem
static const char* kModelPath = "/app_demo/app_assets/models/yolov8n_coco_256.m1model";

// Snapshot service for the PC-side zone planner.
static const int   kSnapshotHttpPort      = 8081;
static const int   kSnapshotUpdateIntervalMs = 500;
static const int   kRunSnapshotUpdateIntervalMs = 5000;
static const bool  kSaveSnapshotFileInRun = false;
static const char* kSnapshotRoute         = "/?action=snapshot";
static const char* kSnapshotAltRoute      = "/latest_snapshot.pgm";
static const char* kSnapshotFilePath      = "/app_demo/latest_snapshot.pgm";
static const char* kZoneConfigPath        = "/app_demo/zone_config.json";
static const int   kSerialPreviewWidth    = 128;
static const int   kSerialPreviewHeight   = 96;
static const bool  kEnableSerialSetup     = true;
static const int   kSerialBaudrate        = 115200;

// Acceptance/performance mode keeps the field demo observable while reducing
// UART/file I/O in the hot path used by scoring.
static const bool  kEnableAcceptanceMode  = true;
static const int   kAcceptanceDurationMs  = 60000;
static const bool  kVerboseDetectionLog   = false;
static const int   kDetectionSummaryLogMs = 2000;

// Detection parameters
static const int   kNumClasses      = 80;
static const int   kRegMax          = 16;   // YOLOv8 DFL bins
static const int   kNumHeads        = 6;    // P3_box,P3_cls,P4_box,P4_cls,P5_box,P5_cls
static const float kConfThreshold   = 0.3f;
static const float kNmsThreshold    = 0.45f;
static const int   kKeepTopK        = 30;
static const int   kAlarmConfirmMs  = 800;
static const int   kAlarmClearMs    = 500;
static const int   kAlarmHoldMs     = 0;      // 检测丢失立即停止报警
static const int   kLowLightAlarmHoldMs = 0;
static const int   kEnvLowLightY     = 40;
static const int   kEnvBrightY       = 210;
static const int   kEnvPolicyStableSamples = 3;
static const int   kEnvLogIntervalMs = 2000;
static const float kLowLightConfThreshold = 0.25f;
static const float kBrightConfThreshold   = 0.35f;

// OSD color LUT indices (colorLUT.sscl). Tweak if the rendered hue does not match.
static const int   kColorAlarmBox   = 0;   // Red    - object inside danger zone
static const int   kColorNormalBox  = 1;   // Green  - normal detection ("white" alternative)
static const int   kColorZoneBox    = 3;   // Yellow - danger zone outline
static const int   kZoneBorderPx    = 5;   // 危险区域黄框线宽
static const int   kBoxBorderPx     = 5;   // 检测框线宽：3 -> 5，大框轮廓更醒目

// Alarm indicator bitmap (English ALERT). Shown top-left when alarm is active.
static const char* kAlarmBitmapName = "alert.ssbmp";
static const int   kAlarmBitmapPosX = 30;
static const int   kAlarmBitmapPosY = 30;

static const std::array<int, 3> kStrides = {8, 16, 32};

// 80 COCO class names (index matches model output class id)
static const std::array<const char*, 80> kClassNames = {{
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
}};

}  // namespace coco_config
