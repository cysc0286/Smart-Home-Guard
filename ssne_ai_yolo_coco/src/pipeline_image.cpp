/*
 * pipeline_image.cpp — SSNE在线pipeline图像采集
 */
#include "../include/common.hpp"
#include <iostream>
#include <unistd.h>

bool IMAGEPROCESSOR::Initialize(std::array<int, 2>* in_img_shape)
{
    if (initialized_) {
        return true;
    }

    img_shape = *in_img_shape;

    uint16_t img_width = static_cast<uint16_t>(img_shape[0]);
    uint16_t img_height = static_cast<uint16_t>(img_shape[1]);
    format_online = SSNE_YUV422_16;

    // 1920x1080 -> 左右各裁240px -> 1440x1080
    OnlineSetCrop(kPipeline0, 240, 1680, 0, 1080);
    OnlineSetOutputImage(kPipeline0, format_online, 1440, 1080);

    int res0 = OpenOnlinePipeline(kPipeline0);
    if (res0 != 0) {
        printf("[ERROR] Failed to open online pipeline!\n");
        printf("ret: %d\n", res0);
        return false;
    }
    initialized_ = true;
    printf("[INFO] open online pipe0: %d \n", res0);
    return true;
}

bool IMAGEPROCESSOR::GetImage(ssne_tensor_t* img_sensor) {
    if (!initialized_) {
        return false;
    }
    const int capture_code = GetImageData(img_sensor, kPipeline0, kSensor0, 0);
    if (capture_code != 0) {
        return false;
    }
    return true;
}

void IMAGEPROCESSOR::Release()
{
    if (!initialized_) {
        return;
    }
    CloseOnlinePipeline(kPipeline0);
    initialized_ = false;
    printf("[INFO] OnlinePipe closed!\n");
}
