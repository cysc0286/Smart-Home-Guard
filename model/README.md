# PC 端工具集

本文件夹是比赛项目的 **PC 端工具目录**，不参与板端编译，仅负责模型导出和上位机开发。

## 文件说明

| 文件 | 用途 |
|------|------|
| `yolov8n.pt` | YOLOv8n 预训练权重（COCO 80类，来源：Ultralytics） |
| `yolov8n_coco_head6.onnx` | 从 `.pt` 导出的 Head6 格式 ONNX，用于上传思思AI平台转换 |
| `export_onnx.py` | 导出脚本，将 `.pt` 转为板端所需的 Head6 ONNX 格式 |
| `app.py` | 上位机控制程序（待开发），规划用于摄像头画面观察 + 危险区域框定 |
| `requirements.txt` | Python 依赖 |

## 模型导出流程

```
yolov8n.pt
  ↓  python export_onnx.py
yolov8n_coco_head6.onnx
  ↓  上传思思AI平台（单文件，约13MB）
yolov8n_coco.m1model
  ↓  放入板端路径
  ssne_ai_yolo_coco/app_assets/models/yolov8n_coco.m1model
```

导出的 ONNX 格式（Head6 解耦头）：

| 输出张量 | 形状 | 含义 |
|----------|------|------|
| P3_box | [1, 80, 80, 64] | 小目标 DFL box logits（未解码） |
| P3_cls | [1, 80, 80, 80] | 小目标类别 logits（未 sigmoid） |
| P4_box | [1, 40, 40, 64] | 中目标 DFL box logits |
| P4_cls | [1, 40, 40, 80] | 中目标类别 logits |
| P5_box | [1, 20, 20, 64] | 大目标 DFL box logits |
| P5_cls | [1, 20, 20, 80] | 大目标类别 logits |

板端 C++ 代码负责 DFL 解码、sigmoid、NMS，模型本身输出裸 logits。

## app.py（上位机，规划中）

当前文件内容为旧版视频帧提取脚本（已废弃），**待改写**为：

- 接收板端 MJPEG 视频流，实时显示摄像头画面
- 鼠标在画面上绘制危险区域（矩形 / 多边形）
- 将区域坐标发送到板端（TCP），板端读取后判断入侵并报警
- 满足比赛"可视化展示界面"评分项

## 板端编译流程（参考）

```bash
docker start A1_Builder
docker exec -it A1_Builder bash
cd /home/smartsens_flying_chip_a1_sdk/A1_SDK_SC235HAI/smartsens_sdk
bash scripts/build_app.sh
bash scripts/build_release_sdk.sh
```

生成物在 `output/images/`，烧录到 SD 卡即可。
