# PC 端工具集

本文件夹是比赛项目的 **PC 端工具目录**，不参与板端编译，仅负责模型导出和上位机开发。

## 文件说明

| 文件 | 用途 |
|------|------|
| `yolov8n.pt` | YOLOv8n 预训练权重（COCO 80类，来源：Ultralytics） |
| `yolov8n_coco_head6.onnx` | 从 `.pt` 导出的 Head6 格式 ONNX，用于上传思思AI平台转换 |
| `export_onnx.py` | 导出脚本，将 `.pt` 转为板端所需的 Head6 ONNX 格式 |
| `app.py` | 视频帧提取工具，用于从视频中筛选含有人物的帧 |
| `requirements.txt` | Python 依赖 |

## 模型导出流程

```text
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

## app.py（视频帧提取工具）

当前 `app.py` 是一个独立的视频帧人物提取脚本，可用于：

- 从本地视频中筛选包含人物的帧
- 保存整帧或人物裁剪图
- 为后续数据整理、演示素材准备提供支持

上位机禁区画框控制程序已移至项目根目录下的 `pc_controller/` 目录。

## 板端编译流程（参考）

```bash
docker start A1_Builder
docker exec -it A1_Builder bash
cd /home/smartsens_flying_chip_a1_sdk/A1_SDK_SC235HAI/smartsens_sdk
bash scripts/build_app.sh
bash scripts/build_release_sdk.sh
```

生成物在 `output/images/`，烧录到 SD 卡即可。
