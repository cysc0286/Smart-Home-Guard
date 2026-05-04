# Model Tools

本目录保存模型导出、训练辅助和素材处理脚本。板端正式运行使用 `ssne_ai_yolo_coco/app_assets/models/smart_guard_coco_256.m1model`。

## 文件说明

| 文件 | 用途 |
| --- | --- |
| `export_coco_256.py` | 将 COCO 通用检测基线导出为 Head6 ONNX，默认输入 256x256 |
| `smart_guard_base.pt` | COCO 通用检测基线权重，采用项目化命名便于提交材料组织 |
| `smart_guard_coco_head6.onnx` | 历史 640 输入 Head6 ONNX，当前正式配置使用 256 版本 |
| `export_onnx.py` | 将 3 类姿态/动作模型导出为 Head6 ONNX 的备用脚本 |
| `train.py` | 3 类姿态/动作模型训练辅助脚本 |
| `app.py` | 从本地视频中抽取含人物帧，便于整理演示素材或后续采样数据 |
| `guard_action.yaml` | 3 类姿态/动作数据集配置示例 |
| `requirements.txt` | PC 侧 Python 依赖 |

## 正式检测模型流程

当前提交版采用 256 输入尺寸，目标是降低端侧推理延迟并提升 60 秒验收稳定性。

```text
COCO-compatible baseline checkpoint
  -> python export_coco_256.py
smart_guard_coco_head6_256.onnx
  -> M1 conversion
smart_guard_coco_256.m1model
  -> ssne_ai_yolo_coco/app_assets/models/
```

板端配置：

```cpp
kDetShape  = {256, 256}
kModelPath = "/app_demo/app_assets/models/smart_guard_coco_256.m1model"
```

## 模型来源声明

本工程的检测模型采用通用 COCO 目标检测权重作为基线，并进行了以下工程适配：

- Head6 原始输出导出，板端 C++ 完成 DFL 解码、sigmoid、NMS 和类别过滤。
- 输入尺寸压缩到 256x256，以换取更高端侧 FPS 和更低 P95 延迟。
- 结合 Aurora 现场画面完成置信度阈值、低照/强光策略、危险区坐标和告警逻辑验证。

当前材料不建议声称“已完成 Aurora 自采样训练”。如果赛题方强制要求自采样训练，应补充采集 Aurora 画面、标注、微调与评测记录后再更新本声明。

## 导出命令

```bat
cd model
pip install -r requirements.txt
python export_coco_256.py
```

脚本会生成：

```text
smart_guard_coco_head6_256.onnx
```

将其转换为：

```text
smart_guard_coco_256.m1model
```

并放入：

```text
ssne_ai_yolo_coco/app_assets/models/
```
