# HALO (Home Alert & Location Observer)

HALO (Home Alert & Location Observer) 是基于飞凌微 A1 开发套件和思特威图像传感器的端侧 AI 家居告警系统。工程运行在 SmartSens FlyingChip A1 平台，完成摄像头取流、人员/宠物检测、危险区域判断、OSD 可视化、GPIO 声光报警、串口配置和 60 秒验收日志输出。

## 核心能力

- 端侧检测：使用 256x256 输入的 `smart_guard_coco_256.m1model`，保留 person、dog、cat 三类告警目标。
- 危险区配置：PC 上位机通过串口获取 Aurora 当前画面快照，绘制危险区后发送到板端。
- 区域判断：矩形和多边形危险区均支持；多边形判断使用真实点集的 point-in-polygon 算法。
- OSD 展示：检测框、危险区、ALERT 图标分图层显示。当前正式演示中，多边形危险区的 OSD 显示为外接矩形，便于控制 OSD buffer 和现场稳定性；告警判断仍按真实多边形完成。
- 声光报警：目标进入危险区时触发 GPIO LED/蜂鸣器。
- 鲁棒性：覆盖摄像头、数据、推理和资源异常，支持降级运行和日志告警。
- 验收日志：输出 `[FPS]`、`[LAT]`、`[ENV]`、`[CHECK][SUMMARY]`，用于 60 秒稳定性、实时性和延迟统计。

## 目录结构

```text
ssne_ai_yolo_coco/      板端 C++ 主程序、检测器、OSD、GPIO、UART 和启动脚本
pc_controller/          PC 上位机危险区绘制与串口/TCP 配置工具
model/                  模型导出、训练辅助脚本和数据处理工具
tools/                  辅助资源生成脚本
submission_materials/   作品 PPT、技术文档等提交材料草稿
```

## 板端运行

板端模型路径在 `ssne_ai_yolo_coco/include/coco_config.hpp` 中配置：

```cpp
static const char* kModelPath = "/app_demo/app_assets/models/smart_guard_coco_256.m1model";
```

运行脚本：

```sh
cd /app_demo
sh ssne_ai_yolo_coco/scripts/run.sh
```

`run.sh` 会加载 GPIO/UART 内核模块，并在程序异常退出后自动重启。

## PC 端配置流程

1. 修改 `pc_controller/controller_config.json` 中的串口号，例如 `COM6`。
2. 板端启动 `ssne_ai_yolo_coco` 后停留在串口配置阶段。
3. PC 端运行：

```bat
cd pc_controller
run.bat
```

4. 在窗口中绘制危险区，按 `S` 发送到板端并启动正式检测。

## 模型说明

提交版本采用项目化命名的 `smart_guard_coco_256.m1model`。模型来源和转换流程见 `model/README.md` 与 `ssne_ai_yolo_coco/app_assets/models/README.md`。

材料中建议使用如下真实表述：

> 检测模型采用 COCO 通用目标检测权重作为基线，经 Head6 输出适配、M1 模型转换、256x256 输入压缩，并结合 Aurora 现场画面完成阈值、危险区、低照/强光策略和端侧性能验证调优。当前工程重点为端侧实时部署、危险区告警闭环和现场鲁棒性。

## 验收关注日志

正式演示时建议保存串口日志截图，重点包含：

```text
[CHECK][BEGIN]
[FPS]
[ENV]
[DET]
[ALARM]
[LAT]
[CHECK][SUMMARY]
[CHECK][COUNTS]
```

其中 `[CHECK][SUMMARY]` 给出 60 秒运行时长、平均应用 FPS、R 值、P95 延迟和评分估算；`[CHECK][COUNTS]` 给出检测帧数、告警帧数、异常恢复次数和资源告警次数。
