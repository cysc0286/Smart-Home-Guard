# 待补素材清单

- Aurora 调试画面截图：正常光、黑暗、强光各 1 张。
- 串口 60 秒验收日志截图：包含 [CHECK][SUMMARY] 与 [CHECK][COUNTS]。
- 作品演示视频素材：危险区配置、进入危险区、OSD 红框、ALERT、蜂鸣器/LED。
- 实物图：A1 板、摄像头、FT232RL 接线、报警外设。
- 模型转换证据：smart_guard_coco_head6_256.onnx 转 smart_guard_coco_256.m1model 的平台截图。
- 技术数据压缩包截图：源码、模型、可执行文件、脚本、README。

## 模型声明建议

检测模型采用 COCO 通用目标检测权重作为基线，经 Head6 输出适配、M1 模型转换、256x256 输入压缩，并结合 Aurora 现场画面完成阈值、危险区、低照/强光策略和端侧性能验证调优。当前材料不声称已完成 Aurora 自采样再训练。
