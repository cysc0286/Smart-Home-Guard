# Board Model Assets

板端正式运行需要在本目录放置：

```text
smart_guard_coco_256.m1model
```

该文件对应 `ssne_ai_yolo_coco/include/coco_config.hpp`：

```cpp
static const std::array<int, 2> kDetShape = {256, 256};
static const char* kModelPath = "/app_demo/app_assets/models/smart_guard_coco_256.m1model";
```

## 获取方式

1. 在 PC 端执行：

```bat
cd model
python export_coco_256.py
```

2. 将生成的 `smart_guard_coco_head6_256.onnx` 转换为 M1 模型。
3. 将转换结果命名为 `smart_guard_coco_256.m1model`。
4. 拷贝到本目录，并随提交材料压缩包一起提交。

## 注意

- Git 默认忽略 `*.m1model`，因此制作技术数据压缩包时必须手动确认模型文件已包含。
- `smart_guard_coco_640_legacy.m1model` 仅作为历史兼容资源保留，正式配置不再使用。
