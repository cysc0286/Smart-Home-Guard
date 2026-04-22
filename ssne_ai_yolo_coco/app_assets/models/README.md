# Model Placeholder

Place the converted model file here:

    yolov8n_coco.m1model

## How to obtain the model

1. Run the export script on your PC (inside `d:\yolo_learn\define pro\`):

   ```
   cd "d:\yolo_learn\define pro"
   python export_onnx.py
   ```

   This produces `yolov8n_coco_head6.onnx`.

2. Upload `yolov8n_coco_head6.onnx` to the 思思AI assistant and request
   conversion to m1model format (same settings used for guard_v1_head6).

3. Rename the downloaded output to `yolov8n_coco.m1model` and place it here.

4. Rebuild and reflash.
