"""
Export the Smart Guard COCO baseline to Head6 ONNX at 256x256 by default.

Output: 6 tensors: P3_box, P3_cls, P4_box, P4_cls, P5_box, P5_cls
  box shape: [1, h, w, 64]   (raw DFL logits)
  cls shape: [1, h, w, 80]   (raw logits, 80 COCO classes)
  At 256x256: P3=32x32, P4=16x16, P5=8x8

After export, convert the .onnx to smart_guard_coco_256.m1model.
Then place it in: ssne_ai_yolo_coco/app_assets/models/smart_guard_coco_256.m1model
And update coco_config.hpp: kDetShape = {256, 256}
                             kModelPath = ".../smart_guard_coco_256.m1model"

Usage:
    python export_coco_256.py
    python export_coco_256.py --imgsz 416   # for a larger variant
"""

import argparse
import os

import onnx
import torch
import torch.nn as nn


def export_coco_head6(
    model_path: str = "smart_guard_base.pt",
    imgsz: int = 256,
    opset: int = 11,
) -> None:
    from ultralytics import YOLO

    output_path = f"smart_guard_coco_head6_{imgsz}.onnx"

    yolo = YOLO(model_path)
    inner = yolo.model
    inner.eval()

    class Head6Exporter(nn.Module):
        def __init__(self, inner_model: nn.Module) -> None:
            super().__init__()
            self.inner = inner_model

        def forward(self, x: torch.Tensor):
            model = self.inner.model
            y = []
            for m in model[:-1]:
                if m.f != -1:
                    x = (
                        y[m.f]
                        if isinstance(m.f, int)
                        else [x if j == -1 else y[j] for j in m.f]
                    )
                x = m(x)
                y.append(x if m.i in self.inner.save else None)

            det = model[-1]
            feats = [y[j] for j in det.f]

            outputs = []
            for i in range(det.nl):
                box = det.cv2[i](feats[i]).permute(0, 2, 3, 1).contiguous()
                cls = det.cv3[i](feats[i]).permute(0, 2, 3, 1).contiguous()
                outputs.extend([box, cls])
            return tuple(outputs)

    exporter = Head6Exporter(inner)
    exporter.eval()

    dummy = torch.zeros(1, 3, imgsz, imgsz)
    with torch.no_grad():
        outs = exporter(dummy)

    assert len(outs) == 6, f"Expected 6 outputs, got {len(outs)}"
    print(f"[OK] imgsz={imgsz}, {len(outs)} output tensors:")
    for i, o in enumerate(outs):
        tag = "box" if i % 2 == 0 else "cls"
        print(f"  [{i}] P{3 + i // 2}_{tag}: {tuple(o.shape)}")

    output_names = ["P3_box", "P3_cls", "P4_box", "P4_cls", "P5_box", "P5_cls"]
    torch.onnx.export(
        exporter,
        dummy,
        output_path,
        opset_version=opset,
        input_names=["images"],
        output_names=output_names,
        do_constant_folding=True,
    )

    model_proto = onnx.load(output_path)
    onnx.save_model(model_proto, output_path, save_as_external_data=False)
    data_file = output_path + ".data"
    if os.path.exists(data_file):
        os.remove(data_file)

    size_mb = os.path.getsize(output_path) / 1024 / 1024
    print(f"\n[DONE] {output_path}  ({size_mb:.1f} MB)")
    print("\nNext steps:")
    print(f"  1. Convert {output_path} -> smart_guard_coco_{imgsz}.m1model")
    print(f"  2. Copy m1model to board: /app_demo/app_assets/models/smart_guard_coco_{imgsz}.m1model")
    print("  3. In coco_config.hpp:")
    print(f"       kDetShape  = {{{imgsz}, {imgsz}}}")
    print(f"       kModelPath = \"/app_demo/app_assets/models/smart_guard_coco_{imgsz}.m1model\"")
    print("  4. Rebuild and flash")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--imgsz", type=int, default=256, choices=[224, 256, 320, 416, 480])
    parser.add_argument("--model", type=str, default="smart_guard_base.pt")
    args = parser.parse_args()
    export_coco_head6(model_path=args.model, imgsz=args.imgsz)
