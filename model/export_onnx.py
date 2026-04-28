"""
Export trained guard_action best.pt to ONNX in Head6 decoupled format.

Output: 6 tensors in order: P3_box, P3_cls, P4_box, P4_cls, P5_box, P5_cls
  box shape: [1, h, w, 4*reg_max] = [1, h, w, 64]  (raw DFL logits, NO decoding)
  cls shape: [1, h, w, nc]        = [1, h, w, 3]   (raw logits, NO sigmoid)
  Classes: 0=bending, 1=standing, 2=squatting

Upload the resulting .onnx to the 思思AI assistant to obtain guard_action.m1model.

Usage:
    python export_onnx.py
"""

import torch
import torch.nn as nn


def export_head6(
    model_path: str = "runs/guard_action/weights/best.pt",
    output_path: str = "guard_action_head6.onnx",
    imgsz: int = 640,
    opset: int = 11,
) -> None:
    from ultralytics import YOLO

    yolo = YOLO(model_path)
    inner = yolo.model
    inner.eval()

    class Head6Exporter(nn.Module):
        """Wraps YOLOv8 backbone+neck, outputs 6 raw decoupled heads."""

        def __init__(self, inner_model: nn.Module) -> None:
            super().__init__()
            self.inner = inner_model

        def forward(self, x: torch.Tensor):
            model = self.inner.model
            y = []

            # Run all layers except the final Detect head
            for m in model[:-1]:
                if m.f != -1:
                    x = (
                        y[m.f]
                        if isinstance(m.f, int)
                        else [x if j == -1 else y[j] for j in m.f]
                    )
                x = m(x)
                y.append(x if m.i in self.inner.save else None)

            # Gather Detect layer's feature map inputs (P3, P4, P5)
            det = model[-1]
            feats = [y[j] for j in det.f]

            outputs = []
            for i in range(det.nl):  # nl == 3
                # Raw DFL box logits: [1, 4*reg_max, h, w] -> [1, h, w, 4*reg_max]
                box = det.cv2[i](feats[i]).permute(0, 2, 3, 1).contiguous()
                # Raw class logits:  [1, nc, h, w]        -> [1, h, w, nc]
                cls = det.cv3[i](feats[i]).permute(0, 2, 3, 1).contiguous()
                outputs.extend([box, cls])

            return tuple(outputs)

    exporter = Head6Exporter(inner)
    exporter.eval()

    dummy = torch.zeros(1, 3, imgsz, imgsz)

    # Sanity-check shapes before export
    with torch.no_grad():
        outs = exporter(dummy)

    assert len(outs) == 6, f"Expected 6 outputs, got {len(outs)}"
    print(f"[OK] {len(outs)} output tensors:")
    expected_strides = [8, 16, 32]
    for i, o in enumerate(outs):
        tag = "box" if i % 2 == 0 else "cls"
        scale = i // 2
        h_exp = imgsz // expected_strides[scale]
        print(f"  [{i}] P{3+scale}_{tag}: {tuple(o.shape)}", end="")
        if i % 2 == 0:
            print(f"  (expected h=w={h_exp}, C={4*16}=64)")
        else:
            nc = o.shape[-1]
            print(f"  (expected h=w={h_exp}, nc={nc})")

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

    # Merge external data (.data file) back into a single self-contained .onnx file.
    # PyTorch may split large models into .onnx + .data; this step reunifies them.
    import onnx
    import os
    model_proto = onnx.load(output_path)
    # Save without external data — everything embedded in one file
    onnx.save_model(
        model_proto,
        output_path,
        save_as_external_data=False,
    )
    # Remove leftover .data file if present
    data_file = output_path + ".data"
    if os.path.exists(data_file):
        os.remove(data_file)

    size_mb = os.path.getsize(output_path) / 1024 / 1024
    print(f"\n[DONE] Exported to: {output_path}  ({size_mb:.1f} MB, single file)")
    print("Next: upload this .onnx to 思思AI assistant to get guard_action.m1model")
    print("Then place the .m1model into:")
    print("  ssne_ai_yolo_coco/app_assets/models/guard_action.m1model")


if __name__ == "__main__":
    export_head6()
