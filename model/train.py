"""
基于YOLOv8n微调3类动作检测: bending/standing/squatting
输出: runs/guard_action/weights/best.pt
"""

from pathlib import Path
from ultralytics import YOLO

MODEL_DIR = Path(__file__).parent
DATA_YAML = MODEL_DIR / "guard_action.yaml"
PRETRAINED = MODEL_DIR / "smart_guard_base.pt"


def main():
    model = YOLO(str(PRETRAINED))

    model.train(
        data=str(DATA_YAML),
        epochs=100,
        imgsz=640,
        batch=16,
        lr0=0.01,
        lrf=0.01,
        warmup_epochs=3,
        patience=20,
        project=str(MODEL_DIR / "runs"),
        name="guard_action",
        exist_ok=True,
        device=0,
        workers=4,
        save=True,
        plots=True,
        cls=1.5,        # upweight cls loss to compensate class imbalance
    )

    best = MODEL_DIR / "runs" / "guard_action" / "weights" / "best.pt"
    print(f"\n[DONE] Best weights: {best}")
    print("Next: run  python export_onnx.py  to export guard_action_head6.onnx")


if __name__ == "__main__":
    main()
