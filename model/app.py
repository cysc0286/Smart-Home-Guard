from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
from ultralytics import YOLO

VIDEO_EXTENSIONS = {".mp4", ".mov", ".avi", ".mkv", ".wmv", ".flv", ".webm", ".m4v"}
PERSON_CLASS_ID = 0


@dataclass(slots=True)
class ExtractionStats:
    video_name: str
    total_frames: int = 0
    checked_frames: int = 0
    saved_images: int = 0
    person_frames: int = 0


class PersonFrameExtractor:
    def __init__(
        self,
        model_path: str,
        confidence: float,
        frame_stride: int,
        min_box_area_ratio: float,
        min_save_gap: int,
        save_mode: str,
        image_format: str,
        jpeg_quality: int,
        device: str | None,
    ) -> None:
        self.model = YOLO(model_path)
        self.confidence = confidence
        self.frame_stride = frame_stride
        self.min_box_area_ratio = min_box_area_ratio
        self.min_save_gap = min_save_gap
        self.save_mode = save_mode
        self.image_format = image_format
        self.jpeg_quality = jpeg_quality
        self.device = device

    def process_video(self, video_path: Path, output_root: Path) -> ExtractionStats:
        capture = cv2.VideoCapture(str(video_path))
        if not capture.isOpened():
            raise RuntimeError(f"无法打开视频: {video_path}")

        safe_video_stem = sanitize_name(video_path.stem)
        video_output_dir = output_root / safe_video_stem
        video_output_dir.mkdir(parents=True, exist_ok=True)

        stats = ExtractionStats(video_name=video_path.name)
        frame_index = 0
        last_saved_frame = -self.min_save_gap - 1

        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break

                stats.total_frames += 1

                if frame_index % self.frame_stride != 0:
                    frame_index += 1
                    continue

                stats.checked_frames += 1
                detections = self.detect_people(frame)
                if not detections:
                    frame_index += 1
                    continue

                stats.person_frames += 1

                if frame_index - last_saved_frame < self.min_save_gap:
                    frame_index += 1
                    continue

                saved_count = self.save_detection_result(
                    frame=frame,
                    detections=detections,
                    output_dir=video_output_dir,
                    video_stem=safe_video_stem,
                    frame_index=frame_index,
                )
                if saved_count > 0:
                    stats.saved_images += saved_count
                    last_saved_frame = frame_index

                frame_index += 1
        finally:
            capture.release()

        return stats

    def detect_people(self, frame) -> list[tuple[int, int, int, int, float]]:
        results = self.model.predict(
            source=frame,
            classes=[PERSON_CLASS_ID],
            conf=self.confidence,
            verbose=False,
            device=self.device,
        )

        if not results:
            return []

        result = results[0]
        if result.boxes is None:
            return []

        frame_h, frame_w = frame.shape[:2]
        frame_area = max(frame_h * frame_w, 1)
        detections: list[tuple[int, int, int, int, float]] = []

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            confidence = float(box.conf[0].item())
            left = max(int(x1), 0)
            top = max(int(y1), 0)
            right = min(int(x2), frame_w)
            bottom = min(int(y2), frame_h)

            width = max(right - left, 0)
            height = max(bottom - top, 0)
            if width == 0 or height == 0:
                continue

            box_area_ratio = (width * height) / frame_area
            if box_area_ratio < self.min_box_area_ratio:
                continue

            detections.append((left, top, right, bottom, confidence))

        return detections

    def save_detection_result(
        self,
        frame,
        detections: list[tuple[int, int, int, int, float]],
        output_dir: Path,
        video_stem: str,
        frame_index: int,
    ) -> int:
        extension = self.image_format.lower()
        if self.save_mode == "frame":
            confidence = max(detection[4] for detection in detections)
            image_path = output_dir / f"{video_stem}_frame_{frame_index:06d}_conf_{confidence:.2f}.{extension}"
            save_image(image_path, frame, self.jpeg_quality)
            return 1

        saved = 0
        for person_index, (left, top, right, bottom, confidence) in enumerate(detections, start=1):
            crop = frame[top:bottom, left:right]
            if crop.size == 0:
                continue
            image_path = output_dir / (
                f"{video_stem}_frame_{frame_index:06d}_person_{person_index:02d}_conf_{confidence:.2f}.{extension}"
            )
            save_image(image_path, crop, self.jpeg_quality)
            saved += 1
        return saved


def save_image(path: Path, image, jpeg_quality: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    suffix = path.suffix.lower()
    if suffix in {".jpg", ".jpeg"}:
        ok = cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    else:
        ok = cv2.imwrite(str(path), image)

    if not ok:
        raise RuntimeError(f"保存图片失败: {path}")


def sanitize_name(name: str) -> str:
    cleaned = "".join(char if char.isalnum() or char in {"-", "_"} else "_" for char in name)
    return cleaned.strip("_") or "video"


def iter_video_files(input_dir: Path, recursive: bool) -> Iterable[Path]:
    iterator = input_dir.rglob("*") if recursive else input_dir.glob("*")
    for path in iterator:
        if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS:
            yield path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="从视频中提取包含人物的帧，可保存整帧或人物裁剪图。"
    )
    parser.add_argument("--input-dir", default="input_videos", help="待处理视频目录")
    parser.add_argument("--output-dir", default="output_frames", help="输出图片目录")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO 模型名称或本地模型路径")
    parser.add_argument("--conf", type=float, default=0.45, help="人物检测置信度阈值")
    parser.add_argument("--frame-stride", type=int, default=5, help="每隔多少帧检测一次")
    parser.add_argument(
        "--min-box-area-ratio",
        type=float,
        default=0.015,
        help="人物框最小面积占整帧比例，小于该比例会被忽略",
    )
    parser.add_argument(
        "--min-save-gap",
        type=int,
        default=15,
        help="两次保存之间至少间隔多少帧，用于减少近似重复图片",
    )
    parser.add_argument(
        "--save-mode",
        choices=("frame", "crop"),
        default="frame",
        help="frame 保存整帧；crop 只保存人物框",
    )
    parser.add_argument(
        "--image-format",
        choices=("jpg", "png"),
        default="jpg",
        help="输出图片格式",
    )
    parser.add_argument("--jpeg-quality", type=int, default=95, help="JPG 质量，范围 1-100")
    parser.add_argument("--device", default=None, help="推理设备，例如 cpu、0、0,1")
    parser.add_argument(
        "--recursive",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="是否递归扫描输入目录下的子目录",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.frame_stride < 1:
        raise ValueError("--frame-stride 必须大于等于 1")
    if args.min_save_gap < 0:
        raise ValueError("--min-save-gap 不能小于 0")
    if not 0.0 <= args.conf <= 1.0:
        raise ValueError("--conf 必须位于 0 到 1 之间")
    if not 0.0 <= args.min_box_area_ratio <= 1.0:
        raise ValueError("--min-box-area-ratio 必须位于 0 到 1 之间")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("--jpeg-quality 必须位于 1 到 100 之间")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        validate_args(args)
    except ValueError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2

    input_dir = Path(args.input_dir).resolve()
    output_dir = Path(args.output_dir).resolve()

    if not input_dir.exists():
        print(f"输入目录不存在: {input_dir}", file=sys.stderr)
        return 1

    video_files = sorted(iter_video_files(input_dir, args.recursive))
    if not video_files:
        print(f"未找到可处理视频，请把视频放到: {input_dir}")
        return 0

    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("开始处理视频")
    print(f"输入目录: {input_dir}")
    print(f"输出目录: {output_dir}")
    print(f"视频数量: {len(video_files)}")
    print(f"模型: {args.model}")
    print(f"保存模式: {args.save_mode}")
    print("=" * 72)

    extractor = PersonFrameExtractor(
        model_path=args.model,
        confidence=args.conf,
        frame_stride=args.frame_stride,
        min_box_area_ratio=args.min_box_area_ratio,
        min_save_gap=args.min_save_gap,
        save_mode=args.save_mode,
        image_format=args.image_format,
        jpeg_quality=args.jpeg_quality,
        device=args.device,
    )

    total_saved = 0
    total_checked = 0
    total_person_frames = 0

    for index, video_path in enumerate(video_files, start=1):
        print(f"[{index}/{len(video_files)}] 正在处理: {video_path.name}")
        try:
            stats = extractor.process_video(video_path, output_dir)
        except Exception as exc:
            print(f"  处理失败: {exc}", file=sys.stderr)
            continue

        total_saved += stats.saved_images
        total_checked += stats.checked_frames
        total_person_frames += stats.person_frames

        print(
            "  完成 | "
            f"总帧数: {stats.total_frames} | "
            f"检测帧数: {stats.checked_frames} | "
            f"含人帧数: {stats.person_frames} | "
            f"已保存: {stats.saved_images}"
        )

    print("=" * 72)
    print("全部处理完成")
    print(f"共检测帧数: {total_checked}")
    print(f"共发现含人帧数: {total_person_frames}")
    print(f"共保存图片数: {total_saved}")
    print(f"结果目录: {output_dir}")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
