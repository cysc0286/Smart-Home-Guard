from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol
from urllib.error import URLError, HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np
try:
    import serial
except ImportError:
    serial = None

WINDOW_NAME = "Smart Home Guard - Snapshot Zone Controller"
DEFAULT_ZONE_FILE = "zone_config.json"
DEFAULT_CONFIG_FILE = "controller_config.json"


@dataclass(slots=True)
class Zone:
    shape: str
    points: list[tuple[int, int]]
    alarm_classes: list[str]

    @classmethod
    def from_rect(
        cls,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        alarm_classes: list[str] | None = None,
    ) -> "Zone":
        left = min(x1, x2)
        right = max(x1, x2)
        top = min(y1, y2)
        bottom = max(y1, y2)
        return cls(shape="rect", points=[(left, top), (right, bottom)], alarm_classes=alarm_classes or [])

    def normalized(self) -> "Zone":
        if self.shape == "rect" and len(self.points) >= 2:
            (x1, y1), (x2, y2) = self.points[:2]
            return Zone.from_rect(x1, y1, x2, y2, self.alarm_classes)
        return Zone(shape="polygon", points=list(self.points), alarm_classes=list(self.alarm_classes))

    def is_complete(self) -> bool:
        if self.shape == "rect":
            return len(self.points) >= 2
        return len(self.points) >= 3

    def to_dict(self) -> dict[str, Any]:
        zone = self.normalized()
        if zone.shape == "polygon":
            return {
                "type": "zone_update",
                "shape": "polygon",
                "points": [[int(x), int(y)] for x, y in zone.points],
                "alarm_classes": list(zone.alarm_classes),
            }
        (x1, y1), (x2, y2) = zone.points[:2]
        return {
            "type": "zone_update",
            "shape": "rect",
            "x1": int(x1),
            "y1": int(y1),
            "x2": int(x2),
            "y2": int(y2),
            "alarm_classes": list(zone.alarm_classes),
        }

    def describe(self) -> str:
        zone = self.normalized()
        if zone.shape == "polygon":
            return f"polygon {len(zone.points)} points"
        (x1, y1), (x2, y2) = zone.points[:2]
        return f"rect ({x1},{y1})-({x2},{y2})"


@dataclass(slots=True)
class SnapshotFrame:
    image: np.ndarray
    logical_width: int
    logical_height: int


@dataclass(slots=True)
class ControllerConfig:
    snapshot_source: str = "serial"
    snapshot_url: str = "http://192.168.1.88:8081/?action=snapshot"
    snapshot_file: str = "latest_snapshot.pgm"
    serial_port: str = "COM3"
    serial_baudrate: int = 115200
    serial_timeout_sec: float = 10.0
    serial_idle_timeout_sec: float = 0.08
    board_ip: str = "192.168.1.88"
    board_port: int = 9000
    zone_file: str = DEFAULT_ZONE_FILE
    auto_send: bool = True
    auto_refresh_after_send: bool = True
    auto_exit_after_send: bool = True
    auto_refresh_snapshot: bool = True
    alarm_classes: list[str] | str | None = None
    snapshot_refresh_interval_ms: int = 500
    startup_retry: int = 5
    startup_retry_interval: float = 1.0
    snapshot_timeout_sec: float = 3.0
    tcp_timeout_sec: float = 1.5
    window_width: int = 1280
    window_height: int = 720


def _read_json_file(path: Path) -> dict[str, Any]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"配置文件不存在: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"配置文件 JSON 格式错误: {path}") from exc

    if not isinstance(raw, dict):
        raise ValueError(f"配置文件内容必须是 JSON 对象: {path}")
    return raw


def load_controller_config(config_path: Path) -> ControllerConfig:
    config = ControllerConfig()
    data = _read_json_file(config_path)

    for field_name in config.__dataclass_fields__:
        if field_name in data:
            setattr(config, field_name, data[field_name])

    config.board_port = int(config.board_port)
    config.startup_retry = max(int(config.startup_retry), 1)
    config.startup_retry_interval = max(float(config.startup_retry_interval), 0.0)
    config.snapshot_timeout_sec = max(float(config.snapshot_timeout_sec), 0.1)
    config.tcp_timeout_sec = max(float(config.tcp_timeout_sec), 0.1)
    config.serial_baudrate = max(int(config.serial_baudrate), 9600)
    config.serial_timeout_sec = max(float(config.serial_timeout_sec), 0.1)
    config.serial_idle_timeout_sec = max(float(config.serial_idle_timeout_sec), 0.05)
    config.snapshot_refresh_interval_ms = max(int(config.snapshot_refresh_interval_ms), 100)
    config.window_width = max(int(config.window_width), 320)
    config.window_height = max(int(config.window_height), 240)
    config.auto_send = bool(config.auto_send)
    config.auto_refresh_after_send = bool(config.auto_refresh_after_send)
    config.auto_exit_after_send = bool(config.auto_exit_after_send)
    config.auto_refresh_snapshot = bool(config.auto_refresh_snapshot)
    config.alarm_classes = normalize_alarm_classes(config.alarm_classes)
    config.snapshot_source = str(config.snapshot_source).strip().lower()
    config.snapshot_url = str(config.snapshot_url)
    config.snapshot_file = str(config.snapshot_file)
    config.serial_port = str(config.serial_port)
    config.board_ip = str(config.board_ip)
    config.zone_file = str(config.zone_file)
    if config.snapshot_source not in {"http", "file", "serial"}:
        raise ValueError("snapshot_source 只能是 'http'、'file' 或 'serial'")
    return config


def normalize_alarm_classes(raw: list[str] | str | None) -> list[str]:
    if raw is None:
        return ["person", "dog", "cat"]
    if isinstance(raw, str):
        values = [item.strip() for item in raw.split(",")]
    else:
        values = [str(item).strip() for item in raw]
    classes = [item for item in values if item]
    return classes or ["person", "dog", "cat"]


def merge_args_into_config(config: ControllerConfig, args: argparse.Namespace) -> ControllerConfig:
    if args.snapshot_url is not None:
        config.snapshot_url = args.snapshot_url
    if args.snapshot_source is not None:
        config.snapshot_source = args.snapshot_source.strip().lower()
    if args.snapshot_file is not None:
        config.snapshot_file = args.snapshot_file
    if args.serial_port is not None:
        config.serial_port = args.serial_port
    if args.serial_baudrate is not None:
        config.serial_baudrate = max(args.serial_baudrate, 9600)
    if args.serial_timeout_sec is not None:
        config.serial_timeout_sec = max(args.serial_timeout_sec, 0.1)
    if args.serial_idle_timeout_sec is not None:
        config.serial_idle_timeout_sec = max(args.serial_idle_timeout_sec, 0.05)
    if args.board_ip is not None:
        config.board_ip = args.board_ip
    if args.board_port is not None:
        config.board_port = args.board_port
    if args.zone_file is not None:
        config.zone_file = args.zone_file
    if args.auto_send is not None:
        config.auto_send = args.auto_send
    if args.auto_refresh_after_send is not None:
        config.auto_refresh_after_send = args.auto_refresh_after_send
    if args.auto_exit_after_send is not None:
        config.auto_exit_after_send = args.auto_exit_after_send
    if args.auto_refresh_snapshot is not None:
        config.auto_refresh_snapshot = args.auto_refresh_snapshot
    if args.alarm_classes is not None:
        config.alarm_classes = normalize_alarm_classes(args.alarm_classes)
    if args.snapshot_refresh_interval_ms is not None:
        config.snapshot_refresh_interval_ms = max(args.snapshot_refresh_interval_ms, 100)
    if args.startup_retry is not None:
        config.startup_retry = max(args.startup_retry, 1)
    if args.startup_retry_interval is not None:
        config.startup_retry_interval = max(args.startup_retry_interval, 0.0)
    if args.snapshot_timeout_sec is not None:
        config.snapshot_timeout_sec = max(args.snapshot_timeout_sec, 0.1)
    if args.tcp_timeout_sec is not None:
        config.tcp_timeout_sec = max(args.tcp_timeout_sec, 0.1)
    if args.window_width is not None:
        config.window_width = max(args.window_width, 320)
    if args.window_height is not None:
        config.window_height = max(args.window_height, 240)
    return config


def resolve_config_path(raw_path: str, config_dir: Path) -> Path:
    path = Path(raw_path)
    if not path.is_absolute():
        path = config_dir / path
    return path.resolve()


class ZoneSender(Protocol):
    def display_target(self) -> str:
        ...

    def send(self, zone: Zone) -> None:
        ...


class TcpZoneSender:
    def __init__(self, board_ip: str, board_port: int, timeout_sec: float = 1.5) -> None:
        self.board_ip = board_ip
        self.board_port = board_port
        self.timeout_sec = timeout_sec

    def send(self, zone: Zone) -> None:
        payload = json.dumps(zone.to_dict(), ensure_ascii=False).encode("utf-8") + b"\n"
        with socket.create_connection(
            (self.board_ip, self.board_port), timeout=self.timeout_sec
        ) as conn:
            conn.sendall(payload)

    def display_target(self) -> str:
        return f"{self.board_ip}:{self.board_port}"


class SnapshotClient(Protocol):
    def fetch(self) -> SnapshotFrame:
        ...


class HttpSnapshotClient:
    def __init__(self, snapshot_url: str, timeout_sec: float = 3.0) -> None:
        self.snapshot_url = snapshot_url
        self.timeout_sec = timeout_sec

    def fetch(self) -> SnapshotFrame:
        req = Request(self.snapshot_url, method="GET")
        with urlopen(req, timeout=self.timeout_sec) as resp:
            if resp.status != 200:
                raise RuntimeError(f"HTTP {resp.status}")
            data = resp.read()

        arr = np.frombuffer(data, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("无法解码板端返回的图片数据")
        height, width = image.shape[:2]
        return SnapshotFrame(image=image, logical_width=width, logical_height=height)


class FileSnapshotClient:
    def __init__(self, snapshot_file: Path) -> None:
        self.snapshot_file = snapshot_file

    def fetch(self) -> SnapshotFrame:
        if not self.snapshot_file.exists():
            raise RuntimeError(f"快照文件不存在: {self.snapshot_file}")
        image = cv2.imread(str(self.snapshot_file), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"无法读取快照文件: {self.snapshot_file}")
        height, width = image.shape[:2]
        return SnapshotFrame(image=image, logical_width=width, logical_height=height)


class SerialControlClient:
    def __init__(self, port: str, baudrate: int, timeout_sec: float, idle_timeout_sec: float) -> None:
        if serial is None:
            raise RuntimeError("缺少 pyserial，请先执行: pip install pyserial")
        self.port = port
        self.baudrate = baudrate
        self.timeout_sec = timeout_sec
        self.idle_timeout_sec = idle_timeout_sec

    def fetch(self) -> SnapshotFrame:
        with self._open_serial() as ser:
            ser.reset_input_buffer()
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(b"SNAPSHOT\n")
            ser.flush()
            header = self._read_snapshot_header(ser)
            parts = header.split()
            if len(parts) != 6:
                raise RuntimeError(f"板端快照头格式错误: {header}")
            preview_width = int(parts[1])
            preview_height = int(parts[2])
            logical_width = int(parts[3])
            logical_height = int(parts[4])
            payload_size = int(parts[5])
            payload = self._read_exact(ser, payload_size)
            frame = self._decode_pgm(payload)
            if frame.shape[1] != preview_width or frame.shape[0] != preview_height:
                raise RuntimeError("板端快照尺寸与头信息不一致")
            return SnapshotFrame(
                image=frame,
                logical_width=logical_width,
                logical_height=logical_height,
            )

    def send(self, zone: Zone) -> None:
        with self._open_serial() as ser:
            ser.reset_input_buffer()
            payload = json.dumps(zone.to_dict(), ensure_ascii=False)
            ser.write(f"ZONE {payload}\n".encode("utf-8"))
            ser.flush()
            self._try_wait_for_marker(ser, b"OK ZONE", timeout_sec=0.5)
            ser.write(b"START\n")
            ser.flush()
            self._try_wait_for_marker(ser, b"OK START", timeout_sec=0.5)

    def display_target(self) -> str:
        return f"{self.port}@{self.baudrate}"

    def _open_serial(self):
        return serial.Serial(self.port, self.baudrate, timeout=self.timeout_sec)

    def _wait_for_marker(self, ser, marker: bytes, timeout_sec: float) -> None:
        buffer = bytearray()
        deadline = time.monotonic() + timeout_sec
        max_scan_bytes = 8192
        original_timeout = ser.timeout
        ser.timeout = 0.1
        try:
            while time.monotonic() < deadline:
                chunk = ser.read(64)
                if not chunk:
                    continue
                buffer.extend(chunk)
                if marker in buffer:
                    return
                if len(buffer) > max_scan_bytes:
                    del buffer[:-len(marker)]
        finally:
            ser.timeout = original_timeout
        raise RuntimeError(f"串口等待板端响应超时: {marker.decode('ascii', errors='ignore')}")

    def _try_wait_for_marker(self, ser, marker: bytes, timeout_sec: float) -> bool:
        try:
            self._wait_for_marker(ser, marker, timeout_sec)
            return True
        except RuntimeError:
            return False

    def _read_snapshot_header(self, ser) -> str:
        marker = b"SNAPSHOT "
        buffer = bytearray()
        max_scan_bytes = 8192
        while len(buffer) < max_scan_bytes:
            chunk = ser.read(1)
            if not chunk:
                raise RuntimeError("串口快照超时，未收到板端 SNAPSHOT 响应")
            buffer.extend(chunk)
            marker_pos = buffer.find(marker)
            if marker_pos < 0:
                if len(buffer) > len(marker):
                    del buffer[:-len(marker)]
                continue

            line_end = buffer.find(b"\n", marker_pos)
            if line_end >= 0:
                raw_header = bytes(buffer[marker_pos:line_end])
                return raw_header.decode("utf-8", errors="strict").strip()

        raise RuntimeError("串口快照响应中未找到 SNAPSHOT 头")

    def _read_exact(self, ser, size: int) -> bytes:
        chunks: list[bytes] = []
        remaining = size
        last_data_time = time.monotonic()
        original_timeout = ser.timeout
        ser.timeout = 0.05
        try:
            while remaining > 0:
                chunk = ser.read(remaining)
                if not chunk:
                    received = size - remaining
                    idle_sec = time.monotonic() - last_data_time
                    if received > 0 and (remaining <= 128 or idle_sec >= self.idle_timeout_sec):
                        chunks.append(b"\x00" * remaining)
                        break
                    if idle_sec >= self.timeout_sec:
                        raise RuntimeError("串口读取图片数据超时")
                    continue
                chunks.append(chunk)
                remaining -= len(chunk)
                last_data_time = time.monotonic()
        finally:
            ser.timeout = original_timeout
        return b"".join(chunks)

    def _decode_pgm(self, payload: bytes) -> np.ndarray:
        if payload.startswith(b"qoif"):
            return self._decode_qoi(payload)
        arr = np.frombuffer(payload, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("无法解码板端返回的串口快照")
        return image

    def _decode_qoi(self, payload: bytes) -> np.ndarray:
        if len(payload) < 22 or payload[:4] != b"qoif":
            raise RuntimeError("QOI 快照格式错误")
        width = int.from_bytes(payload[4:8], "big")
        height = int.from_bytes(payload[8:12], "big")
        channels = payload[12]
        if channels not in (3, 4):
            raise RuntimeError(f"不支持的 QOI 通道数: {channels}")

        index = [(0, 0, 0, 0)] * 64
        r = g = b = 0
        a = 255
        pixels: list[tuple[int, int, int]] = []
        pos = 14
        total_pixels = width * height

        while len(pixels) < total_pixels and pos < len(payload) - 8:
            byte = payload[pos]
            pos += 1

            if byte == 0xFE:
                if pos + 3 > len(payload):
                    break
                r, g, b = payload[pos], payload[pos + 1], payload[pos + 2]
                pos += 3
            elif byte == 0xFF:
                if pos + 4 > len(payload):
                    break
                r, g, b, a = payload[pos], payload[pos + 1], payload[pos + 2], payload[pos + 3]
                pos += 4
            else:
                tag = byte & 0xC0
                if tag == 0x00:
                    r, g, b, a = index[byte]
                elif tag == 0x40:
                    r = (r + ((byte >> 4) & 0x03) - 2) & 0xFF
                    g = (g + ((byte >> 2) & 0x03) - 2) & 0xFF
                    b = (b + (byte & 0x03) - 2) & 0xFF
                elif tag == 0x80:
                    if pos >= len(payload):
                        break
                    byte2 = payload[pos]
                    pos += 1
                    dg = (byte & 0x3F) - 32
                    dr_dg = ((byte2 >> 4) & 0x0F) - 8
                    db_dg = (byte2 & 0x0F) - 8
                    r = (r + dg + dr_dg) & 0xFF
                    g = (g + dg) & 0xFF
                    b = (b + dg + db_dg) & 0xFF
                else:
                    run = byte & 0x3F
                    for _ in range(run + 1):
                        pixels.append((b, g, r))
                    continue

            index_pos = (r * 3 + g * 5 + b * 7 + a * 11) % 64
            index[index_pos] = (r, g, b, a)
            pixels.append((b, g, r))

        if len(pixels) < total_pixels:
            pixels.extend([(0, 0, 0)] * (total_pixels - len(pixels)))
        arr = np.array(pixels[:total_pixels], dtype=np.uint8).reshape((height, width, 3))
        return arr


class ZoneDrawerApp:
    def __init__(
        self,
        snapshot_client: SnapshotClient,
        sender: ZoneSender,
        zone_file: Path,
        auto_send: bool,
        auto_refresh_after_send: bool,
        auto_exit_after_send: bool,
        auto_refresh_snapshot: bool,
        alarm_classes: list[str],
        snapshot_refresh_interval_ms: int,
        startup_retry: int,
        startup_retry_interval_sec: float,
        window_width: int,
        window_height: int,
    ) -> None:
        self.snapshot_client = snapshot_client
        self.sender = sender
        self.zone_file = zone_file
        self.auto_send = auto_send
        self.auto_refresh_after_send = auto_refresh_after_send
        self.auto_exit_after_send = auto_exit_after_send
        self.auto_refresh_snapshot = auto_refresh_snapshot
        self.alarm_classes = list(alarm_classes)
        self.snapshot_refresh_interval_ms = snapshot_refresh_interval_ms
        self.startup_retry = startup_retry
        self.startup_retry_interval_sec = startup_retry_interval_sec
        self.window_width = window_width
        self.window_height = window_height

        self.last_snapshot_fetch_monotonic = 0.0
        self.current_image: np.ndarray | None = None
        self.logical_width = 0
        self.logical_height = 0
        self.draft_points: list[tuple[int, int]] = []
        self.hover_point: tuple[int, int] | None = None
        self.current_zone: Zone | None = self._load_zone()
        self.last_status = "Starting: fetching board snapshot..."
        self.exit_requested = False

    def _load_zone(self) -> Zone | None:
        if not self.zone_file.exists():
            return None
        try:
            data = json.loads(self.zone_file.read_text(encoding="utf-8"))
            if data.get("shape") == "polygon":
                points = data.get("points", [])
                parsed_points = [(int(point[0]), int(point[1])) for point in points]
                if len(parsed_points) >= 3:
                    alarm_classes = normalize_alarm_classes(data.get("alarm_classes", self.alarm_classes))
                    return Zone(shape="polygon", points=parsed_points, alarm_classes=alarm_classes)
                return None
            return Zone.from_rect(
                int(data["x1"]),
                int(data["y1"]),
                int(data["x2"]),
                int(data["y2"]),
                normalize_alarm_classes(data.get("alarm_classes", self.alarm_classes)),
            )
        except Exception:
            return None

    def _save_zone(self, zone: Zone | None) -> None:
        if zone is None:
            if self.zone_file.exists():
                self.zone_file.unlink()
            return
        self.zone_file.write_text(
            json.dumps(zone.to_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _make_blank(self) -> np.ndarray:
        blank = np.full((720, 1280, 3), 25, dtype=np.uint8)
        cv2.putText(
            blank,
            "No snapshot available",
            (430, 330),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (130, 130, 130),
            2,
        )
        cv2.putText(
            blank,
            "Press N to fetch snapshot",
            (455, 370),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (130, 130, 130),
            2,
        )
        return blank

    def _fetch_snapshot(self) -> bool:
        try:
            frame = self.snapshot_client.fetch()
        except (URLError, HTTPError, OSError, RuntimeError) as exc:
            self.last_status = f"Snapshot failed: {exc}"
            return False
        self.current_image = self._prepare_display_image(frame.image)
        self.logical_width = frame.logical_width
        self.logical_height = frame.logical_height
        h, w = self.current_image.shape[:2]
        self.last_snapshot_fetch_monotonic = time.monotonic()
        self.last_status = (
            f"Snapshot OK: preview {w}x{h} | logical {self.logical_width}x{self.logical_height}"
        )
        return True

    def _prepare_display_image(self, image: np.ndarray) -> np.ndarray:
        display = image
        if display.ndim == 2:
            gray = display
            min_val = int(gray.min())
            max_val = int(gray.max())
            if max_val > min_val:
                gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        elif display.shape[2] == 1:
            gray = display[:, :, 0]
            min_val = int(gray.min())
            max_val = int(gray.max())
            if max_val > min_val:
                gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            display = display.copy()

        source_h, source_w = display.shape[:2]
        scale = min(
            self.window_width / max(source_w, 1),
            self.window_height / max(source_h, 1),
        )
        scale = max(scale, 1.0)
        target_w = max(1, int(round(source_w * scale)))
        target_h = max(1, int(round(source_h * scale)))
        if target_w != source_w or target_h != source_h:
            display = cv2.resize(display, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
        return display

    def _fetch_snapshot_with_retry(self) -> bool:
        for attempt in range(1, self.startup_retry + 1):
            if self._fetch_snapshot():
                return True
            if attempt < self.startup_retry:
                time.sleep(self.startup_retry_interval_sec)
        return False

    def _draw_overlay(self, frame: np.ndarray) -> np.ndarray:
        display = frame.copy()

        if self.current_zone is not None:
            self._draw_zone(display, self.current_zone, (0, 0, 255), filled=True)

        if self.draft_points:
            draft_display = [self._point_to_display(point) for point in self.draft_points]
            if self.hover_point is not None:
                draft_display.append(self._point_to_display(self.hover_point))
            for start, end in zip(draft_display, draft_display[1:]):
                cv2.line(display, start, end, (0, 220, 255), 2)
            for index, point in enumerate(draft_display[: len(self.draft_points)]):
                radius = 7 if index == 0 else 5
                cv2.circle(display, point, radius, (0, 220, 255), -1)
            first = draft_display[0]
            cv2.circle(display, first, 12, (0, 220, 255), 2)

        cv2.rectangle(display, (0, 0), (display.shape[1], 48), (0, 0, 0), -1)
        cv2.putText(
            display,
            self.last_status,
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            display,
            "Click: add point | click first point: close | Z: undo | N: snapshot | S: send | C: clear | Q: quit",
            (10, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
        )

        return display

    def _draw_zone(
        self,
        display: np.ndarray,
        zone: Zone,
        color: tuple[int, int, int],
        filled: bool,
    ) -> None:
        display_points = np.array(
            [self._point_to_display(point) for point in zone.normalized().points],
            dtype=np.int32,
        )
        if display_points.size == 0:
            return
        overlay = display.copy()
        if zone.shape == "polygon" and len(display_points) >= 3:
            if filled:
                cv2.fillPoly(overlay, [display_points], color)
                cv2.addWeighted(overlay, 0.25, display, 0.75, 0, display)
            cv2.polylines(display, [display_points], True, color, 2)
            label_anchor = tuple(display_points[0])
        elif zone.shape == "rect" and len(display_points) >= 2:
            (x1, y1), (x2, y2) = display_points[:2]
            if filled:
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
                cv2.addWeighted(overlay, 0.25, display, 0.75, 0, display)
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            label_anchor = (x1, y1)
        else:
            return
        cv2.putText(
            display,
            f"Zone: {zone.describe()}",
            (label_anchor[0], max(label_anchor[1] - 8, 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    def _display_to_logical(self, x: int, y: int) -> tuple[int, int]:
        if self.current_image is None or self.logical_width <= 0 or self.logical_height <= 0:
            return x, y
        display_h, display_w = self.current_image.shape[:2]
        logical_x = round(x * self.logical_width / max(display_w, 1))
        logical_y = round(y * self.logical_height / max(display_h, 1))
        logical_x = max(0, min(logical_x, self.logical_width - 1))
        logical_y = max(0, min(logical_y, self.logical_height - 1))
        return logical_x, logical_y

    def _point_to_display(self, point: tuple[int, int]) -> tuple[int, int]:
        if self.current_image is None or self.logical_width <= 0 or self.logical_height <= 0:
            return point
        display_h, display_w = self.current_image.shape[:2]
        x = round(point[0] * display_w / max(self.logical_width, 1))
        y = round(point[1] * display_h / max(self.logical_height, 1))
        return x, y

    def _is_near_first_point(self, x: int, y: int) -> bool:
        if len(self.draft_points) < 3:
            return False
        first_x, first_y = self._point_to_display(self.draft_points[0])
        dx = x - first_x
        dy = y - first_y
        return dx * dx + dy * dy <= 14 * 14

    def _mouse_callback(self, event, x, y, _flags, _param) -> None:
        if self.current_image is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            if self._is_near_first_point(x, y):
                self.current_zone = Zone(
                    shape="polygon",
                    points=list(self.draft_points),
                    alarm_classes=list(self.alarm_classes),
                )
                self.draft_points.clear()
                self.hover_point = None
                self._save_zone(self.current_zone)
                self.last_status = f"Zone set: {self.current_zone.describe()}"
                if self.auto_send:
                    self._send_zone()
                return

            point = self._display_to_logical(x, y)
            self.draft_points.append(point)
            self.current_zone = None
            self._save_zone(None)
            self.last_status = f"Point {len(self.draft_points)} set: ({point[0]},{point[1]})"
        elif event == cv2.EVENT_MOUSEMOVE:
            self.hover_point = self._display_to_logical(x, y) if self.draft_points else None

    def _undo_point(self) -> None:
        if self.draft_points:
            removed = self.draft_points.pop()
            self.hover_point = None
            self.last_status = f"Removed point: ({removed[0]},{removed[1]})"
            return
        self.last_status = "No draft point to remove"

    def _finish_polygon_if_ready(self) -> bool:
        if len(self.draft_points) < 3:
            self.last_status = "Need at least 3 points before closing the polygon"
            return False
        self.current_zone = Zone(
            shape="polygon",
            points=list(self.draft_points),
            alarm_classes=list(self.alarm_classes),
        )
        self.draft_points.clear()
        self.hover_point = None
        self._save_zone(self.current_zone)
        self.last_status = f"Zone set: {self.current_zone.describe()}"
        return True

    def _send_zone(self) -> None:
        if self.current_zone is None and self.draft_points:
            if not self._finish_polygon_if_ready():
                return
        if self.current_zone is None:
            self.last_status = "No zone selected"
            return
        try:
            self.sender.send(self.current_zone)
        except OSError as exc:
            self.last_status = f"Send failed: {exc}"
            return

        self.last_status = (
            f"Sent to board {self.sender.display_target()} -> "
            f"{self.current_zone.describe()}"
        )
        if self.auto_exit_after_send:
            self.exit_requested = True
            return
        if self.auto_refresh_after_send:
            self._fetch_snapshot()

    def _clear_zone(self) -> None:
        self.current_zone = None
        self.draft_points.clear()
        self.hover_point = None
        self._save_zone(None)
        self.last_status = "Zone cleared"

    def _maybe_auto_refresh_snapshot(self) -> None:
        if not self.auto_refresh_snapshot or self.draft_points:
            return
        now = time.monotonic()
        elapsed_ms = (now - self.last_snapshot_fetch_monotonic) * 1000.0
        if elapsed_ms < self.snapshot_refresh_interval_ms:
            return
        self._fetch_snapshot()

    def run(self) -> int:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, self.window_width, self.window_height)
        cv2.setMouseCallback(WINDOW_NAME, self._mouse_callback)

        self._fetch_snapshot_with_retry()

        while True:
            frame = self.current_image if self.current_image is not None else self._make_blank()
            display = self._draw_overlay(frame)
            cv2.imshow(WINDOW_NAME, display)
            if self.exit_requested:
                cv2.destroyAllWindows()
                return 0

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                cv2.destroyAllWindows()
                return 0
            if key in (ord("n"), ord("N")):
                self._fetch_snapshot()
            elif key in (ord("s"), ord("S")):
                self._send_zone()
                if self.exit_requested:
                    cv2.destroyAllWindows()
                    return 0
            elif key in (ord("c"), ord("C")):
                self._clear_zone()
            elif key in (ord("z"), ord("Z"), 8):
                self._undo_point()
            self._maybe_auto_refresh_snapshot()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="上位机禁区控制器（单帧快照模式）：启动自动取图，直接画框并发送禁区。"
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help="控制器参数配置文件",
    )
    parser.add_argument(
        "--snapshot-url",
        default=None,
        help="板端单帧快照 HTTP 地址",
    )
    parser.add_argument(
        "--snapshot-source",
        choices=("http", "file", "serial"),
        default=None,
        help="快照来源：http 或 file",
    )
    parser.add_argument(
        "--snapshot-file",
        default=None,
        help="本地快照文件路径（file 模式使用）",
    )
    parser.add_argument(
        "--serial-port",
        default=None,
        help="串口端口，例如 COM3",
    )
    parser.add_argument(
        "--serial-baudrate",
        type=int,
        default=None,
        help="串口波特率，例如 115200",
    )
    parser.add_argument(
        "--serial-timeout-sec",
        type=float,
        default=None,
        help="串口超时时间（秒）",
    )
    parser.add_argument(
        "--serial-idle-timeout-sec",
        type=float,
        default=None,
        help="串口图片数据空闲补尾等待时间（秒）",
    )
    parser.add_argument(
        "--board-ip",
        default=None,
        help="板端 IP（用于 TCP 发送禁区）",
    )
    parser.add_argument(
        "--board-port",
        type=int,
        default=None,
        help="板端 TCP 端口（接收禁区）",
    )
    parser.add_argument(
        "--zone-file",
        default=None,
        help="本地禁区配置缓存文件",
    )
    parser.add_argument(
        "--auto-send",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="画框完成后是否自动发送",
    )
    parser.add_argument(
        "--auto-refresh-after-send",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="发送禁区后是否自动刷新快照",
    )
    parser.add_argument(
        "--auto-exit-after-send",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="发送禁区后是否自动关闭窗口",
    )
    parser.add_argument(
        "--auto-refresh-snapshot",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="运行期间是否自动刷新快照",
    )
    parser.add_argument(
        "--alarm-classes",
        default=None,
        help="告警类别白名单，逗号分隔，例如 person,dog,cat",
    )
    parser.add_argument(
        "--snapshot-refresh-interval-ms",
        type=int,
        default=None,
        help="自动刷新快照间隔（毫秒）",
    )
    parser.add_argument(
        "--startup-retry",
        type=int,
        default=None,
        help="启动时快照请求重试次数",
    )
    parser.add_argument(
        "--startup-retry-interval",
        type=float,
        default=None,
        help="启动重试间隔（秒）",
    )
    parser.add_argument(
        "--snapshot-timeout-sec",
        type=float,
        default=None,
        help="快照 HTTP 超时时间（秒）",
    )
    parser.add_argument(
        "--tcp-timeout-sec",
        type=float,
        default=None,
        help="TCP 发送超时时间（秒）",
    )
    parser.add_argument(
        "--window-width",
        type=int,
        default=None,
        help="窗口初始宽度",
    )
    parser.add_argument(
        "--window-height",
        type=int,
        default=None,
        help="窗口初始高度",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config_path = Path(args.config).resolve()
    try:
        config = load_controller_config(config_path)
        config = merge_args_into_config(config, args)
    except (FileNotFoundError, ValueError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    if config.snapshot_source == "http":
        snapshot_client: SnapshotClient = HttpSnapshotClient(
            snapshot_url=config.snapshot_url,
            timeout_sec=config.snapshot_timeout_sec,
        )
        sender: ZoneSender = TcpZoneSender(
            board_ip=config.board_ip,
            board_port=config.board_port,
            timeout_sec=config.tcp_timeout_sec,
        )
    else:
        if config.snapshot_source == "file":
            snapshot_client = FileSnapshotClient(
                snapshot_file=resolve_config_path(config.snapshot_file, config_path.parent)
            )
            sender = TcpZoneSender(
                board_ip=config.board_ip,
                board_port=config.board_port,
                timeout_sec=config.tcp_timeout_sec,
            )
        else:
            serial_client = SerialControlClient(
                port=config.serial_port,
                baudrate=config.serial_baudrate,
                timeout_sec=config.serial_timeout_sec,
                idle_timeout_sec=config.serial_idle_timeout_sec,
            )
            snapshot_client = serial_client
            sender = serial_client
    app = ZoneDrawerApp(
        snapshot_client=snapshot_client,
        sender=sender,
        zone_file=resolve_config_path(config.zone_file, config_path.parent),
        auto_send=config.auto_send,
        auto_refresh_after_send=config.auto_refresh_after_send,
        auto_exit_after_send=config.auto_exit_after_send,
        auto_refresh_snapshot=config.auto_refresh_snapshot,
        alarm_classes=config.alarm_classes,
        snapshot_refresh_interval_ms=config.snapshot_refresh_interval_ms,
        startup_retry=config.startup_retry,
        startup_retry_interval_sec=config.startup_retry_interval,
        window_width=config.window_width,
        window_height=config.window_height,
    )
    return app.run()


if __name__ == "__main__":
    raise SystemExit(main())
