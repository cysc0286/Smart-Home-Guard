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
class RectZone:
    x1: int
    y1: int
    x2: int
    y2: int

    def normalized(self) -> "RectZone":
        return RectZone(
            x1=min(self.x1, self.x2),
            y1=min(self.y1, self.y2),
            x2=max(self.x1, self.x2),
            y2=max(self.y1, self.y2),
        )

    def to_dict(self) -> dict[str, int | str]:
        zone = self.normalized()
        return {
            "type": "zone_update",
            "shape": "rect",
            "x1": zone.x1,
            "y1": zone.y1,
            "x2": zone.x2,
            "y2": zone.y2,
        }


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
    board_ip: str = "192.168.1.88"
    board_port: int = 9000
    zone_file: str = DEFAULT_ZONE_FILE
    auto_send: bool = True
    auto_refresh_after_send: bool = True
    auto_refresh_snapshot: bool = True
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
    config.snapshot_refresh_interval_ms = max(int(config.snapshot_refresh_interval_ms), 100)
    config.window_width = max(int(config.window_width), 320)
    config.window_height = max(int(config.window_height), 240)
    config.auto_send = bool(config.auto_send)
    config.auto_refresh_after_send = bool(config.auto_refresh_after_send)
    config.auto_refresh_snapshot = bool(config.auto_refresh_snapshot)
    config.snapshot_source = str(config.snapshot_source).strip().lower()
    config.snapshot_url = str(config.snapshot_url)
    config.snapshot_file = str(config.snapshot_file)
    config.serial_port = str(config.serial_port)
    config.board_ip = str(config.board_ip)
    config.zone_file = str(config.zone_file)
    if config.snapshot_source not in {"http", "file", "serial"}:
        raise ValueError("snapshot_source 只能是 'http'、'file' 或 'serial'")
    return config


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
    if args.auto_refresh_snapshot is not None:
        config.auto_refresh_snapshot = args.auto_refresh_snapshot
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
    def send(self, zone: RectZone) -> None:
        ...


class TcpZoneSender:
    def __init__(self, board_ip: str, board_port: int, timeout_sec: float = 1.5) -> None:
        self.board_ip = board_ip
        self.board_port = board_port
        self.timeout_sec = timeout_sec

    def send(self, zone: RectZone) -> None:
        payload = json.dumps(zone.to_dict(), ensure_ascii=False).encode("utf-8") + b"\n"
        with socket.create_connection(
            (self.board_ip, self.board_port), timeout=self.timeout_sec
        ) as conn:
            conn.sendall(payload)


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
    def __init__(self, port: str, baudrate: int, timeout_sec: float) -> None:
        if serial is None:
            raise RuntimeError("缺少 pyserial，请先执行: pip install pyserial")
        self.port = port
        self.baudrate = baudrate
        self.timeout_sec = timeout_sec

    def fetch(self) -> SnapshotFrame:
        with self._open_serial() as ser:
            ser.reset_input_buffer()
            ser.write(b"SNAPSHOT\n")
            ser.flush()
            while True:
                line = ser.readline()
                if not line:
                    raise RuntimeError("串口快照超时，未收到板端响应")
                text = line.decode("utf-8", errors="ignore").strip()
                if not text.startswith("SNAPSHOT "):
                    continue
                parts = text.split()
                if len(parts) != 6:
                    raise RuntimeError(f"板端快照头格式错误: {text}")
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

    def send(self, zone: RectZone) -> None:
        with self._open_serial() as ser:
            ser.reset_input_buffer()
            payload = json.dumps(zone.to_dict(), ensure_ascii=False)
            ser.write(f"ZONE {payload}\n".encode("utf-8"))
            ser.flush()
            self._wait_for_prefix(ser, "OK ZONE")
            ser.write(b"START\n")
            ser.flush()
            self._wait_for_prefix(ser, "OK START")

    def _open_serial(self):
        return serial.Serial(self.port, self.baudrate, timeout=self.timeout_sec)

    def _wait_for_prefix(self, ser, prefix: str) -> None:
        while True:
            line = ser.readline()
            if not line:
                raise RuntimeError(f"串口等待板端响应超时: {prefix}")
            text = line.decode("utf-8", errors="ignore").strip()
            if text.startswith(prefix):
                return

    def _read_exact(self, ser, size: int) -> bytes:
        chunks: list[bytes] = []
        remaining = size
        while remaining > 0:
            chunk = ser.read(remaining)
            if not chunk:
                raise RuntimeError("串口读取图片数据超时")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _decode_pgm(self, payload: bytes) -> np.ndarray:
        arr = np.frombuffer(payload, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("无法解码板端返回的串口快照")
        return image


class ZoneDrawerApp:
    def __init__(
        self,
        snapshot_client: SnapshotClient,
        sender: ZoneSender,
        zone_file: Path,
        auto_send: bool,
        auto_refresh_after_send: bool,
        auto_refresh_snapshot: bool,
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
        self.auto_refresh_snapshot = auto_refresh_snapshot
        self.snapshot_refresh_interval_ms = snapshot_refresh_interval_ms
        self.startup_retry = startup_retry
        self.startup_retry_interval_sec = startup_retry_interval_sec
        self.window_width = window_width
        self.window_height = window_height

        self.last_snapshot_fetch_monotonic = 0.0
        self.current_image: np.ndarray | None = None
        self.logical_width = 0
        self.logical_height = 0
        self.dragging = False
        self.drag_start: tuple[int, int] | None = None
        self.preview_zone: RectZone | None = None
        self.current_zone: RectZone | None = self._load_zone()
        self.last_status = "启动中，正在请求板端快照..."

    def _load_zone(self) -> RectZone | None:
        if not self.zone_file.exists():
            return None
        try:
            data = json.loads(self.zone_file.read_text(encoding="utf-8"))
            return RectZone(
                x1=int(data["x1"]),
                y1=int(data["y1"]),
                x2=int(data["x2"]),
                y2=int(data["y2"]),
            ).normalized()
        except Exception:
            return None

    def _save_zone(self, zone: RectZone | None) -> None:
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
            self.last_status = f"快照获取失败: {exc}"
            return False
        self.current_image = frame.image
        self.logical_width = frame.logical_width
        self.logical_height = frame.logical_height
        h, w = self.current_image.shape[:2]
        self.last_snapshot_fetch_monotonic = time.monotonic()
        self.last_status = (
            f"快照获取成功: 预览 {w}x{h} | 逻辑坐标 {self.logical_width}x{self.logical_height}"
        )
        return True

    def _fetch_snapshot_with_retry(self) -> bool:
        for attempt in range(1, self.startup_retry + 1):
            if self._fetch_snapshot():
                return True
            if attempt < self.startup_retry:
                time.sleep(self.startup_retry_interval_sec)
        return False

    def _draw_overlay(self, frame: np.ndarray) -> np.ndarray:
        display = frame.copy()

        zone = self.preview_zone if self.dragging and self.preview_zone is not None else self.current_zone
        if zone is not None:
            z = self._logical_to_display(zone)
            overlay = display.copy()
            cv2.rectangle(overlay, (z.x1, z.y1), (z.x2, z.y2), (0, 0, 255), -1)
            cv2.addWeighted(overlay, 0.25, display, 0.75, 0, display)
            cv2.rectangle(display, (z.x1, z.y1), (z.x2, z.y2), (0, 0, 255), 2)
            cv2.putText(
                display,
                f"Zone: ({z.x1},{z.y1})-({z.x2},{z.y2})",
                (z.x1, max(z.y1 - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

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
            "Left-drag: draw | N: new snapshot | S: send | C: clear | Q: quit",
            (10, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
        )

        return display

    def _display_to_logical(self, x: int, y: int) -> tuple[int, int]:
        if self.current_image is None or self.logical_width <= 0 or self.logical_height <= 0:
            return x, y
        display_h, display_w = self.current_image.shape[:2]
        logical_x = round(x * self.logical_width / max(display_w, 1))
        logical_y = round(y * self.logical_height / max(display_h, 1))
        logical_x = max(0, min(logical_x, self.logical_width - 1))
        logical_y = max(0, min(logical_y, self.logical_height - 1))
        return logical_x, logical_y

    def _logical_to_display(self, zone: RectZone) -> RectZone:
        if self.current_image is None or self.logical_width <= 0 or self.logical_height <= 0:
            return zone.normalized()
        display_h, display_w = self.current_image.shape[:2]
        x1 = round(zone.x1 * display_w / max(self.logical_width, 1))
        y1 = round(zone.y1 * display_h / max(self.logical_height, 1))
        x2 = round(zone.x2 * display_w / max(self.logical_width, 1))
        y2 = round(zone.y2 * display_h / max(self.logical_height, 1))
        return RectZone(x1, y1, x2, y2).normalized()

    def _mouse_callback(self, event, x, y, _flags, _param) -> None:
        if self.current_image is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.drag_start = self._display_to_logical(x, y)
            self.preview_zone = RectZone(*self.drag_start, *self.drag_start)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging and self.drag_start is not None:
            current = self._display_to_logical(x, y)
            self.preview_zone = RectZone(self.drag_start[0], self.drag_start[1], current[0], current[1])
        elif event == cv2.EVENT_LBUTTONUP and self.dragging and self.drag_start is not None:
            self.dragging = False
            current = self._display_to_logical(x, y)
            self.current_zone = RectZone(
                self.drag_start[0], self.drag_start[1], current[0], current[1]
            ).normalized()
            self.preview_zone = None
            self.drag_start = None
            self._save_zone(self.current_zone)
            z = self.current_zone
            self.last_status = f"禁区已设定: ({z.x1},{z.y1})-({z.x2},{z.y2})"
            if self.auto_send:
                self._send_zone()

    def _send_zone(self) -> None:
        if self.current_zone is None:
            self.last_status = "当前没有禁区，请先画框"
            return
        try:
            self.sender.send(self.current_zone)
        except OSError as exc:
            self.last_status = f"发送失败: {exc}"
            return

        z = self.current_zone.normalized()
        self.last_status = (
            f"已发送到板端 {self.sender.board_ip}:{self.sender.board_port} -> "
            f"({z.x1},{z.y1})-({z.x2},{z.y2})"
        )
        if self.auto_refresh_after_send:
            self._fetch_snapshot()

    def _clear_zone(self) -> None:
        self.current_zone = None
        self.preview_zone = None
        self.drag_start = None
        self._save_zone(None)
        self.last_status = "禁区已清除"

    def _maybe_auto_refresh_snapshot(self) -> None:
        if not self.auto_refresh_snapshot or self.dragging:
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

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                cv2.destroyAllWindows()
                return 0
            if key in (ord("n"), ord("N")):
                self._fetch_snapshot()
            elif key in (ord("s"), ord("S")):
                self._send_zone()
            elif key in (ord("c"), ord("C")):
                self._clear_zone()
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
        "--auto-refresh-snapshot",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="运行期间是否自动刷新快照",
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
            )
            snapshot_client = serial_client
            sender = serial_client
    app = ZoneDrawerApp(
        snapshot_client=snapshot_client,
        sender=sender,
        zone_file=resolve_config_path(config.zone_file, config_path.parent),
        auto_send=config.auto_send,
        auto_refresh_after_send=config.auto_refresh_after_send,
        auto_refresh_snapshot=config.auto_refresh_snapshot,
        snapshot_refresh_interval_ms=config.snapshot_refresh_interval_ms,
        startup_retry=config.startup_retry,
        startup_retry_interval_sec=config.startup_retry_interval,
        window_width=config.window_width,
        window_height=config.window_height,
    )
    return app.run()


if __name__ == "__main__":
    raise SystemExit(main())
