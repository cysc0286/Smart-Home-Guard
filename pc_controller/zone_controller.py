from __future__ import annotations

import argparse
import json
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from urllib.error import URLError, HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np

WINDOW_NAME = "Smart Home Guard - Snapshot Zone Controller"
DEFAULT_ZONE_FILE = "zone_config.json"


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


class ZoneSender:
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


class SnapshotClient:
    def __init__(self, snapshot_url: str, timeout_sec: float = 3.0) -> None:
        self.snapshot_url = snapshot_url
        self.timeout_sec = timeout_sec

    def fetch(self) -> np.ndarray:
        req = Request(self.snapshot_url, method="GET")
        with urlopen(req, timeout=self.timeout_sec) as resp:
            if resp.status != 200:
                raise RuntimeError(f"HTTP {resp.status}")
            data = resp.read()

        arr = np.frombuffer(data, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("无法解码板端返回的图片数据")
        return image


class ZoneDrawerApp:
    def __init__(
        self,
        snapshot_client: SnapshotClient,
        sender: ZoneSender,
        zone_file: Path,
        auto_send: bool,
        auto_refresh_after_send: bool,
        startup_retry: int,
        startup_retry_interval_sec: float,
    ) -> None:
        self.snapshot_client = snapshot_client
        self.sender = sender
        self.zone_file = zone_file
        self.auto_send = auto_send
        self.auto_refresh_after_send = auto_refresh_after_send
        self.startup_retry = startup_retry
        self.startup_retry_interval_sec = startup_retry_interval_sec

        self.current_image: np.ndarray | None = None
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
            self.current_image = self.snapshot_client.fetch()
        except (URLError, HTTPError, OSError, RuntimeError) as exc:
            self.last_status = f"快照获取失败: {exc}"
            return False
        h, w = self.current_image.shape[:2]
        self.last_status = f"快照获取成功: {w}x{h}，可直接画框"
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
            z = zone.normalized()
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

    def _mouse_callback(self, event, x, y, _flags, _param) -> None:
        if self.current_image is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.drag_start = (x, y)
            self.preview_zone = RectZone(x, y, x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging and self.drag_start is not None:
            self.preview_zone = RectZone(self.drag_start[0], self.drag_start[1], x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.dragging and self.drag_start is not None:
            self.dragging = False
            self.current_zone = RectZone(
                self.drag_start[0], self.drag_start[1], x, y
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

    def run(self) -> int:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1280, 720)
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="上位机禁区控制器（单帧快照模式）：启动自动取图，直接画框并发送禁区。"
    )
    parser.add_argument(
        "--snapshot-url",
        default="http://192.168.1.88:8081/snapshot.jpg",
        help="板端单帧快照 HTTP 地址",
    )
    parser.add_argument(
        "--board-ip",
        default="192.168.1.88",
        help="板端 IP（用于 TCP 发送禁区）",
    )
    parser.add_argument(
        "--board-port",
        type=int,
        default=9000,
        help="板端 TCP 端口（接收禁区）",
    )
    parser.add_argument(
        "--zone-file",
        default=DEFAULT_ZONE_FILE,
        help="本地禁区配置缓存文件",
    )
    parser.add_argument(
        "--auto-send",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="画框完成后是否自动发送（默认关闭）",
    )
    parser.add_argument(
        "--auto-refresh-after-send",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="发送禁区后是否自动刷新快照（默认开启）",
    )
    parser.add_argument(
        "--startup-retry",
        type=int,
        default=5,
        help="启动时快照请求重试次数",
    )
    parser.add_argument(
        "--startup-retry-interval",
        type=float,
        default=1.0,
        help="启动重试间隔（秒）",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    snapshot_client = SnapshotClient(snapshot_url=args.snapshot_url)
    sender = ZoneSender(board_ip=args.board_ip, board_port=args.board_port)
    app = ZoneDrawerApp(
        snapshot_client=snapshot_client,
        sender=sender,
        zone_file=Path(args.zone_file).resolve(),
        auto_send=args.auto_send,
        auto_refresh_after_send=args.auto_refresh_after_send,
        startup_retry=args.startup_retry,
        startup_retry_interval_sec=args.startup_retry_interval,
    )
    return app.run()


if __name__ == "__main__":
    raise SystemExit(main())
