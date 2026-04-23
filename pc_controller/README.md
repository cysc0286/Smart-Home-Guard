# PC 端禁区画框控制器（单帧快照模式）

本目录是 **PC 上位机** 工具，和 `model/`、`ssne_ai_yolo_coco/` 并列。

本程序不依赖实时视频流，而是：

1. 启动后自动向板端请求一张当前快照（HTTP）
2. 在可视化窗口里直接画禁区
3. 通过 TCP 将禁区发送到板端

这套方式对低性能板子更友好。

## 一键启动（推荐）

### 第一次使用

```bash
cd pc_controller
pip install -r requirements.txt
```

### 之后每次使用

双击运行：

- `run.bat`

如果你板子 IP 不是 `192.168.1.88`，请先编辑 `run.bat` 顶部三行参数：

```bat
set BOARD_IP=192.168.1.88
set BOARD_PORT=9000
set SNAPSHOT_URL=http://%BOARD_IP%:8081/?action=snapshot
```

## 快捷键

- 鼠标左键拖拽：画矩形禁区
- `N`：请求最新快照
- `S`：发送当前禁区到板端
- `C`：清除禁区
- `Q` / `ESC`：退出

## 可选参数（命令行运行时）

- `--auto-send`：画框后自动发送禁区（默认关闭）
- `--auto-refresh-after-send`：发送后自动刷新快照（默认开启）
- `--startup-retry`：启动时快照重试次数（默认 5）
- `--startup-retry-interval`：启动重试间隔秒数（默认 1.0）

## 板端必须配合的接口

### 1) HTTP 快照接口（必须）

板端需提供：

```text
GET /?action=snapshot
```

示例：

```text
http://<board_ip>:8081/?action=snapshot
```

返回一张 JPEG 图片（当前帧）。

> 注意：如果你使用 `mjpg_streamer`，通常是 `?action=snapshot`，不是 `/snapshot.jpg`。

### 2) TCP 禁区接收接口（必须）

板端监听端口（默认 9000），接收上位机发送的单行 JSON：

```json
{"type":"zone_update","shape":"rect","x1":500,"y1":300,"x2":900,"y2":700}
```

## 板端自动化（已给模板）

本目录已提供 `board_autostart_mjpg.sh`，用于板端开机自启快照服务：

1. 将脚本拷到板端（如 `/usr/local/bin/start_snapshot.sh`）
2. 执行：`chmod +x /usr/local/bin/start_snapshot.sh`
3. 在板端 `/etc/rc.local` 的 `exit 0` 前加：

```bash
/usr/local/bin/start_snapshot.sh &
```

重启板子后，快照地址可用：

```text
http://<board_ip>:8081/?action=snapshot
```
