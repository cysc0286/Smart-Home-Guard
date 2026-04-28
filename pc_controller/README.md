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

先修改：

- `controller_config.json`

重点参数：

- `snapshot_source`：`serial`、`file` 或 `http`，当前无网络且仅有串口时推荐 `serial`
- `board_ip`：板端 IP
- `snapshot_url`：板端快照地址，推荐 `http://<board_ip>:8081/?action=snapshot`
- `snapshot_file`：本地快照文件路径，`file` 模式使用
- `serial_port`：串口端口，例如 `COM3`
- `serial_baudrate`：串口波特率，当前推荐 `115200`
- `board_port`：板端接收禁区的 TCP 端口，默认 `9000`
- `auto_send`：画框后是否自动发送
- `auto_refresh_snapshot`：是否持续自动刷新快照
- `snapshot_refresh_interval_ms`：自动刷新间隔
- `startup_retry`：启动时快照重试次数

然后双击运行：

- `run.bat`

`run.bat` 会自动读取 `controller_config.json`。

## 快捷键

- 鼠标左键拖拽：画矩形禁区
- `N`：请求最新快照
- `S`：发送当前禁区到板端
- `C`：清除禁区
- `Q` / `ESC`：退出

自动刷新开启时，程序会持续更新快照；**正在按住鼠标画框时会自动暂停刷新，松手后恢复**。

如果使用 `serial` 模式，当前推荐流程是：板子上电后先停在配置阶段，上位机取一张串口预览图，画框后按 `S` 发送区域并让板子开始正式检测。

## 可选参数（命令行运行时）

默认推荐直接改 `controller_config.json`。如果临时想覆盖配置文件，也可以命令行传参：

- `--snapshot-source serial|file|http`：选择快照来源
- `--snapshot-file`：本地快照文件路径
- `--serial-port`：串口端口
- `--serial-baudrate`：串口波特率
- `--serial-timeout-sec`：串口超时时间
- `--auto-send`：画框后自动发送禁区（默认开启，可用 `--no-auto-send` 临时关闭）
- `--auto-refresh-after-send`：发送后自动刷新快照（默认开启）
- `--auto-refresh-snapshot`：运行期间自动刷新快照（默认开启）
- `--snapshot-refresh-interval-ms`：自动刷新快照间隔毫秒数
- `--startup-retry`：启动时快照重试次数（默认 5）
- `--startup-retry-interval`：启动重试间隔秒数（默认 1.0）
- `--snapshot-timeout-sec`：快照 HTTP 超时时间
- `--tcp-timeout-sec`：TCP 发送超时时间
- `--window-width` / `--window-height`：窗口初始大小

## 板端必须配合的接口

### 1) HTTP 快照接口（必须）

如果板子可联网，推荐使用 **板端主程序内置的快照服务**。启动 `ssne_ai_yolo_coco` 后，板端会监听：

```text
GET /?action=snapshot
```

示例：

```text
http://<board_ip>:8081/?action=snapshot
```

返回一张当前帧图片。

当前内置服务实际返回的是 **PGM 灰度图**，但浏览器和本程序都可以正常打开。

如果板子**不能联网**，`ssne_ai_yolo_coco` 还会持续写出：

```text
/app_demo/latest_snapshot.pgm
```

此时上位机把 `snapshot_source` 设为 `file`，并让 `snapshot_file` 指向你拷到电脑上的 `latest_snapshot.pgm` 即可。

### 2) 串口启动配置（离线推荐）

如果板子不能联网，但串口可用，启动 `ssne_ai_yolo_coco` 后会先等待串口命令：

- `SNAPSHOT`：返回一张低分辨率灰度预览图
- `ZONE <json>`：写入禁区配置
- `START`：结束配置并开始正式检测

上位机 `serial` 模式会自动完成这三步。你只需要：

1. 板子上运行 `./ssne_ai_yolo_coco`
2. 电脑上把 `controller_config.json` 里的 `serial_port` 改成实际串口
3. 双击 `run.bat`
4. 画框后按 `S`

> 如果你还在使用旧版 `mjpg_streamer` 方案，通常也是 `?action=snapshot`，不是 `/snapshot.jpg`。

### 2) TCP 禁区接收接口（必须）

板端监听端口（默认 9000），接收上位机发送的单行 JSON：

```json
{"type":"zone_update","shape":"rect","x1":500,"y1":300,"x2":900,"y2":700}
```

## 板端自动化

当前推荐让 `ssne_ai_yolo_coco` 作为板端常驻程序启动。只要该程序在运行，快照接口就会同时可用：

```text
http://<board_ip>:8081/?action=snapshot
```

旧版 `board_autostart_mjpg.sh` 仅适用于 `mjpg_streamer + /dev/video0` 路线，不适用于当前这块基于 SmartSens SDK 取图的板子。
