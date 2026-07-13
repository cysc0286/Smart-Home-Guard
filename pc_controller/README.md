# HALO PC Zone Controller

本目录是 HALO (Home Alert & Location Observer) 的 PC 上位机工具，用于现场配置危险区。推荐使用串口快照模式：板端通过 UART 发送 Aurora 当前画面的低分辨率预览，PC 端在预览图上绘制区域，再把区域 JSON 发回板端。

## 快速使用

第一次使用：

```bat
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
- `serial_tx_chunk_bytes` / `serial_tx_chunk_gap_ms`：串口命令分块大小和间隔；当前板端推荐 `2` 字节、`30` 毫秒
- `serial_require_ack`：板端回传线路不稳定时设为 `false`，命令仍会按兼容模式发送
- `board_port`：板端接收禁区的 TCP 端口，默认 `9000`
- `auto_send`：画框后是否自动发送
- `auto_exit_after_send`：发送后是否退出；需要随时重画时应设为 `false`
- `auto_refresh_snapshot`：是否持续自动刷新快照
- `snapshot_refresh_interval_ms`：自动刷新间隔
- `startup_retry`：启动时快照重试次数

然后双击运行：

- `run.bat`

`run.bat` 会自动读取 `controller_config.json`。

## 快捷键

- 鼠标左键依次点击：绘制多边形顶点；点击第一个点闭合
- `N`：请求最新快照
- `S`：闭合并应用当前草稿；没有草稿时重发当前生效区域
- `Z` / `Backspace`：撤销最后一个草稿点
- `C`：取消草稿或待应用区域，不会清除板端正在生效的区域
- `Q` / `ESC`：退出

窗口采用“生效区域 / 草稿 / 待应用区域”三态：黄色填充表示当前生效区域，青色折线表示正在绘制的草稿，紫色轮廓表示已闭合但尚未发送的区域。重画期间板端继续检测旧区域；新区域只有在发送成功后才替换旧区域并保存。发送失败时旧区域保持生效，新区域保留为待应用状态，可以再次按 `S` 重试。`auto_exit_after_send=false` 时窗口会一直保留，可以反复重画。

串口模式下发送新区域时仍会补发一次 `START`。板端已经处于检测状态时，这不会停止当前检测；区域命令完整接收后会切换到新区域。

如果使用 `serial` 模式，当前推荐流程是：板子上电后先停在配置阶段，上位机取一张串口预览图，画框后按 `S` 发送区域并让板子开始正式检测。

## 可选参数（命令行运行时）

默认推荐直接改 `controller_config.json`。如果临时想覆盖配置文件，也可以命令行传参：

- `--snapshot-source serial|file|http`：选择快照来源
- `--snapshot-file`：本地快照文件路径
- `--serial-port`：串口端口
- `--serial-baudrate`：串口波特率
- `--serial-timeout-sec`：串口超时时间
- `--serial-tx-chunk-bytes`：串口命令发送分块大小
- `--serial-tx-chunk-gap-ms`：串口分块间隔毫秒数
- `--serial-ack-timeout-sec`：板端命令响应等待时间
- `--serial-command-retries`：严格响应模式下的命令重试次数
- `--serial-require-ack` / `--no-serial-require-ack`：切换严格响应或兼容发送模式
- `--auto-send`：画框后自动发送禁区（默认开启，可用 `--no-auto-send` 临时关闭）
- `--auto-refresh-after-send`：发送后自动刷新快照（默认开启）
- `--auto-exit-after-send`：发送后退出（常驻重画时使用 `--no-auto-exit-after-send`）
- `--auto-refresh-snapshot`：运行期间自动刷新快照（默认开启）
- `--snapshot-refresh-interval-ms`：自动刷新快照间隔毫秒数
- `--startup-retry`：启动时快照重试次数（默认 5）
- `--startup-retry-interval`：启动重试间隔秒数（默认 1.0）
- `--snapshot-timeout-sec`：快照 HTTP 超时时间
- `--tcp-timeout-sec`：TCP 发送超时时间
- `--window-width` / `--window-height`：窗口初始大小

```json
{
  "snapshot_source": "serial",
  "serial_port": "COM6",
  "serial_baudrate": 115200,
  "alarm_classes": ["person", "dog", "cat"]
}
```

运行：

```bat
run.bat
```

## 操作键

| 操作 | 说明 |
| --- | --- |
| 鼠标左键 | 逐点绘制多边形危险区 |
| `Z` / Backspace | 撤销最后一个点 |
| `C` | 清除危险区 |
| `N` | 重新请求一张快照 |
| `S` | 发送危险区到板端 |
| `Q` / Esc | 退出 |

## 工作模式

`snapshot_source` 支持三种：

- `serial`：推荐；无网络时可用，配合 `SNAPSHOT`、`ZONE`、`START` 串口命令。
- `http`：板端 HTTP 快照服务可用时使用。
- `file`：离线读取本地 `latest_snapshot.pgm`。

当前版本的自动刷新已移入后台线程，避免快照请求阻塞窗口交互。绘制过程中会暂停自动刷新，防止区域点位漂移。

## 坐标与显示约定

- 上位机发送的点位使用 1440x1080 crop 坐标。
- 板端危险区判断对多边形使用真实 point-in-polygon。
- 板端 OSD 对多边形危险区显示外接矩形，这是出于 OSD buffer 和现场稳定性的取舍；该显示方式不影响真实多边形告警判断。

## 串口配置阶段

板端启动后会等待配置命令：

```text
SNAPSHOT
ZONE <json>
START
```

旧版 `board_autostart_mjpg.sh` 仅适用于 `mjpg_streamer + /dev/video0` 路线，不适用于当前这块基于 SmartSens SDK 取图的板子。

## 排障记录

- [2026-07-13：串口无日志、Aurora 断连与画框控制排障](../docs/TROUBLESHOOTING_2026-07-13.md)
