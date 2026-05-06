# HALO PC Zone Controller

本目录是 HALO (Home Alert & Location Observer) 的 PC 上位机工具，用于现场配置危险区。推荐使用串口快照模式：板端通过 UART 发送 Aurora 当前画面的低分辨率预览，PC 端在预览图上绘制区域，再把区域 JSON 发回板端。

## 快速使用

第一次使用：

```bat
cd pc_controller
pip install -r requirements.txt
```

每次使用前检查 `controller_config.json`：

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

上位机在 `serial` 模式下会自动完成这些步骤。发送成功后，板端进入正式检测并输出 `[FPS]`、`[LAT]`、`[CHECK]` 等验收日志。
