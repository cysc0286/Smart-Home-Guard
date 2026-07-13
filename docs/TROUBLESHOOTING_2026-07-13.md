# 2026-07-13 串口、Aurora 与画框控制排障记录

本文记录一次已经完成实机验证的排障过程，供后续重新搭建和烧录时快速恢复工作状态。

## 已验证的链路与端口

本工程同时使用三条相互独立的链路，不要仅凭其中一条正常就判断整块板卡正常。

| 用途 | Windows 设备/端口 | 参数 | 作用 |
| --- | --- | --- | --- |
| 上位机控制串口 | 外接 CH340，实测为 `COM6` | 115200，8N1，3.3 V TTL | `SNAPSHOT`、`ZONE <json>`、`START` |
| Linux 调试串口 | CH347-A，实测为 `COM2` | 115200，8N1 | 查看 `[FPS]`、`[LAT]`、`[ENV]`、`[DET]` 等运行日志 |
| Aurora 视频 | UVC 设备 `Smartsens-FlyingChip-A1-2` | USB VID:PID `04B4:00C9` | Aurora 图像和 OSD 显示 |

CH347-B 在本次机器上不是 Linux 控制台，选择它会表现为串口已连接但没有任何输出。

控制串口接线为 TX/RX 交叉并共地：板端 `TX0=GPIO0` 接 USB-UART 的 RX，板端 `RX0=GPIO2` 接 USB-UART 的 TX。板端启动日志应包含：

```text
[UART] Ready at 115200 baud, 8N1, TX0=GPIO0, RX0=GPIO2
```

COM 编号由 Windows 动态分配，换电脑或 USB 接口后可能改变，应以设备管理器中的设备名称为准。

## 问题一：Aurora 串口窗口没有日志

### 现象

Aurora 串口工具显示已连接，但接收区为空；板端检测和视频可能仍在运行。

### 根因与处理

1. 程序中的 `printf`/`stderr` 日志输出到 Linux 控制台，不是上位机控制串口。
2. Linux 控制台位于 CH347-A；本次稳定配置为 `COM2`、115200。
3. CH347-A 最初被分配到与蓝牙串口冲突的 COM 号，Windows 报 Code 31/名称冲突。
4. 改为 `COM10` 后独立串口程序能读取日志，但 Aurora 未列出该两位数端口。
5. 最终把 CH347-A 改为 `COM2`，Aurora 能正常显示持续日志。

Aurora 中的已验证设置：

- 端口：`COM2 (USB-HiSpeed-SERIAL-A CH347F)`
- 波特率：115200
- Shell 模式：可开启
- 自动连接：按需要开启，不影响日志内容

如果端口被占用，先关闭其他串口工具。一个串口同一时间只应由一个程序打开。

## 问题二：画框能发送，但 ACK 不稳定

上位机到板端的命令已经实测可用，但板端返回上位机的 ACK 字节曾出现不可读或超时。为避免“板端已收到区域，上位机却因 ACK 异常判定失败”，当前配置采用：

```json
{
  "serial_tx_chunk_bytes": 2,
  "serial_tx_chunk_gap_ms": 30.0,
  "serial_ack_timeout_sec": 0.5,
  "serial_command_retries": 3,
  "serial_require_ack": false
}
```

兼容模式仍会发送完整的 `ZONE` 和 `START`，但不会因为 ACK 不可读而撤销新区域。若后续修复板端 TX 回传，可将 `serial_require_ack` 改回 `true`，恢复严格确认与重试。

## 问题三：重新画框时检测被打断

已验证方案采用“生效区域、草稿、待应用区域”三态：

- 画新多边形时，板端继续使用旧区域检测。
- 闭合后新区域先进入待应用状态，不覆盖旧区域。
- 按 `S` 发送成功后，新区域才成为生效区域并保存。
- 发送失败时旧区域继续生效，待应用区域保留以便重试。
- `auto_exit_after_send=false` 让窗口发送后保持打开，可随时重新划定区域。

串口模式每次应用新区域时会补发一次 `START`。板端已在检测时，重复 `START` 不会停止检测。

## 问题四：串口恢复后 Aurora 画面冻结或设备离线

CH347 串口和 Aurora UVC 是两个独立 USB 功能。串口持续有日志，只能说明 CH347 正常，不能证明 UVC 摄像头已经枚举。

本次断电重连后，Windows 中 `VID_04B4&PID_00C9` 一度消失，Aurora 左侧设备显示红色离线。恢复步骤如下：

1. 完全退出 Aurora，释放摄像头句柄。
2. 检查板卡供电和 USB 数据线，执行一次受控断电重启。
3. 等待板端启动以及 Windows 重新枚举 UVC 设备。
4. 确认设备管理器中重新出现 `Smartsens-FlyingChip-A1-2` 后再启动 Aurora。
5. 分别确认 COM2 调试日志和 Aurora 视频，不能互相替代。
6. 板端重启后如果停在等待命令阶段，运行上位机或发送 `START` 恢复检测。

PowerShell 可用以下命令快速确认设备：

```powershell
Get-PnpDevice -PresentOnly | Where-Object {
  $_.InstanceId -match 'VID_04B4&PID_00C9|VID_1A86&PID_55DE'
} | Format-Table Status, Class, FriendlyName, InstanceId -AutoSize

[System.IO.Ports.SerialPort]::GetPortNames()
```

## 推荐的排查顺序

1. 先查 Windows 是否枚举出目标设备，不要先在 Aurora 中反复切换端口。
2. 用 CH347-A/115200 验证 Linux 运行日志。
3. 用外接 CH340 验证 `SNAPSHOT`、`ZONE`、`START` 控制链路。
4. 单独确认 UVC `04B4:00C9` 和 Aurora 视频。
5. 最后运行 `pc_controller/run.bat`，确认黄框可重复更新、绿框检测正常、人物进入危险区域时显示红框。

截至 2026-07-13，以上配置已实机验证可以同时完成画框、检测、OSD 显示和调试日志输出。
