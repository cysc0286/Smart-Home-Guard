# A1 UART 与 FT232RL USB-TTL 接线说明

## 电平设置

先把 FT232RL 小板的电平选择切到 **1.8V**，再连接 A1。A1 平台 UART 信号电平是 1.8V，不能接 3.3V 或 5V TTL，否则有损坏 GPIO 的风险。

## 接线

| A1 平台 | FT232RL USB-TTL 小板 | 说明 |
| --- | --- | --- |
| GPIO_PIN_0 / UART_TX0 | RXD | A1 发送接 USB-TTL 接收 |
| GPIO_PIN_2 / UART_RX0 | TXD | A1 接收接 USB-TTL 发送 |
| GND | GND | 必须共地 |
| 1.8V | 不接 | 一般不要接，除非确认要由 A1 给外设供电 |
| 5V / 3.3V | 不接 | 不要接到 A1 UART 引脚 |

口诀：**TX 接 RX，RX 接 TX，GND 接 GND，电平选 1.8V**。

## 串口参数

当前项目默认参数：

```text
baudrate: 115200
data bits: 8
parity: none
stop bits: 1
flow control: none
```

对应文件：

- 板端：[ssne_ai_yolo_coco/include/coco_config.hpp](ssne_ai_yolo_coco/include/coco_config.hpp)
- PC 端：[pc_controller/controller_config.json](pc_controller/controller_config.json)

如果后续确认串口链路稳定，也可以把两个文件里的波特率同时提高到 `921600`。

## FT232RL 小板上的 1.8V 引脚怎么理解

很多 FT232RL 多电平小板会同时标出 `5V`、`3.3V`、`1.8V`，它们的含义要看小板跳帽/拨码设计：

- `TXD` / `RXD`：真正传 UART 数据的引脚。
- `GND`：地线，必须接。
- `1.8V`：通常是小板的 1.8V 电平参考或 1.8V 电源输出，不是 UART 数据线。

本项目里 A1 只需要和 USB-TTL 模块通信，不需要由 USB-TTL 给 A1 供电，所以推荐：

```text
只接 TXD、RXD、GND 三根线。
FT232RL 小板电平选择设为 1.8V。
FT232RL 小板的 1.8V 引脚不要接 A1。
```

只有一种情况可能需要接 `1.8V`：你的 USB-TTL 小板不是通过跳帽/拨码选择电平，而是要求外部把目标板的 IO 电压接到 `VCCIO` / `VREF` / `1.8V` 引脚，模块才知道 TXD/RXD 要用 1.8V。若你的板子丝印明确写的是 `VCCIO` 或 `VREF`，再把 A1 的 1.8V 接到这个参考脚。普通标着 `1.8V` 电源输出的脚不要接。

不确定时，用万用表量一下 USB-TTL 小板的 `TXD` 空闲电压：设置为 1.8V 后，`TXD` 对 `GND` 空闲应接近 1.8V。

## 使用流程

1. FT232RL 切到 1.8V。
2. 按上表接线，最后再插 USB。
3. Windows 设备管理器里确认串口号，例如 `COM9`。
4. 修改 [pc_controller/controller_config.json](pc_controller/controller_config.json) 里的 `serial_port` 为实际串口号。
5. 板端运行 `ssne_ai_yolo_coco/scripts/run.sh`，脚本会加载 `gpio_kmod.ko` 和 `uart_kmod.ko`。
6. PC 端运行 [pc_controller/run.bat](pc_controller/run.bat)。
7. 上位机通过串口发送 `SNAPSHOT` 获取预览，画框后按 `S` 发送 `ZONE ...` 和 `START`。

## 调试检查

- 板端启动日志应出现：`[UART] Ready at 115200 baud, 8N1, TX0=GPIO0, RX0=GPIO2`
- PC 端收不到快照时，先确认 `serial_port` 和 `serial_baudrate` 与板端一致。
- 串口调试工具手动测试时，发送命令必须带换行：

```text
SNAPSHOT
START
```

- 如果串口工具看到乱码，通常是波特率不一致或电平没有切到 1.8V。
- 如果完全无响应，优先检查 TX/RX 是否交叉、GND 是否共地、`uart_kmod.ko` 是否已加载。
