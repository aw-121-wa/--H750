# 工训 H750 导航系统

STM32H750VBT6 四轮麦轮底盘固件和浏览器地图上位机。浏览器负责地图编辑、A* 路径规划和路线下发；H750 负责定位、路线跟踪、ZDT X42S 电机控制和故障停车。

## 硬件配置

| 设备 | 接口 | 参数 |
| --- | --- | --- |
| ZDT X42S 电机 1 至 4 | FDCAN1 PA11/PA12 | Classic CAN，1 Mbit/s，扩展帧 |
| HWT101 | USART2 PD5/PD6 | 115200 8N1，循环 RX DMA |
| 浏览器上位机 | USART1 PA9/PA10 | 115200 8N1，RX/TX DMA |

电机必须使用 **X 固件**、地址 `1/2/3/4`、`CAN1_MAP`、1 Mbit/s 和固定 `0x6B` 校验。CAN 总线两端安装 120 欧姆终端电阻，MCU 引脚必须通过 3.3V CAN 收发器连接 CAN_H/CAN_L。

X 固件速度命令：

```text
扩展 ID = motor_id << 8
F6 DIR ACC_H ACC_L SPEED_H SPEED_L SYNC 6B
```

- 加速度单位：RPM/s，底盘默认 `1000`。
- 速度单位：0.1 RPM，软件限制为 `±300.0 RPM`。
- 四轮命令使用 `SYNC=01` 缓存，再发送广播帧 `ID=0x000, FF 66 6B` 同步启动。
- 实时位置查询为 `36 6B`，X 固件回复的位置单位为 0.1 度。

## 目录

```text
control/                     底盘、航向、定位和导航
devices/                     HWT101 与 ZDT X42S 驱动
platform/communication/      USART 协议、命令分发和 DMA 适配
platform/time/               DWT 微秒延时
host/map-console/            Vite + TypeScript 浏览器上位机
tests/                       Linux 单元测试和 H750 软件模拟器
scripts/verify_software_link.sh  WSL 一键验证
Core/ Drivers/ Middlewares/  CubeMX 生成代码
```

FreeRTOS 任务：

- `START_TASK` 初始化 DWT、CAN、底盘、IMU、定位和上位机链路后退出。
- `IMU_TASK` 每 5 ms 处理 HWT101 数据。
- `HOST_TASK` 每 5 ms 解析 USART1，并以 10 Hz 上报姿态。
- `MAIN_TASK` 每 5 ms 更新定位、执行主机命令和导航状态机。

导航中发生电机反馈超时、IMU 超时、CAN 故障、连续发送失败或主机心跳超过 500 ms 时，底盘同步停车且不会自动恢复。

## 无硬件验证

在项目根目录运行：

```powershell
wsl.exe -d Ubuntu-24.04 -- bash scripts/verify_software_link.sh
```

脚本会校验固定版本的官方 Node.js 包，并在 WSL `/tmp` 中完成：

1. Linux GCC `-Wall -Wextra -Werror` 固件单元测试。
2. 浏览器上位机协议、规划和地图测试。
3. 浏览器 `SerialLink` 与真实 C 协议栈双向通信。
4. 路线提交、导航、麦轮解算和最终 FDCAN 帧逐字节校验。
5. 正负电机位置回复解析和 500 ms 心跳急停验证。
6. 上位机生产构建。

该测试证明软件数据链路、CRC、消息状态机和 ZDT 协议字节一致，不能代替 UART/CAN 收发器、电气连接、终端电阻和电机固件的实机验证。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release

cd host/map-console
npm.cmd ci
npm.cmd test
npm.cmd run build
```

固件输出为 `build/Debug/GongxunH750.elf` 和 `build/Release/GongxunH750.elf`。上位机使用 Chrome 或 Edge 打开 Vite 地址，通过 Web Serial 选择 USART1 对应串口。

## 首次实车检查

1. 架空四个车轮并将测试速度限制为 20 RPM。
2. 用 USB-CAN 确认 `0x100/0x200/0x300/0x400/0x000` 均为扩展数据帧。
3. 确认 `36 6B` 能收到四个电机的 X 固件位置回复。
4. 分别验证前进、横移、原地旋转和急停。
5. 单轮方向错误只修改对应电机方向配置，不修改麦轮解算公式。

## CAN 可靠性诊断与实机测试

### 诊断字段

- `motor_online_mask` 和定位状态位图使用 bit0 至 bit3 对应电机 1 至 4；`0x0F` 表示四台均在新鲜度窗口内。
- `bus_error_count` 是自启动以来的累计统计，不代表当前 CAN 是否健康。
- 当前 CAN 状态分为 `OK`、`WARNING`、`ERROR_PASSIVE` 和 `BUS_OFF`；`last_error_code`、`last_error_ms`、warning/passive/bus-off 计数用于定位故障时刻。
- 合法协议回复、非法 DLC/尾字节/帧、RX FIFO 满和 RX FIFO 丢帧分别有独立统计，便于区分“电机没回”和“MCU 收到但 FIFO 丢了”。

### CAN 急停限制

CAN Bus-Off 时，软件无法保证通过同一条已经失效的 CAN 总线向 X42S 发送 STOP。实车必须检查 PCB 是否已有电机使能、驱动失能、断电继电器或独立急停 GPIO；没有现成硬件时不要凭空定义 GPIO，也不要猜测未确认的电机协议命令。Bus-Off 恢复后固件会先尝试重新启动 FDCAN、发送四轮零速同步帧，并等待电机重新在线；导航故障保持锁存，不会自动继续之前的运动命令。

### Test A：单电机在线

架空小车，确认四台电机使用 X 固件、地址 1/2/3/4、`CAN1_MAP`、1 Mbit/s 和扩展数据帧。分别观察四台 `36 6B` 位置回复，确认 `motor_online_mask=0x0F`。

### Test B：20 RPM 前进

只允许测试速度 20 RPM。用 USB-CAN 确认 `0x100/0x200/0x300/0x400` 为扩展数据帧，随后确认广播同步帧 `0x000 FF 66 6B`；先验证前进，再验证横移、原地旋转和零速停止。

### Test C：运行中断开一台电机

在低速、架空条件下断开其中一台电机 CAN。期望在 200 ms 内对应在线位清零、导航进入 `FAULT` 并停止；其余电机的位置回复和定位采样应继续运行，不能因缺一台回复永久卡住。

### Test D：重新接回电机

重新接回后确认对应在线位恢复并最终得到 `0x0F`。导航不得自动恢复运动，必须由操作者重新启动导航；确认恢复前发送过零速同步帧。

### Test E：拔除 CAN 主线或制造 Bus-Off

记录 `bus_off_count`、`last_error_code`、`last_error_ms`、RX FIFO 统计和在线位图。确认底盘进入故障，恢复流程先零速、再等待四台在线，且不会自动恢复上一条速度命令。若需要真正的断能急停，必须验证独立硬件急停路径。

## 日志

- 2026.8.11 完成四轮 CAN 底盘、ZDT X42S 协议、IMU 和基础测试；新增 WSL vcan0 验证程序，主机测试、Debug 构建和虚拟 CAN 抓包均通过。
- 2026.8.12 编写上位机，并通过stm32H723ZGt6验证能正常收发数据
- 2026.8.13 wsl2训练物料与色环yolo26n模型并转化成onnx


## 待完成
- 验证代码与电路板是否真的能通过can通信控制电机
- 色环模型在图片上偶有漏检准确度需提高，将yolo26n量化部署到maixcam2
