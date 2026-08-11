# 工训 H750 固件

STM32H750VBT6 机器人固件。当前构建包含四轮 CAN 底盘、HWT101 IMU 接收、航向控制、FreeRTOS 任务和 DWT 延时。

## 硬件

| 设备 | 接口 | 配置 |
| --- | --- | --- |
| ZDT X42S 电机 | FDCAN1, PA11/PA12 | 经典 CAN, 1 Mbit/s, 扩展帧 ID |
| HWT101 IMU | USART2, PD5/PD6 | 115200 波特率, 循环 RX DMA |

电机 ID 为 `1`、`2`、`3` 和 `4`。速度命令使用扩展帧 ID `0x100`、`0x200`、`0x300` 和 `0x400`；同步启动帧使用扩展帧 ID `0x000`。

USART2 DMA 缓冲区放置在 SRAM D2 中，因为 DMA1 在 STM32H7 上无法访问 DTCM。

## 项目结构

```text
app/          可选的应用诊断及后续比赛流程
control/      底盘运动学、航向控制器及可复用的控制代码
devices/      外部硬件驱动：ZDT X42S 和 HWT101
platform/     MCU 工具函数，如基于 DWT 的延时
Core/         STM32CubeMX 生成的启动、外设、中断和 RTOS 代码
Drivers/      STM32 HAL 和 CMSIS
Middlewares/  FreeRTOS
tests/        镜像生产目录结构的本地单元测试
```

自定义代码请放在 `Core/`、`Drivers/` 和 `Middlewares/` 之外。CubeMX 可能会重新生成这些目录。自定义源文件在顶层 `CMakeLists.txt` 中注册。

## 运行时

`START_TASK` 初始化 DWT 定时、FDCAN/底盘和 IMU，发送同步零速命令后退出。`IMU_TASK` 每 5 ms 处理循环 DMA 缓冲区。`MAIN_TASK` 不会自动移动底盘。

主要 API：

```c
HAL_StatusTypeDef Chassis_SetSpeed(float vy, float vx, float vw, bool fine);
HAL_StatusTypeDef ZdtX42s_SetSpeedX10(uint8_t id, int16_t rpm_x10,
                                     uint8_t acceleration, bool synchronized);
void Hwt101_Process(void);
float Heading_TurnOutput(float target_angle);
void Time_DelayUs(uint32_t microseconds);
```

旧名称如 `Motor_setspeed`、`ZDT_X42S_SetSpeedX10`、`IMU_Process`、`Gyro_Init` 和 `DWT_DelayUs` 仍作为兼容别名可用。

`Time_DelayUs()` 仅用于短时阻塞延时。FreeRTOS 任务的毫秒级等待请使用 `osDelay()` 或 `osDelayUntil()`。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug

cmake --preset Release
cmake --build --preset Release
```

固件输出：

```text
build/Debug/GongxunH750.elf
build/Release/GongxunH750.elf
```

## 单元测试

```powershell
cmake -S . -B build/host-tests -G Ninja -DBUILD_HOST_TESTS=ON
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

## 硬件测试安全

- 首次电机测试前请架起所有车轮。
- 初始命令限制为 20 RPM。
- CAN 总线两端安装 120 欧姆终端电阻。
- 确认每帧 CAN 帧为 1 Mbit/s 的扩展数据帧。
- 单个车轮方向请在电机配置中修正，而非在底盘运动学公式中修正。

## 日志

- 2026.8.11 完成四轮 CAN 底盘、ZDT X42S 协议、IMU 和基础测试；新增 WSL `vcan0` 验证程序，主机测试、Debug 构建和虚拟 CAN 抓包均通过。
