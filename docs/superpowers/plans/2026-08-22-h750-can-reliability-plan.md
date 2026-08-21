# H750 CAN 底盘可靠性重构 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with review checkpoints.

**Goal:** 修复 H750 四轮 CAN 底盘的采样死锁、RX 堆积、快照竞争和实时健康判断，并保持现有协议与分层。

**Architecture:** 先在定位服务中加入有超时的批量采样状态机，再在 ZDT 驱动中加入一致性快照、FIFO 排空、严格解析和实时诊断；底盘只消费驱动/定位健康结果，并在运动学编码前做整体比例限幅。

**Tech Stack:** C11、STM32H7 HAL FDCAN、CMake/Ninja、Linux GCC 主机单元测试、WSL 软件链路验证。

## Global Constraints

- 保留 Classic CAN、1 Mbit/s、扩展帧和现有 X42S `F6`、`36 6B`、`FF 66 6B` 协议。
- 不引入动态内存、C++、新的通信协议或未经 `stm32h7xx_hal_fdcan.h` 确认的 HAL API。
- 不删除现有测试；每个任务先写回归测试并确认 RED，再写最小实现并确认 GREEN。
- 每个任务完成后运行对应测试和 `git diff --check`，确认修改范围未扩大。

---

### Task 1: 建立基线与测试入口

**Files:**
- Read: `CMakeLists.txt`, `tests/CMakeLists.txt`, `README.md`
- Verify: `scripts/verify_software_link.sh`

**Interfaces:**
- Produces: 可重复执行的主机测试和固件构建基线。

- [ ] **Step 1: Confirm repository state**

Run:

```powershell
git status --short --branch
git log -1 --oneline
```

Expected: 当前分支为 `zt`，HEAD 为 `8a4ea997` 或其后续用户确认提交，且不覆盖已有未提交改动。

- [ ] **Step 2: Run existing host tests**

Run:

```powershell
cmake -S . -B build/HostTests -G Ninja -DBUILD_HOST_TESTS=ON
cmake --build build/HostTests
ctest --test-dir build/HostTests --output-on-failure
```

Expected: 记录基线结果；若失败，保留失败证据，不把它误报为本次回归。

- [ ] **Step 3: Check patch whitespace**

Run `git diff --check`; expected no output.

### Task 2: 位置采样 timeout

**Files:**
- Modify: `control/localization/localization_service.c`
- Modify: `control/localization/localization_service.h`
- Test: `tests/control/localization/localization_service_test.c`

**Interfaces:**
- Produces: `LocalizationService_Tick(now_ms, imu_yaw_deg)` 在 IDLE/WAIT 状态间有界运行；状态结构新增 timeout、缺失位图、跨度拒绝和跳变计数，保持现有调用兼容。

- [ ] **Step 1: Write the failing test**

构造 M1/M2/M3 新序列、M4 不更新，调用超过 20 ms 的 tick，断言没有定位更新且下一次 tick 又产生四个请求；增加 M3 请求返回 `HAL_BUSY` 时本批次不进入 pending 的测试。

- [ ] **Step 2: Run RED**

```powershell
cmake --build build/HostTests --target localization_service_test
ctest --test-dir build/HostTests -R localization_service_test --output-on-failure
```

Expected: 当前实现因无 timeout 或半启动状态而失败。

- [ ] **Step 3: Implement the minimal state machine**

加入 `POSITION_REPLY_TIMEOUT_MS=20U`、`POSITION_SAMPLE_PERIOD_MS=20U`、`POSITION_STALE_MS=200U`、`POSITION_MAX_SAMPLE_SPAN_MS=10U`；`StartSample(now_ms)` 先检查 TX 容量、保存四台 sequence，再连续入队四个请求。任一失败清除 pending/mask；等待超时记录 `missing_mask` 和计数，清除 pending，安排下一周期。

- [ ] **Step 4: Run GREEN and inspect diff**

运行同一 `ctest` 命令和 `git diff --check`；确认完整批次、旧回复隔离、不完整批次和请求失败重试均通过。

### Task 3: 位置快照一致性

**Files:**
- Modify: `devices/motor/zdt_x42s/zdt_x42s.c`
- Modify: `devices/motor/zdt_x42s/zdt_x42s.h`
- Test: `tests/devices/motor/zdt_x42s/zdt_x42s_driver_test.c`

**Interfaces:**
- Produces: `ZdtX42s_GetPositionSample()` 返回同一帧的 position/timestamp/valid/sequence，公开 sequence 每个合法位置帧递增一次。

- [ ] **Step 1: Write and run the failing consistency test**

连续注入两个不同时间和位置的合法位置帧，断言每次读取的字段属于同一帧，非法帧不改变快照；先运行 `ctest --test-dir build/HostTests -R zdt_x42s_driver_test --output-on-failure` 确认测试能捕获当前边界。

- [ ] **Step 2: Implement the internal write guard**

每个位置槽增加内部 guard；ISR 写入前后用 `__DMB()` 和奇偶 guard 标记更新区间，读取侧循环比较前后 guard，禁止复制更新中的槽。保留公开 `sample.sequence` 的一帧一次递增语义。

- [ ] **Step 3: Run GREEN**

```powershell
cmake --build build/HostTests --target zdt_x42s_driver_test
ctest --test-dir build/HostTests -R zdt_x42s_driver_test --output-on-failure
git diff --check
```

### Task 4: RX FIFO、严格解析和过滤器

**Files:**
- Modify: `devices/motor/zdt_x42s/zdt_x42s.c`
- Modify: `devices/motor/zdt_x42s/zdt_x42s.h`
- Modify: `tests/devices/motor/zdt_x42s/zdt_x42s_driver_test.c`
- Read: `Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h`

**Interfaces:**
- Produces: 独立 RX parser、FIFO 排空、非法帧诊断、每台电机合法回复时间戳和显式 filter 配置。

- [ ] **Step 1: Verify actual HAL symbols**

在 `stm32h7xx_hal_fdcan.h` 中确认 `HAL_FDCAN_GetRxFifoFillLevel`、FIFO full/message lost 通知宏及回调签名；只使用实际存在的名称。

- [ ] **Step 2: Extend the fake and write RED tests**

用固定数组保存多帧 header/data，fake fill level 和逐帧弹出；测试一次 callback 处理全部合法帧，并测试非法 DLC、尾字节、未知功能、错误 ID、标准帧和远程帧不会刷新位置或合法回复计数。断言 filter 为显式 ID 范围而非零掩码。

- [ ] **Step 3: Run RED**

```powershell
cmake --build build/HostTests --target zdt_x42s_driver_test
ctest --test-dir build/HostTests -R zdt_x42s_driver_test --output-on-failure
```

Expected: 当前 callback 只取一帧且 filter ID 为 0，新断言失败。

- [ ] **Step 4: Implement parser and FIFO drain**

先验证扩展数据帧、ID 1..4、DLC、功能码和尾字节；合法 `F6` 更新 motor reply，合法 `36` 更新 position reply，非法路径只更新对应诊断计数。callback 循环读取 FIFO0 直到为空，每帧调用 parser。filter 设置扩展 RANGE `0x100..0x4FF`，全局拒收标准/非匹配/远程帧。

- [ ] **Step 5: Run GREEN**

运行驱动测试和 `git diff --check`；确认多帧排空、sequence 每帧一次递增、timestamp 和非法帧诊断均通过。

### Task 5: 电机在线与 CAN 实时健康

**Files:**
- Modify: `devices/motor/zdt_x42s/zdt_x42s.c`
- Modify: `devices/motor/zdt_x42s/zdt_x42s.h`
- Modify: `Core/Src/freertos.c` only at the existing 5 ms service point
- Test: `tests/devices/motor/zdt_x42s/zdt_x42s_driver_test.c`

**Interfaces:**
- Produces: `ZdtX42s_GetMotorOnlineMask(uint32_t now_ms)`、`ZdtX42s_GetHealth(uint32_t now_ms)` 和 `ZdtX42s_Service(uint32_t now_ms)`；`bus_error_count` 只作累计统计。

- [ ] **Step 1: Write and run RED tests**

注入四台不同时间的合法回复，验证 200 ms 新鲜度位图；调用错误状态回调，验证 warning/passive/bus-off 分别更新当前枚举和独立计数，而不是只累加 `bus_error_count`。

- [ ] **Step 2: Verify recovery API before implementation**

若 HAL 头文件提供明确 Bus-Off 恢复 API，则 fake 它并测试恢复后先 STOP、等待在线 `0x0F`、导航仍保持 FAULT；若没有，保留状态锁存和停止尝试，并在 README 明确恢复限制，不猜函数名。

- [ ] **Step 3: Implement diagnostics and service**

增加 `OK/WARNING/ERROR_PASSIVE/BUS_OFF` 枚举、各计数、`last_error_ms` 和每台电机 `last_*_reply_ms`；在现有 5 ms 任务中调用 service，只把健康结果传给导航层。

- [ ] **Step 4: Run GREEN**

```powershell
cmake --build build/HostTests --target zdt_x42s_driver_test navigation_test
ctest --test-dir build/HostTests -R "zdt_x42s_driver_test|navigation_test" --output-on-failure
git diff --check
```

### Task 6: 麦轮比例限幅与有限浮点输入

**Files:**
- Modify: `control/chassis/chassis.c`
- Modify: `control/chassis/chassis_kinematics.c`
- Modify: `control/chassis/chassis_kinematics.h`
- Modify: `tests/control/chassis/chassis_kinematics_test.c`

**Interfaces:**
- Produces: `Chassis_NormalizeWheelRpm(float wheel_rpm[CHASSIS_WHEEL_COUNT])`；`Chassis_SetSpeed()` 对 NaN/Inf 返回 `HAL_ERROR`，四轮按统一比例缩放。

- [ ] **Step 1: Write and run RED tests**

验证 `[400,200,100,300] -> [300,150,75,225]`，并验证 NaN、正 Inf、负 Inf 不会产生 CAN 发送帧。

- [ ] **Step 2: Implement and run GREEN**

入口使用 `isfinite()`；运动学计算后取四轮最大绝对值，超过 `CHASSIS_MAX_RPM` 时统一缩放，再调用现有编码函数。运行：

```powershell
cmake --build build/HostTests --target chassis_kinematics_test chassis_driver_test
ctest --test-dir build/HostTests -R "chassis_kinematics_test|chassis_driver_test" --output-on-failure
git diff --check
```

### Task 7: README 与最终验证

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Document hardware boundary and Test A-E**

写明 CAN Bus-Off 时同一条失效总线不能保证发送 STOP；只记录工程中已存在的独立使能/继电器/急停硬件，不凭空定义 GPIO 或协议命令。加入 20 RPM 架空、四台在线、拔单台、重新接回不自动恢复和 Bus-Off 记录步骤。

- [ ] **Step 2: Run software-link verification**

```powershell
wsl.exe -d Ubuntu-24.04 -- bash scripts/verify_software_link.sh
```

Expected: 退出码 0，主机、协议、模拟器和上位机检查全部通过。

- [ ] **Step 3: Build Debug and Release**

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

Expected: 两种配置退出码均为 0。

- [ ] **Step 4: Review final scope**

运行 `git diff --check`、`git status --short` 和 `git diff --stat`；最终报告列出修改文件、测试结果和仍需实机确认的项目。
