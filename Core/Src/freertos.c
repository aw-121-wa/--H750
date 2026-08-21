/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control/chassis/chassis.h"
#include "control/heading/heading_control.h"
#include "control/localization/localization_service.h"
#include "control/navigation/navigation.h"
#include "devices/imu/hwt101/hwt101.h"
#include "devices/motor/zdt_x42s/zdt_x42s.h"
#include "platform/communication/host_link.h"
#include "platform/time/dwt_delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static NavigationRoute s_host_route;

/* USER CODE END Variables */
/* Definitions for IMU_TASK */
osThreadId_t IMU_TASKHandle;
const osThreadAttr_t IMU_TASK_attributes = {
  .name = "IMU_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for START_TASK */
osThreadId_t START_TASKHandle;
const osThreadAttr_t START_TASK_attributes = {
  .name = "START_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MAIN_TASK */
osThreadId_t MAIN_TASKHandle;
const osThreadAttr_t MAIN_TASK_attributes = {
  .name = "MAIN_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HOST_TASK */
osThreadId_t HOST_TASKHandle;
const osThreadAttr_t HOST_TASK_attributes = {
  .name = "HOST_TASK",
  .stack_size = 324 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void IMU_Task(void *argument);
void START_Task(void *argument);
void MAIN_Task(void *argument);
void HOST_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of IMU_TASK */
  IMU_TASKHandle = osThreadNew(IMU_Task, NULL, &IMU_TASK_attributes);

  /* creation of START_TASK */
  START_TASKHandle = osThreadNew(START_Task, NULL, &START_TASK_attributes);

  /* creation of MAIN_TASK */
  MAIN_TASKHandle = osThreadNew(MAIN_Task, NULL, &MAIN_TASK_attributes);

  /* creation of HOST_TASK */
  HOST_TASKHandle = osThreadNew(HOST_Task, NULL, &HOST_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/*
 * IMU_Task — 200Hz IMU 数据采集
 *
 * 职责：从 HWT101 环形 DMA 缓冲区中解析角度帧，更新 hwt101_data.yaw。
 * 调用链：IMU_Task → Hwt101_Process()
 *                                  ↓
 *                    Hwt101_Peek() → 校验帧头 0x55
 *                                  → Hwt101_ParseFrame() 校验和 + 解码
 *                                  → 写入 hwt101_data.yaw
 *
 * 输出：hwt101_data.yaw 被 MAIN_Task 读取用于定位积分和航向控制。
 * 周期：5ms（200Hz），匹配 HWT101 的 200Hz 输出率。
 */
void IMU_Task(void *argument)
{
  const uint32_t period_ticks = pdMS_TO_TICKS(5U);
  uint32_t next_wake;

  (void)argument;
  next_wake = osKernelGetTickCount();

  for(;;)
  {
    Hwt101_Process();          /* 解析 DMA 环形缓冲区中的所有完整帧 */
    next_wake += period_ticks;
    (void)osDelayUntil(next_wake);
  }
}

/*
 * START_Task — 一次性初始化任务（启动后自销毁）
 *
 * 职责：按依赖顺序初始化所有子系统，完成后退出释放资源。
 * 初始化顺序：
 *   1. Time_Init()               → 启用 DWT 周期计数器
 *   2. Chassis_Init()            → 启动 FDCAN，配置电机滤波器，发送同步停止
 *      → 内部调用 ZdtX42s_Init() + Chassis_SetWheelSpeedX10({0,0,0,0})
 *   3. Heading_Init()            → 初始化航向 PID，启动 HWT101 DMA
 *      → 内部调用 Pid_Init() + Hwt101_Init()
 *   4. LocalizationService_Init()→ 清零里程计状态，调用 Localization_Init()
 *   5. Navigation_Init()         → 清零导航状态机，置为 IDLE
 *   6. HostLink_Init()           → 启动 USART1 循环 DMA，创建命令队列
 *   7. Chassis_Stop()            → 发送零速确认底盘静止
 *   8. osThreadExit()            → 任务自销毁，释放栈内存
 */
void START_Task(void *argument)
{
  (void)argument;

  if (!Time_Init() ||                          /* DWT 周期计数器 */
      Chassis_Init() != HAL_OK ||              /* FDCAN + 电机驱动 */
      Heading_Init() != HAL_OK)                /* 航向 PID + IMU DMA */
  {
    Error_Handler();
  }
  LocalizationService_Init();                  /* 里程计轮询服务 */
  Navigation_Init();                           /* 导航状态机 */
  if (HostLink_Init() != HAL_OK)               /* 上位机串口协议 */
  {
    Error_Handler();
  }
  (void)Chassis_Stop();                        /* 确认底盘静止 */
  osThreadExit();                              /* 自销毁 */
}

/*
 * MAIN_Task — 200Hz 主控制循环（系统核心）
 *
 * 职责：定位积分、上位机命令分发、健康监测、导航推进、遥测回传。
 * 每个周期执行 5 个阶段，按顺序不可打乱：
 *
 * 阶段1 定位更新：
 *   LocalizationService_Tick()
 *     → 轮询电机位置（ZdtX42s_RequestPosition / ZdtX42s_GetPosition）
 *     → 四轮快照完成后调用 FinishSample() 校验
 *     → Localization_Update() 里程计积分，结合 IMU 偏航角
 *   LocalizationService_GetStatus() → 电机有效性、异常跳变
 *   Localization_GetStatus()        → 定位锚定状态
 *
 * 阶段2 命令处理：
 *   HostLink_TakeCommand() 从队列逐条取出上位机指令
 *   → SET_POSE          → Localization_SetMapPose()    设置地图位姿
 *   → SET_CALIBRATION   → Localization_SetCalibration() 设置轮径/轮距修正
 *   → ROUTE_COMMIT      → Navigation_LoadRoute()       加载路径点序列
 *   → NAV_START         → Navigation_Start()           启动路线跟踪
 *   → NAV_PAUSE         → Navigation_Pause()           暂停运动
 *   → NAV_RESUME        → Navigation_Resume()          恢复运动
 *   → NAV_STOP          → Navigation_Stop()            停止并清除路径
 *   HostLink_CompleteCommand() 回复 ACK/NAK
 *
 * 阶段3 健康监测：
 *   imu_valid       ← Hwt101_GetDiagnostics(): 有效帧>0 且 100ms 内有新帧
 *   can_valid       ← ZdtX42s_GetHealth(): 当前状态非 passive/bus-off 且无异常跳变
 *   heartbeat_valid ← HostLink_GetHeartbeatAgeMs(): 500ms 内有上位机帧
 *   Navigation_SetHealth() → 任一不健康则设置故障标志
 *
 * 阶段4 导航状态机：
 *   Navigation_Tick()
 *     → CheckHealth() 检查四项健康指标，不健康则 Chassis_Stop() + FAULT
 *     → ALIGNING: 原地旋转对齐目标点（Heading_TurnOutput）
 *     → MOVING:   直线行驶 + 航向纠偏（Chassis_SetSpeed）
 *     → 到达判定: 距离 < 10mm 切换下一个路径点
 *
 * 阶段5 状态上报：
 *   HostLink_SendPose()     → 回传位姿 (x, y, yaw)
 *   HostLink_SetStatus()    → 回传健康状态、故障标志、导航状态、当前路径点
 */
void MAIN_Task(void *argument)
{
  const uint32_t period_ticks = pdMS_TO_TICKS(5U);
  uint32_t next_wake = osKernelGetTickCount();

  (void)argument;
  for(;;)
  {
    const uint32_t now_ms = HAL_GetTick();
    const volatile Hwt101Diagnostics *imu_diagnostics = Hwt101_GetDiagnostics();
    LocalizationServiceStatus localization_status;
    ZdtX42sHealth can_health;
    LocalizationStatus pose_status;
    HostCommand command;
    bool imu_valid;
    bool can_valid;
    bool heartbeat_valid;
    NavigationPose pose;

    /* ── 阶段1：定位更新 ── */
    LocalizationService_Tick(now_ms, hwt101_data.yaw);
    localization_status = LocalizationService_GetStatus(now_ms);
    pose_status = Localization_GetStatus();
    ZdtX42s_Service(now_ms);

    /* ── 阶段2：命令处理 ── */
    while (HostLink_TakeCommand(&command))
    {
      NavigationState state = Navigation_GetState();
      bool stopped = state != NAVIGATION_ALIGNING &&
                     state != NAVIGATION_MOVING;
      bool command_success = false;

      switch (command.type)
      {
        case HOST_COMMAND_SET_POSE:
          if (stopped && pose_status.wheel_data_valid &&
              pose_status.imu_data_valid)
          {
            command_success = Localization_SetMapPose(&command.data.pose);
          }
          break;
        case HOST_COMMAND_SET_CALIBRATION:
          if (stopped)
          {
            command_success = Localization_SetCalibration(
                &command.data.calibration);
          }
          break;
        case HOST_COMMAND_ROUTE_COMMIT:
          if (HostLink_TakeCommittedRoute(&s_host_route))
          {
            command_success = Navigation_LoadRoute(&s_host_route) ==
                              NAVIGATION_OK;
          }
          break;
        case HOST_COMMAND_NAV_START:
          if (pose_status.pose_state == LOCALIZATION_ANCHORED)
          {
            command_success = Navigation_Start() == NAVIGATION_OK;
          }
          break;
        case HOST_COMMAND_NAV_PAUSE:
          command_success = state == NAVIGATION_ALIGNING ||
                            state == NAVIGATION_MOVING;
          if (command_success) Navigation_Pause();
          break;
        case HOST_COMMAND_NAV_RESUME:
          command_success = state == NAVIGATION_PAUSED &&
                            Navigation_GetFaultFlags() == 0U;
          if (command_success) Navigation_Resume();
          break;
        case HOST_COMMAND_NAV_STOP:
          Navigation_Stop();
          command_success = true;
          break;
        default:
          break;
      }
      HostLink_CompleteCommand(&command, command_success,
                               command_success ? 0U : 2U);
    }

    /* ── 阶段3：健康监测 ── */
    imu_valid = imu_diagnostics->angle_frame_count != 0U &&
                (now_ms - imu_diagnostics->last_angle_frame_ms) <= 100U;
    can_health = ZdtX42s_GetHealth(now_ms);
    can_valid = can_health.bus_state != ZDT_CAN_ERROR_PASSIVE &&
                can_health.bus_state != ZDT_CAN_BUS_OFF &&
                !localization_status.unreasonable_jump;
    heartbeat_valid = HostLink_GetHeartbeatAgeMs(now_ms) <= 500U;
    Navigation_SetHealth(localization_status.motor_valid_mask == 0x0FU &&
                         can_health.motor_online_mask == 0x0FU,
                         imu_valid, can_valid, heartbeat_valid);

    /* ── 阶段4：导航状态机 ── */
    Navigation_Tick(now_ms);

    /* ── 阶段5：状态上报 ── */
    pose = Localization_GetPose();
    HostLink_SendPose(&pose);
    HostLink_SetStatus(localization_status.motor_valid_mask, imu_valid,
                       Navigation_GetFaultFlags(), Navigation_GetState(),
                       Navigation_GetCurrentWaypoint());
    next_wake += period_ticks;
    (void)osDelayUntil(next_wake);
  }
}

/*
 * HOST_Task — 200Hz 上位机通信处理
 *
 * 职责：处理 USART1 收发，解析上位机协议帧，维护心跳。
 * 调用链：HOST_Task → HostLink_Process()
 *                                  ↓
 *                    解析 RX DMA 环形缓冲区中的协议帧
 *                    → 命令帧：入队到命令队列（供 MAIN_Task 取出）
 *                    → ACK/NAK：推进 TX DMA 发送队列
 *                    → 10Hz 姿态遥测打包
 *
 * 与 MAIN_Task 的分工：
 *   HOST_TASK 负责协议解析和入队（通信层）
 *   MAIN_TASK 负责命令执行和业务逻辑（控制层）
 *   通过 HostLink 内部的命令队列解耦
 */
void HOST_Task(void *argument)
{
  const uint32_t period_ticks = pdMS_TO_TICKS(5U);
  uint32_t next_wake = osKernelGetTickCount();

  (void)argument;
  for(;;)
  {
    HostLink_Process();          /* 解析协议帧、入队命令、发送遥测 */
    next_wake += period_ticks;
    (void)osDelayUntil(next_wake);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

