#ifndef PLATFORM_TIME_DWT_DELAY_H
#define PLATFORM_TIME_DWT_DELAY_H

#include <stdbool.h>
#include <stdint.h>

/** 启用阻塞延时使用的 DWT 周期计数器。 */
bool Time_Init(void);

/** 忙等待指定的微秒数。 */
void Time_DelayUs(uint32_t microseconds);

/** 忙等待毫秒；RTOS 任务应使用 osDelay 代替。 */
void Time_DelayMsBlocking(uint32_t milliseconds);

/* 旧版 DWT 延时 API 别名。 */
#define DWT_Delay_Init Time_Init
#define DWT_DelayUs    Time_DelayUs

#endif
