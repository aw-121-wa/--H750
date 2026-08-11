#ifndef PLATFORM_TIME_DELAY_H
#define PLATFORM_TIME_DELAY_H

#include <stdint.h>

/** 初始化基于 DWT 的旧版延时 API。 */
void Delay_Init(void);

/** 忙等待指定的微秒数。 */
void Delay_us(uint32_t microseconds);

/** 忙等待指定的毫秒数。 */
void Delay_ms(uint16_t milliseconds);

#endif
