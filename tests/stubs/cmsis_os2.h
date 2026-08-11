#ifndef TEST_CMSIS_OS2_H
#define TEST_CMSIS_OS2_H

#include <stdint.h>

typedef enum
{
    osOK = 0
} osStatus_t;

osStatus_t osDelay(uint32_t ticks);

#endif
