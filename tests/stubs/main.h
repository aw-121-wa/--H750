#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

#ifndef TEST_HAL_STATUS_TYPE_DEFINED
#define TEST_HAL_STATUS_TYPE_DEFINED
typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;
#endif

typedef struct
{
    uint32_t ErrorCode;
} UART_HandleTypeDef;

typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
    volatile uint32_t LAR;
} DWT_Type;

typedef struct
{
    volatile uint32_t DEMCR;
} CoreDebug_Type;

extern DWT_Type test_dwt;
extern CoreDebug_Type test_core_debug;
extern uint32_t SystemCoreClock;
uint32_t test_read_cycles(void);

#define DWT (&test_dwt)
#define CoreDebug (&test_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk     (1UL << 0)

#endif
