#include "devices/imu/hwt101/hwt101.h"

#include <stdbool.h>
#include <string.h>

#include "usart.h"

#define HWT101_RING_BUFFER_SIZE 256U
#define HWT101_COMMAND_TIMEOUT_MS 20U

extern DMA_HandleTypeDef hdma_usart2_rx;

static const uint8_t k_unlock[] = {0xFFU, 0xAAU, 0x69U, 0x88U, 0xB5U};
static const uint8_t k_zero_yaw[] = {0xFFU, 0xAAU, 0x76U, 0x00U, 0x00U};
static const uint8_t k_output_200hz[] = {0xFFU, 0xAAU, 0x03U, 0x0BU, 0x00U};
static const uint8_t k_baud_115200[] = {0xFFU, 0xAAU, 0x04U, 0x06U, 0x00U};
static const uint8_t k_save[] = {0xFFU, 0xAAU, 0x00U, 0x00U, 0x00U};

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_rx_buffer[HWT101_RING_BUFFER_SIZE];

static uint16_t s_read_index;
static bool s_dma_started;
static volatile bool s_restart_requested;
static volatile Hwt101Diagnostics s_diagnostics;

Hwt101Data hwt101_data;
uint8_t hwt101_update_flag;

static uint16_t Hwt101_BufferedBytes(uint16_t write_index)
{
    if (write_index >= s_read_index)
    {
        return write_index - s_read_index;
    }
    return HWT101_RING_BUFFER_SIZE - s_read_index + write_index;
}

static uint8_t Hwt101_Peek(uint16_t offset)
{
    return s_rx_buffer[(s_read_index + offset) % HWT101_RING_BUFFER_SIZE];
}

static void Hwt101_InvalidateCache(void)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx_buffer,
                                    HWT101_RING_BUFFER_SIZE);
    }
}

static HAL_StatusTypeDef Hwt101_Send(const uint8_t command[5])
{
    return HAL_UART_Transmit(&huart2, command, 5U,
                             HWT101_COMMAND_TIMEOUT_MS);
}

HAL_StatusTypeDef Hwt101_Init(void)
{
    HAL_StatusTypeDef status;

    if (s_dma_started)
    {
        (void)HAL_UART_AbortReceive(&huart2);
        s_diagnostics.dma_restart_count++;
    }

    s_read_index = 0U;
    s_restart_requested = false;
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    SCB_CleanDCache_by_Addr((uint32_t *)s_rx_buffer,
                            HWT101_RING_BUFFER_SIZE);

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    status = HAL_UART_Receive_DMA(&huart2, s_rx_buffer,
                                  sizeof(s_rx_buffer));
    if (status != HAL_OK)
    {
        s_dma_started = false;
        s_restart_requested = true;
        return status;
    }

    s_dma_started = true;
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT | DMA_IT_TC);
    return HAL_OK;
}

void Hwt101_Process(void)
{
    uint16_t write_index;

    if (s_restart_requested)
    {
        (void)Hwt101_Init();
        return;
    }
    if (!s_dma_started)
    {
        return;
    }

    Hwt101_InvalidateCache();
    write_index = (uint16_t)(HWT101_RING_BUFFER_SIZE -
                             __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
    write_index %= HWT101_RING_BUFFER_SIZE;

    while (Hwt101_BufferedBytes(write_index) >= HWT101_FRAME_SIZE)
    {
        uint8_t frame[HWT101_FRAME_SIZE];
        Hwt101ParseResult result;

        if (Hwt101_Peek(0U) != HWT101_FRAME_HEADER)
        {
            s_read_index = (s_read_index + 1U) % HWT101_RING_BUFFER_SIZE;
            s_diagnostics.discarded_byte_count++;
            continue;
        }

        for (uint32_t i = 0U; i < HWT101_FRAME_SIZE; ++i)
        {
            frame[i] = Hwt101_Peek((uint16_t)i);
        }

        result = Hwt101_ParseFrame(frame, &hwt101_data);
        if (result == HWT101_PARSE_OK)
        {
            s_read_index = (s_read_index + HWT101_FRAME_SIZE) %
                           HWT101_RING_BUFFER_SIZE;
            s_diagnostics.valid_frame_count++;
            if (frame[1] == HWT101_FRAME_ANGLE)
            {
                hwt101_update_flag = (uint8_t)~hwt101_update_flag;
            }
        }
        else
        {
            s_read_index = (s_read_index + 1U) % HWT101_RING_BUFFER_SIZE;
            s_diagnostics.discarded_byte_count++;
            if (result == HWT101_PARSE_BAD_CHECKSUM)
            {
                s_diagnostics.checksum_error_count++;
            }
        }
    }
}

const volatile Hwt101Diagnostics *Hwt101_GetDiagnostics(void)
{
    return &s_diagnostics;
}

void Hwt101_ZeroYaw(void)
{
    (void)Hwt101_Send(k_zero_yaw);
    hwt101_data.yaw = 0.0f;
}

void Hwt101_Unlock(void)
{
    (void)Hwt101_Send(k_unlock);
}

void Hwt101_SetBaud115200(void)
{
    (void)Hwt101_Send(k_baud_115200);
}

void Hwt101_SetOutput200Hz(void)
{
    (void)Hwt101_Send(k_output_200hz);
}

void Hwt101_SaveSettings(void)
{
    (void)Hwt101_Send(k_save);
}

/* USART2 错误后请求 DMA 接收恢复。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        s_diagnostics.uart_error_count++;
        s_diagnostics.last_uart_error = huart->ErrorCode;
        s_restart_requested = true;
    }
}
