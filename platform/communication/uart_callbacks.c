#include "platform/communication/host_link.h"

#include "devices/imu/hwt101/hwt101.h"

/** Routes shared HAL UART errors to the owning device driver. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Hwt101_OnUartError(huart);
    HostLink_OnUartError(huart);
}

/** Routes USART1 DMA completion to HostLink. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HostLink_OnUartTxComplete(huart);
}
