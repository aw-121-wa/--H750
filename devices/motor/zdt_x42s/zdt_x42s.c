#include "devices/motor/zdt_x42s/zdt_x42s.h"

#include <string.h>

#include "devices/motor/zdt_x42s/zdt_x42s_protocol.h"

static FDCAN_HandleTypeDef *s_fdcan;
static volatile ZdtX42sDiagnostics s_diagnostics;

static HAL_StatusTypeDef ZdtX42s_Send(
    uint32_t identifier,
    uint32_t data_length,
    uint8_t data[ZDT_X42S_CAN_MAX_DATA_LENGTH])
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef status;

    if (s_fdcan == NULL || data == NULL)
    {
        s_diagnostics.tx_failure_count++;
        return HAL_ERROR;
    }

    header.Identifier = identifier;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = data_length;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    status = HAL_FDCAN_AddMessageToTxFifoQ(s_fdcan, &header, data);
    if (status == HAL_OK)
    {
        s_diagnostics.tx_success_count++;
    }
    else
    {
        s_diagnostics.tx_failure_count++;
    }
    return status;
}

HAL_StatusTypeDef ZdtX42s_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;
    const uint32_t notifications = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                   FDCAN_IT_BUS_OFF |
                                   FDCAN_IT_ERROR_WARNING |
                                   FDCAN_IT_ERROR_PASSIVE;

    if (hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    memset((void *)&s_diagnostics, 0, sizeof(s_diagnostics));
    s_fdcan = NULL;

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    status = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ConfigGlobalFilter(
            hfdcan, FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_Start(hfdcan);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ActivateNotification(hfdcan, notifications, 0U);
    }
    if (status == HAL_OK)
    {
        s_fdcan = hfdcan;
    }
    return status;
}

HAL_StatusTypeDef ZdtX42s_SetSpeedX10(uint8_t motor_id,
                                     int16_t rpm_x10,
                                     uint8_t acceleration,
                                     bool synchronized)
{
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH] = {0};

    if (motor_id == 0U ||
        ZdtX42s_BuildSpeedPayload(rpm_x10, acceleration,
                                  synchronized, payload) == 0U)
    {
        return HAL_ERROR;
    }

    return ZdtX42s_Send(ZdtX42s_CommandId(motor_id, 0U),
                        FDCAN_DLC_BYTES_7, payload);
}

HAL_StatusTypeDef ZdtX42s_Sync(void)
{
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH] = {0};

    if (ZdtX42s_BuildSyncPayload(payload) == 0U)
    {
        return HAL_ERROR;
    }
    return ZdtX42s_Send(0U, FDCAN_DLC_BYTES_3, payload);
}

bool ZdtX42s_HasTxCapacity(uint32_t frame_count)
{
    return s_fdcan != NULL &&
           HAL_FDCAN_GetTxFifoFreeLevel(s_fdcan) >= frame_count;
}

const volatile ZdtX42sDiagnostics *ZdtX42s_GetDiagnostics(void)
{
    return &s_diagnostics;
}

/* 解码一个电机回复并更新接收诊断信息。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t data[ZDT_X42S_CAN_MAX_DATA_LENGTH] = {0};
    uint8_t motor_id;

    if (hfdcan != s_fdcan ||
        (rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                               &header, data) != HAL_OK)
    {
        s_diagnostics.bus_error_count++;
        return;
    }

    s_diagnostics.rx_frame_count++;
    s_diagnostics.last_rx_id = header.Identifier;
    s_diagnostics.last_function = data[0];
    s_diagnostics.last_status = data[1];

    motor_id = (uint8_t)(header.Identifier >> 8U);
    if (header.IdType == FDCAN_EXTENDED_ID &&
        motor_id >= 1U && motor_id <= ZDT_X42S_MOTOR_COUNT)
    {
        s_diagnostics.motor_reply_count[motor_id - 1U]++;
    }
}

/* 记录控制器级 FDCAN 故障用于现场诊断。 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == s_fdcan)
    {
        s_diagnostics.bus_error_count++;
        s_diagnostics.last_error_code = hfdcan->ErrorCode;
    }
}

/* 记录警告、被动和总线关闭状态通知。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t error_status_its)
{
    if (hfdcan == s_fdcan)
    {
        s_diagnostics.bus_error_count++;
        s_diagnostics.last_error_code = error_status_its;
    }
}
