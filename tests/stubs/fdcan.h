#ifndef TEST_FDCAN_H
#define TEST_FDCAN_H

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
    void *Instance;
    uint32_t ErrorCode;
} FDCAN_HandleTypeDef;

extern FDCAN_HandleTypeDef hfdcan1;

typedef struct
{
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
} FDCAN_RxHeaderTypeDef;

#define FDCAN_EXTENDED_ID                 1U
#define FDCAN_DATA_FRAME                  2U
#define FDCAN_DLC_BYTES_3                 3U
#define FDCAN_DLC_BYTES_7                 7U
#define FDCAN_ESI_ACTIVE                  4U
#define FDCAN_BRS_OFF                     5U
#define FDCAN_CLASSIC_CAN                 6U
#define FDCAN_NO_TX_EVENTS                7U
#define FDCAN_FILTER_MASK                 8U
#define FDCAN_FILTER_TO_RXFIFO0           9U
#define FDCAN_REJECT                      10U
#define FDCAN_REJECT_REMOTE               11U
#define FDCAN_RX_FIFO0                    12U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE     (1UL << 0)
#define FDCAN_IT_BUS_OFF                  (1UL << 1)
#define FDCAN_IT_ERROR_WARNING            (1UL << 2)
#define FDCAN_IT_ERROR_PASSIVE            (1UL << 3)

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                                         FDCAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                               uint32_t non_matching_std,
                                               uint32_t non_matching_ext,
                                               uint32_t reject_remote_std,
                                               uint32_t reject_remote_ext);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                                 uint32_t notifications,
                                                 uint32_t buffer_indexes);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                FDCAN_TxHeaderTypeDef *header,
                                                uint8_t *data);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t fifo,
                                         FDCAN_RxHeaderTypeDef *header,
                                         uint8_t *data);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_its);
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t error_status_its);

#endif
