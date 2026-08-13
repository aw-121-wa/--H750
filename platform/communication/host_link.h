#ifndef PLATFORM_COMMUNICATION_HOST_LINK_H
#define PLATFORM_COMMUNICATION_HOST_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "control/localization/localization.h"
#include "control/navigation/navigation.h"
#include "main.h"
#include "platform/communication/host_dispatcher.h"

/** Starts USART1 circular RX DMA and creates the command queue. */
HAL_StatusTypeDef HostLink_Init(void);

/** Parses received bytes, advances TX DMA and emits 10 Hz telemetry. */
void HostLink_Process(void);

/** Publishes the latest pose snapshot for the host task. */
void HostLink_SendPose(const NavigationPose *pose);

/** Updates health fields included in pose and status reports. */
void HostLink_SetStatus(uint8_t motor_valid_mask,
                        bool imu_valid,
                        uint16_t fault_flags,
                        NavigationState navigation_state,
                        uint16_t current_waypoint);

/** Removes one validated host command for execution by MAIN_TASK. */
bool HostLink_TakeCommand(HostCommand *command);

/** Sends the final ACK or NACK after MAIN_TASK executes a command. */
void HostLink_CompleteCommand(const HostCommand *command,
                              bool success,
                              uint8_t result_code);

/** Copies the atomically committed route exactly once. */
bool HostLink_TakeCommittedRoute(NavigationRoute *route);

/** Returns time since the most recent valid host frame. */
uint32_t HostLink_GetHeartbeatAgeMs(uint32_t now_ms);

void HostLink_OnUartError(UART_HandleTypeDef *huart);
void HostLink_OnUartTxComplete(UART_HandleTypeDef *huart);

#endif
