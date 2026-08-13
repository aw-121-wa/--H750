#ifndef CONTROL_NAVIGATION_NAVIGATION_H
#define CONTROL_NAVIGATION_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

#include "control/localization/localization.h"
#include "main.h"

#define NAVIGATION_MAX_WAYPOINTS 128U

typedef struct
{
    int32_t x_x10_mm;
    int32_t y_x10_mm;
} NavigationWaypoint;

typedef struct
{
    uint16_t point_count;
    NavigationWaypoint points[NAVIGATION_MAX_WAYPOINTS];
} NavigationRoute;

typedef enum
{
    NAVIGATION_IDLE = 0,
    NAVIGATION_ROUTE_READY,
    NAVIGATION_ALIGNING,
    NAVIGATION_MOVING,
    NAVIGATION_ARRIVED,
    NAVIGATION_PAUSED,
    NAVIGATION_FAULT,
    NAVIGATION_E_STOP
} NavigationState;

typedef enum
{
    NAVIGATION_OK = 0,
    NAVIGATION_INVALID_ARGUMENT,
    NAVIGATION_INVALID_STATE
} NavigationStatus;

enum
{
    NAVIGATION_FAULT_MOTOR = 1U << 0,
    NAVIGATION_FAULT_IMU = 1U << 1,
    NAVIGATION_FAULT_CAN = 1U << 2,
    NAVIGATION_FAULT_HEARTBEAT = 1U << 3,
    NAVIGATION_FAULT_COMMAND = 1U << 4
};

/** Initializes an empty, stopped navigation state machine. */
void Navigation_Init(void);

/** Atomically loads a validated route while navigation is stopped. */
NavigationStatus Navigation_LoadRoute(const NavigationRoute *route);

/** Starts following the committed route. */
NavigationStatus Navigation_Start(void);

/** Pauses motion and preserves the current waypoint. */
void Navigation_Pause(void);

/** Resumes a manually paused route. */
void Navigation_Resume(void);

/** Stops motion and clears the active route. */
void Navigation_Stop(void);

/** Supplies current motor, IMU, CAN and host heartbeat health. */
void Navigation_SetHealth(bool motor_ok,
                          bool imu_ok,
                          bool can_ok,
                          bool heartbeat_ok);

/** Advances alignment, movement, arrival and safety handling. */
void Navigation_Tick(uint32_t now_ms);

NavigationState Navigation_GetState(void);
uint16_t Navigation_GetCurrentWaypoint(void);
uint16_t Navigation_GetFaultFlags(void);

#endif
