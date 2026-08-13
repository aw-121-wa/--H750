#include "control/navigation/navigation.h"

#include <math.h>
#include <string.h>

#include "control/chassis/chassis.h"
#include "control/heading/heading_control.h"

#define NAVIGATION_PI 3.14159265358979323846f
#define NAVIGATION_ANGLE_TOLERANCE_DEG 0.6f
#define NAVIGATION_POSITION_TOLERANCE_MM 10.0f
#define NAVIGATION_ALIGN_SETTLE_COUNT 5U
#define NAVIGATION_CRUISE_RPM 20.0f
#define NAVIGATION_MIN_RPM 4.0f
#define NAVIGATION_DECEL_DISTANCE_MM 200.0f
#define NAVIGATION_COMMAND_PERIOD_MS 20U
#define NAVIGATION_MAX_COMMAND_FAILURES 3U

static NavigationRoute s_route;
static NavigationState s_state;
static NavigationState s_resume_state;
static uint16_t s_waypoint;
static uint16_t s_fault_flags;
static uint8_t s_align_settle_count;
static bool s_motor_ok;
static bool s_imu_ok;
static bool s_can_ok;
static bool s_heartbeat_ok;
static uint32_t s_last_command_ms;
static uint8_t s_command_failure_count;
static bool s_stop_pending;

static float NormalizeAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void StopWithState(NavigationState state)
{
    s_stop_pending = Chassis_Stop() != HAL_OK;
    s_state = state;
}

void Navigation_Init(void)
{
    memset(&s_route, 0, sizeof(s_route));
    s_state = NAVIGATION_IDLE;
    s_resume_state = NAVIGATION_IDLE;
    s_waypoint = 0U;
    s_fault_flags = 0U;
    s_align_settle_count = 0U;
    s_motor_ok = true;
    s_imu_ok = true;
    s_can_ok = true;
    s_heartbeat_ok = true;
    s_last_command_ms = 0U;
    s_command_failure_count = 0U;
    s_stop_pending = false;
}

NavigationStatus Navigation_LoadRoute(const NavigationRoute *route)
{
    if (route == NULL || route->point_count == 0U ||
        route->point_count > NAVIGATION_MAX_WAYPOINTS)
    {
        return NAVIGATION_INVALID_ARGUMENT;
    }
    if (s_state == NAVIGATION_ALIGNING || s_state == NAVIGATION_MOVING)
    {
        return NAVIGATION_INVALID_STATE;
    }
    s_route = *route;
    s_waypoint = 0U;
    s_fault_flags = 0U;
    s_state = NAVIGATION_ROUTE_READY;
    return NAVIGATION_OK;
}

NavigationStatus Navigation_Start(void)
{
    if (s_state != NAVIGATION_ROUTE_READY)
    {
        return NAVIGATION_INVALID_STATE;
    }
    s_align_settle_count = 0U;
    s_state = NAVIGATION_ALIGNING;
    return NAVIGATION_OK;
}

void Navigation_Pause(void)
{
    if (s_state == NAVIGATION_ALIGNING || s_state == NAVIGATION_MOVING)
    {
        s_resume_state = s_state;
        StopWithState(NAVIGATION_PAUSED);
    }
}

void Navigation_Resume(void)
{
    if (s_state == NAVIGATION_PAUSED && s_fault_flags == 0U)
    {
        s_state = s_resume_state;
    }
}

void Navigation_Stop(void)
{
    StopWithState(NAVIGATION_IDLE);
    memset(&s_route, 0, sizeof(s_route));
    s_waypoint = 0U;
    s_fault_flags = 0U;
}

void Navigation_SetHealth(bool motor_ok,
                          bool imu_ok,
                          bool can_ok,
                          bool heartbeat_ok)
{
    s_motor_ok = motor_ok;
    s_imu_ok = imu_ok;
    s_can_ok = can_ok;
    s_heartbeat_ok = heartbeat_ok;
}

static bool CheckHealth(void)
{
    uint16_t flags = 0U;

    if (!s_motor_ok) flags |= NAVIGATION_FAULT_MOTOR;
    if (!s_imu_ok) flags |= NAVIGATION_FAULT_IMU;
    if (!s_can_ok) flags |= NAVIGATION_FAULT_CAN;
    if (!s_heartbeat_ok) flags |= NAVIGATION_FAULT_HEARTBEAT;
    if (flags == 0U)
    {
        return true;
    }
    s_fault_flags |= flags;
    StopWithState((flags & NAVIGATION_FAULT_HEARTBEAT) != 0U ?
                  NAVIGATION_E_STOP : NAVIGATION_FAULT);
    return false;
}

void Navigation_Tick(uint32_t now_ms)
{
    NavigationPose pose;
    float target_x;
    float target_y;
    float dx;
    float dy;
    float distance;
    float target_yaw;
    float yaw_error;
    float speed;

    if (s_stop_pending &&
        (now_ms - s_last_command_ms) >= NAVIGATION_COMMAND_PERIOD_MS)
    {
        s_last_command_ms = now_ms;
        s_stop_pending = Chassis_Stop() != HAL_OK;
    }
    if (s_state != NAVIGATION_ALIGNING && s_state != NAVIGATION_MOVING)
    {
        return;
    }
    if (!CheckHealth())
    {
        return;
    }
    pose = Localization_GetPose();
    target_x = (float)s_route.points[s_waypoint].x_x10_mm * 0.1f;
    target_y = (float)s_route.points[s_waypoint].y_x10_mm * 0.1f;
    dx = target_x - pose.x_mm;
    dy = target_y - pose.y_mm;
    distance = sqrtf(dx * dx + dy * dy);

    if (distance <= NAVIGATION_POSITION_TOLERANCE_MM)
    {
        if (++s_waypoint >= s_route.point_count)
        {
            s_waypoint = s_route.point_count;
            StopWithState(NAVIGATION_ARRIVED);
        }
        else
        {
            s_align_settle_count = 0U;
            StopWithState(NAVIGATION_ALIGNING);
        }
        return;
    }

    target_yaw = atan2f(dy, dx) * 180.0f / NAVIGATION_PI;
    yaw_error = NormalizeAngle(target_yaw - pose.yaw_deg);
    if (s_state == NAVIGATION_ALIGNING)
    {
        if (fabsf(yaw_error) <= NAVIGATION_ANGLE_TOLERANCE_DEG)
        {
            if (s_align_settle_count == 0U)
            {
                s_stop_pending = Chassis_Stop() != HAL_OK;
            }
            if (++s_align_settle_count >= NAVIGATION_ALIGN_SETTLE_COUNT)
            {
                s_state = NAVIGATION_MOVING;
            }
        }
        else
        {
            s_align_settle_count = 0U;
            if ((now_ms - s_last_command_ms) >= NAVIGATION_COMMAND_PERIOD_MS)
            {
                s_last_command_ms = now_ms;
                if (Chassis_SetSpeed(0.0f, 0.0f,
                                     Heading_TurnOutputForPose(target_yaw,
                                                               pose.yaw_deg),
                                     true) == HAL_OK)
                {
                    s_command_failure_count = 0U;
                }
                else if (++s_command_failure_count >=
                         NAVIGATION_MAX_COMMAND_FAILURES)
                {
                    s_fault_flags |= NAVIGATION_FAULT_COMMAND;
                    StopWithState(NAVIGATION_FAULT);
                }
            }
        }
        return;
    }

    if (fabsf(yaw_error) > 5.0f)
    {
        s_align_settle_count = 0U;
        StopWithState(NAVIGATION_ALIGNING);
        return;
    }
    speed = distance < NAVIGATION_DECEL_DISTANCE_MM ?
            NAVIGATION_CRUISE_RPM * distance / NAVIGATION_DECEL_DISTANCE_MM :
            NAVIGATION_CRUISE_RPM;
    if (speed < NAVIGATION_MIN_RPM)
    {
        speed = NAVIGATION_MIN_RPM;
    }
    if ((now_ms - s_last_command_ms) < NAVIGATION_COMMAND_PERIOD_MS)
    {
        return;
    }
    s_last_command_ms = now_ms;
    if (Chassis_SetSpeed(speed, 0.0f,
                         Heading_TurnOutputForPose(target_yaw, pose.yaw_deg),
                         true) == HAL_OK)
    {
        s_command_failure_count = 0U;
    }
    else if (++s_command_failure_count >= NAVIGATION_MAX_COMMAND_FAILURES)
    {
        s_fault_flags |= NAVIGATION_FAULT_COMMAND;
        StopWithState(NAVIGATION_FAULT);
    }
}

NavigationState Navigation_GetState(void)
{
    return s_state;
}

uint16_t Navigation_GetCurrentWaypoint(void)
{
    return s_waypoint;
}

uint16_t Navigation_GetFaultFlags(void)
{
    return s_fault_flags;
}
