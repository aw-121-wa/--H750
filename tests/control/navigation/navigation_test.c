#include <assert.h>
#include <stdio.h>

#include "control/navigation/navigation.h"

static NavigationPose fake_pose;
static float last_vy;
static float last_vw;
static unsigned stop_count;
static unsigned speed_command_count;

NavigationPose Localization_GetPose(void)
{
    return fake_pose;
}

HAL_StatusTypeDef Chassis_SetSpeed(float vy, float vx, float vw, bool fine)
{
    (void)vx;
    (void)fine;
    last_vy = vy;
    last_vw = vw;
    speed_command_count++;
    return HAL_OK;
}

HAL_StatusTypeDef Chassis_Stop(void)
{
    last_vy = 0.0f;
    last_vw = 0.0f;
    stop_count++;
    return HAL_OK;
}

float Heading_TurnOutputForPose(float target_angle, float current_angle)
{
    return target_angle - current_angle;
}

static void test_aligns_moves_and_arrives(void)
{
    NavigationRoute route = {.point_count = 1U, .points = {{1000, 0}}};

    Navigation_Init();
    fake_pose = (NavigationPose){0};
    assert(Navigation_LoadRoute(&route) == NAVIGATION_OK);
    assert(Navigation_Start() == NAVIGATION_OK);
    Navigation_SetHealth(true, true, true, true);

    Navigation_Tick(10U);
    assert(Navigation_GetState() == NAVIGATION_ALIGNING);
    for (unsigned i = 0; i < 6U; ++i)
    {
        Navigation_Tick(20U + i * 5U);
    }
    assert(Navigation_GetState() == NAVIGATION_MOVING);
    Navigation_Tick(60U);
    assert(last_vy > 0.0f);
    {
        unsigned count = speed_command_count;
        Navigation_Tick(61U);
        assert(speed_command_count == count);
        Navigation_Tick(80U);
        assert(speed_command_count == count + 1U);
    }

    fake_pose.x_mm = 100.0f;
    Navigation_Tick(70U);
    assert(Navigation_GetState() == NAVIGATION_ARRIVED);
    assert(stop_count > 0U);
}

static void test_heartbeat_failure_stops_navigation(void)
{
    NavigationRoute route = {.point_count = 1U, .points = {{1000, 0}}};

    Navigation_Init();
    fake_pose = (NavigationPose){0};
    assert(Navigation_LoadRoute(&route) == NAVIGATION_OK);
    assert(Navigation_Start() == NAVIGATION_OK);
    Navigation_SetHealth(true, true, true, false);
    Navigation_Tick(10U);
    assert(Navigation_GetState() == NAVIGATION_E_STOP);
    assert((Navigation_GetFaultFlags() & NAVIGATION_FAULT_HEARTBEAT) != 0U);
}

static void test_health_recovery_does_not_resume_faulted_navigation(void)
{
    NavigationRoute route = {.point_count = 1U, .points = {{1000, 0}}};

    Navigation_Init();
    fake_pose = (NavigationPose){0};
    assert(Navigation_LoadRoute(&route) == NAVIGATION_OK);
    assert(Navigation_Start() == NAVIGATION_OK);
    Navigation_SetHealth(false, true, false, true);
    Navigation_Tick(10U);
    assert(Navigation_GetState() == NAVIGATION_FAULT);

    Navigation_SetHealth(true, true, true, true);
    Navigation_Tick(20U);
    assert(Navigation_GetState() == NAVIGATION_FAULT);
}

int main(void)
{
    test_aligns_moves_and_arrives();
    test_heartbeat_failure_stops_navigation();
    test_health_recovery_does_not_resume_faulted_navigation();
    puts("Navigation tests passed");
    return 0;
}
