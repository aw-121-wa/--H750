#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control/localization/localization.h"

static void assert_near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static void test_reports_relative_motion_before_anchor(void)
{
    const int32_t zero[4] = {0, 0, 0, 0};
    const int32_t forward[4] = {1000, 1000, -1000, -1000};
    NavigationPose pose;

    Localization_Init();
    Localization_Update(zero, 0.0f, 0U);
    Localization_Update(forward, 0.0f, 100U);
    pose = Localization_GetPose();

    assert(Localization_GetStatus().pose_state == LOCALIZATION_UNANCHORED);
    assert_near(pose.x_mm, 67.23f, 0.1f);
    assert_near(pose.y_mm, 0.0f, 0.1f);
    assert_near(pose.yaw_deg, 0.0f, 0.01f);
}

static void test_initial_yaw_is_relative_zero(void)
{
    const int32_t zero[4] = {0, 0, 0, 0};

    Localization_Init();
    Localization_Update(zero, 42.0f, 100U);
    assert_near(Localization_GetPose().yaw_deg, 0.0f, 0.01f);
    Localization_Update(zero, 43.0f, 200U);
    assert_near(Localization_GetPose().yaw_deg, 1.0f, 0.01f);
}

static void test_set_pose_keeps_future_odometry_continuous(void)
{
    const int32_t zero[4] = {0, 0, 0, 0};
    const int32_t first[4] = {100, 100, -100, -100};
    const int32_t second[4] = {200, 200, -200, -200};
    const NavigationPose anchor = {1000.0f, 500.0f, 90.0f, 0.0f, 0.0f, 0.0f};
    NavigationPose pose;

    Localization_Init();
    Localization_Update(zero, 0.0f, 0U);
    Localization_Update(first, 0.0f, 100U);
    assert(Localization_SetMapPose(&anchor));
    Localization_Update(second, 0.0f, 200U);
    pose = Localization_GetPose();

    assert(Localization_GetStatus().pose_state == LOCALIZATION_ANCHORED);
    assert_near(pose.x_mm, 1000.0f, 0.1f);
    assert_near(pose.y_mm, 506.723f, 0.1f);
    assert_near(pose.yaw_deg, 90.0f, 0.01f);
}

int main(void)
{
    test_reports_relative_motion_before_anchor();
    test_initial_yaw_is_relative_zero();
    test_set_pose_keeps_future_odometry_continuous();
    puts("Localization tests passed");
    return 0;
}
