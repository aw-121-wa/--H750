#include "app/diagnostics/chassis_smoke_test.h"

#include "cmsis_os2.h"
#include "control/chassis/chassis.h"

#define CHASSIS_TEST_SPEED_RPM 20.0f
#define CHASSIS_TEST_RUN_MS    1000U
#define CHASSIS_TEST_STOP_MS   500U

static void Chassis_SmokeTestStep(float vy, float vx, float vw)
{
    Motor_setspeed(vy, vx, vw);
    (void)osDelay(CHASSIS_TEST_RUN_MS);
    Motor_Stop();
    (void)osDelay(CHASSIS_TEST_STOP_MS);
}

void Chassis_SmokeTest(void)
{
    Motor_Stop();
    (void)osDelay(CHASSIS_TEST_STOP_MS);

    Chassis_SmokeTestStep(CHASSIS_TEST_SPEED_RPM, 0.0f, 0.0f);
    Chassis_SmokeTestStep(-CHASSIS_TEST_SPEED_RPM, 0.0f, 0.0f);
    Chassis_SmokeTestStep(0.0f, CHASSIS_TEST_SPEED_RPM, 0.0f);
    Chassis_SmokeTestStep(0.0f, -CHASSIS_TEST_SPEED_RPM, 0.0f);
    Chassis_SmokeTestStep(0.0f, 0.0f, CHASSIS_TEST_SPEED_RPM);
}
