#include "MotorTestRunner.h"

MOTOR_TEST(host_test_runner_starts)
{
    MOTOR_ASSERT_TRUE(true);
}

int main()
{
    return MotorHostTest::runAll();
}
