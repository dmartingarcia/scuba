#include <unity.h>
#include "../../src/logic/motor_direction_guard.h"

void setUp(void) {}
void tearDown(void) {}

void test_first_request_in_a_direction_is_applied_immediately(void) {
    MotorDirectionGuard guard;
    TEST_ASSERT_EQUAL_INT(100, guard.apply(100, 0));
}

void test_same_direction_requests_are_never_blocked(void) {
    MotorDirectionGuard guard;
    guard.apply(100, 0);
    TEST_ASSERT_EQUAL_INT(150, guard.apply(150, 10));
    TEST_ASSERT_EQUAL_INT(50, guard.apply(50, 20));
}

void test_stop_is_always_applied_immediately(void) {
    MotorDirectionGuard guard;
    guard.apply(100, 0);
    TEST_ASSERT_EQUAL_INT(0, guard.apply(0, 10));
}

void test_reversal_is_held_at_zero_until_deadtime_elapses(void) {
    MotorDirectionGuard guard;
    guard.apply(100, 0); // driving forward

    // Reversal requested right away - must be forced to neutral, not applied.
    TEST_ASSERT_EQUAL_INT(0, guard.apply(-100, 10));
    // Still within the dead time window.
    TEST_ASSERT_EQUAL_INT(0, guard.apply(-100, 100));
    TEST_ASSERT_EQUAL_INT(0, guard.apply(-100, 509));
}

void test_reversal_is_allowed_once_deadtime_elapses(void) {
    MotorDirectionGuard guard;
    guard.apply(100, 0);
    guard.apply(-100, 10); // starts the dead-time window at t=10

    TEST_ASSERT_EQUAL_INT(-100, guard.apply(-100, 510));
}

void test_stopping_first_does_not_bypass_the_deadtime(void) {
    MotorDirectionGuard guard;
    guard.apply(100, 0);   // driving forward
    guard.apply(0, 5);     // explicit stop
    // Reversal still shouldn't be allowed immediately just because the
    // caller went through an explicit stop first.
    TEST_ASSERT_EQUAL_INT(0, guard.apply(-100, 6));
    TEST_ASSERT_EQUAL_INT(-100, guard.apply(-100, 506));
}

void test_reconsidering_the_same_direction_during_deadtime_resumes_instantly(void) {
    MotorDirectionGuard guard;
    guard.apply(100, 0);
    guard.apply(-100, 10); // reversal attempt starts dead time, held at 0

    // Caller changes its mind back to the original direction before the
    // dead time elapses - since we never actually reversed, this is safe.
    TEST_ASSERT_EQUAL_INT(80, guard.apply(80, 50));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_first_request_in_a_direction_is_applied_immediately);
    RUN_TEST(test_same_direction_requests_are_never_blocked);
    RUN_TEST(test_stop_is_always_applied_immediately);
    RUN_TEST(test_reversal_is_held_at_zero_until_deadtime_elapses);
    RUN_TEST(test_reversal_is_allowed_once_deadtime_elapses);
    RUN_TEST(test_stopping_first_does_not_bypass_the_deadtime);
    RUN_TEST(test_reconsidering_the_same_direction_during_deadtime_resumes_instantly);
    return UNITY_END();
}
