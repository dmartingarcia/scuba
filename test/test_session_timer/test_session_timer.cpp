#include <unity.h>
#include "../../src/logic/session_timer.h"

void setUp(void) {}
void tearDown(void) {}

void test_unlimited_when_duration_zero(void) {
    TEST_ASSERT_FALSE(isSessionTimeUp(1000000, 0, 0));
}

void test_not_up_before_duration_elapsed(void) {
    TEST_ASSERT_FALSE(isSessionTimeUp(5000, 1000, 60000));
}

void test_up_exactly_at_duration(void) {
    TEST_ASSERT_TRUE(isSessionTimeUp(61000, 1000, 60000));
}

void test_up_after_duration(void) {
    TEST_ASSERT_TRUE(isSessionTimeUp(120000, 1000, 60000));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_unlimited_when_duration_zero);
    RUN_TEST(test_not_up_before_duration_elapsed);
    RUN_TEST(test_up_exactly_at_duration);
    RUN_TEST(test_up_after_duration);
    return UNITY_END();
}
