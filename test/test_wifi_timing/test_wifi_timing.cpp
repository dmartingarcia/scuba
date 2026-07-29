#include <unity.h>
#include "../../src/logic/wifi_timing.h"

void setUp(void) {}
void tearDown(void) {}

void test_isReconnectDue_false_before_time(void) {
    TEST_ASSERT_FALSE(isReconnectDue(1000, 2000));
}

void test_isReconnectDue_true_at_time(void) {
    TEST_ASSERT_TRUE(isReconnectDue(2000, 2000));
}

void test_isReconnectDue_true_after_time(void) {
    TEST_ASSERT_TRUE(isReconnectDue(5000, 2000));
}

void test_nextReconnectAttempt_adds_interval(void) {
    TEST_ASSERT_EQUAL_UINT32(61000, nextReconnectAttempt(1000, 60000));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_isReconnectDue_false_before_time);
    RUN_TEST(test_isReconnectDue_true_at_time);
    RUN_TEST(test_isReconnectDue_true_after_time);
    RUN_TEST(test_nextReconnectAttempt_adds_interval);
    return UNITY_END();
}
