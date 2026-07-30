#include <unity.h>
#include "../../src/logic/ha_state.h"

void setUp(void) {}
void tearDown(void) {}

void test_active_is_cleaning(void) {
    TEST_ASSERT_TRUE(HaState::Cleaning == deriveHaState(true, false));
    TEST_ASSERT_TRUE(HaState::Cleaning == deriveHaState(true, true));
}

void test_inactive_without_timer_is_paused(void) {
    TEST_ASSERT_TRUE(HaState::Paused == deriveHaState(false, false));
}

void test_inactive_with_timer_is_finished(void) {
    TEST_ASSERT_TRUE(HaState::Finished == deriveHaState(false, true));
}

void test_names(void) {
    TEST_ASSERT_EQUAL_STRING("cleaning", haStateName(HaState::Cleaning));
    TEST_ASSERT_EQUAL_STRING("paused", haStateName(HaState::Paused));
    TEST_ASSERT_EQUAL_STRING("finished", haStateName(HaState::Finished));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_active_is_cleaning);
    RUN_TEST(test_inactive_without_timer_is_paused);
    RUN_TEST(test_inactive_with_timer_is_finished);
    RUN_TEST(test_names);
    return UNITY_END();
}
