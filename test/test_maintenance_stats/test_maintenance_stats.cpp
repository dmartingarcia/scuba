#include <unity.h>
#include "../../src/logic/maintenance_stats.h"

void setUp(void) {}
void tearDown(void) {}

void test_recordBoot_increments_count(void) {
    MaintenanceStats stats = {0, 0};
    stats = recordBoot(stats);
    TEST_ASSERT_EQUAL_UINT32(1, stats.bootCount);
    stats = recordBoot(stats);
    TEST_ASSERT_EQUAL_UINT32(2, stats.bootCount);
}

void test_accumulateActiveSeconds_adds_when_active(void) {
    MaintenanceStats stats = {0, 100};
    stats = accumulateActiveSeconds(stats, 30, true);
    TEST_ASSERT_EQUAL_UINT32(130, stats.totalRuntimeSeconds);
}

void test_accumulateActiveSeconds_skips_when_idle(void) {
    MaintenanceStats stats = {0, 100};
    stats = accumulateActiveSeconds(stats, 30, false);
    TEST_ASSERT_EQUAL_UINT32(100, stats.totalRuntimeSeconds);
}

void test_shouldCommitStats_true_on_session_end_regardless_of_interval(void) {
    TEST_ASSERT_TRUE(shouldCommitStats(1000, 999999, 0, true));
}

void test_shouldCommitStats_false_when_interval_disabled_and_not_ended(void) {
    TEST_ASSERT_FALSE(shouldCommitStats(1000, 500, 0, false));
}

void test_shouldCommitStats_false_before_periodic_interval(void) {
    TEST_ASSERT_FALSE(shouldCommitStats(1000, 2000, 60000, false));
}

void test_shouldCommitStats_true_at_periodic_interval(void) {
    TEST_ASSERT_TRUE(shouldCommitStats(2000, 2000, 60000, false));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_recordBoot_increments_count);
    RUN_TEST(test_accumulateActiveSeconds_adds_when_active);
    RUN_TEST(test_accumulateActiveSeconds_skips_when_idle);
    RUN_TEST(test_shouldCommitStats_true_on_session_end_regardless_of_interval);
    RUN_TEST(test_shouldCommitStats_false_when_interval_disabled_and_not_ended);
    RUN_TEST(test_shouldCommitStats_false_before_periodic_interval);
    RUN_TEST(test_shouldCommitStats_true_at_periodic_interval);
    return UNITY_END();
}
