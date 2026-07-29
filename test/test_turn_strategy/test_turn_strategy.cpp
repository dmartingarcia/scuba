#include <unity.h>
#include "../../src/logic/turn_strategy.h"

void setUp(void) {}
void tearDown(void) {}

void test_parse_legacy(void) {
    TEST_ASSERT_TRUE(TurnStrategy::Legacy == parseTurnStrategy("legacy", TurnStrategy::KalmanFusion));
}

void test_parse_duration(void) {
    TEST_ASSERT_TRUE(TurnStrategy::FixedDuration == parseTurnStrategy("duration", TurnStrategy::Legacy));
}

void test_parse_kalman(void) {
    TEST_ASSERT_TRUE(TurnStrategy::KalmanFusion == parseTurnStrategy("kalman", TurnStrategy::Legacy));
}

void test_parse_unknown_returns_fallback(void) {
    TEST_ASSERT_TRUE(TurnStrategy::FixedDuration == parseTurnStrategy("bogus", TurnStrategy::FixedDuration));
}

void test_name_round_trips(void) {
    TEST_ASSERT_TRUE(TurnStrategy::Legacy == parseTurnStrategy(turnStrategyName(TurnStrategy::Legacy), TurnStrategy::KalmanFusion));
    TEST_ASSERT_TRUE(TurnStrategy::FixedDuration == parseTurnStrategy(turnStrategyName(TurnStrategy::FixedDuration), TurnStrategy::Legacy));
    TEST_ASSERT_TRUE(TurnStrategy::KalmanFusion == parseTurnStrategy(turnStrategyName(TurnStrategy::KalmanFusion), TurnStrategy::Legacy));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_parse_legacy);
    RUN_TEST(test_parse_duration);
    RUN_TEST(test_parse_kalman);
    RUN_TEST(test_parse_unknown_returns_fallback);
    RUN_TEST(test_name_round_trips);
    return UNITY_END();
}
