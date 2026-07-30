#include <unity.h>
#include "../../src/logic/attitude.h"

void setUp(void) {}
void tearDown(void) {}

void test_level_gives_zero_pitch_and_roll(void) {
    Attitude a = computeAttitude(0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, a.pitchDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, a.rollDeg);
}

void test_pitched_forward_90_degrees(void) {
    // Gravity fully on X, robot pitched 90deg (nose down/up)
    Attitude a = computeAttitude(1.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, a.pitchDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, a.rollDeg);
}

void test_rolled_90_degrees(void) {
    // Gravity fully on Y, robot rolled onto its side
    Attitude a = computeAttitude(0.0f, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, a.pitchDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, a.rollDeg);
}

void test_partial_tilt_both_axes(void) {
    Attitude a = computeAttitude(0.5f, 0.5f, 0.707f);
    TEST_ASSERT_TRUE(a.pitchDeg > 20.0f && a.pitchDeg < 50.0f);
    TEST_ASSERT_TRUE(a.rollDeg > 20.0f && a.rollDeg < 50.0f);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_level_gives_zero_pitch_and_roll);
    RUN_TEST(test_pitched_forward_90_degrees);
    RUN_TEST(test_rolled_90_degrees);
    RUN_TEST(test_partial_tilt_both_axes);
    return UNITY_END();
}
