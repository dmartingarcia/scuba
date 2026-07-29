#include <unity.h>
#include "../../src/logic/kalman_filter.h"

void setUp(void) {}
void tearDown(void) {}

void test_converges_to_constant_measurement(void) {
    KalmanState state = kalmanInit(0.0f);
    for (int i = 0; i < 20; i++) {
        state = kalmanUpdate(state, 10.0f, 0.01f, 1.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.0f, state.rate);
}

void test_covariance_shrinks_over_iterations(void) {
    KalmanState state = kalmanInit(0.0f);
    float initialP = state.p;
    for (int i = 0; i < 10; i++) {
        state = kalmanUpdate(state, 5.0f, 0.01f, 1.0f);
    }
    TEST_ASSERT_TRUE(state.p < initialP);
}

void test_attenuates_single_outlier(void) {
    // Settle near 0 first, then hit it with one big outlier measurement.
    KalmanState state = kalmanInit(0.0f);
    for (int i = 0; i < 10; i++) {
        state = kalmanUpdate(state, 0.0f, 0.01f, 1.0f);
    }
    KalmanState afterOutlier = kalmanUpdate(state, 100.0f, 0.01f, 1.0f);
    // The filtered estimate should move toward the outlier, but nowhere near all the way.
    TEST_ASSERT_TRUE(afterOutlier.rate < 50.0f);
    TEST_ASSERT_TRUE(afterOutlier.rate > state.rate);
}

void test_high_measurement_noise_trusts_prior_more(void) {
    KalmanState state = kalmanInit(0.0f);
    KalmanState lowNoise = kalmanUpdate(state, 10.0f, 0.01f, 0.1f);
    KalmanState highNoise = kalmanUpdate(state, 10.0f, 0.01f, 100.0f);
    // With high measurement noise, the filter should move less toward the new reading.
    TEST_ASSERT_TRUE(highNoise.rate < lowNoise.rate);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_converges_to_constant_measurement);
    RUN_TEST(test_covariance_shrinks_over_iterations);
    RUN_TEST(test_attenuates_single_outlier);
    RUN_TEST(test_high_measurement_noise_trusts_prior_more);
    return UNITY_END();
}
