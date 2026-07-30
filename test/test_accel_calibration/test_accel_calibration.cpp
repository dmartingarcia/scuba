#include <unity.h>
#include "../../src/logic/accel_calibration.h"

void setUp(void) {}
void tearDown(void) {}

void test_default_is_uncalibrated_and_passthrough(void) {
    AccelCalibration cal = defaultAccelCalibration();
    TEST_ASSERT_FALSE(cal.calibrated);
    TEST_ASSERT_EQUAL_FLOAT(0.37f, applyAccelCalibration(0.37f, cal));
}

void test_calibrate_stores_offset_and_marks_calibrated(void) {
    AccelCalibration cal = calibrateAccelZero(0.15f);
    TEST_ASSERT_TRUE(cal.calibrated);
    TEST_ASSERT_EQUAL_FLOAT(0.15f, cal.zOffset);
}

void test_apply_subtracts_offset_once_calibrated(void) {
    AccelCalibration cal = calibrateAccelZero(0.15f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, applyAccelCalibration(0.15f, cal));
    TEST_ASSERT_EQUAL_FLOAT(0.85f, applyAccelCalibration(1.0f, cal));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_default_is_uncalibrated_and_passthrough);
    RUN_TEST(test_calibrate_stores_offset_and_marks_calibrated);
    RUN_TEST(test_apply_subtracts_offset_once_calibrated);
    return UNITY_END();
}
