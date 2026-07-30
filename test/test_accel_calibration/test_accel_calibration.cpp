#include <unity.h>
#include "../../src/logic/accel_calibration.h"

void setUp(void) {}
void tearDown(void) {}

void test_default_is_uncalibrated_and_passthrough(void) {
    AccelCalibration cal = defaultAccelCalibration();
    TEST_ASSERT_FALSE(cal.calibrated);
    TEST_ASSERT_EQUAL_FLOAT(0.37f, calibratedX(0.37f, cal));
    TEST_ASSERT_EQUAL_FLOAT(0.22f, calibratedY(0.22f, cal));
    TEST_ASSERT_EQUAL_FLOAT(0.11f, calibratedZ(0.11f, cal));
}

void test_calibrate_stores_all_three_offsets(void) {
    AccelCalibration cal = calibrateAccel(0.1f, 0.2f, 0.15f);
    TEST_ASSERT_TRUE(cal.calibrated);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, cal.xOffset);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, cal.yOffset);
    TEST_ASSERT_EQUAL_FLOAT(0.15f, cal.zOffset);
}

void test_apply_subtracts_matching_axis_offset(void) {
    AccelCalibration cal = calibrateAccel(0.1f, 0.2f, 0.15f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calibratedX(0.1f, cal));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calibratedY(0.2f, cal));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calibratedZ(0.15f, cal));
    TEST_ASSERT_EQUAL_FLOAT(0.9f, calibratedX(1.0f, cal));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_default_is_uncalibrated_and_passthrough);
    RUN_TEST(test_calibrate_stores_all_three_offsets);
    RUN_TEST(test_apply_subtracts_matching_axis_offset);
    return UNITY_END();
}
