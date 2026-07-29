#include <unity.h>
#include "../../src/logic/imu_detect.h"

void setUp(void) {}
void tearDown(void) {}

void test_detects_lsm6ds3(void) {
    TEST_ASSERT_TRUE(ImuType::LSM6DS3 == detectImuType(0x69));
}

void test_detects_mpu9250(void) {
    TEST_ASSERT_TRUE(ImuType::MPU9250 == detectImuType(0x71));
}

void test_detects_mpu9255_variant(void) {
    TEST_ASSERT_TRUE(ImuType::MPU9250 == detectImuType(0x73));
}

void test_unknown_for_unrecognized_byte(void) {
    TEST_ASSERT_TRUE(ImuType::Unknown == detectImuType(0xFF));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_detects_lsm6ds3);
    RUN_TEST(test_detects_mpu9250);
    RUN_TEST(test_detects_mpu9255_variant);
    RUN_TEST(test_unknown_for_unrecognized_byte);
    return UNITY_END();
}
