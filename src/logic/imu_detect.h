#ifndef IMU_DETECT_H
#define IMU_DETECT_H

#include <stdint.h>

enum class ImuType { MPU9250, LSM6DS3, Unknown };

// Decide which IMU chip is wired up from its WHO_AM_I register byte, so the
// firmware doesn't need a rebuild to swap the old MPU9250 for the new
// LSM6DS3 - probe once at boot and pick the matching driver.
//
// LSM6DS3 WHO_AM_I (reg 0x0F) is always 0x69.
// MPU9250 family WHO_AM_I (reg 0x75): 0x71 (MPU9250), 0x73 (MPU9255),
// 0x70/0x68 seen on some MPU6500-based clones sold as "MPU9250".
inline ImuType detectImuType(uint8_t whoAmI) {
    if (whoAmI == 0x69) return ImuType::LSM6DS3;
    if (whoAmI == 0x71 || whoAmI == 0x73 || whoAmI == 0x70 || whoAmI == 0x68) return ImuType::MPU9250;
    return ImuType::Unknown;
}

#endif // IMU_DETECT_H
