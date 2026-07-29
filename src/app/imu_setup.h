#ifndef IMU_SETUP_H
#define IMU_SETUP_H

#include <Wire.h>
#include "../hal/imu.h"

// Probes the LSM6DS3's WHO_AM_I register over I2C to decide which IMU chip
// is physically wired up, and returns a ready-to-use driver for it. Caller
// owns the returned pointer.
ImuSensor* detectAndBeginImu(TwoWire &wire);

#endif // IMU_SETUP_H
