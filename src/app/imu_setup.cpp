#include "imu_setup.h"
#include "../config.h"
#include "../logic/imu_detect.h"
#include "../hal/imu_mpu9250.h"
#include "../hal/imu_lsm6ds3.h"

ImuSensor* detectAndBeginImu(TwoWire &wire) {
  uint8_t whoAmI = 0;
  wire.beginTransmission(LSM6DS3_WHO_AM_I_ADDR);
  wire.write(LSM6DS3_WHO_AM_I_REG);
  wire.endTransmission(false);
  wire.requestFrom((int)LSM6DS3_WHO_AM_I_ADDR, 1);
  if (wire.available()) {
    whoAmI = wire.read();
  }

  ImuSensor* detected;
  if (detectImuType(whoAmI) == ImuType::LSM6DS3) {
    detected = new Lsm6ds3Imu(LSM6DS3_WHO_AM_I_ADDR);
  } else {
    detected = new Mpu9250Imu(&wire);
  }

  detected->begin();
  return detected;
}
