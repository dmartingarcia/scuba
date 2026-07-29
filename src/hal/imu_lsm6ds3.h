#ifndef IMU_LSM6DS3_H
#define IMU_LSM6DS3_H

#include <math.h>
#include "imu.h"
#include "SparkFunLSM6DS3.h"

class Lsm6ds3Imu : public ImuSensor {
public:
    explicit Lsm6ds3Imu(uint8_t address) : imu(I2C_MODE, address) {}

    bool begin() override {
        return imu.begin() == IMU_SUCCESS;
    }

    bool readAccel(float &x, float &y, float &z, float &sqrtMag) override {
        x = imu.readFloatAccelX();
        y = imu.readFloatAccelY();
        z = imu.readFloatAccelZ();
        sqrtMag = sqrtf(x * x + y * y + z * z);
        return true;
    }

    bool readGyro(float &x, float &y, float &z) override {
        x = imu.readFloatGyroX();
        y = imu.readFloatGyroY();
        z = imu.readFloatGyroZ();
        return true;
    }

    const char* name() const override { return "LSM6DS3"; }

    bool hasReliableGyro() const override { return true; }

    // 6-axis chip - no magnetometer exists on this hardware, period.
    bool hasMagnetometer() override { return false; }
    bool readHeading(float &heading) override { (void)heading; return false; }

private:
    LSM6DS3 imu;
};

#endif // IMU_LSM6DS3_H
