#ifndef IMU_MPU9250_H
#define IMU_MPU9250_H

#include "imu.h"
#include "MPU9250_asukiaaa.h"

class Mpu9250Imu : public ImuSensor {
public:
    Mpu9250Imu(TwoWire *wire, uint8_t address = MPU9250_ADDRESS_AD0_LOW) : mpu(address) {
        mpu.setWire(wire);
    }

    bool begin() override {
        mpu.beginAccel();
        mpu.beginGyro();
        return true;
    }

    bool readAccel(float &x, float &y, float &z, float &sqrtMag) override {
        if (mpu.accelUpdate() != 0) return false;
        x = mpu.accelX();
        y = mpu.accelY();
        z = mpu.accelZ();
        sqrtMag = mpu.accelSqrt();
        return true;
    }

    bool readGyro(float &x, float &y, float &z) override {
        if (mpu.gyroUpdate() != 0) return false;
        x = mpu.gyroX();
        y = mpu.gyroY();
        z = mpu.gyroZ();
        return true;
    }

    const char* name() const override { return "MPU9250"; }

private:
    MPU9250_asukiaaa mpu;
};

#endif // IMU_MPU9250_H
