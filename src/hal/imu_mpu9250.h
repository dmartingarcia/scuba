#ifndef IMU_MPU9250_H
#define IMU_MPU9250_H

#include "imu.h"
#include "MPU9250_asukiaaa.h"

#define AK8963_I2C_ADDRESS 0x0C
#define AK8963_WHO_AM_I_REG 0x00
#define AK8963_WHO_AM_I_EXPECTED 0x48

class Mpu9250Imu : public ImuSensor {
public:
    Mpu9250Imu(TwoWire *wire, uint8_t address = MPU9250_ADDRESS_AD0_LOW) : mpu(address), wire(wire) {
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

    // False: gyro output on this chip has been unreliable in practice.
    bool hasReliableGyro() const override { return false; }

    // Real runtime probe, not an assumption - a lot of "MPU9250" breakout
    // boards in the wild are actually MPU6500 clones with no AK8963 die at
    // all. beginMag() enables I2C bypass so the AK8963 becomes directly
    // addressable at 0x0C; read its own WHO_AM_I (expects 0x48) rather than
    // trusting that any I2C transaction there "worked".
    bool hasMagnetometer() override {
        if (!magProbed) {
            magProbed = true;
            mpu.beginMag();

            wire->beginTransmission(AK8963_I2C_ADDRESS);
            wire->write(AK8963_WHO_AM_I_REG);
            wire->endTransmission(false);
            wire->requestFrom((int)AK8963_I2C_ADDRESS, 1);

            magPresent = wire->available() && (wire->read() == AK8963_WHO_AM_I_EXPECTED);
        }
        return magPresent;
    }

    bool readHeading(float &heading) override {
        if (!hasMagnetometer()) return false;
        if (mpu.magUpdate() != 0) return false;
        heading = mpu.magHorizDirection();
        return true;
    }

private:
    MPU9250_asukiaaa mpu;
    TwoWire* wire;
    bool magProbed = false;
    bool magPresent = false;
};

#endif // IMU_MPU9250_H
