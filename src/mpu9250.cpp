#include "mpu9250.h"
#include "Wire.h"
#include "MPU9250_asukiaaa.h"

MPU9250::MPU9250(uint8_t sdaPin, uint8_t sclPin)
    : sda(sdaPin), scl(sclPin), ax(-1), ay(-1), az(-1), aSqrt(-1), gx(-1), gy(-1), gz(-1), mx(-1), my(-1), mz(-1), mDirection(-1) {}

bool MPU9250::init() {
    Wire.begin(sda, scl);
    mpu.setWire(&Wire);
    mpu.beginAccel();
    mpu.beginGyro();
    mpu.beginMag();
    update(); // Initial update to read sensor values
    logging_data(); // Log initial data
    return true;
}

void MPU9250::logging_data() {
    Serial.print("Accel X:" + String(ax));
    Serial.print("Accel Y:" + String(ay));
    Serial.print("Accel Z:" + String(az));
    Serial.print("Accel Sqrt:" + String(aSqrt));
    Serial.print("Gyro X:" + String(gx));
    Serial.print("Gyro Y:" + String(gy));
    Serial.print("Gyro Z:" + String(gz));
    Serial.print("Mag X:" + String(mx));
    Serial.print("Mag Y:" + String(my));
    Serial.print("Mag Z:" + String(mz));
    Serial.print("Mag Direction:" + String(mDirection));
}

void MPU9250::update() {
    auto result = mpu.accelUpdate();
    if (result != 0) {
        mpu.beginAccel();
        result = mpu.accelUpdate();
        if (result != 0) {
            Serial.println("Failed to update accelerometer data.");
        }
    }

    result = mpu.gyroUpdate();
    if (result != 0) {
        mpu.beginGyro();
        result = mpu.gyroUpdate();
        if (result != 0) {
            Serial.println("Failed to update gyroscope data.");
        }
    }

    result = mpu.magUpdate();
    if (result != 0) {
        mpu.beginMag();
        result = mpu.magUpdate();
        if (result != 0) {
            Serial.println("Failed to update magnetometer data.");
        }
    }

    ax = mpu.accelX();
    ay = mpu.accelY();
    az = mpu.accelZ();
    aSqrt = mpu.accelSqrt();

    gx = mpu.gyroX();
    gy = mpu.gyroY();
    gz = mpu.gyroZ();

    mx = mpu.magX();
    my = mpu.magY();
    mz = mpu.magZ();
    mDirection = mpu.magHorizDirection();
}

float MPU9250::getAccelX() { return ax; }
float MPU9250::getAccelY() { return ay; }
float MPU9250::getAccelZ() { return az; }
float MPU9250::getSqrtAccel() { return aSqrt; }
float MPU9250::getInclination() {
    return abs(az);
}

float MPU9250::getGyroX() { return gx; }
float MPU9250::getGyroY() { return gy; }
float MPU9250::getGyroZ() { return gz; }

float MPU9250::getMagX() { return mx; }
float MPU9250::getMagY() { return my; }
float MPU9250::getMagZ() { return mz; }
float MPU9250::getMagDirection() { return mDirection; }