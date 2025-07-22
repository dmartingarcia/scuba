#include "mpu9250.h"

MPU9250::MPU9250(uint8_t sdaPin, uint8_t sclPin)
    : sda(sdaPin), scl(sclPin), ax(0), ay(0), az(0), aSqrt(0), gx(0), gy(0), gz(0), mx(0), my(0), mz(0), mDirection(0) {}

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
    ("Accel X: "); Serial.println(ax);
    Serial.print("Accel Y: "); Serial.println(ay);
    Serial.print("Accel Z: "); Serial.println(az);
    Serial.print("Accel Sqrt: "); Serial.println(aSqrt);
    Serial.print("Gyro X: "); Serial.println(gx);
    Serial.print("Gyro Y: "); Serial.println(gy);
    Serial.print("Gyro Z: "); Serial.println(gz);
    Serial.print("Mag X: "); Serial.println(mx);
    Serial.print("Mag Y: "); Serial.println(my);
    Serial.print("Mag Z: "); Serial.println(mz);
    Serial.print("Mag Direction: "); Serial.println(mDirection);
}

void MPU9250::update() {
    mpu.accelUpdate();
    mpu.gyroUpdate();
    mpu.magUpdate();

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

float MPU9250::getAccelX() const { return ax; }
float MPU9250::getAccelY() const { return ay; }
float MPU9250::getAccelZ() const { return az; }
float MPU9250::getSqrtAccel() const { return aSqrt; }
float MPU9250::getInclination() const {
    return abs(az);
}

float MPU9250::getGyroX() const { return gx; }
float MPU9250::getGyroY() const { return gy; }
float MPU9250::getGyroZ() const { return gz; }

float MPU9250::getMagX() const { return mx; }
float MPU9250::getMagY() const { return my; }
float MPU9250::getMagZ() const { return mz; }
float MPU9250::getMagDirection() const { return mDirection; }