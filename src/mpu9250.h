#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

class MPU9250 {
public:
    MPU9250(uint8_t sdaPin, uint8_t sclPin);
    bool init();
    void logging_data();
    void update();

    float getAccelX();
    float getAccelY();
    float getAccelZ();
    float getSqrtAccel();
    float getInclination();

    float getGyroX();
    float getGyroY();
    float getGyroZ();

    float getMagX();
    float getMagY();
    float getMagZ();
    float getMagDirection();

    MPU9250_asukiaaa mpu;
    uint8_t sda, scl;
    float ax, ay, az, aSqrt;
    float gx, gy, gz;
    float mx, my, mz;
    float mDirection;
};

#endif // MPU9250_H