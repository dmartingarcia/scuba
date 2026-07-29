#ifndef IMU_H
#define IMU_H

// HAL boundary: robot logic talks to this interface only, never to a
// specific chip driver (MPU9250_asukiaaa, SparkFunLSM6DS3, ...) directly.
class ImuSensor {
public:
    virtual ~ImuSensor() {}
    virtual bool begin() = 0;
    virtual bool readAccel(float &x, float &y, float &z, float &sqrtMag) = 0;
    virtual bool readGyro(float &x, float &y, float &z) = 0;
    virtual const char* name() const = 0;
};

#endif // IMU_H
