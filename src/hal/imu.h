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

    // Whether this chip's gyro readings are trustworthy enough to steer a
    // precise turn by. Some MPU9250 units in the wild have unreliable gyro
    // output - robot_logic falls back to a fixed-duration turn when false.
    virtual bool hasReliableGyro() const = 0;

    // Whether a magnetometer actually answers on this chip - a genuine
    // runtime probe (see Mpu9250Imu), not an assumption. Many "MPU9250"
    // breakout boards are mislabeled MPU6500 clones with no magnetometer
    // die at all. Not const: the concrete implementation may need to do
    // I2C I/O (and cache the result) the first time this is called.
    virtual bool hasMagnetometer() = 0;

    // Absolute heading in degrees. Only meaningful if hasMagnetometer().
    virtual bool readHeading(float &heading) = 0;
};

#endif // IMU_H
