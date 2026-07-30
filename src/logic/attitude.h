#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <math.h>

// Genuine 2-axis tilt from a (roughly static) accelerometer reading: the
// gravity vector alone gives real pitch and roll (unlike yaw, which needs
// a gyro or compass - see logic/kalman_filter.h and the magnetometer
// probe). Standard accelerometer tilt formulas.
struct Attitude {
    float pitchDeg;
    float rollDeg;
};

inline Attitude computeAttitude(float x, float y, float z) {
    Attitude a;
    a.pitchDeg = atan2(x, sqrt(y * y + z * z)) * 180.0 / M_PI;
    a.rollDeg = atan2(y, sqrt(x * x + z * z)) * 180.0 / M_PI;
    return a;
}

#endif // ATTITUDE_H
