#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <math.h>

// Genuine 2-axis tilt from a (roughly static) accelerometer reading: the
// gravity vector alone gives real pitch and roll (unlike yaw, which needs
// a gyro or compass - see logic/kalman_filter.h and the magnetometer
// probe).
//
// Axis convention here is NOT the textbook one (gravity on Z) - on this
// chassis's IMU mount, gravity sits on X at rest (see the calibration
// readings logged in sensors.cpp: X≈-1, Y≈0, Z≈0 flat; Z swings ±1 on
// front/back tilt, matching angle()'s use of raw Z for wall detection).
// So X (not Z) goes in both denominators, and Z (not X) is the pitch
// numerator - putting the near-zero-at-rest axis on top keeps atan2 away
// from its near-singular case (large numerator over a near-zero
// denominator), which is what was causing the attitude UI to jump around.
struct Attitude {
    float pitchDeg;
    float rollDeg;
};

inline Attitude computeAttitude(float x, float y, float z) {
    Attitude a;
    a.pitchDeg = atan2(z, sqrt(x * x + y * y)) * 180.0 / M_PI;
    a.rollDeg = atan2(y, sqrt(x * x + z * z)) * 180.0 / M_PI;
    return a;
}

#endif // ATTITUDE_H
