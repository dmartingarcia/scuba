#ifndef ACCEL_CALIBRATION_H
#define ACCEL_CALIBRATION_H

// Corrects all 3 accel axes for an IMU that isn't mounted perfectly level
// in the chassis: calibrate once with the robot resting flat, subtract
// that reading from every future one. Needed for both angle() (Z-only,
// wall detection) and pitch/roll attitude display (X+Y too).
struct AccelCalibration {
    float xOffset;
    float yOffset;
    float zOffset;
    bool calibrated;
};

inline AccelCalibration defaultAccelCalibration() {
    return {0.0f, 0.0f, 0.0f, false};
}

inline AccelCalibration calibrateAccel(float x, float y, float z) {
    return {x, y, z, true};
}

inline float calibratedX(float rawX, const AccelCalibration& cal) {
    return cal.calibrated ? (rawX - cal.xOffset) : rawX;
}

inline float calibratedY(float rawY, const AccelCalibration& cal) {
    return cal.calibrated ? (rawY - cal.yOffset) : rawY;
}

inline float calibratedZ(float rawZ, const AccelCalibration& cal) {
    return cal.calibrated ? (rawZ - cal.zOffset) : rawZ;
}

#endif // ACCEL_CALIBRATION_H
