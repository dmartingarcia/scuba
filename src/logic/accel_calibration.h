#ifndef ACCEL_CALIBRATION_H
#define ACCEL_CALIBRATION_H

// Corrects angle() for an IMU that isn't mounted perfectly level in the
// chassis: calibrate once with the robot resting flat, subtract that
// reading from every future one.
struct AccelCalibration {
    float zOffset;
    bool calibrated;
};

inline AccelCalibration defaultAccelCalibration() {
    return {0.0f, false};
}

inline AccelCalibration calibrateAccelZero(float measuredZ) {
    return {measuredZ, true};
}

inline float applyAccelCalibration(float rawZ, const AccelCalibration& cal) {
    return cal.calibrated ? (rawZ - cal.zOffset) : rawZ;
}

#endif // ACCEL_CALIBRATION_H
