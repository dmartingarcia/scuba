#ifndef ACCEL_CALIBRATION_STORE_H
#define ACCEL_CALIBRATION_STORE_H

void accelCalibrationInit();     // Load persisted calibration from LittleFS
void calibrateAccelZeroNow();    // Sample the accelerometer at rest, persist the offset
void clearAccelCalibration();    // Back to uncalibrated passthrough, persist

#endif // ACCEL_CALIBRATION_STORE_H
