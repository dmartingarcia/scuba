#ifndef SENSORS_H
#define SENSORS_H

#include "../logic/attitude.h"

void updateSensors(); // Reads BMP280 + accel, throttled to DELAY_UPDATING_SENSORS
void updateYaw();     // Integrates gyro readings into the running yaw estimate
float angle();        // Tilt angle derived from the latest accel reading (Z-only, wall detection)
Attitude currentAttitude(); // Full pitch+roll from the accelerometer, for the UI's 3D model

#endif // SENSORS_H
