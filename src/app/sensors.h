#ifndef SENSORS_H
#define SENSORS_H

void updateSensors(); // Reads BMP280 + accel, throttled to DELAY_UPDATING_SENSORS
void updateYaw();     // Integrates gyro readings into the running yaw estimate
float angle();        // Tilt angle derived from the latest accel reading

#endif // SENSORS_H
