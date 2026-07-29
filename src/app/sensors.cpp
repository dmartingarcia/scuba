#include "sensors.h"
#include "../globals.h"
#include "error_reporter.h"

void updateSensors() {
  if ((long)millis() < nextUpdate) return;
  nextUpdate += DELAY_UPDATING_SENSORS;

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();

  if (imu->readAccel(aX, aY, aZ, aSqrt)) {
    clearErrorCode(ErrorCode::ImuReadFailed);
  } else {
    logBuffer.println("Cannot read accel values");
    logError(ErrorCode::ImuReadFailed);
  }
}

void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastYawUpdate) / 1000.0; // en segundos
  lastYawUpdate = now;

  if (!imu->readGyro(gX, gY, gZ)) {
    logBuffer.println("Cannot read gyro values");
    logError(ErrorCode::ImuReadFailed);
    return; // No gyro data, cannot update yaw
  }

  float gX2, gY2, gZ2;
  if (imu->readGyro(gX2, gY2, gZ2)) {
    gX = (gX2 + gX) / 2;
    gY = (gY2 + gY) / 2;
    gZ = (gZ2 + gZ) / 2;
    clearErrorCode(ErrorCode::ImuReadFailed);
  } else {
    logBuffer.println("Cannot read gyro values");
    logError(ErrorCode::ImuReadFailed);
    return; // No gyro data, cannot update yaw
  }

  if (abs(gX) > 5.0) { // Threshold to avoid noise
    yaw += gX * dt; // Integración simple
  }

  // Normaliza el ángulo entre 0 y 360
  while (yaw < 0) yaw += 360;
  while (yaw >= 360) yaw -= 360;
}

float angle() {
  updateSensors();
  // accel - X:-1.01 Y: 0.07 Z:-0.01 Sqrt:1.02 reposo
  // accel - X:-0.05 Y:-0.04 Z:-1.04 Sqrt:1.04 subiendo de un lado
  // accel - X: 0.02 Y: 0.06 Z: 0.97 Sqrt:0.97 subiendo del otro lado
  // accel - X: 0.98 Y:-0.11 Z:-0.09 Sqrt:0.99  boca arriba
  // accel - X: 0.04 Y: 1.01 Z:-0.09 Sqrt:1.01 de canto 1
  // accel - X:-0.08 Y:-1.00 Z:-0.00 Sqrt:1.00 de canto 2
  return 90.0f * aZ;
}
