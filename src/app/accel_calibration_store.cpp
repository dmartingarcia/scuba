#include <LittleFS.h>
#include <ArduinoJson.h>
#include "accel_calibration_store.h"
#include "../globals.h"
#include "../config.h"

static void persist() {
  File f = LittleFS.open(ACCEL_CALIBRATION_PATH, "w");
  if (!f) return;

  JsonDocument doc;
  doc["xOffset"] = accelCalibration.xOffset;
  doc["yOffset"] = accelCalibration.yOffset;
  doc["zOffset"] = accelCalibration.zOffset;
  doc["calibrated"] = accelCalibration.calibrated;
  serializeJson(doc, f);
  f.close();
}

void accelCalibrationInit() {
  LittleFS.begin(true); // no-op if already mounted elsewhere

  File f = LittleFS.open(ACCEL_CALIBRATION_PATH, "r");
  if (!f) {
    accelCalibration = defaultAccelCalibration();
    return;
  }

  JsonDocument doc;
  deserializeJson(doc, f);
  f.close();

  accelCalibration.xOffset = doc["xOffset"] | 0.0f;
  accelCalibration.yOffset = doc["yOffset"] | 0.0f;
  accelCalibration.zOffset = doc["zOffset"] | 0.0f;
  accelCalibration.calibrated = doc["calibrated"] | false;
}

void calibrateAccelZeroNow() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 0;

  for (int i = 0; i < ACCEL_CALIBRATION_SAMPLES; i++) {
    float x, y, z, sqrtMag;
    if (imu->readAccel(x, y, z, sqrtMag)) {
      sumX += x;
      sumY += y;
      sumZ += z;
      samples++;
    }
    delay(50);
  }

  if (samples == 0) {
    logBuffer.println("Accel calibration failed: no readings from IMU");
    return;
  }

  accelCalibration = calibrateAccel(sumX / samples, sumY / samples, sumZ / samples);
  persist();
  logBuffer.println("Accel calibrated: x=" + String(accelCalibration.xOffset) +
                     " y=" + String(accelCalibration.yOffset) +
                     " z=" + String(accelCalibration.zOffset));
}

void clearAccelCalibration() {
  accelCalibration = defaultAccelCalibration();
  persist();
  logBuffer.println("Accel calibration cleared");
}
