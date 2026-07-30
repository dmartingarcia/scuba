#include <LittleFS.h>
#include <ArduinoJson.h>
#include "accel_calibration_store.h"
#include "../globals.h"
#include "../config.h"

static void persist() {
  File f = LittleFS.open(ACCEL_CALIBRATION_PATH, "w");
  if (!f) return;

  JsonDocument doc;
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

  accelCalibration.zOffset = doc["zOffset"] | 0.0f;
  accelCalibration.calibrated = doc["calibrated"] | false;
}

void calibrateAccelZeroNow() {
  float sum = 0;
  int samples = 0;

  for (int i = 0; i < ACCEL_CALIBRATION_SAMPLES; i++) {
    float x, y, z, sqrtMag;
    if (imu->readAccel(x, y, z, sqrtMag)) {
      sum += z;
      samples++;
    }
    delay(50);
  }

  if (samples == 0) {
    logBuffer.println("Accel calibration failed: no readings from IMU");
    return;
  }

  accelCalibration = calibrateAccelZero(sum / samples);
  persist();
  logBuffer.println("Accel calibrated: zOffset=" + String(accelCalibration.zOffset));
}

void clearAccelCalibration() {
  accelCalibration = defaultAccelCalibration();
  persist();
  logBuffer.println("Accel calibration cleared");
}
