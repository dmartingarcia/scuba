#include <Arduino.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "globals.h"
#include "app/sensors.h"
#include "app/robot_logic.h"
#include "app/position_tracker.h"
#include "app/imu_setup.h"
#include "app/maintenance.h"
#include "app/error_reporter.h"
#include "app/accel_calibration_store.h"
#include "app/mqtt_config_store.h"
#include "app/tuning_store.h"
#include "net/wifi_manager.h"
#include "net/ota_manager.h"
#include "net/web_server.h"
#include "net/mqtt_manager.h"

// Defers the ESP-IDF bootloader's OTA self-validation instead of letting it
// auto-confirm the new image before setup() even runs (its default weak
// implementation just returns true unconditionally). setupWifi() is what
// actually calls esp_ota_mark_app_valid_cancel_rollback()/
// esp_ota_mark_app_invalid_rollback_and_reboot() once it knows whether the
// new firmware can reach WiFi at all.
extern "C" bool verifyRollbackLater() { return true; }

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  errorReporterInit(); // Mount LittleFS, load persisted fault log
  checkResetReason(); // Logs UnexpectedReset if last boot was a crash/watchdog/brownout
  setupWifi(); // Connect to WiFi
  setupOta(); // Setup OTA updates
  setupWebServer(); // Setup web server

  led.init();
  motorMovimiento.init(); // Initialize movement motor
  motorAgua.init(); // Initialize water motor

  Wire.begin(SDA_PIN, SCL_PIN);

  // One-shot bus scan at boot so a wiring/pull-up problem shows up in
  // /logs directly, instead of guessing address-by-address from symptoms.
  logBuffer.println("I2C scan on SDA=" + String(SDA_PIN) + " SCL=" + String(SCL_PIN) + ":");
  int devicesFound = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      logBuffer.println("  found 0x" + String(addr, HEX));
      devicesFound++;
    }
  }
  if (devicesFound == 0) {
    logBuffer.println("  nothing responded - check wiring/pull-ups before anything else");
  }

  if (bmp.begin(0x76)) {
    clearErrorCode(ErrorCode::BmpInitFailed);
  } else {
    logBuffer.println("Could not find a valid BMP280 sensor, check wiring!");
    logError(ErrorCode::BmpInitFailed);
  }

  imu = detectAndBeginImu(Wire); // Auto-detects MPU9250 vs LSM6DS3
  logBuffer.println("IMU detected: " + String(imu->name()));
  logBuffer.println(String("Magnetometer: ") + (imu->hasMagnetometer() ? "present (real WHO_AM_I probe passed)" : "absent/not responding"));

  // Default turn strategy per chip - override anytime via /config?turnStrategy=
  turnStrategy = imu->hasReliableGyro() ? TurnStrategy::Legacy : TurnStrategy::FixedDuration;

  maintenanceInit(); // Load persisted stats, record this boot
  accelCalibrationInit(); // Load persisted accelerometer zero-offset, if any
  mqttConfigInit(); // Load persisted MQTT broker config, if any
  tuningInit(); // Load persisted motor/angle/timing tuning, if any
  setupMqtt();

  // From here on, loop() must call esp_task_wdt_reset() regularly (also fed
  // inside recoverFromWall()/turn() in robot_logic.cpp+turn_controller.cpp,
  // which can legitimately block for seconds at a time) - if it ever stops,
  // most likely an I2C lockup talking to the IMU/BMP280, reboot instead of
  // sitting frozen underwater until someone pulls the robot out.
  esp_task_wdt_init(WATCHDOG_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset();
  ArduinoOTA.handle();
  maintainWifi();
  robotLogic();
  updatePosition();
  updateYaw(); // Update yaw angle based on gyro data
  maintenanceTick();
  maintainMqtt();
  led.handleBlink();
}
