#include <Arduino.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "globals.h"
#include "app/sensors.h"
#include "app/robot_logic.h"
#include "app/position_tracker.h"
#include "app/imu_setup.h"
#include "app/maintenance.h"
#include "app/error_reporter.h"
#include "net/wifi_manager.h"
#include "net/ota_manager.h"
#include "net/web_server.h"

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  errorReporterInit(); // Mount LittleFS, load persisted fault log
  setupWifi(); // Connect to WiFi
  setupOta(); // Setup OTA updates
  setupWebServer(); // Setup web server

  led.init();
  motorMovimiento.init(); // Initialize movement motor
  motorAgua.init(); // Initialize water motor

  Wire.begin(SDA_PIN, SCL_PIN);

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
}

void loop() {
  ArduinoOTA.handle();
  maintainWifi();
  robotLogic();
  updatePosition();
  updateYaw(); // Update yaw angle based on gyro data
  maintenanceTick();
  led.handleBlink();
}
