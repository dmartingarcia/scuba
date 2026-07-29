#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "globals.h"
#include "sensors.h"
#include "robot_logic.h"
#include "position_tracker.h"
#include "wifi_manager.h"
#include "ota_manager.h"
#include "web_server.h"
#include "imu_setup.h"

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  setupWifi(); // Connect to WiFi
  setupOta(); // Setup OTA updates
  setupWebServer(); // Setup web server

  led.init();
  motorMovimiento.init(); // Initialize movement motor
  motorAgua.init(); // Initialize water motor

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bmp.begin(0x76)) {
    logBuffer.println("Could not find a valid BMP280 sensor, check wiring!");
  }

  imu = detectAndBeginImu(Wire); // Auto-detects MPU9250 vs LSM6DS3
  logBuffer.println("IMU detected: " + String(imu->name()));
}

void loop() {
  ArduinoOTA.handle();
  maintainWifi();
  robotLogic();
  updatePosition();
  updateYaw(); // Update yaw angle based on gyro data
  led.handleBlink();
}
