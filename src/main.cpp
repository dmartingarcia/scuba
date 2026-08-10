#include <Arduino.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
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
// implementation just returns true unconditionally, which is what let a bad
// OTA image get permanently confirmed with zero chance of a fallback). With
// this override, the image stays in ESP_OTA_IMG_PENDING_VERIFY until setup()
// explicitly confirms it at the very end (see esp_ota_mark_app_valid_cancel_
// rollback() below) - if the chip never gets there (crash/hang anywhere in
// setup(), caught by the watchdog below), the next boot attempt still finds
// it PENDING_VERIFY and the bootloader itself marks it ABORTED and falls
// back to the previous working image (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE,
// confirmed on in this SDK build) - no app code needs to run for that.
// setupWifi() (wifi_manager.cpp) additionally calls
// esp_ota_mark_app_invalid_rollback_and_reboot() itself, faster than waiting
// for a crash, if the new image can't even reach WiFi.
extern "C" bool verifyRollbackLater() { return true; }

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging

  // Arm the watchdog before anything else below, not just before loop() - a
  // crash/hang anywhere in setup() (most likely an I2C lockup talking to the
  // IMU/BMP280) needs to turn into a reboot too, both to recover a healthy
  // board and so a genuinely bad OTA image gets the reboot it needs for the
  // bootloader's rollback check above to ever see it again.
  esp_task_wdt_init(WATCHDOG_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(NULL);

  errorReporterInit(); // Mount LittleFS, load persisted fault log
  checkResetReason(); // Logs UnexpectedReset if last boot was a crash/watchdog/brownout
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
  accelCalibrationInit(); // Load persisted accelerometer zero-offset, if any
  mqttConfigInit(); // Load persisted MQTT broker config, if any
  tuningInit(); // Load persisted motor/angle/timing tuning, if any
  setupMqtt();

  // Only now, after every step above has run without crashing or hanging -
  // not just after WiFi connected - tell the bootloader this OTA image is
  // good. Confirming any earlier (previously done right after WiFi connected,
  // in wifi_manager.cpp) meant a crash in sensor/motor init immediately
  // afterwards would already be "valid" and crash-loop forever with no way
  // back to the last known-good firmware.
  esp_ota_mark_app_valid_cancel_rollback();
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
