#include <WiFi.h>
#include <esp_ota_ops.h>
#include "wifi_manager.h"
#include "../globals.h"
#include "../secrets.h"
#include "../logic/wifi_timing.h"
#include "../app/error_reporter.h"

// Whether the recovery AP (see AP_FALLBACK_SSID in config.h) is currently up.
static bool apFallbackActive = false;

static void startApFallback() {
  if (apFallbackActive) return;
  apFallbackActive = true;
  WiFi.mode(WIFI_AP_STA); // keep retrying the home network in the background
  WiFi.softAP(AP_FALLBACK_SSID, AP_FALLBACK_PASSWORD);
  logBuffer.println("STA unreachable - recovery AP up at " + WiFi.softAPIP().toString());
}

static void stopApFallback() {
  if (!apFallbackActive) return;
  apFallbackActive = false;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  logBuffer.println("STA reconnected - recovery AP down");
}

// If this boot is running freshly-flashed OTA firmware that hasn't proven
// itself yet (see verifyRollbackLater() in main.cpp) and it can't even reach
// the WiFi that worked for every previous firmware, that's a strong signal
// the new image is broken. Revert to the last known-good firmware and
// reboot automatically - no cable, no opening the sealed enclosure.
static void rollbackIfPendingVerify() {
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t otaState;
  if (esp_ota_get_state_partition(running, &otaState) == ESP_OK && otaState == ESP_OTA_IMG_PENDING_VERIFY) {
    logBuffer.println("New OTA firmware failed to reach WiFi - rolling back to previous firmware");
    esp_ota_mark_app_invalid_rollback_and_reboot(); // reboots immediately if a previous app exists; otherwise returns
  }
}

void setupWifi() {
  if ((timeToConnectWifi > (long)millis()) || (RobotState::STARTING != currentState && RobotState::STOPPED != currentState)) {
    return;
  }
  logBuffer.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = RETRIES_WIFI_CONNECT;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    logBuffer.println(".");

    retries--;
    if (retries <= 0) {
      logBuffer.println("Failed to connect to WiFi. Please check your credentials.");
      logError(ErrorCode::WifiConnectFailed);
      rollbackIfPendingVerify();
      startApFallback();
      return;
    }
    timeToConnectWifi = millis() + TIME_TO_CONNECT_WIFI; // 60 seconds to connect
  }

  logBuffer.println("");
  logBuffer.println("WiFi connected");
  logBuffer.println("IP address: ");
  logBuffer.println(WiFi.localIP().toString());
  clearErrorCode(ErrorCode::WifiConnectFailed);
  esp_ota_mark_app_valid_cancel_rollback(); // first successful connect after an OTA confirms the new image is good
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    clearErrorCode(ErrorCode::WifiConnectFailed);
    stopApFallback();
    return;
  }
  if (!isReconnectDue(millis(), timeToConnectWifi)) return;

  logBuffer.println("WiFi disconnected, attempting reconnect...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  timeToConnectWifi = nextReconnectAttempt(millis(), TIME_TO_CONNECT_WIFI);
  startApFallback();
}
