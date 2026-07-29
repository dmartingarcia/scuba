#include <WiFi.h>
#include "wifi_manager.h"
#include "../globals.h"
#include "../secrets.h"
#include "../logic/wifi_timing.h"
#include "../app/error_reporter.h"

void setupWifi() {
  if (timeToConnectWifi > (long)millis()) {
    return;
  }

  logBuffer.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 40;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    logBuffer.println(".");

    retries--;
    if (retries <= 0) {
      logBuffer.println("Failed to connect to WiFi. Please check your credentials.");
      logError(ErrorCode::WifiConnectFailed);
      return;
    }
    timeToConnectWifi = millis() + 60000; // 60 seconds to connect
  }

  logBuffer.println("");
  logBuffer.println("WiFi connected");
  logBuffer.println("IP address: ");
  logBuffer.println(WiFi.localIP().toString());
  clearErrorCode(ErrorCode::WifiConnectFailed);
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    clearErrorCode(ErrorCode::WifiConnectFailed);
    return;
  }
  if (!isReconnectDue(millis(), timeToConnectWifi)) return;

  logBuffer.println("WiFi disconnected, attempting reconnect...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  timeToConnectWifi = nextReconnectAttempt(millis(), 60000);
}
