// Minimal OTA-only bridge firmware - NOT part of the normal app.
//
// Purpose: the real firmware's watchdog (armed early in setup(), 12s
// timeout) can starve during a large OTA transfer on a slow/congested link,
// causing the device to reboot mid-transfer before the new image (which
// contains the fix for that exact problem) ever lands. This bridge is tiny
// (WiFi + ArduinoOTA only, no sensors/motors/webserver/mqtt) and never arms
// a watchdog at all, so it can absorb a full-size OTA transfer no matter how
// long it takes. Flash this once via OTA, then flash the real firmware
// through env:ota again - it'll land on this bridge without issue.
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "secrets.h"

void setup() {
  Serial.begin(9600);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();
}
