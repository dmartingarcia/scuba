#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include "ota_manager.h"
#include "../globals.h"

void setupOta() {
  ArduinoOTA.onStart([]() {
    motorMovimiento.setSpeed(0);
    motorAgua.setSpeed(0);
    currentState = STOPPED; // Stop motors during OTA
  });

  // ArduinoOTA.handle() blocks synchronously while it erases/writes each
  // flash chunk - long enough on some writes to starve loop()'s
  // esp_task_wdt_reset() past WATCHDOG_TIMEOUT_SECONDS, which was crashing
  // the transfer around 70-90% (reset reason ESP_RST_TASK_WDT). Feed it here
  // too, on every progress tick this callback gets during the transfer.
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    esp_task_wdt_reset();
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      logBuffer.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      logBuffer.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      logBuffer.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      logBuffer.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      logBuffer.println("End Failed");
    }
  });

  ArduinoOTA.begin();
}
