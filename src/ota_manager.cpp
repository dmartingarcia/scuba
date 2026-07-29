#include <ArduinoOTA.h>
#include "ota_manager.h"
#include "globals.h"

void setupOta() {
  ArduinoOTA.onStart([]() {
    motorMovimiento.setSpeed(0);
    motorAgua.setSpeed(0);
    currentState = STOPPED; // Stop motors during OTA
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
