#include <ArduinoJson.h>
#include "web_server.h"
#include "../globals.h"
#include "../app/sensors.h"
#include "../app/error_reporter.h"
#include "../app/accel_calibration_store.h"
#include "../app/mqtt_config_store.h"
#include "../index.h" // HTML content for the web interface

void setupWebServer() {
  // Serve main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  // Serve logs
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", logBuffer.get());
  });

  // Control endpoint
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    String action;
    if (request->hasParam("action")) {
      action = request->getParam("action")->value();

      if (action == "start") {
        // Optional ?duration=<minutes> sets/resets how long this run should
        // last (0 or omitted = unlimited, keeps the last configured value).
        if (request->hasParam("duration")) {
          sessionDurationMs = (unsigned long)request->getParam("duration")->value().toInt() * 60000UL;
        }
        sessionStartMillis = millis();
        sessionCompletedByTimer = false;
        currentState = MOVING_FORWARD;
      } else if (action == "stop") {
        sessionCompletedByTimer = false; // manual stop = paused, not finished
        currentState = STOPPED;
      } else if (action == "turn") {
        currentState = TURNING;
      }
    }
    request->send(200, "text/plain", "OK");
  });

  // Runtime configuration endpoint
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("sessionDuration")) {
      sessionDurationMs = (unsigned long)request->getParam("sessionDuration")->value().toInt() * 60000UL;
    }
    if (request->hasParam("statsSaveInterval")) {
      statsSaveIntervalMs = (unsigned long)request->getParam("statsSaveInterval")->value().toInt() * 60000UL;
    }
    if (request->hasParam("turnStrategy")) {
      turnStrategy = parseTurnStrategy(request->getParam("turnStrategy")->value().c_str(), turnStrategy);
    }

    JsonDocument doc;
    doc["sessionDurationMinutes"] = sessionDurationMs / 60000;
    doc["statsSaveIntervalMinutes"] = statsSaveIntervalMs / 60000;
    doc["turnStrategy"] = turnStrategyName(turnStrategy);

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["state"] = resolveState(currentState);
    doc["angle"] = angle();
    doc["yaw"] = yaw;
    doc["x"] = currentX;
    doc["y"] = currentY;
    doc["sessionDurationMinutes"] = sessionDurationMs / 60000;
    doc["sessionElapsedSeconds"] = (millis() - sessionStartMillis) / 1000;
    doc["imuName"] = imu->name();
    doc["imuHasMagnetometer"] = imu->hasMagnetometer();
    doc["accelCalibrated"] = accelCalibration.calibrated;
    doc["accelZeroOffset"] = accelCalibration.zOffset;

    Attitude attitude = currentAttitude();
    doc["pitchDeg"] = attitude.pitchDeg;
    doc["rollDeg"] = attitude.rollDeg;

    JsonObject maintenance = doc["maintenance"].to<JsonObject>();
    maintenance["bootCount"] = maintenanceStats.bootCount;
    maintenance["totalRuntimeHours"] = maintenanceStats.totalRuntimeSeconds / 3600.0;

    JsonArray map = doc["map"].to<JsonArray>();
    for (int y = 0; y < GRID_SIZE; y++) {
      JsonArray row = map.add<JsonArray>();
      for (int x = 0; x < GRID_SIZE; x++) {
        row.add(cleanedArea[y][x]);
      }
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Fault log endpoint (ECU-style): GET for the log, ?action=clear to wipe it
  server.on("/errors", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("action") && request->getParam("action")->value() == "clear") {
      clearAllErrors();
      request->send(200, "text/plain", "OK");
      return;
    }

    JsonDocument doc;
    JsonArray entries = doc["entries"].to<JsonArray>();
    for (int i = 0; i < errorLog.count; i++) {
      JsonObject e = entries.add<JsonObject>();
      e["code"] = errorLog.entries[i].code;
      e["name"] = errorCodeName(errorLog.entries[i].code);
      e["timestamp"] = errorLog.entries[i].timestamp;
      e["active"] = isErrorActive(errorLog, errorLog.entries[i].code);
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Accelerometer zero calibration: place the robot flat, GET /calibrate.
  // ?action=clear wipes it back to uncalibrated passthrough.
  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("action") && request->getParam("action")->value() == "clear") {
      clearAccelCalibration();
    } else {
      calibrateAccelZeroNow();
    }

    JsonDocument doc;
    doc["calibrated"] = accelCalibration.calibrated;
    doc["zOffset"] = accelCalibration.zOffset;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // MQTT / Home Assistant config: GET updates whatever params are present,
  // ?action=reset wipes it back to defaults (disabled). Password is never
  // echoed back in the response.
  server.on("/mqtt", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("action") && request->getParam("action")->value() == "reset") {
      resetMqttConfig();
    } else {
      if (request->hasParam("enabled")) mqttConfig.enabled = request->getParam("enabled")->value() == "1";
      if (request->hasParam("host")) mqttConfig.host = request->getParam("host")->value();
      if (request->hasParam("port")) mqttConfig.port = (uint16_t)request->getParam("port")->value().toInt();
      if (request->hasParam("user")) mqttConfig.user = request->getParam("user")->value();
      if (request->hasParam("password")) mqttConfig.password = request->getParam("password")->value();
      if (request->hasParam("topicPrefix")) mqttConfig.topicPrefix = request->getParam("topicPrefix")->value();
      saveMqttConfig();
    }

    JsonDocument doc;
    doc["enabled"] = mqttConfig.enabled;
    doc["host"] = mqttConfig.host;
    doc["port"] = mqttConfig.port;
    doc["user"] = mqttConfig.user;
    doc["topicPrefix"] = mqttConfig.topicPrefix;
    doc["hasPassword"] = mqttConfig.password.length() > 0;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.begin();
}
