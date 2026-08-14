#include <ArduinoJson.h>
#include "web_server.h"
#include "../globals.h"
#include "../app/sensors.h"
#include "../app/error_reporter.h"
#include "../app/accel_calibration_store.h"
#include "../app/mqtt_config_store.h"
#include "../app/tuning_store.h"
#include "../app/maintenance.h"
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

      // MAINTENANCE (flipped over, see isUpsideDown() in sensors.cpp) is
      // terminal like STOPPED, but stricter: only "start" leaves it -
      // stop/forward/backward/turn are no-ops until you either press Start
      // or physically reboot the robot.
      if (currentState == MAINTENANCE && action != "start") {
        request->send(200, "text/plain", "OK");
        return;
      }

      if (action == "start") {
        // Optional ?duration=<minutes> sets/resets how long this run should
        // last (0 or omitted = unlimited, keeps the last configured value).
        if (request->hasParam("duration")) {
          sessionDurationMs = (unsigned long)request->getParam("duration")->value().toInt() * 60000UL;
        }
        sessionStartMillis = millis();
        sessionCompletedByTimer = false;
        manualControlActive = false;
        currentState = MOVING_FORWARD;
      } else if (action == "stop") {
        sessionCompletedByTimer = false; // manual stop = paused, not finished
        manualControlActive = false;
        currentState = STOPPED;
      } else if (action == "turn") {
        // One-shot: same previousState mechanism the autonomous
        // wall-avoidance turn uses (see handleWallDetection() in
        // robot_logic.cpp) - turnToDirection() restores it automatically
        // once the turn completes, no follow-up request needed.
        previousState = currentState;
        currentState = TURNING;
        maxTurningMillis = millis() + tuning.maxTimeTurningMs;
        timeout = millis();
      } else if (action == "forward" || action == "backward") {
        // One-shot pulse: drive for tuning.manualActionDurationMs, then
        // robotLogic() automatically restores manualRevertState - entirely
        // server-side, no follow-up "stop" request needed from the caller.
        if (!manualControlActive) manualRevertState = currentState;
        currentState = action == "forward" ? MOVING_FORWARD : MOVING_BACKWARD;
        manualControlActive = true;
        manualActionDeadlineMillis = millis() + tuning.manualActionDurationMs;
      } else if (action == "maintenance") {
        // Manual entry into MAINTENANCE (normally only reached via
        // isUpsideDown() in robotLogic()) - lets /maintenance actions like
        // rampAgua run without having to physically flip the robot. Same
        // exit path as the automatic case: only action=start leaves it.
        motorMovimiento.setSpeed(0);
        motorAgua.setSpeed(0);
        manualControlActive = false;
        currentState = MAINTENANCE;
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
    doc["firmwareCommit"] = FIRMWARE_COMMIT;
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

  // Live motor/angle/timing tuning, persisted to LittleFS. Defaults live in
  // config.h (DEFAULT_*) via TuningParams - see logic/tuning_params.h.
  server.on("/tuning", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("action") && request->getParam("action")->value() == "reset") {
      resetTuning();
    } else {
      bool changed = false;
      if (request->hasParam("movimientoMoveSpeed")) { tuning.movimientoMoveSpeed = request->getParam("movimientoMoveSpeed")->value().toFloat(); changed = true; }
      if (request->hasParam("movimientoMoveBackwardsSpeed")) { tuning.movimientoMoveBackwardsSpeed = request->getParam("movimientoMoveBackwardsSpeed")->value().toFloat(); changed = true; }
      if (request->hasParam("movimientoIdleSpeed")) { tuning.movimientoIdleSpeed = request->getParam("movimientoIdleSpeed")->value().toFloat(); changed = true; }
      if (request->hasParam("aguaTurnSpeed")) { tuning.aguaTurnSpeed = request->getParam("aguaTurnSpeed")->value().toFloat(); changed = true; }
      if (request->hasParam("aguaMoveSpeed")) { tuning.aguaMoveSpeed = request->getParam("aguaMoveSpeed")->value().toFloat(); changed = true; }
      if (request->hasParam("aguaIdleSpeed")) { tuning.aguaIdleSpeed = request->getParam("aguaIdleSpeed")->value().toFloat(); changed = true; }
      if (request->hasParam("wallAngleThreshold")) { tuning.wallAngleThreshold = request->getParam("wallAngleThreshold")->value().toFloat(); changed = true; }
      if (request->hasParam("wallAngleRecoverThreshold")) { tuning.wallAngleRecoverThreshold = request->getParam("wallAngleRecoverThreshold")->value().toFloat(); changed = true; }
      if (request->hasParam("floorInclinationPrecision")) { tuning.floorInclinationPrecision = request->getParam("floorInclinationPrecision")->value().toFloat(); changed = true; }
      if (request->hasParam("turnAngleDeg")) { tuning.turnAngleDeg = request->getParam("turnAngleDeg")->value().toInt(); changed = true; }
      if (request->hasParam("upsideDownThreshold")) { tuning.upsideDownThreshold = request->getParam("upsideDownThreshold")->value().toFloat(); changed = true; }
      if (request->hasParam("movingTimeoutMs")) { tuning.movingTimeoutMs = request->getParam("movingTimeoutMs")->value().toInt(); changed = true; }
      if (request->hasParam("maxTimeTurningMs")) { tuning.maxTimeTurningMs = request->getParam("maxTimeTurningMs")->value().toInt(); changed = true; }
      if (request->hasParam("delayAutostartMs")) { tuning.delayAutostartMs = request->getParam("delayAutostartMs")->value().toInt(); changed = true; }
      if (request->hasParam("turnDurationMs")) { tuning.turnDurationMs = request->getParam("turnDurationMs")->value().toInt(); changed = true; }
      if (request->hasParam("attitudeSmoothingAlpha")) { tuning.attitudeSmoothingAlpha = request->getParam("attitudeSmoothingAlpha")->value().toFloat(); changed = true; }
      if (request->hasParam("manualActionDurationMs")) { tuning.manualActionDurationMs = request->getParam("manualActionDurationMs")->value().toInt(); changed = true; }
      if (request->hasParam("aguaDisabledFake")) {
        tuning.aguaDisabledFake = request->getParam("aguaDisabledFake")->value() == "1";
        motorAgua.setFakeDisabled(tuning.aguaDisabledFake);
        changed = true;
      }
      if (changed) saveTuning();
    }

    JsonDocument doc;
    doc["movimientoMoveSpeed"] = tuning.movimientoMoveSpeed;
    doc["movimientoMoveBackwardsSpeed"] = tuning.movimientoMoveBackwardsSpeed;
    doc["movimientoIdleSpeed"] = tuning.movimientoIdleSpeed;
    doc["aguaTurnSpeed"] = tuning.aguaTurnSpeed;
    doc["aguaMoveSpeed"] = tuning.aguaMoveSpeed;
    doc["aguaIdleSpeed"] = tuning.aguaIdleSpeed;
    doc["wallAngleThreshold"] = tuning.wallAngleThreshold;
    doc["wallAngleRecoverThreshold"] = tuning.wallAngleRecoverThreshold;
    doc["floorInclinationPrecision"] = tuning.floorInclinationPrecision;
    doc["turnAngleDeg"] = tuning.turnAngleDeg;
    doc["upsideDownThreshold"] = tuning.upsideDownThreshold;
    doc["movingTimeoutMs"] = tuning.movingTimeoutMs;
    doc["maxTimeTurningMs"] = tuning.maxTimeTurningMs;
    doc["delayAutostartMs"] = tuning.delayAutostartMs;
    doc["turnDurationMs"] = tuning.turnDurationMs;
    doc["attitudeSmoothingAlpha"] = tuning.attitudeSmoothingAlpha;
    doc["manualActionDurationMs"] = tuning.manualActionDurationMs;
    doc["aguaDisabledFake"] = tuning.aguaDisabledFake;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Maintenance actions, from the Maintenance card in the web UI:
  //   ?action=reboot        - restart the ESP32
  //   ?action=resetStats    - zero bootCount/totalRuntimeSeconds
  //   ?action=factoryReset  - wipe MQTT/tuning/calibration/fault-log config
  //                           and stats back to defaults, then reboot
  //   ?action=rampAgua                - ramp the water motor 0->255
  //   ?action=rampMovimientoForward   - ramp the movement motor 0->255
  //   ?action=rampMovimientoBackward  - ramp the movement motor 0->-255
  //                           (all three over MAINTENANCE_RAMP_DURATION_MS;
  //                           only while flipped into MAINTENANCE - see
  //                           robotLogic()'s MAINTENANCE case)
  server.on("/maintenance", HTTP_GET, [](AsyncWebServerRequest *request) {
    String action = request->hasParam("action") ? request->getParam("action")->value() : "";

    if (action == "reboot" || action == "factoryReset") {
      if (action == "factoryReset") {
        resetMqttConfig();
        resetTuning();
        clearAccelCalibration();
        clearAllErrors();
        resetMaintenanceStats();
      }
      request->send(200, "text/plain", "OK");
      // Deferred so this response has time to flush before the connection drops.
      rebootRequested = true;
      rebootAtMillis = millis() + 500;
      return;
    }

    if (action == "resetStats") {
      resetMaintenanceStats();
    } else if (action == "rampAgua" || action == "rampMovimientoForward" || action == "rampMovimientoBackward") {
      if (currentState != MAINTENANCE) {
        request->send(409, "text/plain", "Flip the robot into MAINTENANCE first");
        return;
      }
      if (action == "rampAgua") {
        aguaRampActive = true;
        aguaRampStartMillis = millis();
      } else {
        movimientoRampActive = true;
        movimientoRampStartMillis = millis();
        movimientoRampDirection = action == "rampMovimientoForward" ? 1 : -1;
      }
    }

    request->send(200, "text/plain", "OK");
  });

  server.begin();
}
