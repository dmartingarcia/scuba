#include <LittleFS.h>
#include <ArduinoJson.h>
#include "tuning_store.h"
#include "../globals.h"
#include "../config.h"

void saveTuning() {
  File f = LittleFS.open(TUNING_CONFIG_PATH, "w");
  if (!f) return;

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
  serializeJson(doc, f);
  f.close();
}

void tuningInit() {
  LittleFS.begin(true); // no-op if already mounted elsewhere

  File f = LittleFS.open(TUNING_CONFIG_PATH, "r");
  if (!f) {
    tuning = defaultTuningParams();
    return;
  }

  JsonDocument doc;
  deserializeJson(doc, f);
  f.close();

  TuningParams d = defaultTuningParams();
  tuning.movimientoMoveSpeed = doc["movimientoMoveSpeed"] | d.movimientoMoveSpeed;
  tuning.movimientoMoveBackwardsSpeed = doc["movimientoMoveBackwardsSpeed"] | d.movimientoMoveBackwardsSpeed;
  tuning.movimientoIdleSpeed = doc["movimientoIdleSpeed"] | d.movimientoIdleSpeed;
  tuning.aguaTurnSpeed = doc["aguaTurnSpeed"] | d.aguaTurnSpeed;
  tuning.aguaMoveSpeed = doc["aguaMoveSpeed"] | d.aguaMoveSpeed;
  tuning.aguaIdleSpeed = doc["aguaIdleSpeed"] | d.aguaIdleSpeed;
  tuning.wallAngleThreshold = doc["wallAngleThreshold"] | d.wallAngleThreshold;
  tuning.wallAngleRecoverThreshold = doc["wallAngleRecoverThreshold"] | d.wallAngleRecoverThreshold;
  tuning.floorInclinationPrecision = doc["floorInclinationPrecision"] | d.floorInclinationPrecision;
  tuning.turnAngleDeg = doc["turnAngleDeg"] | d.turnAngleDeg;
  tuning.upsideDownThreshold = doc["upsideDownThreshold"] | d.upsideDownThreshold;
  tuning.movingTimeoutMs = doc["movingTimeoutMs"] | d.movingTimeoutMs;
  tuning.maxTimeTurningMs = doc["maxTimeTurningMs"] | d.maxTimeTurningMs;
  tuning.delayAutostartMs = doc["delayAutostartMs"] | d.delayAutostartMs;
  tuning.turnDurationMs = doc["turnDurationMs"] | d.turnDurationMs;
  tuning.attitudeSmoothingAlpha = doc["attitudeSmoothingAlpha"] | d.attitudeSmoothingAlpha;
  tuning.manualActionDurationMs = doc["manualActionDurationMs"] | d.manualActionDurationMs;
}

void resetTuning() {
  tuning = defaultTuningParams();
  saveTuning();
  logBuffer.println("Tuning params reset to defaults");
}
