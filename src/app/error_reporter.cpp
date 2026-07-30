#include <LittleFS.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include "error_reporter.h"
#include "../globals.h"
#include "../config.h"

static void loadErrorLog() {
  File f = LittleFS.open(ERROR_LOG_PATH, "r");
  if (!f) {
    errorLog = {};
    return;
  }

  JsonDocument doc;
  deserializeJson(doc, f);
  f.close();

  errorLog = {};
  errorLog.count = doc["count"] | 0;
  errorLog.nextIndex = doc["nextIndex"] | 0;
  errorLog.activeMask = doc["activeMask"] | 0;

  JsonArrayConst entries = doc["entries"].as<JsonArrayConst>();
  int i = 0;
  for (JsonObjectConst e : entries) {
    if (i >= ERROR_LOG_CAPACITY) break;
    errorLog.entries[i].code = e["code"] | 0;
    errorLog.entries[i].timestamp = e["timestamp"] | 0;
    i++;
  }
}

static void persistErrorLog() {
  File f = LittleFS.open(ERROR_LOG_PATH, "w");
  if (!f) return;

  JsonDocument doc;
  doc["count"] = errorLog.count;
  doc["nextIndex"] = errorLog.nextIndex;
  doc["activeMask"] = errorLog.activeMask;

  JsonArray entries = doc["entries"].to<JsonArray>();
  for (int i = 0; i < errorLog.count; i++) {
    JsonObject e = entries.add<JsonObject>();
    e["code"] = errorLog.entries[i].code;
    e["timestamp"] = errorLog.entries[i].timestamp;
  }

  serializeJson(doc, f);
  f.close();
}

void errorReporterInit() {
  LittleFS.begin(true); // no-op if maintenanceInit() already mounted it
  loadErrorLog();
}

void logError(ErrorCode code) {
  if (isErrorActive(errorLog, (uint8_t)code)) return; // already active, don't spam

  logBuffer.println("*** ERROR: " + String(errorCodeName((uint8_t)code)) + " ***");
  errorLog = pushError(errorLog, (uint8_t)code, millis());
  persistErrorLog();
}

void clearErrorCode(ErrorCode code) {
  if (!isErrorActive(errorLog, (uint8_t)code)) return;

  errorLog = clearError(errorLog, (uint8_t)code);
  persistErrorLog();
}

void clearAllErrors() {
  errorLog = {};
  persistErrorLog();
}

void checkResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  bool unexpected = reason == ESP_RST_PANIC
                 || reason == ESP_RST_INT_WDT
                 || reason == ESP_RST_TASK_WDT
                 || reason == ESP_RST_WDT
                 || reason == ESP_RST_BROWNOUT;

  if (!unexpected) return;

  logBuffer.println("Last boot was NOT clean - reset reason code: " + String((int)reason));
  // One-shot event, not an ongoing condition: log it, then immediately clear
  // so the NEXT bad reset (even the same reason) also gets recorded instead
  // of being deduped away by the "already active" rule.
  logError(ErrorCode::UnexpectedReset);
  clearErrorCode(ErrorCode::UnexpectedReset);
}
