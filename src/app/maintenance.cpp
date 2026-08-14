#include <LittleFS.h>
#include <ArduinoJson.h>
#include "maintenance.h"
#include "../globals.h"
#include "../config.h"
#include "../logic/maintenance_stats.h"

static unsigned long lastTickMillis = 0;
static unsigned long nextStatsCommit = 0;
static bool wasActive = false;

static void loadStats() {
  File f = LittleFS.open(MAINTENANCE_STATS_PATH, "r");
  if (!f) {
    maintenanceStats = {0, 0};
    return;
  }

  JsonDocument doc;
  deserializeJson(doc, f);
  f.close();

  maintenanceStats.bootCount = doc["bootCount"] | 0;
  maintenanceStats.totalRuntimeSeconds = doc["totalRuntimeSeconds"] | 0;
}

static void persistStats() {
  File f = LittleFS.open(MAINTENANCE_STATS_PATH, "w");
  if (!f) return;

  JsonDocument doc;
  doc["bootCount"] = maintenanceStats.bootCount;
  doc["totalRuntimeSeconds"] = maintenanceStats.totalRuntimeSeconds;
  serializeJson(doc, f);
  f.close();
}

static bool isActiveState() {
  return currentState == MOVING_FORWARD || currentState == MOVING_BACKWARD || currentState == TURNING;
}

void maintenanceInit() {
  LittleFS.begin(true); // format on first boot / corrupt filesystem

  loadStats();
  maintenanceStats = recordBoot(maintenanceStats);
  persistStats();

  lastTickMillis = millis();
  nextStatsCommit = millis() + statsSaveIntervalMs;
  wasActive = isActiveState();
}

void resetMaintenanceStats() {
  maintenanceStats = {0, 0};
  persistStats();
}

void maintenanceTick() {
  unsigned long now = millis();
  uint32_t elapsedSeconds = (now - lastTickMillis) / 1000;
  if (elapsedSeconds == 0) return;
  lastTickMillis = now;

  bool active = isActiveState();
  maintenanceStats = accumulateActiveSeconds(maintenanceStats, elapsedSeconds, active);

  bool sessionJustEnded = wasActive && !active;
  wasActive = active;

  if (shouldCommitStats(now, nextStatsCommit, statsSaveIntervalMs, sessionJustEnded)) {
    persistStats();
    nextStatsCommit = now + statsSaveIntervalMs;
  }
}
