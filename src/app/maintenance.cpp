#include <EEPROM.h>
#include "maintenance.h"
#include "../globals.h"
#include "../logic/maintenance_stats.h"

#define EEPROM_SIZE 16
#define EEPROM_MAGIC 0xA5C3EEDA
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_STATS 4

static unsigned long lastTickMillis = 0;
static unsigned long nextStatsCommit = 0;
static bool wasActive = false;

static void persistStats() {
  EEPROM.put(EEPROM_ADDR_MAGIC, (uint32_t)EEPROM_MAGIC);
  EEPROM.put(EEPROM_ADDR_STATS, maintenanceStats);
  EEPROM.commit();
}

static bool isActiveState() {
  return currentState == MOVING_FORWARD || currentState == MOVING_BACKWARD || currentState == TURNING;
}

void maintenanceInit() {
  EEPROM.begin(EEPROM_SIZE);

  uint32_t magic = 0;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_ADDR_STATS, maintenanceStats);
  } else {
    maintenanceStats = {0, 0};
  }

  maintenanceStats = recordBoot(maintenanceStats);
  persistStats();

  lastTickMillis = millis();
  nextStatsCommit = millis() + statsSaveIntervalMs;
  wasActive = isActiveState();
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
