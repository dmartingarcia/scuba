#ifndef MAINTENANCE_STATS_H
#define MAINTENANCE_STATS_H

#include <stdint.h>

// Persisted across reboots (EEPROM) for maintenance tracking: how many
// times the robot has been powered on, and how many seconds it has
// actually spent cleaning (not idle/stopped).
struct MaintenanceStats {
    uint32_t bootCount;
    uint32_t totalRuntimeSeconds;
};

inline MaintenanceStats recordBoot(MaintenanceStats stats) {
    stats.bootCount += 1;
    return stats;
}

inline MaintenanceStats accumulateActiveSeconds(MaintenanceStats stats, uint32_t elapsedSeconds, bool isActive) {
    if (isActive) stats.totalRuntimeSeconds += elapsedSeconds;
    return stats;
}

// Decides whether it's time to persist stats to EEPROM:
// - always commit right when a cleaning session just ended (avoid losing
//   the last stretch of runtime to a power loss before the next periodic tick)
// - otherwise, on the configured periodic interval - intervalMs == 0
//   disables periodic saving entirely (session-end is then the only trigger)
inline bool shouldCommitStats(unsigned long now, unsigned long nextCommitAt, unsigned long intervalMs, bool sessionJustEnded) {
    if (sessionJustEnded) return true;
    if (intervalMs == 0) return false;
    return now >= nextCommitAt;
}

#endif // MAINTENANCE_STATS_H
