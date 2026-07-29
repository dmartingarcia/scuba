#ifndef MAINTENANCE_H
#define MAINTENANCE_H

void maintenanceInit(); // Call once from setup(): loads stats, records this boot
void maintenanceTick(); // Call every loop(): accumulates active runtime, persists per shouldCommitStats()

#endif // MAINTENANCE_H
