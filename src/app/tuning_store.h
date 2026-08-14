#ifndef TUNING_STORE_H
#define TUNING_STORE_H

void tuningInit();   // Load persisted tuning params from LittleFS
void saveTuning();   // Persist current globals.tuning
void resetTuning();  // Back to config.h defaults, persist

#endif // TUNING_STORE_H
