#ifndef ERROR_REPORTER_H
#define ERROR_REPORTER_H

#include "../logic/error_log.h"

void errorReporterInit();              // Load persisted error log from LittleFS
void logError(ErrorCode code);         // Record + persist immediately; deduped while active
void clearErrorCode(ErrorCode code);   // Mark resolved so the next occurrence logs again
void clearAllErrors();                 // Wipe the whole fault log (manual "clear codes" action)

#endif // ERROR_REPORTER_H
