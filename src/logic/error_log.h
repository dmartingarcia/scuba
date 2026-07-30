#ifndef ERROR_LOG_H
#define ERROR_LOG_H

#include <stdint.h>

// Fault codes, ECU-style: something failed, gets recorded with a timestamp,
// and stays "active" (won't log a duplicate) until the condition clears.
enum class ErrorCode : uint8_t {
    ImuInitFailed = 1,
    ImuReadFailed = 2,
    BmpInitFailed = 3,
    TurnTimeout = 4,
    WifiConnectFailed = 5,
    MqttConnectFailed = 6,
    UnexpectedReset = 7, // Boot caused by panic/watchdog/brownout, not power-on or a clean reset
};

#define ERROR_LOG_CAPACITY 16

struct ErrorEntry {
    uint8_t code;
    uint32_t timestamp;
};

struct ErrorLog {
    ErrorEntry entries[ERROR_LOG_CAPACITY];
    uint8_t count;       // valid entries, caps at ERROR_LOG_CAPACITY
    uint8_t nextIndex;   // next write slot (circular)
    uint32_t activeMask; // bit N set = error code N is currently active (not yet cleared)
};

// Records an occurrence of `code`, unless it's already active (repeated
// failures of the same kind don't spam the log - only the first one, until
// clearError() runs). Overwrites the oldest entry once at capacity.
inline ErrorLog pushError(ErrorLog log, uint8_t code, uint32_t timestamp) {
    uint32_t bit = (1UL << code);
    if (log.activeMask & bit) return log; // already active, don't duplicate

    log.activeMask |= bit;
    log.entries[log.nextIndex] = {code, timestamp};
    log.nextIndex = (log.nextIndex + 1) % ERROR_LOG_CAPACITY;
    if (log.count < ERROR_LOG_CAPACITY) log.count++;
    return log;
}

// Marks `code` as no longer active - the next occurrence will be logged again.
inline ErrorLog clearError(ErrorLog log, uint8_t code) {
    log.activeMask &= ~(1UL << code);
    return log;
}

inline bool isErrorActive(const ErrorLog& log, uint8_t code) {
    return log.activeMask & (1UL << code);
}

inline const char* errorCodeName(uint8_t code) {
    switch ((ErrorCode)code) {
        case ErrorCode::ImuInitFailed: return "ImuInitFailed";
        case ErrorCode::ImuReadFailed: return "ImuReadFailed";
        case ErrorCode::BmpInitFailed: return "BmpInitFailed";
        case ErrorCode::TurnTimeout: return "TurnTimeout";
        case ErrorCode::WifiConnectFailed: return "WifiConnectFailed";
        case ErrorCode::MqttConnectFailed: return "MqttConnectFailed";
        case ErrorCode::UnexpectedReset: return "UnexpectedReset";
        default: return "Unknown";
    }
}

#endif // ERROR_LOG_H
