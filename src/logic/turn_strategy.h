#ifndef TURN_STRATEGY_H
#define TURN_STRATEGY_H

#include <string.h>

// Turning has always been finicky on this hardware (thrust-against-water,
// unreliable MPU9250 gyro, no usable magnetometer near the motors) - keep
// multiple selectable strategies instead of committing to one.
enum class TurnStrategy {
    Legacy,        // Raw gyro-yaw tracking (the original behavior, unchanged)
    FixedDuration, // Pulsed spin for a calibrated duration, no yaw tracking
    KalmanFusion,  // Gyro-yaw tracking with a Kalman-smoothed rate
};

inline const char* turnStrategyName(TurnStrategy s) {
    switch (s) {
        case TurnStrategy::Legacy: return "legacy";
        case TurnStrategy::FixedDuration: return "duration";
        case TurnStrategy::KalmanFusion: return "kalman";
    }
    return "legacy";
}

inline TurnStrategy parseTurnStrategy(const char* name, TurnStrategy fallback) {
    if (strcmp(name, "legacy") == 0) return TurnStrategy::Legacy;
    if (strcmp(name, "duration") == 0) return TurnStrategy::FixedDuration;
    if (strcmp(name, "kalman") == 0) return TurnStrategy::KalmanFusion;
    return fallback;
}

#endif // TURN_STRATEGY_H
