#ifndef TUNING_PARAMS_H
#define TUNING_PARAMS_H

#include "../config.h"

// Live, UI-editable versions of the motor/angle/timing constants that used
// to be hardcoded in config.h. Persisted to LittleFS (see
// app/tuning_store.h) - config.h's DEFAULT_* constants only seed the
// defaults below and what /tuning?action=reset restores.
struct TuningParams {
    float movimientoMoveSpeed = DEFAULT_MOVIMIENTO_MOVE_SPEED;
    float movimientoMoveBackwardsSpeed = DEFAULT_MOVIMIENTO_MOVE_BACKWARDS_SPEED;
    float movimientoIdleSpeed = DEFAULT_MOVIMIENTO_IDLE_SPEED;
    float aguaTurnSpeed = DEFAULT_AGUA_TURN_SPEED;
    float aguaMoveSpeed = DEFAULT_AGUA_MOVE_SPEED;
    float aguaIdleSpeed = DEFAULT_AGUA_IDLE_SPEED;

    float wallAngleThreshold = DEFAULT_WALL_ANGLE_THRESHOLD;
    float wallAngleRecoverThreshold = DEFAULT_WALL_ANGLE_RECOVER_THRESHOLD;
    float floorInclinationPrecision = DEFAULT_FLOOR_INCLINATION_PRECISION;
    int turnAngleDeg = DEFAULT_TURN_ANGLE;
    float upsideDownThreshold = DEFAULT_UPSIDE_DOWN_THRESHOLD;

    long movingTimeoutMs = DEFAULT_MOVING_TIMEOUT;
    long maxTimeTurningMs = DEFAULT_MAX_TIME_TURNING;
    long delayAutostartMs = DEFAULT_DELAY_AUTOSTART;
    long turnDurationMs = DEFAULT_TURN_DURATION_MS;
    long manualActionDurationMs = DEFAULT_MANUAL_ACTION_DURATION_MS;

    float attitudeSmoothingAlpha = DEFAULT_ATTITUDE_SMOOTHING_ALPHA;

    // Diagnostic override: when true, motorAgua.setSpeed() calls are tracked
    // (getSpeed(), logs, /status all keep reporting as normal) but never
    // actually reach the physical pins - see Motor::setFakeDisabled().
    bool aguaDisabledFake = false;
};

inline TuningParams defaultTuningParams() {
    return TuningParams();
}

#endif // TUNING_PARAMS_H
