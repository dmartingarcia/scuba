#ifndef MOTOR_DIRECTION_GUARD_H
#define MOTOR_DIRECTION_GUARD_H

// Minimum time a motor must sit at zero output before reversing direction.
// Without R_EN/L_EN hard-cutting the H-bridge (see MOTOR_HAS_ENABLE_PINS in
// config.h), a fast forward<->reverse flip can drive current through the
// bridge before the motor's back-EMF has decayed - this is the software
// equivalent of that cutoff.
const unsigned long MOTOR_DIRECTION_DEADTIME_MS = 500;

inline int motorSign(int speed) {
    if (speed > 0) return 1;
    if (speed < 0) return -1;
    return 0;
}

// Call apply() every control tick with the desired speed and use its return
// value - not the input - as what's actually written to the PWM pins.
struct MotorDirectionGuard {
    int lastDrivenSign = 0;
    unsigned long neutralSinceMs = 0;
    bool reversalPending = false;

    int apply(int requestedSpeed, unsigned long nowMs) {
        int requestedSign = motorSign(requestedSpeed);

        bool isReversal = requestedSign != 0 && lastDrivenSign != 0 && requestedSign != lastDrivenSign;
        if (isReversal) {
            if (!reversalPending) {
                reversalPending = true;
                neutralSinceMs = nowMs;
                return 0;
            }
            if (nowMs - neutralSinceMs < MOTOR_DIRECTION_DEADTIME_MS) {
                return 0;
            }
            // Dead time satisfied - fall through and allow the reversal.
        }

        reversalPending = false;
        if (requestedSign != 0) {
            lastDrivenSign = requestedSign;
        }
        return requestedSpeed;
    }
};

#endif // MOTOR_DIRECTION_GUARD_H
