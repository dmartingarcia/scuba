#include "motor.h"
#include <Arduino.h>
#include "../logic/speed_utils.h"

bool Motor::init() {
    pinMode(rpwmPin, OUTPUT);
    speed = 0;
    analogWrite(rpwmPin, 0);

    if (singleDirection) {
        initialized = true;
        return true;
    }

    pinMode(lpwmPin, OUTPUT);
    analogWrite(lpwmPin, 0);
#if MOTOR_HAS_ENABLE_PINS
    pinMode(rEnablePin, OUTPUT);
    pinMode(lEnablePin, OUTPUT);
    digitalWrite(rEnablePin, LOW); // Disable both sides until a direction is set
    digitalWrite(lEnablePin, LOW);
#endif
    initialized = true;
    return true;
}

int Motor::checkSpeed(int speed) {
    return clampSpeed(speed);
}

void Motor::setFakeDisabled(bool disabled) {
    fakeDisabled = disabled;
    if (!disabled || !initialized) return;

    // Force the physical outputs off immediately, regardless of whatever
    // speed was last actually applied.
    analogWrite(rpwmPin, 0);
    if (singleDirection) return; // no lpwm/enable pins to touch in this mode

    analogWrite(lpwmPin, 0);
#if MOTOR_HAS_ENABLE_PINS
    digitalWrite(rEnablePin, LOW);
    digitalWrite(lEnablePin, LOW);
#endif
}

bool Motor::isFakeDisabled() const {
    return fakeDisabled;
}

void Motor::setSpeed(int newSpeed) {
    if (!initialized) return;

    if (singleDirection) {
        int requested = checkSpeed(newSpeed);
        speed = requested < 0 ? 0 : requested; // hardware only drives one direction
        if (fakeDisabled) return; // tracked via `speed`/getSpeed(), pin untouched
        analogWrite(rpwmPin, speed);
        return;
    }

    int requested = checkSpeed(newSpeed);
    speed = directionGuard.apply(requested, millis());

    if (fakeDisabled) return; // tracked above via `speed`/getSpeed(), physical pins untouched

    if (speed > 0) {
#if MOTOR_HAS_ENABLE_PINS
        digitalWrite(rEnablePin, HIGH); // Enable both sides - direction comes from RPWM/LPWM
        digitalWrite(lEnablePin, HIGH);
#endif
        // Forward rotation
        analogWrite(rpwmPin, abs(speed)); // Set PWM for forward rotation
        analogWrite(lpwmPin, 0);     // No PWM for reverse
    } else if (speed < 0) {
#if MOTOR_HAS_ENABLE_PINS
        digitalWrite(rEnablePin, HIGH);
        digitalWrite(lEnablePin, HIGH);
#endif
        // Reverse rotation
        analogWrite(rpwmPin, 0);         // No PWM for forward
        analogWrite(lpwmPin, abs(speed));    // Set PWM for reverse rotation
    } else {
        // Stop (or held at zero by directionGuard during a reversal's dead time)
        analogWrite(rpwmPin, 0);
        analogWrite(lpwmPin, 0);
#if MOTOR_HAS_ENABLE_PINS
        digitalWrite(rEnablePin, LOW);
        digitalWrite(lEnablePin, LOW);
#endif
    }
}

int Motor::getSpeed() const {
    return speed;
}