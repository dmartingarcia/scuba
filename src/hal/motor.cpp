#include "motor.h"
#include <Arduino.h>
#include "../logic/speed_utils.h"

bool Motor::init() {
    pinMode(rpwmPin, OUTPUT);
    pinMode(lpwmPin, OUTPUT);
    pinMode(rEnablePin, OUTPUT);
    pinMode(lEnablePin, OUTPUT);
    speed = 0;
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
    digitalWrite(rEnablePin, LOW); // Enable right motor
    digitalWrite(lEnablePin, LOW); // Enable left motor
    initialized = true;
    return true;
}

int Motor::checkSpeed(int speed) {
    return clampSpeed(speed);
}

void Motor::setFakeDisabled(bool disabled) {
    fakeDisabled = disabled;
    if (disabled && initialized) {
        // Force the physical outputs off immediately, regardless of
        // whatever speed was last actually applied.
        analogWrite(rpwmPin, 0);
        analogWrite(lpwmPin, 0);
        digitalWrite(rEnablePin, LOW);
        digitalWrite(lEnablePin, LOW);
    }
}

bool Motor::isFakeDisabled() const {
    return fakeDisabled;
}

void Motor::setSpeed(int newSpeed) {
    if (!initialized) return;

    speed = checkSpeed(newSpeed);

    if (fakeDisabled) return; // tracked above via `speed`/getSpeed(), physical pins untouched

    if (speed > 0) {
        // Forward rotation
        digitalWrite(rEnablePin, HIGH); // Disable right motor
        digitalWrite(lEnablePin, HIGH); // Disable left motor

        analogWrite(rpwmPin, abs(speed)); // Set PWM for forward rotation
        analogWrite(lpwmPin, 0);     // No PWM for reverse
    } else if (speed < 0) {
        digitalWrite(rEnablePin, HIGH); // Disable right motor
        digitalWrite(lEnablePin, HIGH); // Disable left motor

        // Reverse rotation
        analogWrite(rpwmPin, 0);         // No PWM for forward
        analogWrite(lpwmPin, abs(speed));    // Set PWM for reverse rotation
    } else {
        // Stop
        analogWrite(rpwmPin, 0);
        analogWrite(lpwmPin, 0);
        digitalWrite(rEnablePin, LOW); // Disable right motor
        digitalWrite(lEnablePin, LOW); // Disable left motor

    }
}

int Motor::getSpeed() const {
    return speed;
}