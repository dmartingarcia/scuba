#include "motor.h"
#include <Arduino.h>

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
    if (speed > 255) {
        speed = 255;
    } else if (speed < -255) {
        speed = -255;
    }
    return speed;
}

void Motor::setSpeed(int newSpeed) {
    if (!initialized) return;

    speed = checkSpeed(newSpeed);

    if (speed > 0) {
        // Forward rotation
        digitalWrite(rEnablePin, HIGH); // Disable right motor
        digitalWrite(lEnablePin, HIGH); // Disable left motor

        analogWrite(rpwmPin, speed); // Set PWM for forward rotation
        analogWrite(lpwmPin, 0);     // No PWM for reverse
    } else if (speed < 0) {
        digitalWrite(rEnablePin, HIGH); // Disable right motor
        digitalWrite(lEnablePin, HIGH); // Disable left motor

        // Reverse rotation
        analogWrite(rpwmPin, 0);         // No PWM for forward
        analogWrite(lpwmPin, -speed);    // Set PWM for reverse rotation
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